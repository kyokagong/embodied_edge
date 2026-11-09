"""Analyze actual onboard JSONL recordings; never creates robot data or commands."""
import argparse
import bisect
import collections
import json
import math
from pathlib import Path
import statistics


def records(directory):
    rows=[];bad=0
    for path in directory.glob('part-*.jsonl'):
        with path.open() as f:
            for line in f:
                try:
                    item=json.loads(line)
                    assert item['schema']==1
                    rows.append(item)
                except (ValueError,KeyError,AssertionError):bad+=1
    rows.sort(key=lambda x:x['log_seq'])
    return [x['record'] for x in rows],bad


def distribution(values):
    data=sorted(values)
    if not data:return None
    def percentile(p):
        index=(len(data)-1)*p;lower=int(index);upper=min(lower+1,len(data)-1)
        return data[lower]+(data[upper]-data[lower])*(index-lower)
    return dict(count=len(data),mean=statistics.fmean(data),min=data[0],p50=percentile(.5),p95=percentile(.95),p99=percentile(.99),max=data[-1])


def rate(times):
    return (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--actor-dir',type=Path,required=True)
    p.add_argument('--rt-dir',type=Path,required=True)
    p.add_argument('--last-state',type=Path)
    p.add_argument('--report',type=Path,required=True)
    o=p.parse_args();actor,bad_actor=records(o.actor_dir);rt,bad_rt=records(o.rt_dir)
    manifest=json.loads((o.actor_dir/'manifest.json').read_text());hardware=manifest['hardware'];home=hardware['stand_trial']['home']
    inputs={r['id']:r for r in actor if r['kind']=='input'}
    inference={r['id']:r for r in actor if r['kind']=='inference'}
    targets=[r for r in actor if r['kind']=='target'];targets.sort(key=lambda r:r['at'])
    by_sequence={(r['target']['epoch'],r['target']['sequence']):r for r in targets}
    active=[r for r in rt if r['mode']=='ACTIVE'];active.sort(key=lambda r:r['at'])
    policy_targets=[r for r in targets if r['phase'] in ['POLICY_STAND','STANDING']]
    policy_ids={r['observation_id'] for r in policy_targets}
    policy_inference=[r for i,r in inference.items() if i in policy_ids]
    first_rt={};hold_counts=collections.Counter();delays=[];input_delays=[]
    for r in active:
        key=(r['epoch'],r['accepted']);hold_counts[key]+=1
        if key not in first_rt:
            first_rt[key]=r
            if r['generated'] is not None:delays.append((r['at']-r['generated'])*1000)
            if r['observed'] is not None:input_delays.append((r['at']-r['observed'])*1000)
    previous_checks=previous_mismatch=shape_errors=gravity_errors=feedback_errors=0
    target_times=[r['at'] for r in targets]
    for r in inputs.values():
        x=r['input']
        if len(x)!=1376 or not all(math.isfinite(v) for v in x):shape_errors+=1;continue
        latest=x[702:780]
        if abs(sum(v*v for v in latest[47:50])-1)>.01:gravity_errors+=1
        if max(abs(latest[i]-(r['sensors']['q'][i]-home[i])) for i in range(22))>1e-5:feedback_errors+=1
        if max(abs(a-b) for a,b in zip(latest[50:72],r['previous_raw']))>1e-5:feedback_errors+=1
        index=bisect.bisect_right(target_times,r['at'])
        recent=[t for t in targets[max(0,index-2):index] if t['target']['epoch']==r['epoch'] and t['phase'] in ['POLICY_STAND','STANDING']]
        if recent:
            previous_checks+=1
            if min(max(abs(a-b) for a,b in zip(t['target']['raw'],r['previous_raw'])) for t in recent)>1e-5:previous_mismatch+=1
    joints=[]
    for i,j in enumerate(manifest['contract']['joints']):
        q=[r['q'][i] for r in active];errors=[r['commanded_q'][i]-r['q'][i] for r in active]
        joints.append(dict(name=j['name'],q_min=min(q,default=0),q_max=max(q,default=0),max_speed=max((abs(r['qd'][i]) for r in active),default=0),tracking_rms=math.sqrt(statistics.fmean(v*v for v in errors)) if errors else None,max_estimated_effort=max((abs(r['estimated_effort'][i]) for r in active),default=0),max_measured_effort=max((abs(r['measured_effort'][i]) for r in active),default=0),limit_frames=sum(r['saturation'][i]!=0 for r in active)))
    policy_raw_max=[max(abs(v) for v in r['candidate_raw']) for r in policy_inference]
    issued_raw_max=[max(abs(v) for v in r['target']['raw']) for r in policy_targets]
    states=[r for r in actor if r['kind']=='state']
    last=json.loads(o.last_state.read_text()) if o.last_state else None
    report=dict(source='real_onboard_flight_records',model_sha256=manifest['model_sha256'],profile_sha256=manifest['profile_sha256'],
        counts=dict(inputs=len(inputs),inference=len(inference),targets=len(targets),published_frames=len(active),partial_or_invalid_json_lines=bad_actor+bad_rt),
        cadence=dict(model_hz=rate(sorted(r['completed'] for r in inference.values())),hal_hz=rate([r['at'] for r in active]),requested_hal_hz=sorted({r['requested_publish_hz'] for r in active}),publish_interval_ms=distribution(r['publish_interval_ms'] for r in active[1:]),monitor_interval_ms=distribution(r['interval_ms'] for r in active),hal_frames_per_policy_target=distribution(hold_counts.values()),inference_ms=distribution(r['ms'] for r in inference.values()),prediction_to_hal_ms=distribution(delays),observation_to_hal_ms=distribution(input_delays)),
        observation_audit=dict(shape_errors=shape_errors,gravity_norm_errors=gravity_errors,real_feedback_or_previous_field_errors=feedback_errors,previous_raw_recent_issued_checks=previous_checks,previous_raw_recent_issued_mismatches=previous_mismatch),
        policy=dict(candidate_raw_max_abs=distribution(policy_raw_max),issued_raw_max_abs=distribution(issued_raw_max),saturated_axes_per_target=distribution(sum(r['model_limited']) for r in policy_targets)),
        phases=[dict(at=r['at'],phase=r['phase'],fault=r['fault']) for r in states],
        max_temperature=max((max(r['temperature']) for r in active),default=None),
        hardware_faults=sorted({r['fault'] for r in rt if r['fault']}),joints=joints,
        rejected_targets=max((r.get('rejected_targets',0) for r in rt),default=0),
        rejection_reasons=sorted({r.get('last_rejection') for r in rt if r.get('last_rejection')}),
        standing_verified=bool(last and last['standing_verified']),ground_support_verified=False,
        log_health=dict(actor=last.get('flight_log') if last else None,rt_written=last['bridge']['log_written'] if last else None,rt_dropped=last['bridge']['log_dropped'] if last else None,rt_failed=last['bridge']['log_failed'] if last else None))
    o.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');o.report.chmod(0o600)
    print(json.dumps({k:report[k] for k in ['counts','cadence','observation_audit','policy','hardware_faults','standing_verified']},ensure_ascii=False))


if __name__=='__main__':main()
