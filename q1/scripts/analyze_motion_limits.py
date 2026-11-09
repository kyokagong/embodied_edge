"""Read real actor/HAL files; quantify each limiter in the last played dance."""
import argparse,collections,datetime,json,math,re,statistics
from pathlib import Path


def rows(root,kinds=None,window=None):
    for p in Path(root).glob('part-*.jsonl'):
        with p.open('rb') as f:
            remaining=p.stat().st_size
            while remaining>0:
                line=f.readline();remaining-=len(line)
                if not line:break
                if kinds and not any(('"kind":"'+k+'"').encode() in line[:200] for k in kinds):continue
                if window:
                    match=re.search(rb'"(?:at|completed)":([0-9.]+)',line)
                    if not match or not window[0]<=float(match[1])<=window[1]:continue
                try:yield json.loads(line)
                except ValueError:continue


def dist(v):
    v=sorted(v)
    if not v:return None
    return dict(n=len(v),mean=statistics.fmean(v),p50=v[len(v)//2],p95=v[min(len(v)-1,int(len(v)*.95))],max=v[-1])


def main():
    p=argparse.ArgumentParser();p.add_argument('--actor-dir',required=True);p.add_argument('--rt-dir',required=True);p.add_argument('--report',required=True);p.add_argument('--baseline-profile');p.add_argument('--play-request-id');a=p.parse_args()
    events=[r['record'] for r in rows(a.actor_dir,['event']) if not r['record']['error']]
    play=max((r for r in events if r['action']=='play_dance' and (not a.play_request_id or r['request_id']==a.play_request_id)),key=lambda r:r['at'])
    stop=min((r for r in events if r['action'] in ['cancel_dance','release','hold'] and r['at']>play['at']),key=lambda r:r['at'])
    window=(play['at'],stop['at'])
    manifest=json.loads((Path(a.actor_dir)/'manifest.json').read_text());h=manifest['hardware'];home=h['stand_trial']['home'];res=h['stand_trial'].get('policy_residual');names=[r['name'] for r in manifest['contract']['joints']]
    actor=[r['record'] for r in rows(a.actor_dir,['target','state','inference'],window)]
    targets=sorted([r for r in actor if r['kind']=='target' and r['phase']=='DANCING'],key=lambda r:r['at']);by_seq={(r['target']['epoch'],r['target']['sequence']):r for r in targets}
    published=[r['record'] for r in rows(a.rt_dir,window=window)]
    active=[r for r in published if r['mode']=='ACTIVE'];active.sort(key=lambda r:r['at'])
    bits={'joint_bound':1,'rt_step':2,'tracking_correction':4,'torque':8,'measured_speed':16,'tilt':32}
    baseline=json.loads(Path(a.baseline_profile).read_text())['stand_trial']['effort_limit'] if a.baseline_profile else None
    per_joint=[]
    for i,name in enumerate(names):
        requested=[home[i]+.25*max(-20,min(20,r['candidate_raw'][i])) for r in targets]
        limited=[r['target']['q'][i] for r in targets]
        amp=sum(abs(q-home[i])>res[i]+1e-6 for q in requested) if res is not None else 0
        native=sum(q<h['lower'][i]-1e-6 or q>h['upper'][i]+1e-6 for q in requested)
        slew=sum(r['stages']['slew_limited'][i] if 'stages' in r else abs(home[i]+.25*r['target']['raw'][i]-r['target']['q'][i])>1e-5 for r in targets)
        matched=[r for r in active if (r['epoch'],r['accepted']) in by_seq]
        error=[abs(by_seq[(r['epoch'],r['accepted'])]['target']['q'][i]-r['q'][i]) for r in matched]
        changed=[abs(by_seq[(r['epoch'],r['accepted'])]['target']['q'][i]-r['commanded_q'][i]) for r in matched]
        per_joint.append(dict(name=name,amplitude_rad=res[i] if res is not None else None,raw_range=[min(requested),max(requested)],sent_range=[min(limited),max(limited)],amplitude_clipped_fraction=amp/len(targets),native_clipped_fraction=native/len(targets),actor_slew_fraction=slew/len(targets),hal_saturation_fraction={k:sum(bool(r['saturation'][i]&b) for r in active)/len(active) for k,b in bits.items()},max_actual_speed=max(abs(r['qd'][i]) for r in active),policy_to_measured_error_rad=dist(error),target_to_motor_change_rad=dist(changed),max_effort=max(abs(r['measured_effort'][i]) for r in active)))
    for i,joint in enumerate(per_joint):
        stage_frames=[r for r in active if 'stages' in r and r['stages']['pd_active'][i]]
        if stage_frames:
            joint['requested_pd_abs_nm']=dist([abs(r['stages']['before_effort_tau'][i]) for r in stage_frames])
            joint['after_effort_cap_abs_nm']=dist([abs(r['stages']['after_effort_tau'][i]) for r in stage_frames])
            joint['after_speed_scale_abs_nm']=dist([abs(r['stages']['after_speed_tau'][i]) for r in stage_frames])
            joint['effort_cap_nm']=h['stand_trial']['effort_limit'][i]
            joint['actual_effort_cap_fraction']=sum(abs(r['stages']['before_effort_tau'][i]-r['stages']['after_effort_tau'][i])>1e-6 for r in stage_frames)/len(stage_frames)
            joint['actual_speed_scale_fraction']=sum(r['stages']['speed_scale'][i]<.999 for r in stage_frames)/len(stage_frames)
            if baseline:
                joint['baseline_effort_cap_nm']=baseline[i]
                joint['fraction_published_above_baseline_cap']=sum(abs(r['estimated_effort'][i])>baseline[i]+1e-4 for r in stage_frames)/len(stage_frames)
    inf=[r for r in actor if r['kind']=='inference'];dt=[r['publish_interval_ms'] for r in active]
    any_actor_amp=sum(any(r['model_limited']) for r in targets)/len(targets)
    report=dict(source='real_onboard_logs',profile_sha256=manifest['profile_sha256'],window_monotonic=window,duration_s=window[1]-window[0],counts=dict(targets=len(targets),hal=len(active)),model_amplitude_or_native_clip_frames_fraction=any_actor_amp,hal_any_saturation_fraction={k:sum(any(s&b for s in r['saturation']) for r in active)/len(active) for k,b in bits.items()},hal_hz=(len(active)-1)/(active[-1]['at']-active[0]['at']),hal_publish_interval_ms=dist(dt),inference_ms=dist([r['ms'] for r in inf]),observation_to_hal_ms=dist([(r['at']-r['observed'])*1000 for r in active if r['observed'] is not None]),faults=sorted({r['fault'] for r in published if r['fault']}),hal_modes=dict(collections.Counter(r['mode'] for r in published)),state_phases=collections.Counter(r['phase'] for r in actor if r['kind']=='state'),joints=per_joint)
    stage_targets=[r['stages'] for r in targets if 'stages' in r]
    report['explicit_stage_target_records']=len(stage_targets)
    if stage_targets:
        report['actor_stage_metrics']=dict(slew_frames_fraction=sum(any(r['slew_limited']) for r in stage_targets)/len(stage_targets),prediction_reused_fraction=sum(r['prediction_reused'] for r in stage_targets)/len(stage_targets),desired_to_sent_max_rad=dist([max(abs(v) for v in r['desired_to_sent_delta_rad']) for r in stage_targets]),prediction_age_ms=dist([r['prediction_age_ms'] for r in stage_targets if r['prediction_age_ms'] is not None]))
    stage_hal=[r for r in active if 'stages' in r]
    if stage_hal:
        report['separate_pd_limit_metrics']=dict(effort_cap_frames_fraction=sum(any(abs(a-b)>1e-6 for a,b in zip(r['stages']['before_effort_tau'],r['stages']['after_effort_tau'])) for r in stage_hal)/len(stage_hal),speed_scaling_frames_fraction=sum(any(v<.999 for v in r['stages']['speed_scale']) for r in stage_hal)/len(stage_hal),legacy_bit8_meaning='combined PD output adjustment, including speed scaling; not pure effort clipping')
    report['inference_queue_wait_ms']=dist([r['queue_wait_ms'] for r in inf if 'queue_wait_ms' in r])
    after=[r['record'] for r in rows(a.actor_dir,['target','state'],(stop['at'],stop['at']+7))]
    returned=sorted([r for r in after if r['kind']=='target' and r['phase']=='RETURNING_STAND'],key=lambda r:r['at'])
    report['last_return_target']=returned[-1]['target']['q'] if returned else None
    report['returned_to_policy']=any(r['kind']=='state' and r['phase'] in ['POLICY_STAND','STANDING'] and r['at']>stop['at']+5 for r in after)
    report['policy_output_mode']='model_native' if res is None else 'legacy_trial_envelope'
    report['dance_ids']=sorted({r['dance'] for r in actor if r['kind']=='state' and r.get('dance')})
    target=Path(a.report);target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');target.chmod(0o600);print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
