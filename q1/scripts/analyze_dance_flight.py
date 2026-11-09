"""Read private real Q1 recordings on Q1; emits a summary without full tensors."""
import argparse
import collections
import json
from pathlib import Path


def records(directory):
    for path in directory.glob('part-*.jsonl'):
        # Snapshot the extent: the live recorder may append while being audited.
        with path.open('rb') as stream:
            remaining=path.stat().st_size
            while remaining>0:
                line=stream.readline();remaining-=len(line)
                if not line:break
                try:yield json.loads(line)['record']
                except (ValueError,KeyError):continue


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--actor-dir',type=Path,required=True)
    parser.add_argument('--rt-dir',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    events=[r for r in records(args.actor_dir) if r.get('kind')=='event' and not r['error']]
    play=max((r for r in events if r['action']=='play_dance'),key=lambda r:r['at'])
    cancel=min((r for r in events if r['action']=='cancel_dance' and r['at']>play['at']),key=lambda r:r['at'])
    began,ended=play['at'],cancel['at'];until=ended+7
    manifest=json.loads((args.actor_dir/'manifest.json').read_text());hardware=manifest['hardware'];home=hardware['stand_trial']['home']
    modes=collections.Counter();phases=collections.Counter();shapes=bad_feedback=nonzero_goals=0
    input_ids=set();inferred=set();targets=[];return_targets=[];states=[];latencies=[]
    for r in records(args.actor_dir):
        at=r.get('at',r.get('completed',0))
        if not began<=at<=until:continue
        kind=r.get('kind')
        if kind=='input':
            x=r['input'];mode=x[780];modes[str(mode)]+=1
            if mode==1:
                input_ids.add(r['id']);shapes+=len(x)!=1376
                nonzero_goals+=any(abs(v)>1e-8 for v in x[786:])
                expected=[hardware['signs'][i]*(r['sensors']['q'][i]-hardware['zeros'][i])-home[i]+hardware['encoder_bias'][i] for i in range(22)]
                bad_feedback+=max(abs(a-b) for a,b in zip(x[702:724],expected))>1e-5
                bad_feedback+=max(abs(a-b) for a,b in zip(x[752:774],r['previous_raw']))>1e-5
        elif kind=='inference':inferred.add(r['id']);latencies.append(r['ms'])
        elif kind=='target':
            phases[r['phase']]+=1
            if r['phase']=='DANCING':targets.append(r)
            elif r['phase']=='RETURNING_STAND':return_targets.append(r)
        elif kind=='state':states.append(r)
    targets.sort(key=lambda r:r['at']);return_targets.sort(key=lambda r:r['at']);states.sort(key=lambda r:r['at'])
    saved=next((r.get('stand_pose') for r in states if r.get('stand_pose')),None)
    actual=[]
    for r in records(args.rt_dir):
        if began<=r['at']<=until:actual.append(r)
    actual.sort(key=lambda r:r['at']);dance_rt=[r for r in actual if r['at']<=ended]
    assert targets and actual
    initial=targets[0]['target']['q']
    # The test report carries the target captured atomically by play_dance.
    summary=dict(real_hardware=True,source='private_onboard_jsonl',play_request=play['request_id'],cancel_request=cancel['request_id'],
        played_wall_s=ended-began,dance_inputs=len(input_ids),matched_inferences=len(input_ids&inferred),nonzero_goal_inputs=nonzero_goals,
        input_shape_errors=shapes,real_feedback_or_previous_errors=bad_feedback,modes=dict(modes),target_phases=dict(phases),
        last_return_target=return_targets[-1]['target']['q'] if return_targets else None,return_duration_s=return_targets[-1]['at']-return_targets[0]['at'] if return_targets else None,
        cancel_behavior='pose_interpolation' if return_targets else 'zero_velocity_condition',
        dance_hal_frames=len(dance_rt),hal_hz=(len(dance_rt)-1)/(dance_rt[-1]['at']-dance_rt[0]['at']),
        actual_joint_motion_range_rad=[max(r['q'][i] for r in dance_rt)-min(r['q'][i] for r in dance_rt) for i in range(22)],
        hal_faults=sorted({r['fault'] for r in actual if r['fault']}),hal_modes=sorted({r['mode'] for r in actual}),
        model_sha256=manifest['model_sha256'],profile_sha256=manifest['profile_sha256'],
        returned_to_policy=any(r['phase'] in ['POLICY_STAND','STANDING'] and r['at']>ended+5 for r in states))
    args.report.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');args.report.chmod(0o600)
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':main()
