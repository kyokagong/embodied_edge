"""Read the real controller's HTTP snapshots. Never sends a POST or ROS command."""
import argparse
import json
from pathlib import Path
import time
import urllib.request
import statistics


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--url', default='http://127.0.0.1:8768')
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--guardian-fallback', action='store_true',
                   help='Observe a latched real commissioning guard; its fault hold writes HAL, the model stays shadow-only')
    args=p.parse_args()
    if not 1<=args.seconds<=1800:
        p.error('seconds must be in [1,1800]')
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    rows=[]
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:
        with opener.open(args.url+'/api/state', timeout=2) as f:
            s=json.load(f)
        if s['backend']!='ros2' or s['controller']!='supermimic':
            raise ValueError('Not the real SuperMimic backend')
        if args.guardian_fallback:
            if s['bridge']['mode']!='FAULT' or not s['bridge']['fault'] or s['enabled']:
                raise ValueError('Expected a latched guard with model permission disabled')
        elif s['bridge']['tx_count']!=0 or s['actuator_active']:
            raise ValueError('This read-only acceptance requires no actuator writes')
        rows.append(s)
        time.sleep(.1)
    s=rows[-1]
    sensors=s['bridge']['sensors']
    delta=s['policy_steps']-rows[0]['policy_steps']
    report={'backend':'real_ros2','hardware_command_sent':args.guardian_fallback,
            'model_command_sent':False,'guardian_fault_hold':args.guardian_fallback,'samples':len(rows),
            'duration_s':time.monotonic()-start,'joint_complete':sensors['complete'],
            'imu_valid':sensors['imu_valid'],'joint_names':s['joint_names'],
            'joint_frames':sensors['joint_frames'],'imu_frames':sensors['imu_frames'],
            'policy_steps':s['policy_steps'],'parity':s['parity'],
            'policy_steps_during_test':delta,
            'observed_policy_hz':delta/(time.monotonic()-start),
            'fresh_feedback_samples':sum(0<=v['sensor_age_ms']<=60 for v in rows),
            'feedback_age_ms_max_sampled':max(v['sensor_age_ms'] for v in rows),
            'root_velocity_available':sensors.get('root_velocity') is not None,
            'scheduler_hz':s['bridge']['scheduler_hz'],
            'scheduler_max_gap_ms':s['bridge']['scheduler_max_gap_ms'],
            'blockers':s['blockers'],'fault':s['fault'],
            'inference_ms_max_sampled':max(v['inference_ms'] for v in rows),
            'motion_qualified':False}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    args.output.with_suffix('.last-state.json').write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))
    if not sensors['complete'] or not sensors['imu_valid'] or delta==0:
        raise SystemExit('Real observation/inference did not become ready; inspect blockers')


if __name__=='__main__':
    main()
