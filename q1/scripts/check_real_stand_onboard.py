"""Run ON THE Q1: real UI commands and real HAL/encoder evidence.

No fake ROS peer, generated joint feedback, or simulated gamepad input.
"""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.request
import urllib.error
import uuid


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--takeover-only',action='store_true')
    p.add_argument('--leave-standing',action='store_true')
    p.add_argument('--release-first',action='store_true')
    p.add_argument('--chain-only',action='store_true',help='Verify real bounded model pipeline; does not certify ground standing')
    p.add_argument('--leave-running',action='store_true')
    p.add_argument('--release-after',action='store_true')
    o=p.parse_args()
    if not 1<=o.seconds<=90:raise ValueError('bounded test is 1..90 seconds')
    client='real-onboard-'+uuid.uuid4().hex;sequence=0
    op=urllib.request.build_opener(urllib.request.ProxyHandler({}));base='http://127.0.0.1:8768'
    def get():
        with op.open(base+'/api/state',timeout=2) as f:return json.load(f)
    def post(action):
        nonlocal sequence
        sequence+=1
        body={'action':action,'client_id':client,'sequence':sequence,'request_id':uuid.uuid4().hex}
        r=urllib.request.Request(base+'/api/action',data=json.dumps(body).encode(),headers={'Content-Type':'application/json','Origin':base})
        try:
            with op.open(r,timeout=2) as f:return json.load(f)
        except urllib.error.HTTPError as e:
            raise RuntimeError(f'{action}: HTTP {e.code}: {e.read().decode()}') from e
    before=get();assert before['controller']=='supermimic_stand' and before['bridge']['controlled_joints']==22
    post('pair')
    if o.release_first:
        released_before=before;post('release');release_rows=[];settled=None;until=time.monotonic()+15
        while time.monotonic()<until:
            post('heartbeat');s=get();release_rows.append(s)
            ready=s['bridge']['released'] and s['bridge']['applied_gain_scale']==0 and max(abs(v) for v in s['bridge']['sensors']['qd'])<.25
            settled=(settled or time.monotonic()) if ready else None
            if settled and time.monotonic()-settled>1:break
            time.sleep(.1)
        release_report={'real_hardware':True,'executed_on':'Q1 localhost','operation':'zero_stiffness_low_damping_release','motor_power_off':False,'before':released_before,'samples':release_rows,'final':get()}
        release_path=o.report.with_suffix('.release.json');release_path.write_text(json.dumps(release_report,ensure_ascii=False,indent=2)+'\n');release_path.chmod(0o600)
        before=get();assert before['bridge']['released'] and before['bridge']['applied_gain_scale']==0
    if before['guard_fault']:
        post('reset')
        for _ in range(40):
            post('heartbeat');before=get()
            if before['guard_fault'] is None:break
            time.sleep(.1)
    # Reset keeps physical support active. Wait for actual encoder velocities,
    # rather than starting in the transient immediately following reset.
    for _ in range(50):
        post('heartbeat');before=get()
        if before['guard_fault'] is None and max(abs(v) for v in before['bridge']['sensors']['qd'])<.25:break
        time.sleep(.1)
    assert before['guard_fault'] is None
    baseline=before['bridge']['sensors']['q'];tx_before=before['bridge']['tx_count'];model_before=before['model_count']
    publish_hz=before['bridge'].get('publish_hz',500)
    rows=[];error=None;started=time.monotonic();success=False;trace_file=None
    try:
        post('stand')
        duration=2. if o.takeover_only else o.seconds
        while time.monotonic()-started<duration:
            post('heartbeat');s=get();rows.append(s)
            assert s['bridge']['sensors']['complete'] and s['bridge']['sensors']['imu_valid']
            assert all(math.isfinite(v) for v in s['bridge']['commanded_q'])
            if s['phase']=='FAULT' or s['guard_fault']:
                success=False;error=s['fault'] or s['guard_fault'];break
            if not o.takeover_only and not o.chain_only:
                success=bool(s['standing_verified'] and s['phase']=='STANDING')
            if trace_file is None and s['phase'] in ['POLICY_STAND','STANDING']:
                try:
                    data=Path('private/model-trace.json').read_text();trace=json.loads(data)
                    if trace['phase'] in ['POLICY_STAND','STANDING'] and trace['accepted_sequence'] is not None:
                        target=o.report.with_suffix('.input-trace.json');target.write_text(data);target.chmod(0o600);trace_file=str(target)
                except (OSError,ValueError,KeyError):pass
            if not o.takeover_only and not o.chain_only and success:
                if time.monotonic()-started>32:break
            time.sleep(.1)
        if o.takeover_only:success=bool(rows and rows[-1]['bridge']['tx_count']-tx_before>publish_hz*1.5 and not error)
        if o.chain_only:
            policy_rows=[s for s in rows if s['phase'] in ['POLICY_STAND','STANDING']]
            limits=json.loads(Path('config/hardware.json').read_text())['stand_trial']['effort_limit']
            bounded=all(abs(v)<=limits[i]+.1 for s in rows for i,v in enumerate(s['bridge']['estimated_effort']))
            success=bool(not error and len(policy_rows)>=100 and rows[-1]['bridge']['tx_count']-tx_before>publish_hz*o.seconds*.8 and bounded)
    except Exception as e:error=str(e)
    finally:
        if not (success and ((o.leave_standing and not o.takeover_only) or (o.chain_only and o.leave_running))):
            try:post('release' if o.release_after else 'hold');time.sleep(.2)
            except Exception as e:error=error or str(e)
    final=get()
    report={'real_hardware':True,'executed_on':'Q1 localhost','mock_used':False,
            'test':'22_joint_actual_takeover' if o.takeover_only else 'bounded_real_model_chain' if o.chain_only else 'supported_supermimic_stand',
            'success':success,'error':error,'seconds':time.monotonic()-started,
            'hal_frames':final['bridge']['tx_count']-tx_before,'model_steps':final['model_count']-model_before,
            'max_measured_delta_rad':max((abs(s['bridge']['sensors']['q'][i]-baseline[i]) for s in rows for i in range(22)),default=0),
            'before':before,'samples':rows,'final':final,'trace_file':trace_file,
            'standing_verified':final['standing_verified'],'ground_support_verified':False,
            'configured_publish_hz':publish_hz,
            'left_running':bool(success and o.chain_only and o.leave_running),
            'left_standing':bool(success and o.leave_standing and not o.takeover_only and not o.chain_only)}
    o.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');o.report.chmod(0o600)
    print(json.dumps({k:v for k,v in report.items() if k not in ['before','samples','final']},ensure_ascii=False))
    if not success:raise SystemExit(2)


if __name__=='__main__':main()
