"""Exercise actual Q1 observation-inference tasks through the deployed console.

No synthetic gamepad input or simulated ROS peer is created. Refuses motion mode.
"""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.error
import urllib.request
import uuid


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--url', default='http://127.0.0.1:8768')
    p.add_argument('--operator-key', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    o = p.parse_args()
    key = o.operator_key.read_text().strip()
    client = 'real-q1-flow-'+uuid.uuid4().hex
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def get(path):
        with opener.open(o.url+path, timeout=4) as response:
            return json.load(response)
    def post(path, body, request_id=None):
        payload = {'client_id':client,'request_id':request_id or uuid.uuid4().hex,**body}
        request = urllib.request.Request(o.url+path, data=json.dumps(payload).encode(),
            headers={'Content-Type':'application/json','X-Q1-Operator':key,'Origin':o.url})
        with opener.open(request, timeout=4) as response:
            return json.load(response)
    before = get('/api/state')
    assert before['shadow_only'] and not before['model_actuator_active'] and not before['enabled']
    assert before['deployment']['architecture']=='aarch64' and before['deployment']['inference_location']=='Q1'
    assert before['bridge']['sensors']['complete'] and before['bridge']['sensors']['imu_valid']
    catalog = get('/api/dances')
    assert len(catalog)==8 and all(c['observation_inference_allowed'] for c in catalog)
    clip = next(c['id'] for c in catalog if c['id']=='dance1_subject3')
    post('/api/action', {'action':'stop'})
    post('/api/action', {'action':'enable'})
    assert get('/api/state')['preview_running']
    post('/api/action', {'action':'select','clip_id':clip})
    request_id = uuid.uuid4().hex
    post('/api/dance/queue', {'clip_id':clip}, request_id)
    repeated = post('/api/dance/queue', {'clip_id':clip}, request_id)
    assert repeated.get('duplicate') and get('/api/state')['dance']['queue']==[clip]
    rows = []
    try:
        post('/api/dance/start', {'clip_id':clip})
        for _ in range(15):
            state = get('/api/state')
            assert state['preview_running'] and state['dance']['id']==clip
            assert state['shadow_only'] and not state['enabled'] and not state['model_actuator_active']
            assert state['bridge']['accepted']==before['bridge']['accepted']
            assert all(math.isfinite(v) for v in state['target']) and len(state['target'])==22
            rows.append(state)
            time.sleep(.2)
        assert rows[-1]['dance']['frame']>rows[0]['dance']['frame']
        assert rows[-1]['policy_steps']>rows[0]['policy_steps']
        assert rows[-1]['bridge']['sensors']['joint_frames']>before['bridge']['sensors']['joint_frames']
        assert rows[-1]['bridge']['sensors']['imu_frames']>before['bridge']['sensors']['imu_frames']
    finally:
        post('/api/dance/cancel', {})
    after = get('/api/state')
    assert not after['preview_running'] and after['dance']['id'] is None
    assert after['guard_fault']==before['guard_fault'] and after['bridge']['session']==before['bridge']['session']
    assert after['bridge']['accepted']==before['bridge']['accepted']
    report={'real_q1':True,'mock_used':False,'inference_location':after['deployment'],
            'console':o.url,'flow':['start_observation_inference','select_dance','queue_once','deduplicate_retry','run_dance_inference','advance_reference_frames','cancel'],
            'clip':clip,'first_frame':rows[0]['dance']['frame'],'last_frame':rows[-1]['dance']['frame'],
            'policy_steps_during_dance':rows[-1]['policy_steps']-rows[0]['policy_steps'],
            'joint_feedback_advanced':True,'imu_feedback_advanced':True,
            'model_actuator_commands_sent':False,'guard_session_preserved':True,
            'guard_fault_preserved':after['guard_fault'],'physical_gamepad_tested':False,
            'task_stopped_at_end':True,'motion_qualified':False}
    o.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':
    main()
