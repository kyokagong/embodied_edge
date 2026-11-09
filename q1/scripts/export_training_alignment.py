"""Export captured real Q1 configuration and training differences; no robot writes."""
import argparse
import ast
import csv
import hashlib
import json
import shutil
from pathlib import Path

import yaml


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--contract', type=Path, required=True)
    p.add_argument('--motion-report', type=Path, required=True)
    a = p.parse_args()
    capture = json.loads((a.root / 'capture.json').read_text())
    h = json.loads((a.root / 'deployed_hardware.json').read_text())
    manifest = json.loads((a.root / 'actor_manifest.json').read_text())
    rt = json.loads((a.root / 'rt_manifest.json').read_text())
    c = json.loads(a.contract.read_text())
    assert manifest['profile_sha256'] == hashlib.sha256((a.root/'deployed_hardware.json').read_bytes()).hexdigest()
    assert manifest['hardware'] == rt['hardware']
    for key in ['kp', 'kd', 'lower', 'upper', 'encoder_bias']:
        assert all(abs(x-y) < 1e-5 for x, y in zip(h[key], manifest['hardware'][key]))
    active = capture['last_active_hal_frame']
    assert active and active['mode'] == 'ACTIVE'
    for key in ['kp', 'kd']:
        assert all(abs(x-y) < 1e-5 for x, y in zip(h[key], active[key]))
    factory = yaml.safe_load((a.root/'factory_MotorModule.yaml').read_text())
    hardware = {}
    hal_order = []
    for node in ['upper_node', 'lower_left_node', 'lower_right_node']:
        cfg = factory['manager'][node]['config']
        for i, name in enumerate(cfg['urdf_name']):
            hardware[name] = dict(node=node, min_rad=cfg['min_limits'][i], max_rad=cfg['max_limits'][i],
                                  motor_direction=cfg['direction'][i], motor_offset=cfg['offset'][i],
                                  module_id=cfg['module_id'][i], submodule_id=cfg['submodule_id'][i],
                                  mit_protocol_parameters_raw=cfg['mit_params'][i])
            hal_order.append(name)
    assert len(hardware) == 22 == len(c['joints'])
    # Parse constant dictionaries without importing or executing external project code.
    constants = {}
    for node in ast.parse((a.root/'official/simulator.py').read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id in ['STIFFNESS', 'DAMPING']:
                constants[node.targets[0].id] = ast.literal_eval(node.value)
    rows, training_joints = [], []
    state = capture['state']['bridge']
    for i, joint in enumerate(c['joints']):
        name = joint['name']; f = hardware[name]
        keys = [k for k in constants['STIFFNESS'] if k in name]
        assert len(keys) == 1
        official_kp = constants['STIFFNESS'][keys[0]]
        official_kd = constants['DAMPING'][keys[0]]
        r = dict(index=i, name=name, hal_slot=hal_order.index(name),
                 training_kp=joint['kp'], body_active_kp=h['kp'][i], kp_delta=h['kp'][i]-joint['kp'],
                 training_kd=joint['kd'], body_active_kd=h['kd'][i], kd_delta=h['kd'][i]-joint['kd'],
                 kd_ratio=h['kd'][i]/joint['kd'],
                 training_effort_nm=joint['effort'], body_software_effort_nm=h['stand_trial']['effort_limit'][i],
                 effort_delta_nm=h['stand_trial']['effort_limit'][i]-joint['effort'],
                 training_lower_rad=joint['model_range_rad'][0], training_upper_rad=joint['model_range_rad'][1],
                 factory_lower_rad=f['min_rad'], factory_upper_rad=f['max_rad'],
                 runtime_lower_rad=h['lower'][i], runtime_upper_rad=h['upper'][i],
                 training_velocity_rad_s=joint['velocity_limit'], policy_slew_limit_rad_s=h['stand_trial']['policy_velocity_limit'][i],
                 measured_speed_soft_limit_rad_s=h['stand_trial']['measured_velocity_limit'][i],
                 prepare_only_velocity_rad_s=h['stand_trial']['velocity_limit'][i],
                 training_home_rad=joint['home'], body_home_rad=h['stand_trial']['home'][i],
                 training_armature=joint['armature'], body_measured_armature=None,
                 motor_direction=f['motor_direction'], motor_offset_rad=f['motor_offset'],
                 hal_sign=h['signs'][i], hal_zero=h['zeros'][i], encoder_bias=h['encoder_bias'][i],
                 observed_disabled_kp=state['commanded_kp'][i], observed_disabled_kd=state['commanded_kd'][i],
                 official_training_kp=official_kp, official_training_kd=official_kd)
        rows.append(r)
        training_joints.append(dict(name=name, kp=r['body_active_kp'], kd=r['body_active_kd'],
                                    software_effort_limit_nm=r['body_software_effort_nm'],
                                    command_position_range_rad=[r['runtime_lower_rad'], r['runtime_upper_rad']],
                                    factory_position_range_rad=[r['factory_lower_rad'], r['factory_upper_rad']],
                                    configured_policy_slew_rad_s=r['policy_slew_limit_rad_s'],
                                    configured_measured_speed_soft_limit_rad_s=r['measured_speed_soft_limit_rad_s'],
                                    home_rad=r['body_home_rad'],
                                    physical_torque_speed_curve=None, measured_armature=None,
                                    measured_friction=None, motor_protocol=f))
    with (a.root/'joint_comparison.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    dump(a.root/'joint_comparison.json', rows)
    shutil.copyfile(a.contract, a.root/'training_contract_iter22100.json')
    limits = json.loads((a.motion_report/'episode-2.json').read_text())
    audit = json.loads((a.motion_report/'input-and-tracking-audit.json').read_text())[-1]
    fields = ['t', 'frame', 'observation_age_ms', 'target_age_ms', 'joint_receive_age_ms', 'imu_receive_age_ms']
    with (a.root/'real_latency_samples.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for sample in audit['series']:
            writer.writerow({k: sample[k] if k in ['t', 'frame'] else sample['stages'][k] for k in fields})
    differences = dict(
        kp=[r['name'] for r in rows if abs(r['kp_delta'])>1e-5],
        kd=[r['name'] for r in rows if abs(r['kd_delta'])>1e-5],
        software_effort=[r['name'] for r in rows if abs(r['effort_delta_nm'])>1e-5],
        command_position_range=[r['name'] for r in rows if
                                abs(r['runtime_lower_rad']-r['training_lower_rad'])>1e-5 or
                                abs(r['runtime_upper_rad']-r['training_upper_rad'])>1e-5],
        official_kp=[r['name'] for r in rows if abs(r['body_active_kp']-r['official_training_kp'])>1e-5],
        official_kd=[r['name'] for r in rows if abs(r['body_active_kd']-r['official_training_kd'])>1e-5])
    baseline = dict(
        schema='q1-current-deployment-training-reference-v1',
        purpose='Reproduce this deployed control stack in training; not a calibrated physical robot model or a drop-in runtime config.',
        captured_at_utc=capture['captured_at_utc'], profile_sha256=manifest['profile_sha256'],
        configured_and_actual_active_gains_agree=True,
        inactive_snapshot=dict(phase=capture['state']['phase'], released=state['released'],
                               stiffness=state['commanded_kp'], damping=state['commanded_kd']),
        joint_order=[r['name'] for r in rows], hal_order=hal_order,
        joints=training_joints,
        action=dict(scale=c['action']['scale'], raw_clip=c['action']['clip'],
                    policy_previous_action='clip(raw, -20, 20), before mechanical/slew/RT limits',
                    raw_to_position='home + 0.25 * policy_raw',
                    velocity_target_rad_s=0., feedforward_effort_nm=0.,
                    motor_offset_application='Factory HAL already converts to URDF coordinates. Do not apply factory motor direction/offset again.'),
        runtime=dict(policy_hz=c['policy_hz'], actual_hal_publish_hz=limits['hal_hz'], configured_hal_publish_hz=active['requested_publish_hz'],
                     monitor_scheduler_target_hz=500, driver_poll_config_ms=factory['manager']['publish_interval_ms'],
                     joint_feedback_config_ms=factory['rt_joint_topic_manager']['publish_interval_ms'],
                     imu_feedback_config_ms=factory['rt_imu_topic_manager']['publish_interval_ms'],
                     measured_observation_to_hal_ms=limits['observation_to_hal_ms'],
                     measured_inference_ms=limits['inference_ms'],
                     actuator_internal_servo_hz=None, hal_delivery_to_physical_response_delay_ms=None,
                     inference_prediction_reuse_fraction=limits['actor_stage_metrics']['prediction_reused_fraction']),
        runtime_wrapper=dict(soft_limits=h['stand_trial']['soft_limits'],
                             tracking_error_correction_limit_rad=h['max_tracking_error_rad'],
                             tilt_limit_rad=h['max_tilt_rad'],
                             tilt_scale='clamp(max_tilt_rad / max(tilt,max_tilt_rad), 0.2, 1)',
                             tracking_rule='qcmd=clamp(qcmd,q_measured-0.5*tilt_scale,q_measured+0.5*tilt_scale)',
                             torque_rule='clip(Kp*(qcmd-q)-Kd*qd, +/-effort_limit); convert back to position',
                             speed_rule='When torque*qd>0, multiply torque by clamp((v_limit-abs(qd))/(0.2*v_limit),0,1)',
                             policy_target_max_step_rad=h['max_step_rad'],
                             state_timeout_s=h['state_timeout_s'], target_timeout_s=h['target_timeout_s'],
                             sensor_skew_s=h['sensor_skew_s'], temperature_limit=h['max_temperature'],
                             prepare_seconds=h['stand_trial']['prepare_seconds'],
                             prepare_only_velocity_rad_s=h['stand_trial']['velocity_limit'],
                             stand_takeover_s=3., model_handover_s=3.),
        observation=dict(contract=c['input'], imu_frame=h['imu_frame'], imu_to_pelvis_assumed=h['imu_to_pelvis'],
                         imu_mounting_physically_verified=False, encoder_bias_configured=h['encoder_bias'],
                         encoder_bias_physically_calibrated=False,
                         mode_transition='Preserve real state and history; dance reference starts at frame 0. No reference state teleport.'),
        unmeasured_physics=dict(imu_location_and_mount_rotation=None, link_mass_and_inertia=None,
                                contact_friction=None, motor_torque_speed_curve=None, actuator_armature=None,
                                motor_deadband_and_backlash=None, actual_load_distribution=None),
        model_vs_body_difference_counts={k:len(v) for k,v in differences.items()},
        warnings=['Kp/Kd are real HAL command gains, not independent identification of physical actuator dynamics.',
                  'Software effort/velocity caps are not certified motor ratings.',
                  'MIT arrays are raw protocol parameters, not calibrated continuous torque/speed envelopes.',
                  'Only prepare uses 0.08 rad/s; policy/dance use the separately exported 32/36 rad/s slew bounds.',
                  'Latency CSV is empirical observation-to-HAL timing, not a fitted motor delay distribution.'])
    dump(a.root/'body_training_reference.json', baseline)
    dump(a.root/'differences.json', differences)
    provenance = dict(captured_at_utc=capture['captured_at_utc'], sources={name:{k:v for k,v in info.items() if k!='content'}
                     for name,info in capture['files'].items()}, actual_active_frame=active,
                     processes=capture['processes'], training_contract_sha256=hashlib.sha256(a.contract.read_bytes()).hexdigest())
    dump(a.root/'provenance.json', provenance)
    # Offline-only verification against actual captured configs/commands, never fabricated telemetry.
    verification = dict(real_robot_capture=True, joint_count=len(rows), active_hal_gains_match_profile=True,
                        actor_rt_profile_match=True, configured_profile_hash_verified=True,
                        released_at_capture=state['released'], motion_commands_sent=False,
                        difference_counts={k:len(v) for k,v in differences.items()})
    dump(a.root/'export_verification.json', verification)
    print(json.dumps(verification, ensure_ascii=False))


if __name__ == '__main__':
    main()
