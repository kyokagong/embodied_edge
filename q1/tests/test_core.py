import copy
import unittest
try:
    from q1_edge.core import Controller
    from q1_edge.config import load_config
except ImportError:
    Controller = None

UPPER = {
 "waist_roll_joint":0., "waist_yaw_joint":0.,
 "left_shoulder_pitch_joint":0., "left_shoulder_roll_joint":.2,
 "left_shoulder_yaw_joint":0., "left_elbow_joint":1.,
 "right_shoulder_pitch_joint":0., "right_shoulder_roll_joint":-.2,
 "right_shoulder_yaw_joint":0., "right_elbow_joint":1.}

class Device:
    kind="mock"
    def __init__(self):
        self.action="BIPED_WALK_RUN";self.velocity=(0.,0.,0.);self.upper=dict(UPPER);self.active=False;self.stops=0
    def enable(self):self.active=True
    def request_mode(self,mode):self.action="BIPED_CUSTOM_UPPER" if mode=="teleop" else "BIPED_WALK_RUN"
    def set_velocity(self,forward,lateral,angular):self.velocity=(forward,lateral,angular)
    def set_upper(self,p):self.upper=dict(p)
    def stop(self):self.velocity=(0.,0.,0.);self.active=False;self.stops+=1
    def close(self):self.stop()
    def snapshot(self):return dict(connected=True,action=self.action,action_running=True,joints=dict(UPPER),joints_age_s=0.,source_active=self.active,fault=None)

class CoreTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(Controller,"Q1 controller has not been implemented")
        self.now=100.;self.device=Device();self.seq=0
        self.core=Controller(self.device,load_config(),clock=lambda:self.now)
        self.send()
    def send(self,axes=None,buttons=None,client="a",source="simulator",connected=True,sequence=None):
        self.seq+=1
        return self.core.input(dict(client_id=client,source=source,sequence=self.seq if sequence is None else sequence,axes=axes or [0.,0.,0.,0.],buttons=buttons or [False]*17,connected=connected))
    def start_walk(self):
        self.core.action("calibrate",client_id="a");self.core.action("enable",client_id="a");self.core.action("walk",client_id="a")
        b=[False]*17;b[5]=True;self.send([0.,-1.,0.,0.],b);self.core.tick()
    def test_logger_records_controls_but_not_identical_input_heartbeats(self):
        with self.assertLogs("q1_edge.core", level="INFO") as logs:
            self.core.action("calibrate", client_id="a")
            buttons=[False]*17; buttons[5]=True
            self.send(buttons=buttons)
            for _ in range(5): self.send(buttons=buttons)
            self.assertEqual(sum("input_buttons" in line for line in logs.output), 1)
            self.send(connected=False)
        text="\n".join(logs.output)
        self.assertIn("control_action", text)
        self.assertIn("calibrate", text)
        self.assertIn("input_buttons", text)
        self.assertIn("deadman=True", text)
        self.assertIn("input_connection", text)
        self.assertIn("connected=False", text)
        self.assertEqual(sum("input_buttons" in line for line in logs.output), 2)

    def test_compact_input_does_not_build_full_robot_state(self):
        from unittest.mock import patch
        payload=dict(client_id='a',source='simulator',sequence=2,axes=[0.]*4,buttons=[False]*17,connected=True)
        with patch.object(self.core,'state',side_effect=AssertionError('full state on heartbeat')):
            ack=self.core.input(payload,compact=True)
        self.assertEqual(ack['sequence'],2)
        self.assertEqual(self.core._seq,2)
        self.assertNotIn('robot',ack)

    def test_compact_inputs_reject_replays_and_invalid_axes_without_refreshing(self):
        payload=dict(client_id='a',source='simulator',sequence=2,axes=[0.]*4,buttons=[False]*17,connected=True)
        self.core.input(payload,compact=True);received=self.core._received
        self.now+=.1
        with self.assertRaises(ValueError):self.core.input(payload,compact=True)
        payload.update(sequence=3,axes=[0.,float('nan'),0.,0.])
        with self.assertRaises(ValueError):self.core.input(payload,compact=True)
        self.assertEqual(self.core._received,received)

    def test_input_gap_and_untrusted_link_metrics_are_diagnostic_only(self):
        self.now+=.21
        payload=dict(client_id='a',source='simulator',sequence=2,axes=[0.]*4,buttons=[False]*17,connected=True,
                     telemetry={'rtt_ms':125.,'timeouts':2,'max_rtt_ms':240.,'unsafe':'ignore','errors':float('nan')})
        with self.assertLogs('q1_edge.core',level='WARNING') as logs:self.core.input(payload)
        self.assertIn('input_gap', '\n'.join(logs.output))
        metrics=self.core.state()['input']['link']
        self.assertAlmostEqual(metrics['last_gap_ms'],210.)
        self.assertAlmostEqual(metrics['max_gap_ms'],210.)
        self.assertEqual(metrics['client']['rtt_ms'],125.)
        self.assertNotIn('unsafe',metrics['client']);self.assertNotIn('errors',metrics['client'])
        self.assertFalse(self.core.enabled)

    def test_input_timeout_logs_exact_age_and_sequence_before_recovery(self):
        self.start_walk();self.now+=.3
        with self.assertLogs('q1_edge.core',level='WARNING') as logs:self.core.tick()
        text='\n'.join(logs.output)
        self.assertIn('stop_context',text);self.assertIn('input_age_ms=300.0',text)
        self.assertIn('sequence=',text)
        self.assertFalse(self.core.enabled);self.assertEqual(self.device.velocity,(0.,0.,0.))
        self.send();self.core.tick();self.assertFalse(self.core.enabled)

    def physical_ready(self):
        self.core=Controller(self.device,load_config(),clock=lambda:self.now)
        self.send(source='browser_gamepad')
        self.core.action('calibrate',client_id='a')

    def hold_rb(self,seconds=1.0):
        buttons=[False]*17;buttons[5]=True
        start=self.now;self.send(buttons=buttons,source='browser_gamepad')
        steps=round(seconds/.1)
        for i in range(1,steps+1):
            self.now=start+i*.1
            self.send(buttons=buttons,source='browser_gamepad');self.core.tick()

    def test_rb_long_press_toggles_once_and_release_keeps_armed(self):
        self.physical_ready();self.hold_rb(2.)
        self.assertTrue(self.core.enabled)
        self.assertTrue(self.core.state()['input']['motion_latched'])
        self.send(source='browser_gamepad')
        self.core.action('walk',client_id='a')
        self.send(axes=[0.,-1.,0.,0.],source='browser_gamepad');self.core.tick()
        self.assertGreater(self.device.velocity[0],0)
        self.send(source='browser_gamepad');self.core.tick()
        self.assertEqual(self.device.velocity,(0.,0.,0.))
        self.hold_rb(1.)
        self.assertFalse(self.core.enabled)
        self.assertFalse(self.core.state()['input']['motion_latched'])
        self.assertEqual(self.device.velocity,(0.,0.,0.))

    def test_rb_can_enable_again_after_explicit_toggle_off(self):
        self.physical_ready();self.hold_rb(1.)
        self.send(source='browser_gamepad');self.hold_rb(1.)
        self.assertFalse(self.core.enabled);self.assertTrue(self.core.calibrated)
        self.send(source='browser_gamepad');self.hold_rb(1.)
        self.assertTrue(self.core.enabled)

    def test_toggle_off_requires_release_before_late_enable_edges_or_actions(self):
        self.physical_ready();self.hold_rb(1.)
        self.send(source='browser_gamepad');self.hold_rb(1.)
        buttons=[False]*17;buttons[5]=buttons[9]=True
        self.send(buttons=buttons,source='browser_gamepad');self.core.tick()
        self.assertFalse(self.core.enabled)
        with self.assertRaises(ValueError):self.core.action('enable',client_id='a')
        self.assertFalse(self.core.state()['input']['motion_latched'])
        self.send(source='browser_gamepad');self.hold_rb(1.)
        self.assertTrue(self.core.enabled)

    def test_stop_fences_late_old_enable_frame(self):
        self.physical_ready();self.hold_rb(1.)
        self.send(source='browser_gamepad')
        self.core.action('stop',client_id='a')
        buttons=[False]*17;buttons[8]=buttons[9]=True
        self.send(buttons=buttons,source='browser_gamepad');self.core.tick()
        self.assertFalse(self.core.enabled)
        self.assertFalse(self.core.calibrated)
        self.assertFalse(self.core.state()['input']['motion_latched'])
        self.assertEqual(self.device.velocity,(0.,0.,0.))

    def test_short_rb_or_non_neutral_hold_cannot_arm(self):
        self.physical_ready();self.hold_rb(.9)
        self.assertFalse(self.core.enabled)
        self.send(source='browser_gamepad')
        buttons=[False]*17;buttons[5]=True
        start=self.now
        for i in range(12):
            self.now=start+i*.1
            self.send(axes=[.5,0.,0.,0.],buttons=buttons,source='browser_gamepad')
        self.assertFalse(self.core.enabled)
        self.assertEqual(self.device.velocity,(0.,0.,0.))

    def test_stop_button_wins_over_same_frame_long_press_and_gap_clears_latch(self):
        self.physical_ready();self.hold_rb(.9)
        buttons=[False]*17;buttons[5]=buttons[1]=True
        self.now+=.1;self.send(buttons=buttons,source='browser_gamepad')
        self.assertFalse(self.core.enabled)
        self.send(source='browser_gamepad');self.core.action('calibrate',client_id='a');self.hold_rb(1.)
        self.assertTrue(self.core.enabled)
        self.now+=.3;self.core.tick()
        self.assertFalse(self.core.enabled)
        self.assertFalse(self.core.state()['input']['motion_latched'])
        self.hold_rb(1.2)
        self.assertFalse(self.core.enabled)

    def test_bounded_jitter_and_late_stop_frame_never_auto_rearm(self):
        self.physical_ready();self.hold_rb(1.)
        self.send(source='browser_gamepad');self.core.action('walk',client_id='a')
        for i in range(100):
            self.now+=(.05,.12,.18,.20)[i%4]
            self.send(axes=[0.,-.4,(-.5 if i%2 else .5),0.],source='browser_gamepad');self.core.tick()
            self.assertTrue(self.core.enabled)
        self.now+=.251
        buttons=[False]*17;buttons[8]=buttons[9]=True
        self.send(buttons=buttons,source='browser_gamepad');self.core.tick()
        self.assertFalse(self.core.enabled);self.assertFalse(self.core.calibrated)
        self.assertFalse(self.core.state()['input']['motion_latched'])
        self.assertEqual(self.device.velocity,(0.,0.,0.))
        self.send(source='browser_gamepad');self.hold_rb(1.2)
        self.assertFalse(self.core.enabled)

    def test_right_stick_turns_body_both_ways_after_latched_enable(self):
        self.physical_ready();self.hold_rb(1.)
        self.send(source='browser_gamepad');self.core.action('walk',client_id='a')
        self.send(axes=[0.,0.,1.,0.],source='browser_gamepad');self.core.tick()
        self.assertEqual(self.device.velocity,(0.,0.,-.4))
        self.send(axes=[0.,0.,-1.,0.],source='browser_gamepad');self.core.tick()
        self.assertEqual(self.device.velocity,(0.,0.,.4))
        self.send(source='browser_gamepad');self.core.tick()
        self.assertEqual(self.device.velocity,(0.,0.,0.))

    def test_uncalibrated_cannot_enable(self):
        with self.assertRaises(ValueError):self.core.action("enable",client_id="a")
        self.assertFalse(self.core.state()["enabled"])
        self.assertEqual(self.device.velocity,(0.,0.,0.))
    def test_explicit_walk_and_rb_can_request_bounded_initial_acquisition(self):
        self.core.action("calibrate",client_id="a")
        self.core.action("enable",client_id="a")
        self.core.action("walk",client_id="a")
        self.device.active=False
        original=self.device.snapshot
        self.device.snapshot=lambda:dict(original(),source_can_acquire=True)
        buttons=[False]*17;buttons[5]=True
        self.send([0.,-1.,0.,0.],buttons);self.core.tick()
        self.assertGreater(self.device.velocity[0],0)
        self.send();self.core.tick()
        self.assertEqual(self.device.velocity,(0.,0.,0.))

    def test_deadman_is_required_and_release_immediately_zeros(self):
        self.start_walk();self.assertGreater(self.device.velocity[0],0.)
        self.send([0.,-1.,0.,0.]);self.core.tick()
        self.assertEqual(self.device.velocity,(0.,0.,0.))
        self.assertTrue(self.core.state()["enabled"])
    def test_stale_input_latches_and_reconnect_never_resumes(self):
        self.start_walk();self.now+=.3;self.core.tick()
        self.assertFalse(self.core.state()["enabled"]);self.assertEqual(self.device.velocity,(0.,0.,0.))
        b=[False]*17;b[5]=True;self.send([0.,-1.,0.,0.],b);self.core.tick()
        self.assertFalse(self.core.state()["enabled"]);self.assertEqual(self.device.velocity,(0.,0.,0.))
    def test_replayed_sequence_cannot_refresh_deadman(self):
        self.start_walk();old=self.seq;self.now+=.15
        with self.assertRaises(ValueError):self.send(sequence=old)
        self.now+=.15;self.core.tick();self.assertFalse(self.core.state()["enabled"])
    def test_initial_held_start_does_not_enable(self):
        c=Controller(Device(),load_config(),clock=lambda:self.now)
        b=[False]*17;b[9]=True
        c.input(dict(client_id="new",source="simulator",sequence=0,axes=[0]*4,buttons=b,connected=True))
        self.assertFalse(c.state()["enabled"])
    def test_non_neutral_calibration_refused(self):
        self.send([.8,0,0,0])
        with self.assertRaises(ValueError):self.core.action("calibrate",client_id="a")
        self.assertFalse(self.core.state()["calibrated"])
    def test_non_finite_input_rejected_before_refresh(self):
        self.start_walk();self.now+=.15
        with self.assertRaises(ValueError):self.send([float("nan"),0,0,0])
        self.now+=.15;self.core.tick();self.assertEqual(self.device.velocity,(0.,0.,0.))
    def test_second_tab_cannot_steal_active_controller(self):
        self.start_walk()
        with self.assertRaises(ValueError):self.send(client="other")
        with self.assertRaises(ValueError):self.core.action("stand",client_id="other")
        self.core.action("stop",client_id="other");self.assertFalse(self.core.state()["enabled"])
    def test_disconnect_revokes_calibration_and_motion(self):
        self.start_walk();self.send(connected=False);self.core.tick()
        self.assertFalse(self.core.state()["enabled"]);self.assertFalse(self.core.state()["calibrated"])
        self.assertEqual(self.device.velocity,(0.,0.,0.))
    def test_real_backend_rejects_simulator_input(self):
        self.device.kind="ros2"
        with self.assertRaises(ValueError):self.send()
        self.assertFalse(self.core.state()["enabled"])
    def test_source_preemption_latches_zero(self):
        self.start_walk();self.device.active=False;self.core.tick()
        self.assertFalse(self.core.state()["enabled"]);self.assertEqual(self.device.velocity,(0.,0.,0.))
    def test_explicit_enable_can_recover_backend_latched_fault(self):
        self.start_walk();self.now+=.3;self.core.tick()
        self.send();self.core.action("clear_fault");self.core.action("calibrate",client_id="a")
        original=self.device.snapshot
        backend_fault=["latched watchdog"]
        self.device.snapshot=lambda:dict(original(),fault=backend_fault[0])
        def enable():self.device.active=True;backend_fault[0]=None
        self.device.enable=enable
        self.core.action("enable",client_id="a")
        self.assertTrue(self.core.state()["enabled"])
        self.assertIsNone(backend_fault[0])
    def test_upper_waits_for_owned_source_before_publishing(self):
        from q1_edge.pico import PicoRetargeter
        self.core.retargeter=PicoRetargeter()
        frame=dict(timestamp_ns=1000000000,head=[0,1.6,0,0,0,0,1],left=[-.25,1.2,-.35,0,0,0,1],right=[.25,1.2,-.35,0,0,0,1],left_grip=0.,right_grip=0.,valid=True)
        self.core.pico_frame(frame);self.core.action("calibrate",client_id="a");self.core.action("enable",client_id="a");self.core.action("teleop",client_id="a")
        self.device.active=False
        def refuse(p):raise RuntimeError("upper published before ownership")
        self.device.set_upper=refuse;self.core.tick()
        self.assertTrue(self.core.state()["enabled"])
        self.assertEqual(self.core.state()["mode"],"teleop")
    def test_absent_browser_pad_preserves_native_pico_calibration(self):
        from q1_edge.pico import PicoRetargeter
        c=Controller(Device(),load_config(),PicoRetargeter(),clock=lambda:self.now)
        frame=dict(timestamp_ns=1000000000,head=[0,1.6,0,0,0,0,1],left=[-.25,1.2,-.35,0,0,0,1],right=[.25,1.2,-.35,0,0,0,1],left_grip=0.,right_grip=0.,valid=True)
        c.pico_frame(frame,source="xrobotoolkit");c.action("calibrate");c.action("enable")
        c.input(dict(client_id="absent-pad",source="browser_gamepad",sequence=1,axes=[0]*4,buttons=[],connected=False))
        c.tick()
        self.assertTrue(c.state()["pico"]["calibrated"])
        self.assertTrue(c.state()["enabled"])
    def test_dead_backend_feedback_stops_walking(self):
        self.start_walk()
        original=self.device.snapshot
        self.device.snapshot=lambda:dict(original(),joints_age_s=1.)
        self.core.tick()
        self.assertFalse(self.core.state()["enabled"])
        self.assertEqual(self.device.velocity,(0.,0.,0.))
    def test_failed_stop_transport_does_not_kill_fault_state(self):
        self.start_walk()
        def broken():raise RuntimeError("transport disappeared")
        self.device.stop=broken;self.now+=.3
        self.core.tick()
        self.assertFalse(self.core.state()["enabled"])
        self.assertEqual(self.core.state()["mode"],"fault")
    def test_software_stop_preserves_native_balanced_hold(self):
        self.start_walk();self.core.action("stop");self.core.tick()
        self.assertFalse(self.core.state()["enabled"]);self.assertEqual(self.device.action,"BIPED_WALK_RUN")
        self.assertEqual(self.device.velocity,(0.,0.,0.))
    def test_stop_wins_over_simultaneous_enable_and_mode_buttons(self):
        self.core.action("calibrate");self.core.action("enable")
        b=[False]*17
        for index in (1,9,2):b[index]=True
        self.send(buttons=b);self.core.tick()
        self.assertFalse(self.core.state()["enabled"])
        self.assertEqual(self.device.velocity,(0.,0.,0.))
        self.assertEqual(self.core.state()["mode"],"idle")
    def test_new_input_owner_invalidates_previous_calibration_while_disabled(self):
        self.core.action("calibrate");self.now+=.3
        self.send(client="other")
        self.assertFalse(self.core.state()["calibrated"])
        with self.assertRaises(ValueError):self.core.action("enable",client_id="other")

if __name__=="__main__":unittest.main()
