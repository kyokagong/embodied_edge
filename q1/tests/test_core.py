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
    def test_uncalibrated_cannot_enable(self):
        with self.assertRaises(ValueError):self.core.action("enable",client_id="a")
        self.assertFalse(self.core.state()["enabled"])
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
