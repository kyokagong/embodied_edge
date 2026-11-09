"""SDK boundary checks; only the unavailable ROS transport is replaced."""
import copy
import importlib
import json
import math
from pathlib import Path
import re
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT / "vendor" / "primebot_sdk" / "aimdk_msgs"
NAMES = (
    "waist_roll_joint", "waist_yaw_joint", "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
)


def sdk_types():
    """Construct complete message fields/constants from the pinned .msg/.srv files."""
    classes = {}

    class Stamp:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    classes["builtin_interfaces/Time"] = Stamp

    def build(name, content):
        fields, constants = [], {}
        for raw in content.splitlines():
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            typename, field = line.split(None, 1)
            if "=" in field:
                key, value = field.split("=", 1)
                constants[key.strip()] = int(value.strip(), 0)
            else:
                fields.append((typename, field))

        def init(self):
            for typename, field in fields:
                if typename.endswith("[]"):
                    value = []
                elif typename == "string":
                    value = ""
                elif typename == "bool":
                    value = False
                elif typename.startswith("float"):
                    value = 0.0
                elif typename.startswith(("int", "uint")):
                    value = 0
                else:
                    value = classes[typename]()
                setattr(self, field, value)

        return type(name, (), {"__init__": init, **constants})

    for path in SDK.rglob("*.msg"):
        classes[path.stem] = build(path.stem, path.read_text())
    messages = types.ModuleType("aimdk_msgs.msg")
    for name, cls in classes.items():
        if "/" not in name:
            setattr(messages, name, cls)
    services = types.ModuleType("aimdk_msgs.srv")
    for path in SDK.rglob("*.srv"):
        request, response = re.split(r"(?m)^---\s*$", path.read_text(), maxsplit=1)
        cls = type(path.stem, (), {"Request": build(path.stem + "Request", request),
                                  "Response": build(path.stem + "Response", response)})
        setattr(services, path.stem, cls)
    return messages, services


class Future:
    def __init__(self, response=None, done=True):
        self.response, self.complete, self.cancelled = response, done, False

    def done(self):
        return self.complete

    def result(self):
        if isinstance(self.response, Exception):
            raise self.response
        return self.response

    def cancel(self):
        self.cancelled = True
        return True


class RosTransport:
    def __init__(self):
        self.msg, self.srv = sdk_types()
        self.requests, self.published, self.subscriptions, self.qos = [], {}, {}, {}
        self.responses, self.hanging = {}, set()
        self.action, self.action_status = "BIPED_WALK_RUN", 100
        self.owner, self.registered = "", False
        self.joint_positions = {}
        self.source_name = "edge_q1"
        self.expired_sources = set()
        self.requires_nonzero = False
        self.accepts_source = True
        self.auto_feedback = True
        self.running, self.latest_stamp = True, 0
        self.modules = self.make_modules()

    def stamp(self):
        ns = time.time_ns()
        stamp = types.SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9)
        return stamp

    def response(self, name, request):
        service = getattr(self.srv, name)
        response = service.Response()
        if name == "GetMcAction":
            response.header.code = 0
            response.info.action_desc = self.action
            response.info.status.value = self.action_status
            response.info.switch_ready = True
        else:
            response.response.header.code = 0
            response.response.state.value = 1
            response.response.task_id = 42
        if name == "SetMcInputSource":
            if request.action.value == 1001:
                self.registered = True
                self.source_name = request.input_source.name
            if request.action.value == 1003:
                self.registered = False
                self.expired_sources.add(request.input_source.name)
        if name == "GetCurrentInputSource":
            response.input_source.name = self.owner
            response.input_source.priority = 30 if self.owner == self.source_name or self.owner in self.expired_sources else 80
            response.input_source.timeout = 1000
        return response

    def joints(self, missing=None, duplicate=False, stale=False):
        msg = self.msg.JointStateArray()
        msg.header.stamp = self.stamp()
        if stale:
            msg.header.stamp.sec -= 10
        for i, name in enumerate(NAMES):
            if name == missing:
                continue
            joint = self.msg.JointState()
            joint.name, joint.position = name, self.joint_positions.get(name, 0.01 * i)
            msg.joints.append(joint)
        if duplicate:
            msg.joints.append(copy.deepcopy(msg.joints[0]))
        return msg

    def publish_joints(self, msg=None):
        callback = self.subscriptions.get("/aima/hal/joint/state")
        if callback:
            callback(msg or self.joints())

    def make_modules(self):
        transport = self
        rclpy = types.ModuleType("rclpy")
        context_mod = types.ModuleType("rclpy.context")
        node_mod = types.ModuleType("rclpy.node")
        executor_mod = types.ModuleType("rclpy.executors")
        qos_mod = types.ModuleType("rclpy.qos")

        class Context:
            def ok(self):
                return transport.running

        class Node:
            def __init__(self, name, **kwargs):
                self.name = name

            def get_clock(self):
                return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
                    nanoseconds=time.time_ns(), to_msg=transport.stamp))

            def create_publisher(self, message_type, topic, qos):
                transport.published[topic] = []
                transport.qos[topic] = qos

                def publish(message):
                    transport.published[topic].append(copy.deepcopy(message))
                    if (topic == "/aima/mc/locomotion/velocity" and transport.registered
                            and transport.accepts_source and (not transport.requires_nonzero
                            or any((message.forward_velocity, message.lateral_velocity, message.angular_velocity)))):
                        if transport.owner in ("", transport.source_name) or transport.owner in transport.expired_sources:
                            transport.owner = message.source

                return types.SimpleNamespace(publish=publish)

            def create_subscription(self, message_type, topic, callback, qos):
                transport.subscriptions[topic] = callback
                return object()

            def create_client(self, service_type, topic):
                name = service_type.__name__

                def call_async(request):
                    transport.requests.append((name, copy.deepcopy(request)))
                    if name in transport.hanging:
                        return Future(done=False)
                    response = transport.responses.get(name)
                    if callable(response):
                        response = response(request)
                    if response is None:
                        response = transport.response(name, request)
                    return Future(copy.deepcopy(response))

                return types.SimpleNamespace(call_async=call_async, service_is_ready=lambda: True,
                                             remove_pending_request=lambda future: None)

            def destroy_node(self):
                pass

        class Executor:
            def __init__(self, **kwargs):
                pass

            def add_node(self, node):
                pass

            def spin_once(self, timeout_sec=0):
                if transport.auto_feedback:
                    transport.publish_joints()
                time.sleep(0.002)

            def shutdown(self, **kwargs):
                pass

        class QoSProfile:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)

        sensor_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msg.JointState = type("JointState", (), {})
        rclpy.init = lambda **kwargs: None
        rclpy.shutdown = lambda **kwargs: None
        rclpy.ok = lambda **kwargs: transport.running
        node_mod.Node, executor_mod.SingleThreadedExecutor = Node, Executor
        context_mod.Context, qos_mod.QoSProfile = Context, QoSProfile
        for name in ("HistoryPolicy", "DurabilityPolicy", "ReliabilityPolicy"):
            setattr(qos_mod, name, types.SimpleNamespace(KEEP_LAST=1, VOLATILE=2,
                                                       BEST_EFFORT=3, RELIABLE=4))
        return {"sensor_msgs": types.ModuleType("sensor_msgs"), "sensor_msgs.msg": sensor_msg,
                "rclpy": rclpy, "rclpy.context": context_mod, "rclpy.node": node_mod,
                "rclpy.executors": executor_mod, "rclpy.qos": qos_mod,
                "aimdk_msgs": types.ModuleType("aimdk_msgs"),
                "aimdk_msgs.msg": self.msg, "aimdk_msgs.srv": self.srv}


def wait_for(predicate, seconds=1):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("timed out waiting for backend state")


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.backends = importlib.import_module("q1_edge.backends")

    def test_snapshot_component_ages_and_scheduling_gap_are_diagnostic(self):
        robot=self.make_robot()
        now=time.monotonic()
        with robot._lock, patch('q1_edge.backends.time.monotonic',return_value=now), \
                patch.object(robot,'_complete_requests'),patch.object(robot,'_poll'):
            robot._action_at=now-.12
            robot._owner_at=now-.08
            robot._last_worker_tick=now-.15
            with self.assertLogs('q1_edge.backends',level='WARNING') as logs:
                robot._tick()
            self.assertIn('worker_tick_gap_ms', '\n'.join(logs.output))
            snapshot=robot.snapshot()
            self.assertAlmostEqual(snapshot['action_age_s'],.12)
            self.assertAlmostEqual(snapshot['owner_age_s'],.08)

    def test_mock_requires_enable_and_validates_all_joint_targets(self):
        robot = self.backends.MockBackend()
        with self.assertRaises(RuntimeError):
            robot.set_velocity(0.1, 0, 0)
        robot.enable()
        robot.request_mode("walk")
        robot.set_velocity(0.1, 0, 0)
        self.assertEqual(robot.snapshot()["velocity"]["forward"], 0.1)
        for pose in ({}, {**dict.fromkeys(NAMES, 0.0), "extra": 0.0},
                     {**dict.fromkeys(NAMES, 0.0), NAMES[0]: math.nan}):
            with self.assertRaises(ValueError):
                robot.set_upper(pose)
        robot.stop()
        self.assertEqual(robot.snapshot()["velocity"]["forward"], 0)
        self.assertEqual(robot.snapshot()["action"], "BIPED_WALK_RUN")

    def make_robot(self, **config):
        self.transport = RosTransport()
        self.patch = patch.dict(sys.modules, self.transport.modules)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        settings = {"feedback_timeout_s": 0.15, "command_timeout_s": 0.10,
                    "service_timeout_s": 0.25, "transition_timeout_s": 0.6,
                    "source_poll_interval_s": 0.02, **config}
        robot = self.backends.Ros2Backend({"ros2": settings})
        self.addCleanup(robot.close)
        wait_for(lambda: robot.snapshot()["connected"])
        return robot

    def activate(self, robot):
        robot.enable()
        wait_for(lambda: robot.snapshot()["source_active"])
        robot.request_mode("walk")

    def velocity_messages(self):
        return self.transport.published["/aima/mc/locomotion/velocity"]

    def upper_messages(self):
        return self.transport.published["/aima/mc/custom/joint/command"]

    def standard_sample(self):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=self.transport.stamp()),
            name=list(reversed(NAMES)), position=[0.1 * i for i in range(10)])

    def test_standard_joint_feedback_maps_by_name_and_enforces_freshness(self):
        robot = self.make_robot()
        self.transport.auto_feedback = False
        callback = self.transport.subscriptions.get("/upper_body_joint_states")
        self.assertIsNotNone(callback, "real Q1 JointState topic must be subscribed")
        sample = self.standard_sample()
        callback(sample)
        self.assertAlmostEqual(robot.snapshot()["joints"][NAMES[-1]], 0.0)
        self.assertAlmostEqual(robot.snapshot()["joints"][NAMES[0]], 0.9)
        previous_stamp = robot._joint_stamp
        for kind in ("missing", "duplicate", "length", "nan", "stale", "future", "repeated"):
            with self.subTest(kind=kind):
                bad = self.standard_sample()
                if kind == "missing": bad.name.pop(); bad.position.pop()
                elif kind == "duplicate": bad.name[0] = bad.name[1]
                elif kind == "length": bad.position.pop()
                elif kind == "nan": bad.position[0] = math.nan
                elif kind == "stale": bad.header.stamp.sec -= 5
                elif kind == "future": bad.header.stamp.sec += 5
                else: bad = sample
                callback(bad)
                self.assertEqual(robot._joint_stamp, previous_stamp)
        time.sleep(.17)
        self.assertFalse(robot.snapshot()["connected"])

    def test_readonly_source_query_accepts_observed_unknown_success(self):
        robot = self.make_robot()
        self.activate(robot)
        response = self.transport.response("GetCurrentInputSource", None)
        response.response.state.value = 0
        self.transport.responses["GetCurrentInputSource"] = response
        time.sleep(.2)
        self.assertTrue(robot.snapshot()["source_active"])
        self.assertIsNone(robot.snapshot()["fault"])
        response.input_source.name = "rc"
        wait_for(lambda: bool(robot.snapshot()["fault"]))
        self.assertFalse(robot.snapshot()["source_active"])

    def test_unknown_query_with_failure_reason_does_not_confirm_ownership(self):
        robot = self.make_robot()
        response = self.transport.response("GetCurrentInputSource", None)
        response.response.state.value = 0
        response.response.state.reason = 8
        response.input_source.name = robot._source
        self.transport.responses["GetCurrentInputSource"] = response
        self.activate_query_failure(robot)

    def activate_query_failure(self, robot):
        robot.enable()
        wait_for(lambda: bool(robot.snapshot()["fault"]))
        self.assertFalse(robot.snapshot()["source_active"])

    def test_startup_readonly_timeout_recovers_after_successful_retry(self):
        robot = self.make_robot()
        self.transport.hanging.add("GetMcAction")
        wait_for(lambda: robot.snapshot()["fault"] == "GetMcAction timed out")
        self.transport.hanging.remove("GetMcAction")
        wait_for(lambda: robot.snapshot()["fault"] is None)
        self.assertEqual(self.velocity_messages(), [])
        self.assertFalse(any(n.startswith("Set") for n, _ in self.transport.requests))

    def test_active_readonly_timeout_remains_latched_after_retry(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.hanging.add("GetCurrentInputSource")
        wait_for(lambda: bool(robot.snapshot()["fault"]))
        self.transport.hanging.remove("GetCurrentInputSource")
        time.sleep(.08)
        self.assertIsNotNone(robot.snapshot()["fault"])

    def test_firmware_waits_for_nonzero_before_source_acquisition(self):
        robot = self.make_robot(transition_timeout_s=.1)
        self.transport.requires_nonzero = True
        robot.enable()
        time.sleep(.35)  # Longer than the old enable-time deadline.
        self.assertIsNone(robot.snapshot()["fault"])
        self.assertFalse(robot.snapshot()["source_active"])
        self.assertTrue(robot.snapshot()["source_can_acquire"])
        self.assertTrue(all(m.forward_velocity == 0 for m in self.velocity_messages()))
        robot.set_velocity(.22, 0., 0.)
        wait_for(lambda: robot.snapshot()["source_active"])
        wait_for(lambda: any(m.forward_velocity == .22 for m in self.velocity_messages()))

    def test_first_motion_without_owner_ack_is_bounded_and_released(self):
        robot = self.make_robot()
        self.transport.requires_nonzero = True
        self.transport.accepts_source = False
        robot.enable()
        wait_for(lambda: robot.snapshot()["source_can_acquire"])
        robot.set_velocity(.3, 0., 0.)
        end = time.monotonic()+.4
        while not robot.snapshot()["fault"] and time.monotonic()<end:
            try: robot.set_velocity(.3, 0., 0.)
            except RuntimeError: pass
            time.sleep(.01)
        self.assertIsNotNone(robot.snapshot()["fault"])
        self.assertTrue(any(m.forward_velocity > 0 for m in self.velocity_messages()))
        self.assertLessEqual(max(m.forward_velocity for m in self.velocity_messages()), .2)
        wait_for(lambda: not self.transport.registered)
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)

    def test_late_first_owner_ack_cannot_bypass_bootstrap_deadline(self):
        robot = self.make_robot()
        self.transport.requires_nonzero = True
        self.transport.accepts_source = False
        robot.enable()
        wait_for(lambda: robot.snapshot()["source_can_acquire"])
        with robot._lock:
            robot.set_velocity(.3,0.,0.)
            robot._acquisition_at = time.monotonic()-robot._acquisition_timeout-.01
            response=self.transport.response("GetCurrentInputSource", None)
            response.input_source.name=robot._source
            robot._pending["GetCurrentInputSource"]=(Future(response), time.monotonic()-.02, None)
            robot._tick()
            self.assertIsNotNone(robot.snapshot()["fault"])
            self.assertFalse(robot._ever_owned)
            self.assertFalse(robot.snapshot()["source_active"])
        self.assertTrue(all(m.forward_velocity <= .2 for m in self.velocity_messages()))

    def test_unowned_enable_requires_native_running_walk(self):
        robot = self.make_robot()
        for action in ("BIPED_STAND_DEFAULT", "PASSIVE_DEFAULT", "BIPED_CUSTOM_UPPER"):
            with self.subTest(action=action):
                self.transport.action=action
                wait_for(lambda: robot.snapshot()["action"]==action)
                with self.assertRaisesRegex(RuntimeError, "BIPED_WALK_RUN"):
                    robot.enable()
        self.assertFalse(any(n.startswith("Set") for n,_ in self.transport.requests))

    def test_active_foreign_owner_rejects_bounded_attempt_without_reacquisition(self):
        robot = self.make_robot()
        self.transport.requires_nonzero = True
        self.transport.owner = "rc"
        robot.enable()
        wait_for(lambda: robot.snapshot().get("source_owner") == "rc" and robot.snapshot()["source_can_acquire"])
        robot.set_velocity(.3, 0., 0.)
        wait_for(lambda: bool(robot.snapshot()["fault"]))
        self.assertEqual(self.transport.owner, "rc")
        self.assertFalse(robot.snapshot()["source_active"])
        self.assertTrue(all(m.forward_velocity <= .2 for m in self.velocity_messages()))
        self.assertEqual(self.velocity_messages()[-1].forward_velocity,0)
        with self.assertRaises(RuntimeError): robot.set_velocity(.2,0.,0.)

    def test_cached_deleted_owner_does_not_block_legitimate_acquisition(self):
        robot=self.make_robot()
        self.transport.requires_nonzero=True
        self.transport.owner="expired_previous_source"
        self.transport.expired_sources.add(self.transport.owner)
        robot.enable()
        wait_for(lambda: robot.snapshot()["source_can_acquire"])
        robot.set_velocity(.2,0.,0.)
        wait_for(lambda: robot.snapshot()["source_active"])
        self.assertIsNone(robot.snapshot()["fault"])

    def test_new_registration_epoch_cannot_accept_cached_self_as_ownership(self):
        robot=self.make_robot()
        self.activate(robot)
        old_source=robot._source
        robot.stop()
        wait_for(lambda: not robot._registered)
        self.assertEqual(self.transport.owner,old_source)
        self.transport.requires_nonzero=True
        robot.enable()
        self.assertNotEqual(robot._source,old_source)
        wait_for(lambda: robot.snapshot()["source_can_acquire"])
        time.sleep(.04)
        self.assertFalse(robot.snapshot()["source_active"])
        self.assertFalse(robot._ever_owned)
        robot.set_velocity(.2,0.,0.)
        wait_for(lambda: robot.snapshot()["source_active"])

    def test_rpc_logger_records_registration_and_failure_without_poll_spam(self):
        robot = self.make_robot()
        with self.assertLogs("q1_edge.backends", level="INFO") as logs:
            self.activate(robot)
            response = self.transport.srv.SetMcAction.Response()
            response.response.header.code = 7
            self.transport.responses["SetMcAction"] = response
            robot.request_mode("teleop")
            wait_for(lambda: bool(robot.snapshot()["fault"]))
        text = "\n".join(logs.output)
        for detail in ("rpc_request", "SetMcInputSource", "edge_q1", "rpc_response", "code=7", "backend_fault"):
            self.assertIn(detail, text)
        self.assertNotIn("rpc_response service=Get", text)

    def test_source_registration_accepts_firmware_unset_business_state(self):
        robot = self.make_robot()
        def firmware_response(request):
            response = self.transport.response("SetMcInputSource", request)
            response.response.state.value = self.transport.msg.CommonState.UNKNOWN
            return response
        self.transport.responses["SetMcInputSource"] = firmware_response
        self.activate(robot)
        self.assertIsNone(robot.snapshot()["fault"])
        robot.set_velocity(.22, 0., 0.)
        wait_for(lambda: any(m.forward_velocity == .22 for m in self.velocity_messages()))
        robot.stop()
        wait_for(lambda: not self.transport.registered and not robot._registered)
        self.assertFalse(robot.snapshot()["source_active"])

    def test_unset_source_business_state_with_reason_rejected(self):
        robot = self.make_robot()
        response = self.transport.srv.SetMcInputSource.Response()
        response.response.state.reason = 8
        self.transport.responses["SetMcInputSource"] = response
        robot.enable()
        wait_for(lambda: bool(robot.snapshot()["fault"]))
        self.assertFalse(robot.snapshot()["source_active"])
        self.assertEqual(self.velocity_messages(), [])

    def test_unknown_action_response_is_still_rejected(self):
        robot = self.make_robot()
        self.activate(robot)
        response = self.transport.srv.SetMcAction.Response()
        self.transport.responses["SetMcAction"] = response
        robot.request_mode("teleop")
        wait_for(lambda: bool(robot.snapshot()["fault"]))
        self.assertFalse(self.upper_messages())

    def test_nonzero_source_rpc_does_not_register_source(self):
        robot = self.make_robot()
        response = self.transport.srv.SetMcInputSource.Response()
        response.response.header.code = 1
        response.response.state.value = 0
        self.transport.responses["SetMcInputSource"] = response
        robot.enable()
        wait_for(lambda: bool(robot.snapshot()["fault"]))
        self.assertFalse(robot.snapshot()["source_active"])
        self.assertEqual(self.velocity_messages(), [])

    def test_ros_connection_is_read_only_until_explicit_enable(self):
        robot = self.make_robot()
        time.sleep(0.04)
        self.assertFalse(any(name.startswith("Set") for name, request in self.transport.requests))
        self.assertEqual(self.velocity_messages(), [])
        self.assertEqual(self.upper_messages(), [])
        self.assertFalse(robot.snapshot()["source_active"])
        self.activate(robot)
        adds = [r for n, r in self.transport.requests if n == "SetMcInputSource"]
        self.assertEqual([(r.action.value, r.input_source.name, r.input_source.priority,
                           r.input_source.timeout) for r in adds], [(1001, robot._source, 30, 1000)])

    def test_positive_velocity_requires_running_action_and_owned_source(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.action_status = 200
        wait_for(lambda: not robot.snapshot()["action_running"])
        with self.assertRaises(RuntimeError):
            robot.set_velocity(0.1, 0, 0)
        self.assertTrue(all(m.forward_velocity == 0 for m in self.velocity_messages()))

    def test_command_watchdog_cuts_motion_even_when_owner_still_matches(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.set_velocity(0.1, 0.02, 0.1)
        wait_for(lambda: any(m.forward_velocity == 0.1 for m in self.velocity_messages()))
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)
        self.assertEqual(self.transport.owner, robot._source)
        with self.assertRaises(RuntimeError):
            robot.set_velocity(0.1, 0, 0)

    def test_runtime_source_loss_latches_and_never_fights_rc_mode(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.set_velocity(0.1, 0, 0)
        wait_for(lambda: self.velocity_messages()[-1].forward_velocity > 0)
        self.transport.owner = "rc"
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        self.assertFalse(robot.snapshot()["source_active"])
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)
        self.assertFalse(any(n == "SetMcAction" for n, r in self.transport.requests))

    def test_business_failure_is_not_treated_as_successful_source_registration(self):
        robot = self.make_robot()
        response = self.transport.srv.SetMcInputSource.Response()
        response.response.header.code = 0
        response.response.state.value = 2
        self.transport.responses["SetMcInputSource"] = response
        robot.enable()
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        self.assertFalse(robot.snapshot()["source_active"])
        self.assertEqual(self.velocity_messages(), [])

    def test_hanging_action_request_does_not_block_zero_heartbeat(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.hanging.add("SetMcAction")
        start = time.monotonic()
        robot.request_mode("teleop")
        self.assertLess(time.monotonic() - start, 0.03)
        wait_for(lambda: any(n == "SetMcAction" for n, r in self.transport.requests))
        before = len(self.velocity_messages())
        time.sleep(0.08)
        self.assertGreater(len(self.velocity_messages()), before + 1)
        self.assertTrue(all(m.forward_velocity == 0 for m in self.velocity_messages()))
        wait_for(lambda: robot.snapshot()["fault"] is not None)

    def test_upper_only_after_exact_running_feedback_and_starts_measured(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.request_mode("teleop")
        wait_for(lambda: any(n == "SetMcAction" for n, r in self.transport.requests))
        self.assertEqual(self.upper_messages(), [])
        self.transport.action, self.transport.action_status = "BIPED_CUSTOM_UPPER", 200
        time.sleep(0.04)
        self.assertEqual(self.upper_messages(), [])
        self.transport.action_status = 100
        wait_for(lambda: bool(self.upper_messages()))
        first = self.upper_messages()[0]
        self.assertEqual([j.name for j in first.joints], list(NAMES))
        self.assertEqual([j.position for j in first.joints], [0.01 * i for i in range(10)])
        self.assertTrue(all(j.velocity == 0 and j.effort == 0 and j.stiffness == 20
                            and j.damping == 3 for j in first.joints))
        self.assertEqual(self.transport.qos["/aima/mc/custom/joint/command"].reliability, 3)

    def test_malformed_or_old_joint_feedback_never_refreshes_age(self):
        robot = self.make_robot()
        self.transport.auto_feedback = False
        before = robot.snapshot()["joints"]
        self.transport.publish_joints(self.transport.joints(missing=NAMES[-1]))
        self.transport.publish_joints(self.transport.joints(duplicate=True))
        self.transport.publish_joints(self.transport.joints(stale=True))
        bad = self.transport.joints()
        bad.joints[0].position = math.nan
        self.transport.publish_joints(bad)
        time.sleep(0.04)
        self.assertEqual(robot.snapshot()["joints"], before)
        self.assertGreater(robot.snapshot()["joints_age_s"], 0.03)

    def test_stand_and_stop_hold_balance_and_release_only_owned_source(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.request_mode("stand")
        robot.set_velocity(0, 0, 0)
        robot.stop()
        wait_for(lambda: any(n == "SetMcInputSource" and r.action.value == 1003
                             for n, r in self.transport.requests))
        self.assertTrue(all(r.command.action_desc not in ("PASSIVE_DEFAULT", "DAMPING_DEFAULT",
                            "BIPED_STAND_DEFAULT") for n, r in self.transport.requests if n == "SetMcAction"))
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)

    def test_nonfinite_and_out_of_bounds_commands_rejected(self):
        robot = self.make_robot()
        self.activate(robot)
        for values in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, True), (5, 0, 0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                robot.set_velocity(*values)

    def test_late_teleop_ack_after_stop_does_not_release_before_native_hold(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.request_mode("teleop")
        wait_for(lambda: any(n == "SetMcAction" for n, r in self.transport.requests))
        time.sleep(0.03)
        robot.stop()
        time.sleep(0.10)
        self.assertFalse(any(n == "SetMcInputSource" and r.action.value == 1003
                             for n, r in self.transport.requests))
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)

    def test_lost_source_does_not_resume_pending_mode_after_owner_returns(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.owner = "rc"
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        self.transport.owner = robot._source
        self.transport.action = "BIPED_CUSTOM_UPPER"
        time.sleep(0.06)
        self.assertFalse(any(n == "SetMcAction" for n, r in self.transport.requests))
        self.assertEqual(self.upper_messages(), [])

    def test_controller_stop_preserves_external_takeover_until_explicit_enable(self):
        from q1_edge.core import Controller

        robot = self.make_robot()
        self.transport.joint_positions = dict(self.backends.UPPER_HOME)
        config = json.loads((ROOT / "config" / "default.json").read_text())
        controller = Controller(robot, config)
        frame = {"client_id": "local-pad", "source": "linux_gamepad", "sequence": 0,
                 "axes": [0, 0, 0, 0], "buttons": [False] * 17, "connected": True}
        controller.input(frame)
        controller.action("calibrate")
        controller.action("enable")
        wait_for(lambda: robot.snapshot()["source_active"])
        controller.tick()
        self.assertTrue(controller.enabled)

        self.transport.owner = "rc"
        self.transport.action = "BIPED_CUSTOM_UPPER"
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        controller.tick()
        self.assertFalse(controller.enabled)
        self.assertEqual(controller.mode, "fault")
        self.transport.owner = robot._source
        time.sleep(0.08)
        self.assertFalse(any(n == "SetMcAction" for n, r in self.transport.requests))
        self.assertTrue(all(m.forward_velocity == m.lateral_velocity == m.angular_velocity == 0
                            for m in self.velocity_messages()))

        controller.action("clear_fault")
        controller.input({**frame, "sequence": 1})
        controller.action("calibrate")
        controller.action("enable")
        wait_for(lambda: any(n == "SetMcAction" and r.command.action_desc == "BIPED_WALK_RUN"
                             for n, r in self.transport.requests))

    def test_takeover_stop_and_close_never_resume_old_upper_or_native_mode(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.request_mode("teleop")
        wait_for(lambda: any(n == "SetMcAction" for n, r in self.transport.requests))
        self.transport.action = "BIPED_CUSTOM_UPPER"
        wait_for(lambda: bool(self.upper_messages()))
        self.transport.owner = "rc"
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        upper_count = len(self.upper_messages())
        robot.stop()
        self.transport.owner = robot._source
        self.transport.joint_positions = dict(self.backends.UPPER_HOME)
        time.sleep(0.08)
        robot.close()
        self.assertEqual(len(self.upper_messages()), upper_count)
        actions = [r.command.action_desc for n, r in self.transport.requests if n == "SetMcAction"]
        self.assertEqual(actions, ["BIPED_CUSTOM_UPPER"])
        self.assertFalse(any(n == "SetMcInputSource" and r.action.value == 1003
                             for n, r in self.transport.requests))
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)

    def test_later_feedback_fault_cannot_reenable_recovery_after_takeover(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.joint_positions = dict(self.backends.UPPER_HOME)
        self.transport.owner = "rc"
        self.transport.action = "BIPED_CUSTOM_UPPER"
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        self.transport.owner = robot._source
        self.transport.auto_feedback = False
        time.sleep(0.18)
        self.assertFalse(robot.snapshot()["connected"])
        self.transport.auto_feedback = True
        wait_for(lambda: robot.snapshot()["connected"])
        time.sleep(0.04)
        self.assertFalse(any(n == "SetMcAction" for n, r in self.transport.requests))

    def test_joint_and_owner_staleness_stop_independently_of_fresh_targets(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.set_velocity(0.1, 0, 0)
        self.transport.hanging.add("GetCurrentInputSource")
        deadline = time.monotonic() + 0.20
        while time.monotonic() < deadline and not robot.snapshot()["fault"]:
            try:
                robot.set_velocity(0.1, 0, 0)
            except RuntimeError:
                break
            time.sleep(0.01)
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        self.assertIsNotNone(robot.snapshot()["fault"])
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)

    def test_pending_business_state_does_not_bypass_running_feedback(self):
        robot = self.make_robot()
        self.activate(robot)
        response = self.transport.srv.SetMcAction.Response()
        response.response.state.value = 200
        self.transport.responses["SetMcAction"] = response
        robot.request_mode("teleop")
        wait_for(lambda: any(n == "SetMcAction" for n, r in self.transport.requests))
        time.sleep(0.05)
        self.assertEqual(self.upper_messages(), [])
        self.assertIsNone(robot.snapshot()["fault"])

    def test_upper_watchdog_keeps_full_last_pose_and_does_not_snap_home(self):
        robot = self.make_robot()
        self.activate(robot)
        robot.request_mode("teleop")
        wait_for(lambda: any(n == "SetMcAction" for n, r in self.transport.requests))
        self.transport.action = "BIPED_CUSTOM_UPPER"
        wait_for(lambda: bool(self.upper_messages()))
        wait_for(lambda: robot.snapshot()["fault"] is not None)
        self.assertEqual(len(self.upper_messages()[-1].joints), 10)
        self.assertEqual([j.position for j in self.upper_messages()[-1].joints],
                         [0.01 * i for i in range(10)])
        self.assertFalse(any(n == "SetMcInputSource" and r.action.value == 1003
                             for n, r in self.transport.requests))

    def test_leaving_upper_mode_does_not_keep_its_old_command_watchdog(self):
        robot = self.make_robot()
        self.transport.joint_positions = dict(zip(NAMES, (
            -0.01982023485863345, -0.00021580613966982196,
            0.017233758101518008, 0.22511493021261703, -0.003530979547924497,
            1.0186741542682025, 0.017164215358473386, -0.22262401216020816,
            0.004204617106736518, 1.0185279282650144)))
        self.activate(robot)
        robot.request_mode("teleop")
        wait_for(lambda: any(n == "SetMcAction" for n, r in self.transport.requests))
        self.transport.action = "BIPED_CUSTOM_UPPER"
        wait_for(lambda: bool(self.upper_messages()))
        robot.request_mode("stand")
        wait_for(lambda: any(n == "SetMcAction" and r.command.action_desc == "BIPED_WALK_RUN"
                             for n, r in self.transport.requests))
        self.transport.action = "BIPED_WALK_RUN"
        time.sleep(0.15)
        self.assertIsNone(robot.snapshot()["fault"])

    def test_unexpected_native_action_change_latches_without_switching_back(self):
        robot = self.make_robot()
        self.activate(robot)
        time.sleep(0.03)
        self.transport.action = "BIPED_WALK_RUN_DEFAULT"
        wait_for(lambda: robot.snapshot()["action"] == "BIPED_WALK_RUN_DEFAULT")
        self.assertIsNotNone(robot.snapshot()["fault"])
        self.assertFalse(any(n == "SetMcAction" for n, r in self.transport.requests))

    def test_stop_before_add_reply_waits_for_safe_release(self):
        robot = self.make_robot()
        self.transport.hanging.add("SetMcInputSource")
        robot.enable()
        robot.stop()
        start = time.monotonic()
        robot.close()
        self.assertGreater(time.monotonic() - start, 0.15)
        self.assertIsNotNone(robot.snapshot()["fault"])

    def test_repeated_joint_sample_does_not_refresh_feedback_age(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.auto_feedback = False
        sample = self.transport.joints()
        self.transport.publish_joints(sample)
        time.sleep(0.02)
        self.transport.publish_joints(sample)
        self.assertGreater(robot.snapshot()["joints_age_s"], 0.015)

    def test_unsupported_passive_or_damping_recovery_is_not_requested_on_stop(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.action = "DAMPING_DEFAULT"
        wait_for(lambda: robot.snapshot()["action"] == "DAMPING_DEFAULT")
        robot.stop()
        time.sleep(0.04)
        self.assertFalse(any(n == "SetMcAction" for n, r in self.transport.requests))

    def test_close_is_bounded_when_transport_hangs(self):
        robot = self.make_robot()
        self.activate(robot)
        self.transport.hanging.add("SetMcInputSource")
        start = time.monotonic()
        robot.close()
        self.assertLess(time.monotonic() - start, 0.8)
        self.assertEqual(self.velocity_messages()[-1].forward_velocity, 0)

    def test_headers_monotonically_advance_with_complete_velocity_fields(self):
        robot = self.make_robot()
        self.activate(robot)
        time.sleep(0.07)
        messages = self.velocity_messages()
        stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in messages]
        self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
        self.assertTrue(all(m.source == robot._source and m.pitch_velocity == 0 and m.level == 0
                            for m in messages))


if __name__ == "__main__":
    unittest.main()
