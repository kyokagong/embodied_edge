"""Native Q1 MC adapters. Importing this module never imports or starts ROS."""
from __future__ import annotations

import math
import re
import threading
import time


UPPER_JOINT_NAMES = (
    "waist_roll_joint", "waist_yaw_joint", "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
)
UPPER_HOME = dict(zip(UPPER_JOINT_NAMES, (
    -0.01982023485863345, -0.00021580613966982196,
    0.017233758101518008, 0.22511493021261703, -0.003530979547924497,
    1.0186741542682025, 0.017164215358473386, -0.22262401216020816,
    0.004204617106736518, 1.0185279282650144,
)))
_WALK = "BIPED_WALK_RUN"
_UPPER = "BIPED_CUSTOM_UPPER"
_MODES = {"stand": _WALK, "walk": _WALK, "teleop": _UPPER}
_ZERO = {"forward": 0.0, "lateral": 0.0, "angular": 0.0}


def _number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def _velocity(forward, lateral, angular):
    values = forward, lateral, angular
    if not all(_number(v) and abs(v) <= limit
               for v, limit in zip(values, (0.3, 0.2, 0.5))):
        raise ValueError("Velocity must be finite and within 0.3/0.2 m/s and 0.5 rad/s")
    return dict(zip(_ZERO, values))


def _pose(positions):
    if not isinstance(positions, dict) or set(positions) != set(UPPER_JOINT_NAMES):
        raise ValueError("Upper command requires exactly all ten named joints")
    if not all(_number(v) for v in positions.values()):
        raise ValueError("Upper joint positions must be finite numbers")
    return dict(positions)


class MockBackend:
    kind = "mock"

    def __init__(self):
        self._enabled = False
        self._action = _WALK
        self._velocity = dict(_ZERO)
        self._joints = dict(UPPER_HOME)

    def enable(self):
        self._enabled = True

    def request_mode(self, mode):
        if mode not in _MODES:
            raise ValueError("Mode must be stand, walk or teleop")
        if not self._enabled:
            raise RuntimeError("Explicit enable required")
        self._velocity = dict(_ZERO)
        self._action = _MODES[mode]

    def set_velocity(self, forward, lateral, angular):
        value = _velocity(forward, lateral, angular)
        if any(value.values()) and (not self._enabled or self._action != _WALK):
            raise RuntimeError("Walking requires explicit enable and native MC walk mode")
        self._velocity = value

    def set_upper(self, positions):
        pose = _pose(positions)
        if not self._enabled or self._action != _UPPER:
            raise RuntimeError("Upper motion requires explicit enable and CUSTOM_UPPER")
        self._joints = pose

    def stop(self):
        self._velocity = dict(_ZERO)
        self._enabled = False
        self._action = _WALK

    def close(self):
        self.stop()

    def snapshot(self):
        return {"connected": True, "action": self._action, "action_running": True,
                "joints": dict(self._joints), "joints_age_s": 0.0,
                "source_active": self._enabled, "fault": None,
                "velocity": dict(self._velocity)}


class Ros2Backend:
    """Bounded asynchronous service state machine plus an independent output watchdog.

    No source is registered until enable(). Services never spin or wait on a
    future, and the output worker continues sending zeros while services hang.
    """
    kind = "ros2"

    def __init__(self, config):
        cfg = config.get("ros2", {})
        self._source = cfg.get("source_name", "edge_q1")
        self._priority = cfg.get("priority", 30)
        self._source_timeout = cfg.get("source_timeout_ms", 1000)
        if (not isinstance(self._source, str) or not re.fullmatch(r"edge_q1(?:_[A-Za-z0-9]+)*", self._source)
                or type(self._priority) is not int or not 1 <= self._priority < 80
                or type(self._source_timeout) is not int or not 250 <= self._source_timeout <= 1000):
            raise ValueError("Use a unique edge_q1 source, priority below rc80, timeout 250–1000ms")
        self._feedback_timeout = cfg.get("feedback_timeout_s", 0.25)
        self._command_timeout = cfg.get("command_timeout_s", 0.25)
        self._service_timeout = cfg.get("service_timeout_s", 3.0)
        self._transition_timeout = cfg.get("transition_timeout_s", 20.0)
        self._poll_interval = cfg.get("source_poll_interval_s", 0.1)
        if not all(_number(v) and v > 0 for v in (
                self._feedback_timeout, self._command_timeout, self._service_timeout,
                self._transition_timeout, self._poll_interval)):
            raise ValueError("ROS timeouts must be positive finite numbers")
        if self._feedback_timeout > 0.5 or self._command_timeout > 0.5:
            raise ValueError("Feedback and command watchdogs must be at most 0.5 seconds")
        self._lock = threading.RLock()
        self._quit = threading.Event()
        self._closed = False
        self._fault = None
        self._enabled = self._registered = self._source_active = False
        self._ever_owned = self._stopping = self._release_attempted = False
        self._recovery_inhibited = False
        self._owner_at = self._action_at = self._joints_at = None
        self._action, self._action_running, self._switch_ready = "", False, False
        self._joints, self._joint_stamp = {}, 0
        self._velocity = dict(_ZERO)
        self._velocity_at = self._upper_at = None
        self._upper_command = self._upper_target = None
        self._target = self._waiting_action = None
        self._transition_at = self._request_at = None
        self._enabled_at = self._zero_since = None
        self._pending, self._last_poll = {}, {}
        self._last_stamp = self._sequence = 0
        self._last_output = self._last_velocity_output = 0.0

        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            import aimdk_msgs.msg as messages
            import aimdk_msgs.srv as services
        except ImportError as exc:
            raise RuntimeError("ROS2 Humble and the pinned official aimdk_msgs workspace must be sourced") from exc
        self._rclpy, self._messages, self._services = rclpy, messages, services
        self._context = Context()
        rclpy.init(context=self._context)
        try:
            self._node = Node(self._source, context=self._context)
            reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
            best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
            self._velocity_pub = self._node.create_publisher(
                messages.McLocomotionVelocity, "/aima/mc/locomotion/velocity", reliable)
            self._upper_pub = self._node.create_publisher(
                messages.McCustomJointCommand, "/aima/mc/custom/joint/command", best_effort)
            self._subscription = self._node.create_subscription(
                messages.JointStateArray, "/aima/hal/joint/state", self._on_joints, best_effort)
            self._clients = {name: self._node.create_client(
                getattr(services, name), "/aimdk_5Fmsgs/srv/" + name)
                for name in ("GetMcAction", "SetMcAction", "GetCurrentInputSource", "SetMcInputSource")}
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
        except Exception:
            if hasattr(self, "_node"):
                self._node.destroy_node()
            rclpy.shutdown(context=self._context)
            raise
        self._spin_thread = threading.Thread(target=self._spin, name="q1-ros-feedback", daemon=True)
        self._output_thread = threading.Thread(target=self._run, name="q1-command-watchdog", daemon=True)
        self._spin_thread.start()
        self._output_thread.start()

    def _fresh(self, timestamp, now):
        return timestamp is not None and 0 <= now - timestamp <= self._feedback_timeout

    def _connected(self, now):
        return (not self._closed and self._context.ok() and self._fresh(self._action_at, now)
                and self._fresh(self._joints_at, now))

    def _owns(self, now):
        return self._registered and self._source_active and self._fresh(self._owner_at, now)

    def _ready(self, action, now):
        return self._connected(now) and self._action_running and self._action == action

    def _stamp(self):
        stamp = self._node.get_clock().now().to_msg()
        ns = max(stamp.sec * 10**9 + stamp.nanosec, self._last_stamp + 1)
        self._last_stamp = ns
        stamp.sec, stamp.nanosec = divmod(ns, 10**9)
        return stamp

    def _on_joints(self, message):
        try:
            pose = {}
            for joint in message.joints:
                if joint.name in UPPER_JOINT_NAMES:
                    if joint.name in pose or not _number(joint.position):
                        return
                    pose[joint.name] = joint.position
            pose = _pose(pose)
            stamp = message.header.stamp
            ns = stamp.sec * 10**9 + stamp.nanosec
            clock_ns = self._node.get_clock().now().nanoseconds
            if (not 0 <= stamp.nanosec < 10**9 or ns <= 0
                    or not -0.1 <= (clock_ns - ns) / 1e9 <= self._feedback_timeout):
                return
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        with self._lock:
            if ns <= self._joint_stamp:
                return
            self._joint_stamp, self._joints, self._joints_at = ns, pose, time.monotonic()

    def enable(self):
        with self._lock:
            now = time.monotonic()
            if not self._connected(now):
                raise RuntimeError("Fresh MC and all ten measured joints required before enable")
            if self._stopping or "SetMcInputSource" in self._pending:
                raise RuntimeError("Previous source operation is still finishing")
            if self._enabled and not self._fault:
                return
            if self._recovery_inhibited:
                self._upper_at = self._upper_command = self._upper_target = None
            self._recovery_inhibited = False
            self._fault = None
            self._enabled, self._ever_owned, self._release_attempted = True, False, False
            self._enabled_at = now
            self._velocity = dict(_ZERO)
            self._velocity_at = now
            self._target = _WALK
            self._transition_at = now
            if not self._registered:
                self._source_request(1001, now)

    def request_mode(self, mode):
        if mode not in _MODES:
            raise ValueError("Mode must be stand, walk or teleop")
        with self._lock:
            if not self._enabled or self._fault:
                raise RuntimeError("Explicit enable required before a mode request")
            target = _MODES[mode]
            if self._target == target:
                return
            self._velocity = dict(_ZERO)
            self._target, self._transition_at = target, time.monotonic()
            if target == _UPPER:
                self._upper_at = self._upper_command = self._upper_target = None
            elif self._upper_command:
                self._upper_target = dict(self._upper_command)

    def set_velocity(self, forward, lateral, angular):
        value = _velocity(forward, lateral, angular)
        with self._lock:
            now = time.monotonic()
            if any(value.values()) and (not self._enabled or self._fault or self._target != _WALK
                                       or not self._owns(now) or not self._ready(_WALK, now)
                                       or self._waiting_action is not None):
                raise RuntimeError("Velocity requires owned source and fresh native MC RUNNING feedback")
            self._velocity, self._velocity_at = value, now

    def set_upper(self, positions):
        pose = _pose(positions)
        with self._lock:
            now = time.monotonic()
            if (not self._enabled or self._fault or self._target != _UPPER
                    or not self._owns(now) or not self._ready(_UPPER, now)
                    or self._waiting_action is not None):
                raise RuntimeError("Upper motion requires owned source and fresh CUSTOM_UPPER RUNNING feedback")
            if any(abs(pose[n] - self._joints[n]) > 0.35 for n in UPPER_JOINT_NAMES):
                raise ValueError("Upper target differs from measured pose by more than 0.35 radians")
            self._upper_target, self._upper_at = pose, now

    def stop(self):
        with self._lock:
            self._velocity = dict(_ZERO)
            self._enabled = False
            if not self._registered and "SetMcInputSource" not in self._pending:
                return
            if not self._stopping and not self._recovery_inhibited:
                self._stopping, self._zero_since = True, time.monotonic()
                self._target, self._transition_at = _WALK, self._zero_since
                if self._upper_command:
                    self._upper_target = dict(self._upper_command)
            self._publish_velocity(time.monotonic())

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            age = None if self._joints_at is None else max(0.0, now - self._joints_at)
            return {"connected": self._connected(now), "action": self._action,
                    "action_running": self._action_running and self._fresh(self._action_at, now),
                    "joints": dict(self._joints), "joints_age_s": age,
                    "source_active": self._owns(now), "fault": self._fault,
                    "velocity": dict(self._velocity)}

    def _source_request(self, action, now):
        request = self._services.SetMcInputSource.Request()
        request.request.header.stamp = self._stamp()
        request.action.value = action
        request.input_source.name = self._source
        if action == 1001:
            request.input_source.priority = self._priority
            request.input_source.timeout = self._source_timeout
        return self._send("SetMcInputSource", request, now, action)

    def _send(self, name, request, now, tag=None):
        if name in self._pending:
            return False
        client = self._clients[name]
        if not client.service_is_ready():
            if name.startswith("Set"):
                self._trip(name + " service unavailable")
            return False
        try:
            self._pending[name] = (client.call_async(request), now, tag)
            return True
        except Exception as exc:
            self._trip(name + " transport error: " + str(exc))
            return False

    def _trip(self, message, recover=True):
        self._fault = self._fault or message
        self._velocity = dict(_ZERO)
        self._enabled = False
        if not recover:
            self._recovery_inhibited = True
            self._target = self._transition_at = self._waiting_action = None
            self._stopping = False
        if recover and not self._recovery_inhibited and self._registered and not self._stopping:
            self._stopping, self._zero_since = True, time.monotonic()
            self._target, self._transition_at = _WALK, self._zero_since
            if self._upper_command:
                self._upper_target = dict(self._upper_command)
        if self._registered:
            try:
                self._publish_velocity(time.monotonic())
            except Exception:
                pass

    def _complete_requests(self, now):
        for name, (future, started, tag) in list(self._pending.items()):
            if not future.done():
                if now - started <= self._service_timeout:
                    continue
                future.cancel()
                self._clients[name].remove_pending_request(future)
                del self._pending[name]
                self._trip(name + " timed out")
                continue
            del self._pending[name]
            try:
                response = future.result()
                if name == "GetMcAction":
                    if response.header.code != 0 or not response.info.action_desc:
                        raise ValueError("invalid MC action response")
                    self._action, self._action_at = response.info.action_desc, now
                    self._action_running = response.info.status.value == self._messages.McActionStatus.RUNNING
                    self._switch_ready = response.info.switch_ready
                    if (self._enabled and self._transition_at is None and self._target
                            and (self._action != self._target or not self._action_running)):
                        self._trip("Native MC action changed outside requested transition", recover=False)
                else:
                    task = response.response
                    state = task.state.value
                    accepted = {self._messages.CommonState.SUCCESS}
                    if name == "SetMcAction":
                        accepted |= {self._messages.CommonState.PENDING, self._messages.CommonState.CREATED,
                                     self._messages.CommonState.RUNNING}
                    if task.header.code != 0 or state not in accepted:
                        raise ValueError(f"RPC code={task.header.code}, business state={state}, reason={task.state.reason}")
                    if name == "SetMcInputSource":
                        self._registered = tag == 1001
                        if not self._registered:
                            self._source_active = self._stopping = False
                            self._target = self._transition_at = None
                        elif not self._enabled:
                            self.stop()
                    elif name == "GetCurrentInputSource":
                        owned = response.input_source.name == self._source
                        self._owner_at, self._source_active = now, owned
                        if self._registered and self._ever_owned and not owned:
                            self._trip("MC input source ownership lost", recover=False)
                        self._ever_owned = self._ever_owned or (self._registered and owned)
            except Exception as exc:
                if name.startswith("Set") or self._registered:
                    self._trip(name + " failed: " + str(exc))

    def _poll(self, now):
        for name in ("GetMcAction", "GetCurrentInputSource"):
            if now - self._last_poll.get(name, -math.inf) >= self._poll_interval and name not in self._pending:
                request = getattr(self._services, name).Request()
                request.request.header.stamp = self._stamp()
                if self._send(name, request, now):
                    self._last_poll[name] = now

    def _advance_mode(self, now):
        if (self._recovery_inhibited or not self._target or not (self._enabled or self._stopping)
                or not self._owns(now) or not self._connected(now)):
            return
        if self._transition_at is not None and now - self._transition_at > self._transition_timeout:
            self._trip("MC mode transition timed out", recover=False)
            self._target = None
            return
        if "SetMcAction" in self._pending:
            return
        if self._waiting_action:
            if (self._action == self._waiting_action and self._action_running
                    and self._action_at >= self._request_at):
                self._waiting_action = None
            else:
                return
        if self._action == self._target and self._action_running:
            self._transition_at = None
            if self._target == _UPPER and self._upper_command is None:
                self._upper_command = self._upper_target = dict(self._joints)
                self._upper_at = now
            return
        if not self._action_running or not self._switch_ready:
            return
        if self._target == _WALK and self._action == _UPPER:
            # shortcut: away from HOME needs remote recovery until a return trajectory is hardware validated.
            if any(abs(self._joints[n] - UPPER_HOME[n]) > 0.15 for n in UPPER_JOINT_NAMES):
                self._fault = self._fault or "Upper pose away from HOME; native MC held, recover with remote"
                return
        if self._action == _WALK:
            command, expected = _UPPER, _UPPER
        elif self._action in (_UPPER, "BIPED_STAND_DEFAULT", "BIPED_WALK_RUN_DEFAULT", "BIPED_WHOLE_BODY_CTRL"):
            command, expected = _WALK, _WALK
        elif self._action == "PASSIVE_DEFAULT" and not self._stopping:
            command, expected = "STAND_UP", _WALK
        else:
            self._trip("Unsupported native MC start state; prepare standing with remote", recover=False)
            self._target = None
            return
        request = self._services.SetMcAction.Request()
        request.header.stamp, request.source = self._stamp(), self._source
        request.command.action_desc = command
        if self._send("SetMcAction", request, now):
            self._waiting_action, self._request_at = expected, now

    def _publish_velocity(self, now):
        msg = self._messages.McLocomotionVelocity()
        msg.header.stamp, msg.header.sequence = self._stamp(), self._sequence
        self._sequence = (self._sequence + 1) % 2**32
        msg.source = self._source
        motion = (self._enabled and not self._fault and self._owns(now) and self._target == _WALK
                  and self._ready(_WALK, now) and self._waiting_action is None)
        value = self._velocity if motion else _ZERO
        msg.forward_velocity, msg.lateral_velocity, msg.angular_velocity = value.values()
        msg.pitch_velocity = msg.level = 0.0
        self._velocity_pub.publish(msg)
        self._last_velocity_output = now

    def _publish_upper(self, now):
        if (self._recovery_inhibited or self._upper_command is None
                or not self._owns(now) or not self._ready(_UPPER, now)
                or (not self._enabled and not self._stopping)):
            return
        msg = self._messages.McCustomJointCommand()
        msg.header.stamp, msg.header.sequence = self._stamp(), self._sequence
        self._sequence = (self._sequence + 1) % 2**32
        step = min(max(now - self._last_output, 0), 0.02) * 0.5
        for name in UPPER_JOINT_NAMES:
            joint = self._messages.JointCommand()
            current, target = self._upper_command[name], self._upper_target[name]
            self._upper_command[name] = current + max(-step, min(step, target - current))
            joint.name, joint.position = name, self._upper_command[name]
            joint.velocity = joint.effort = 0.0
            joint.stiffness, joint.damping = 20.0, 3.0
            msg.joints.append(joint)
        self._upper_pub.publish(msg)

    def _tick(self):
        with self._lock:
            now = time.monotonic()
            self._complete_requests(now)
            self._poll(now)
            if self._registered:
                if not self._connected(now):
                    self._trip("MC or full joint feedback is stale")
                if self._ever_owned and not self._fresh(self._owner_at, now):
                    self._source_active = False
                    self._trip("MC input source ownership feedback is stale", recover=False)
                if self._enabled and not self._ever_owned and now - self._enabled_at > self._service_timeout:
                    self._trip("MC source acquisition timed out", recover=False)
                if self._enabled and any(self._velocity.values()) and now - self._velocity_at > self._command_timeout:
                    self._trip("Velocity command watchdog expired")
                if (self._enabled and self._target == _UPPER and self._upper_at is not None
                        and now - self._upper_at > self._command_timeout):
                    self._trip("Upper command watchdog expired")
                self._advance_mode(now)
                if now - self._last_velocity_output >= 0.019:
                    self._publish_velocity(now)
                self._publish_upper(now)
                if (self._stopping and not self._recovery_inhibited and not self._release_attempted
                        and self._ready(_WALK, now)
                        and now - self._zero_since >= 0.08 and self._waiting_action is None
                        and "SetMcAction" not in self._pending):
                    self._release_attempted = True
                    self._source_request(1003, now)
            self._last_output = now

    def _spin(self):
        try:
            while not self._quit.is_set() and self._context.ok():
                self._executor.spin_once(timeout_sec=0.01)
        except Exception as exc:
            with self._lock:
                self._trip("ROS feedback executor failed: " + str(exc))

    def _run(self):
        next_tick = time.monotonic()
        while not self._quit.is_set():
            try:
                self._tick()
            except Exception as exc:
                with self._lock:
                    self._trip("ROS command worker failed: " + str(exc))
            next_tick = max(next_tick + 0.01, time.monotonic())
            self._quit.wait(max(0.0, next_tick - time.monotonic()))

    def close(self):
        if self._closed:
            return
        self.stop()
        deadline = time.monotonic() + min(self._service_timeout + 0.1, 0.5)
        while (self._registered or "SetMcInputSource" in self._pending) and time.monotonic() < deadline:
            self._quit.wait(0.01)
        with self._lock:
            if self._registered:
                self._fault = self._fault or "Close could not confirm safe source release; use native remote"
                try:
                    self._velocity = dict(_ZERO)
                    self._publish_velocity(time.monotonic())
                except Exception:
                    pass
            self._closed = True
            self._quit.set()
        self._output_thread.join(timeout=0.05)
        self._spin_thread.join(timeout=0.05)
        self._executor.shutdown(timeout_sec=0.05)
        self._node.destroy_node()
        if self._context.ok():
            self._rclpy.shutdown(context=self._context)
