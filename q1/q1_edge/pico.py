"""Pico controller sampling and measured-neutral, position-priority Q1 arm IK.

The arms have four joints each: controller orientation is validated but is not
an exact wrist orientation target. Geometry limits are not confirmed HW limits.
"""

from __future__ import annotations

import importlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


UPPER_JOINT_NAMES = (
    "waist_roll_joint", "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
)
PICO_TO_ROBOT = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def _quat_matrix(quaternion):
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _axis_matrix(axis, angle):
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*skew + (1 - math.cos(angle))*(skew @ skew)


def _numeric_vector(value, size, label):
    array = np.asarray(value)
    if (array.shape != (size,) or array.dtype.kind not in "fiu" or not np.isfinite(array).all()
            or any(isinstance(item, (bool, np.bool_)) for item in value)):
        raise ValueError(f"{label} must contain {size} finite numbers")
    return array.astype(float)


def validate_frame(frame):
    """Validate the common native/browser boundary without replacing timestamps."""
    if not isinstance(frame, dict) or frame.get("valid") is not True:
        raise ValueError("Pico tracking is invalid; recalibrate")
    stamp = frame.get("timestamp_ns")
    if type(stamp) is not int or not 0 < stamp < 2**63:
        raise ValueError("Pico timestamp_ns must be a positive native integer")
    poses = {}
    for side in ("head", "left", "right"):
        pose = _numeric_vector(frame.get(side), 7, side)
        if abs(float(np.linalg.norm(pose[3:])) - 1.) > .001:
            raise ValueError(f"{side} quaternion is not unit length; recalibrate")
        poses[side] = pose
    for name in ("left_grip", "right_grip"):
        value = frame.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0. <= value <= 1.:
            raise ValueError(f"{name} must be finite and within [0,1]")
    return poses


class PicoRetargeter:
    """Solve both hand positions from relative controllers and all10 measured joints.

    Call once per advancing Pico frame. Repeated, invalid, jumped or unreachable
    samples latch calibration off. Callers own receipt freshness and grip gating.
    """

    def __init__(self, asset_path=None, limits=None):
        asset_path = Path(asset_path) if asset_path is not None else Path(__file__).resolve().parent.parent / "assets/q1_upper.xml"
        root = ET.parse(asset_path).getroot()
        self.limits = {}
        for joint in root.iter("joint"):
            self.limits[joint.get("name")] = tuple(float(x) for x in joint.get("range").split())
        if set(self.limits) != set(UPPER_JOINT_NAMES):
            raise ValueError("Upper FK asset must contain exactly the ten upper joints")
        for name, bounds in (limits or {}).items():
            if name not in self.limits:
                raise ValueError(f"Unknown upper joint limit: {name}")
            low, high = _numeric_vector(bounds, 2, f"{name} limits")
            if low >= high:
                raise ValueError(f"Invalid limits for {name}")
            self.limits[name] = (float(low), float(high))
        self._low = np.array([self.limits[name][0] for name in UPPER_JOINT_NAMES])
        self._high = np.array([self.limits[name][1] for name in UPPER_JOINT_NAMES])
        self._chains = {}

        def visit(body, parent_chain):
            pos = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
            wxyz = np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
            rotation = _quat_matrix(wxyz[[1, 2, 3, 0]])
            joint = body.find("joint")
            index, pivot, axis = None, np.zeros(3), np.zeros(3)
            if joint is not None:
                index = UPPER_JOINT_NAMES.index(joint.get("name"))
                pivot = np.fromstring(joint.get("pos", "0 0 0"), sep=" ")
                axis = np.fromstring(joint.get("axis", "0 0 1"), sep=" ")
                axis /= np.linalg.norm(axis)
            chain = parent_chain + [(pos, rotation, index, pivot, axis)]
            for site in body.findall("site"):
                if site.get("name") in ("left_hand", "right_hand"):
                    self._chains[site.get("name").split("_")[0]] = (
                        chain, np.fromstring(site.get("pos", "0 0 0"), sep=" "))
            for child in body.findall("body"):
                visit(child, chain)

        for body in root.findall("./worldbody/body"):
            visit(body, [])
        if set(self._chains) != {"left", "right"}:
            raise ValueError("Upper FK asset is missing the two hand sites")
        self._status = {"calibrated": False, "error_m": 0., "fault": None,
                        "max_joint_rate_rad_s": .6, "hardware_limits_confirmed": False,
                        "target": "4DOF arm hand position; wrist-link origin"}
        self._last_timestamp = None

    @property
    def status(self):
        return dict(self._status)

    def _joint_vector(self, joints):
        if not isinstance(joints, dict) or set(joints) != set(UPPER_JOINT_NAMES):
            raise ValueError("Calibration requires all ten measured upper joint positions")
        values = _numeric_vector([joints[name] for name in UPPER_JOINT_NAMES], 10, "upper positions")
        if np.any(values < self._low) or np.any(values > self._high):
            raise ValueError("Measured upper pose is outside configured limits; confirm hardware profile")
        return values

    def _fk(self, values, side):
        position, rotation = np.zeros(3), np.eye(3)
        axes, pivots = {}, {}
        chain, endpoint = self._chains[side]
        for offset, fixed_rotation, index, pivot, axis in chain:
            position = position + rotation @ offset
            rotation = rotation @ fixed_rotation
            if index is not None:
                axes[index] = rotation @ axis
                pivots[index] = position + rotation @ pivot
                turn = _axis_matrix(axis, float(values[index]))
                position = position + rotation @ (pivot - turn @ pivot)
                rotation = rotation @ turn
        hand = position + rotation @ endpoint
        jacobian = np.zeros((3, 10))
        for index in axes:
            jacobian[:, index] = np.cross(axes[index], hand - pivots[index])
        return hand, jacobian

    def hand_positions(self, joints):
        """FK endpoint positions in model world coordinates, in meters."""
        values = self._joint_vector(joints)
        return {side: self._fk(values, side)[0].tolist() for side in ("left", "right")}

    def calibrate(self, frame, joints):
        self._status.update(calibrated=False, fault=None, error_m=0.)
        try:
            poses = validate_frame(frame)
            measured = self._joint_vector(joints)
            head = _quat_matrix(poses["head"][3:])
            forward = PICO_TO_ROBOT @ head @ np.array([0., 0., -1.])
            if np.linalg.norm(forward[:2]) < .1:
                raise ValueError("Look forward for neutral heading calibration")
            yaw = math.atan2(float(forward[1]), float(forward[0]))
            self._vr_to_robot = _axis_matrix(np.array([0., 0., 1.]), -yaw) @ PICO_TO_ROBOT
            self._neutral_poses = poses
            self._previous_poses = poses
            self._head_rotation = head
            self._neutral = measured
            self._output = measured.copy()
            self._solution = measured.copy()
            self._neutral_hands = {side: self._fk(measured, side)[0] for side in ("left", "right")}
            self._calibration_timestamp = frame["timestamp_ns"]
            self._last_timestamp = None
            self._status["calibrated"] = True
        except (ValueError, TypeError, OverflowError) as exc:
            self._status["fault"] = str(exc)
            raise ValueError(str(exc)) from exc

    def _solve_arm(self, values, side, target):
        indices = slice(2, 6) if side == "left" else slice(6, 10)
        for _ in range(60):
            hand, jacobian = self._fk(values, side)
            error = target - hand
            if float(np.linalg.norm(error)) < .0002:
                break
            jacobian = jacobian[:, indices]
            bias = self._neutral[indices] - values[indices]
            delta = np.linalg.solve(jacobian.T @ jacobian + .00011*np.eye(4),
                                    jacobian.T @ error + .00001*bias)
            delta *= min(1., .15 / max(float(np.max(np.abs(delta))), 1e-12))
            old_cost = float(error @ error + .00001*(bias @ bias))
            for _ in range(8):
                candidate = values.copy()
                candidate[indices] = np.clip(values[indices] + delta, self._low[indices], self._high[indices])
                residual = target - self._fk(candidate, side)[0]
                neutral_error = self._neutral[indices] - candidate[indices]
                cost = float(residual @ residual + .00001*(neutral_error @ neutral_error))
                if cost < old_cost:
                    values = candidate
                    break
                delta *= .5
            else:
                break
        residual_m = float(np.linalg.norm(target - self._fk(values, side)[0]))
        if residual_m > .002:
            self._status["error_m"] = residual_m
            raise ValueError(f"{side} hand target is unreachable within limits ({residual_m:.4f}m); recalibrate")
        return values

    def retarget(self, frame, dt=.01):
        if not self._status["calibrated"]:
            raise ValueError("Pico requires measured neutral calibration")
        try:
            poses = validate_frame(frame)
            if isinstance(dt, bool) or not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0.:
                raise ValueError("Pico dt must be positive and finite")
            timestamp = frame["timestamp_ns"]
            previous_timestamp = self._last_timestamp if self._last_timestamp is not None else self._calibration_timestamp
            interval = (timestamp - previous_timestamp) * 1e-9
            first_neutral = (self._last_timestamp is None and timestamp == self._calibration_timestamp
                             and all(np.array_equal(poses[side], self._neutral_poses[side]) for side in poses))
            if interval <= 0. and not first_neutral:
                raise ValueError("Pico sample timestamp did not advance; recalibrate")
            if interval > .25:
                raise ValueError("Pico sample gap exceeded 250ms; recalibrate")
            for side, pose in poses.items():
                previous = self._previous_poses[side]
                if float(np.linalg.norm(pose[:3] - previous[:3])) > .12 + 2.*max(interval, 0.):
                    raise ValueError(f"{side} tracking position jumped; recalibrate")
                similarity = abs(float(pose[3:] @ previous[3:])) / (np.linalg.norm(pose[3:])*np.linalg.norm(previous[3:]))
                if 2.*math.acos(float(np.clip(similarity, -1., 1.))) > .8 + 5.*max(interval, 0.):
                    raise ValueError(f"{side} tracking orientation jumped; recalibrate")
            relative_head = self._vr_to_robot @ _quat_matrix(poses["head"][3:]) @ self._head_rotation.T @ self._vr_to_robot.T
            yaw = math.atan2(float(relative_head[1, 0]), float(relative_head[0, 0]))
            roll = math.atan2(float(relative_head[2, 1]), float(relative_head[2, 2]))
            if np.allclose(relative_head, np.eye(3), atol=1e-12, rtol=0.):
                yaw = roll = 0.
            desired = self._solution.copy()
            desired[:2] = np.clip(self._neutral[:2] + [np.clip(roll, -.12, .12), np.clip(yaw, -.25, .25)],
                                  self._low[:2], self._high[:2])
            targets = {}
            for side in ("left", "right"):
                delta = self._vr_to_robot @ (poses[side][:3] - self._neutral_poses[side][:3])
                targets[side] = self._neutral_hands[side] + delta
                indices = slice(2, 6) if side == "left" else slice(6, 10)
                if np.array_equal(delta, np.zeros(3)) and np.max(np.abs(desired[:2] - self._neutral[:2])) < 1e-12:
                    desired[indices] = self._neutral[indices]
                else:
                    desired = self._solve_arm(desired, side, targets[side])
            max_step = .6 * min(float(dt), .05, interval if interval > 0. else float(dt))
            output = np.clip(self._output + np.clip(desired - self._output, -max_step, max_step), self._low, self._high)
            self._status["error_m"] = max(float(np.linalg.norm(self._fk(output, side)[0] - targets[side])) for side in targets)
            self._output, self._solution = output, desired
            self._previous_poses, self._last_timestamp = poses, timestamp
            return dict(zip(UPPER_JOINT_NAMES, output.tolist()))
        except (ValueError, TypeError, OverflowError, np.linalg.LinAlgError) as exc:
            self._status.update(calibrated=False, fault=str(exc))
            raise ValueError(str(exc)) from exc


class XRoboSource:
    """Read the official xrobotoolkit_sdk getter API; import only when requested."""

    def __init__(self):
        self._sdk = importlib.import_module("xrobotoolkit_sdk")
        self._sdk.init()
        self._last_timestamp = 0
        self._closed = False

    def poll(self):
        if self._closed:
            return None
        timestamp = self._sdk.get_time_stamp_ns()
        if type(timestamp) is not int or timestamp <= self._last_timestamp:
            return None
        frame = {"timestamp_ns": timestamp, "head": list(self._sdk.get_headset_pose()),
                 "left": list(self._sdk.get_left_controller_pose()),
                 "right": list(self._sdk.get_right_controller_pose()),
                 "left_grip": self._sdk.get_left_grip(), "right_grip": self._sdk.get_right_grip(),
                 "valid": True}
        # shortcut: native getters lack an atomic snapshot/controller-valid flags; upgrade when the SDK exposes them.
        if self._sdk.get_time_stamp_ns() != timestamp:
            return None
        self._last_timestamp = timestamp
        try:
            validate_frame(frame)
        except (ValueError, TypeError, OverflowError):
            frame["valid"] = False
        return frame

    def close(self):
        if not self._closed:
            self._closed = True
            self._sdk.close()
