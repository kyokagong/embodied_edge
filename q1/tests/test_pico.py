"""Offline contracts: no SDK runtime, ROS, simulator, or robot is needed."""

import copy
import importlib
import math
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np

from q1_edge.backends import UPPER_HOME

try:
    pico = importlib.import_module("q1_edge.pico")
except ModuleNotFoundError:
    pico = None


NAMES = (
    "waist_roll_joint", "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
)
MEASURED = dict(zip(NAMES, [.03, -.08, -.2, .3, .12, -.2, -.15, -.28, -.1, -.15]))


def frame(timestamp_ns=1_000_000_000):
    return {"timestamp_ns": timestamp_ns,
            "head": [0., 1.6, 0., 0., 0., 0., 1.],
            "left": [-.25, 1.2, -.35, 0., 0., 0., 1.],
            "right": [.25, 1.2, -.35, 0., 0., 0., 1.],
            "left_grip": 1., "right_grip": 1., "valid": True}


class RetargeterTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(pico, "the Pico subsystem has not been implemented")
        self.retargeter = pico.PicoRetargeter()
        self.neutral = frame()

    def test_requires_measured_calibration_before_motion(self):
        with self.assertRaises(ValueError):
            self.retargeter.retarget(self.neutral)
        for joints in ({}, {**MEASURED, NAMES[0]: math.nan},
                       {**MEASURED, NAMES[0]: 12.}):
            with self.subTest(joints=joints), self.assertRaises(ValueError):
                self.retargeter.calibrate(self.neutral, joints)
        self.assertFalse(self.retargeter.status["calibrated"])

    def test_neutral_is_exact_measured_pose_without_a_home_jump(self):
        self.retargeter.calibrate(self.neutral, MEASURED)
        self.assertEqual(self.retargeter.retarget(self.neutral), MEASURED)
        fresh = frame(self.neutral["timestamp_ns"] + 10_000_000)
        self.assertEqual(self.retargeter.retarget(fresh), MEASURED)
        self.assertEqual(set(self.retargeter.retarget(frame(fresh["timestamp_ns"] + 10_000_000))), set(NAMES))

    def test_fk_matches_hand_checked_zero_pose_of_official_model(self):
        hands = self.retargeter.hand_positions(dict.fromkeys(NAMES, 0.))
        np.testing.assert_allclose(hands["left"], [.128088, .12249977, .57281004], atol=2e-6)
        np.testing.assert_allclose(hands["right"], [.128088, -.12249977, .57281004], atol=2e-6)

    def test_five_centimeter_controller_translation_reaches_hand_and_obeys_rate(self):
        self.retargeter.calibrate(self.neutral, MEASURED)
        initial = self.retargeter.hand_positions(MEASURED)
        moved = copy.deepcopy(self.neutral)
        moved["left"][2] -= .05  # PICO -Z is robot +X.
        previous = MEASURED
        for step in range(1, 151):
            moved["timestamp_ns"] = self.neutral["timestamp_ns"] + step * 10_000_000
            result = self.retargeter.retarget(moved, dt=.01)
            self.assertTrue(all(abs(result[name] - previous[name]) <= .00600000001 for name in NAMES))
            self.assertTrue(all(self.retargeter.limits[name][0] <= result[name] <= self.retargeter.limits[name][1]
                                for name in NAMES))
            previous = result
        final = self.retargeter.hand_positions(result)
        np.testing.assert_allclose(final["left"], np.asarray(initial["left"]) + [.05, 0., 0.], atol=.002)
        np.testing.assert_allclose(final["right"], initial["right"], atol=.0001)
        self.assertLess(self.retargeter.status["error_m"], .002)

    def test_sdk_home_positive_elbows_preserve_neutral_and_reach_small_displacements(self):
        cases = (
            ("left", [0., 0., -.01], [.01, 0., 0.]),
            ("left", [0., .035, 0.], [0., 0., .035]),
            ("left", [0., 0., .035], [-.035, 0., 0.]),
            ("right", [0., .035, 0.], [0., 0., .035]),
        )
        initial = self.retargeter.hand_positions(UPPER_HOME)
        for side, pico_delta, robot_delta in cases:
            with self.subTest(side=side, robot_delta=robot_delta):
                self.retargeter.calibrate(self.neutral, UPPER_HOME)
                self.assertEqual(self.retargeter.retarget(self.neutral), UPPER_HOME)
                moved = copy.deepcopy(self.neutral)
                moved[side][:3] = (np.asarray(moved[side][:3]) + pico_delta).tolist()
                previous = UPPER_HOME
                for step in range(1, 151):
                    moved["timestamp_ns"] = self.neutral["timestamp_ns"] + step * 10_000_000
                    result = self.retargeter.retarget(moved)
                    self.assertTrue(all(abs(result[name] - previous[name]) <= .00600000001 for name in NAMES))
                    previous = result
                final = self.retargeter.hand_positions(result)
                np.testing.assert_allclose(final[side], np.asarray(initial[side]) + robot_delta, atol=.002)
                other = "right" if side == "left" else "left"
                np.testing.assert_allclose(final[other], initial[other], atol=.0001)

    def test_sdk_home_forward_35mm_exceeds_arm_length_and_requires_recalibration(self):
        # The pinned geometry's maximum radius is .271529m; this target needs .276480m.
        self.retargeter.calibrate(self.neutral, UPPER_HOME)
        moved = frame(1_010_000_000)
        moved["left"][2] -= .035
        with self.assertRaises(ValueError):
            self.retargeter.retarget(moved)
        self.assertFalse(self.retargeter.status["calibrated"])
        self.assertGreater(self.retargeter.status["error_m"], .0049)

    def test_invalid_tracking_latches_until_recalibrated(self):
        invalid = []
        for field, value in (("valid", False), ("timestamp_ns", True), ("left_grip", math.nan)):
            broken = frame(1_010_000_000)
            broken[field] = value
            invalid.append(broken)
        for side in ("head", "left", "right"):
            for quaternion in ([0., 0., 0., 0.], [0., 0., 0., 2.], [0., 0., 0., 1.005], [math.nan, 0., 0., 1.]):
                broken = frame(1_010_000_000)
                broken[side][3:] = quaternion
                invalid.append(broken)
            for invalid_position in (math.nan, False):
                broken = frame(1_010_000_000)
                broken[side][0] = invalid_position
                invalid.append(broken)
        for broken in invalid:
            with self.subTest(broken=broken):
                self.retargeter.calibrate(self.neutral, MEASURED)
                with self.assertRaises(ValueError):
                    self.retargeter.retarget(broken)
                self.assertFalse(self.retargeter.status["calibrated"])
                with self.assertRaises(ValueError):
                    self.retargeter.retarget(frame(1_020_000_000))

    def test_repeat_and_regressed_timestamps_do_not_keep_tracking_alive(self):
        for timestamp in (1_010_000_000, 1_000_000_000):
            with self.subTest(timestamp=timestamp):
                self.retargeter.calibrate(self.neutral, MEASURED)
                self.retargeter.retarget(frame(1_010_000_000))
                with self.assertRaises(ValueError):
                    self.retargeter.retarget(frame(timestamp))
                self.assertFalse(self.retargeter.status["calibrated"])

    def test_tracking_jump_requires_recalibration(self):
        self.retargeter.calibrate(self.neutral, MEASURED)
        moved = frame(1_010_000_000)
        moved["left"][0] += .4
        with self.assertRaises(ValueError):
            self.retargeter.retarget(moved)
        self.assertFalse(self.retargeter.status["calibrated"])

    def test_gap_or_invalid_dt_requires_recalibration(self):
        for timestamp, dt in ((1_250_000_001, .01), (1_010_000_000, math.nan),
                              (1_010_000_000, 0.), (1_010_000_000, True)):
            with self.subTest(timestamp=timestamp, dt=dt):
                self.retargeter.calibrate(self.neutral, MEASURED)
                with self.assertRaises(ValueError):
                    self.retargeter.retarget(frame(timestamp), dt=dt)
                self.assertFalse(self.retargeter.status["calibrated"])

    def test_large_caller_dt_cannot_bypass_rate_bound(self):
        self.retargeter.calibrate(self.neutral, MEASURED)
        moved = frame(1_010_000_000)
        moved["right"][2] -= .04
        result = self.retargeter.retarget(moved, dt=2.)
        self.assertTrue(all(abs(result[name] - MEASURED[name]) <= .00600000001 for name in NAMES))

    def test_head_heading_calibrates_a_fixed_translation_frame(self):
        self.neutral["head"][3:] = [0., math.sqrt(.5), 0., math.sqrt(.5)]
        self.retargeter.calibrate(self.neutral, MEASURED)
        initial = self.retargeter.hand_positions(MEASURED)
        moved = copy.deepcopy(self.neutral)
        moved["right"][2] -= .03
        for step in range(1, 101):
            moved["timestamp_ns"] = 1_000_000_000 + step * 10_000_000
            result = self.retargeter.retarget(moved)
        final = self.retargeter.hand_positions(result)
        np.testing.assert_allclose(final["right"], np.asarray(initial["right"]) + [0., -.03, 0.], atol=.002)

    def test_head_roll_is_relative_and_limited(self):
        self.retargeter.calibrate(self.neutral, MEASURED)
        moved = frame()
        moved["head"][3:] = [0., 0., math.sin(.15), math.cos(.15)]
        for step in range(1, 61):
            moved["timestamp_ns"] = 1_000_000_000 + step * 10_000_000
            result = self.retargeter.retarget(moved)
        self.assertAlmostEqual(result["waist_roll_joint"], MEASURED["waist_roll_joint"] - .12)

    def test_gradual_unreachable_target_signals_failure_instead_of_a_zero_pose(self):
        self.retargeter.calibrate(self.neutral, MEASURED)
        failed = False
        for step in range(1, 31):
            moved = frame(1_000_000_000 + step * 10_000_000)
            moved["left"][2] -= step * .02
            try:
                self.retargeter.retarget(moved)
            except ValueError:
                failed = True
                break
        self.assertTrue(failed, "an out-of-workspace target must be rejected")
        self.assertFalse(self.retargeter.status["calibrated"])

    def test_head_yaw_is_relative_to_neutral_and_limited(self):
        self.neutral["head"][3:] = [0., math.sin(.3), 0., math.cos(.3)]
        self.retargeter.calibrate(self.neutral, MEASURED)
        self.assertEqual(self.retargeter.retarget(self.neutral), MEASURED)
        moved = copy.deepcopy(self.neutral)
        moved["head"][3:] = [0., math.sin(.5), 0., math.cos(.5)]
        for step in range(1, 101):
            moved["timestamp_ns"] = 1_000_000_000 + step * 10_000_000
            result = self.retargeter.retarget(moved)
        self.assertLessEqual(abs(result["waist_yaw_joint"] - MEASURED["waist_yaw_joint"]), .250001)
        self.assertGreater(abs(result["waist_yaw_joint"] - MEASURED["waist_yaw_joint"]), .05)


class SDKDouble(types.ModuleType):
    """Only the official native getter API; hand optics are deliberately unavailable."""

    def __init__(self):
        super().__init__("xrobotoolkit_sdk")
        self.timestamp = 1_000_000_000
        self.current = frame()

    def init(self):
        pass

    def close(self):
        pass

    def get_time_stamp_ns(self):
        return self.timestamp

    def get_headset_pose(self):
        return self.current["head"]

    def get_left_controller_pose(self):
        return self.current["left"]

    def get_right_controller_pose(self):
        return self.current["right"]

    def get_left_grip(self):
        return .9

    def get_right_grip(self):
        return .8

    def get_left_hand_is_active(self):
        raise AssertionError("Optical hands do not establish controller tracking")

    get_right_hand_is_active = get_left_hand_is_active


class XRoboSourceTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(pico, "the Pico subsystem has not been implemented")
        self.sdk = SDKDouble()
        self.patch = patch.dict(sys.modules, {"xrobotoolkit_sdk": self.sdk})
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.source = pico.XRoboSource()
        self.addCleanup(self.source.close)

    def test_native_sample_timestamp_is_preserved_and_cached_frames_are_suppressed(self):
        actual = self.source.poll()
        self.assertEqual(actual["timestamp_ns"], 1_000_000_000)
        self.assertTrue(actual["valid"])
        self.assertEqual(actual["left_grip"], .9)
        self.assertEqual(actual["right_grip"], .8)
        self.assertIsNone(self.source.poll())
        self.sdk.timestamp = 999_999_999
        self.assertIsNone(self.source.poll())
        self.sdk.timestamp = 1_000_000_001
        self.assertEqual(self.source.poll()["timestamp_ns"], 1_000_000_001)

    def test_invalid_controller_pose_is_reported_invalid(self):
        self.sdk.current["left"][3:] = [0., 0., 0., 0.]
        self.assertFalse(self.source.poll()["valid"])
        self.assertIsNone(self.source.poll())

    def test_concurrent_sample_change_is_not_mislabeled_as_a_coherent_snapshot(self):
        original = self.sdk.get_left_controller_pose

        def update_during_read():
            self.sdk.timestamp += 1
            return original()

        self.sdk.get_left_controller_pose = update_during_read
        self.assertIsNone(self.source.poll())
        self.sdk.get_left_controller_pose = original
        self.assertEqual(self.source.poll()["timestamp_ns"], 1_000_000_001)

    def test_close_is_idempotent_and_no_longer_emits_frames(self):
        self.source.close()
        self.source.close()
        self.assertIsNone(self.source.poll())


if __name__ == "__main__":
    unittest.main()
