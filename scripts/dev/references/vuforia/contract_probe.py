#!/usr/bin/env python3
"""Independent arithmetic checks of published contracts; zero Vuforia calls.

This is a research fixture, not an implementation of or SDK adapter for Vuforia.
The expected geometry is specified analytically in contract-fixture.json.
"""
import hashlib
import json
import math
import os
import pathlib
import unittest

BASE = pathlib.Path(__file__).resolve().parent
FIXTURE = json.loads((BASE / "contract-fixture.json").read_text())
FLIP = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rigid_inverse(t):
    r = [[t[j][i] for j in range(3)] for i in range(3)]
    return [r[i] + [-sum(r[i][k] * t[k][3] for k in range(3))] for i in range(3)] + [[0, 0, 0, 1]]


def point(t, p):
    return [sum(t[i][j] * p[j] for j in range(4)) for i in range(4)]


def close(a, b):
    return max(abs(x - y) for x, y in zip(a, b)) < 1e-12


def pair_is_valid(events):
    # This predicate checks an input ledger, without calling any SDK.
    return len(events) == 2 and events[0][0] == "pose" and events[1][0] == "frame" and events[0][1] == events[1][1]


class ContractChecks(unittest.TestCase):
    def test_01_cv_world_to_camera_known_points(self):
        tcw = multiply(FLIP, rigid_inverse(FIXTURE["camera_to_world_gl"]))
        for world, expected in zip(FIXTURE["world_points"], FIXTURE["expected_camera_points_cv"]):
            self.assertTrue(close(point(tcw, world), expected))
        self.assertTrue(close([tcw[i][3] for i in range(3)], FIXTURE["expected_driver_translation_cv"]))

    def test_02_wrong_flip_side_is_detected(self):
        wrong = multiply(rigid_inverse(FIXTURE["camera_to_world_gl"]), FLIP)
        err = max(abs(x - y) for x, y in zip(point(wrong, FIXTURE["world_points"][0]), FIXTURE["expected_camera_points_cv"][0]))
        self.assertGreater(err, 1)

    def test_03_observer_camera_to_world_recovers_pose(self):
        tcw = multiply(FLIP, rigid_inverse(FIXTURE["camera_to_world_gl"]))
        recovered = multiply(rigid_inverse(tcw), FLIP)
        for row, expected in zip(recovered, FIXTURE["camera_to_world_gl"]):
            self.assertTrue(close(row, expected))

    def test_04_view_matrix_and_device_pose_are_inverse(self):
        twc = FIXTURE["camera_to_world_gl"]
        identity = multiply(rigid_inverse(twc), twc)
        for i, row in enumerate(identity):
            self.assertTrue(close(row, [int(i == j) for j in range(4)]))

    def test_05_target_relative_position_independent_expected(self):
        target_to_camera = multiply(rigid_inverse(FIXTURE["camera_to_world_gl"]), FIXTURE["target_to_world"])
        device_to_target = rigid_inverse(target_to_camera)
        self.assertTrue(close([device_to_target[i][3] for i in range(3)], FIXTURE["expected_device_position_in_target"]))

    def test_06_column_major_translation_indices(self):
        twc = FIXTURE["camera_to_world_gl"]
        values = [twc[row][col] for col in range(4) for row in range(4)]
        self.assertEqual(values[12:15], [1, 2, 3])
        self.assertEqual([values[i] for i in (3, 7, 11)], [0, 0, 0])

    def test_07_body_camera_lever_arm_is_required(self):
        twc = multiply(FIXTURE["body_to_world_gl"], FIXTURE["camera_to_body_gl"])
        world = FIXTURE["lever_arm_world_point"]
        expected = FIXTURE["expected_camera_point_with_lever_arm_cv"]
        self.assertTrue(close(point(multiply(FLIP, rigid_inverse(twc)), world), expected))
        missing_extrinsic = point(multiply(FLIP, rigid_inverse(FIXTURE["body_to_world_gl"])), world)
        self.assertAlmostEqual(math.dist(missing_extrinsic[:3], expected[:3]), 0.1)

    def test_08_exact_pose_first_pair(self):
        ns = FIXTURE["exact_pair_ns"]
        self.assertTrue(pair_is_valid([("pose", ns), ("frame", ns)]))

    def test_09_one_ns_mismatch_is_rejected(self):
        self.assertFalse(pair_is_valid([("pose", FIXTURE["exact_pair_ns"]), ("frame", FIXTURE["mismatched_pair_ns"])]))

    def test_10_frame_first_is_rejected(self):
        ns = FIXTURE["exact_pair_ns"]
        self.assertFalse(pair_is_valid([("frame", ns), ("pose", ns)]))

    def test_11_exposure_end_and_sensitivity(self):
        end = FIXTURE["exposure_end_ns"]
        duration = FIXTURE["exposure_duration_ns"]
        self.assertEqual(end - duration, FIXTURE["expected_exposure_start_ns"])
        self.assertEqual(end - duration // 2, FIXTURE["expected_exposure_midpoint_ns"])
        rate = FIXTURE["example_constant_angular_velocity_deg_s"]
        self.assertAlmostEqual(rate * duration * 1e-9, FIXTURE["expected_start_to_end_rotation_deg"])
        self.assertAlmostEqual(rate * duration * 0.5e-9, FIXTURE["expected_midpoint_to_end_rotation_deg"])

    def test_12_nv21_stride_buffer_accounting(self):
        f = FIXTURE["nv21"]
        total = f["stride"] * f["height"] + f["stride"] * (f["height"] // 2)
        self.assertEqual(total, f["expected_bytes"])
        self.assertGreater(total, f["incorrect_tightly_packed_bytes"])


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ContractChecks)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    out = {
        "schema": "vuforia-public-contract-probe-v1",
        "evidence_scope": "framework_conformance_independent_arithmetic",
        "vuforia_runtime_calls": 0,
        "sdk_abi_verified": False,
        "fixture_sha256": hashlib.sha256((BASE / "contract-fixture.json").read_bytes()).hexdigest(),
        "tests_run": result.testsRun,
        "failures": len(result.failures),
        "errors": len(result.errors),
        "passed": result.wasSuccessful(),
        "limitations": ["No Vuforia SDK linked or run", "No Vuforia accuracy or latency measured", "Known point arithmetic and input ledger rules only", "Exposure sensitivity is analytical, not measured camera timing"],
    }
    (pathlib.Path(os.environ.get("VUFORIA_EVIDENCE_DIR", BASE)) / "contract-results.json").write_text(json.dumps(out, indent=2) + "\n")
    raise SystemExit(0 if result.wasSuccessful() else 1)
