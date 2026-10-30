"""Analytic metric fixtures independent of either VIO implementation."""

import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location(
    "audit_metrics", Path(__file__).resolve().parents[1] / "scripts/evaluation/audit_metrics.py")
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


def trajectory():
    return np.array([[0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 1, 0, 0, 0, 0, 0, 1],
                     [2, 1, 1, 0, 0, 0, 0, 1],
                     [3, 0, 1, 1, 0, 0, 0, 1]], dtype=float)


class AuditMetricsTest(unittest.TestCase):
    def test_identical_is_zero(self):
        result = metrics.evaluate(trajectory(), trajectory(), expected_frames=5)
        self.assertLess(result["ate_translation_m"]["rmse"], 1e-12)
        self.assertEqual(result["rpe"]["rotation_deg"]["rmse"], 0)
        self.assertEqual(result["pose_coverage"], 0.8)

    def test_scale_error_cannot_disappear(self):
        est = trajectory()
        est[:, 1:4] *= 2
        result = metrics.evaluate(est, trajectory())
        self.assertGreater(result["ate_translation_m"]["rmse"], 0.5)
        self.assertLess(result["diagnostic_sim3_only"]["ate_rmse_m"], 1e-12)
        self.assertAlmostEqual(result["diagnostic_sim3_only"]["fitted_scale"], 0.5)

    def test_known_rotation_drift(self):
        est = trajectory()
        est[:, 4:8] = Rotation.from_euler("z", [0, 90, 180, 270], degrees=True).as_quat()
        result = metrics.evaluate(est, trajectory())
        self.assertAlmostEqual(result["rpe"]["rotation_deg"]["rmse"], 90)

    def test_global_frame_rotation_is_removed(self):
        gt = trajectory()
        est = gt.copy()
        r = Rotation.from_euler("xyz", [20, -30, 60], degrees=True)
        est[:, 1:4] = r.apply(gt[:, 1:4]) + [8, -2, 1]
        est[:, 4:8] = r.as_quat()
        result = metrics.evaluate(est, gt)
        self.assertLess(result["ate_translation_m"]["rmse"], 1e-12)
        self.assertLess(result["ape_rotation_deg"]["rmse"], 1e-10)
        self.assertLess(result["rpe"]["translation_m"]["rmse"], 1e-12)

    def test_stationary_yaw_gauge_is_not_attitude_error(self):
        gt = trajectory()
        gt[:, 1:4] = 0
        est = gt.copy()
        est[:, 4:8] = Rotation.from_euler("z", 60, degrees=True).as_quat()
        result = metrics.evaluate(est, gt)
        self.assertEqual(result["alignment_covariance_rank"], 0)
        self.assertEqual(result["orientation_alignment"], "ambiguous_from_positions")
        self.assertIsNone(result["ape_rotation_deg"])
        self.assertEqual(result["rpe"]["rotation_deg"]["rmse"], 0)

    def test_dropped_association_retains_real_timestamps(self):
        gt = trajectory()
        est = np.insert(gt, 1, [0.2, 50, 0, 0, 0, 0, 0, 1], axis=0)
        result = metrics.evaluate(est, gt)
        self.assertEqual(result["associated_poses"], 4)
        self.assertEqual(result["rpe"]["pairs"], 3)
        self.assertLess(result["rpe"]["translation_m"]["rmse"], 1e-12)

    def test_invalid_and_empty_data_rejected(self):
        for bad in ([], np.zeros((3, 8)), trajectory()[::-1]):
            with self.assertRaises(ValueError):
                metrics.validate_poses(bad)
        bad = trajectory()
        bad[0, 1] = np.nan
        with self.assertRaises(ValueError):
            metrics.validate_poses(bad)
        gt = trajectory()
        gt[:, 0] += 20
        with self.assertRaises(ValueError):
            metrics.evaluate(trajectory(), gt)

    def test_gt_association_is_unique(self):
        est = np.insert(trajectory(), 1, [0.005, 0, 0, 0, 0, 0, 0, 1], axis=0)
        pairs = metrics.associate(est, trajectory(), 0.01)
        self.assertEqual(len(pairs), 4)
        self.assertEqual(len({j for _, j in pairs}), 4)

    def test_camera_body_transform_uses_rotation_and_lever_arm(self):
        body = trajectory()
        body[:, 4:8] = Rotation.from_euler("z", [0, 30, 60, 90], degrees=True).as_quat()
        r_ic = Rotation.from_euler("x", 90, degrees=True).as_matrix()
        t_ic = np.array([0.2, 0.1, 0.05])
        r_wi = Rotation.from_quat(body[:, 4:8]).as_matrix()
        camera = body.copy()
        camera[:, 1:4] += np.einsum("nij,j->ni", r_wi, t_ic)
        camera[:, 4:8] = Rotation.from_matrix(r_wi @ r_ic).as_quat()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.yaml"
            path.write_text("%YAML:1.0\nextrinsicRotation: !!opencv-matrix\n  rows: 3\n  cols: 3\n  data: "
                            + str(r_ic.flatten().tolist())
                            + "\nextrinsicTranslation: !!opencv-matrix\n  rows: 3\n  cols: 1\n  data: "
                            + str(t_ic.tolist()) + "\n")
            transformed = metrics.camera_to_body(camera, path)
        np.testing.assert_allclose(transformed[:, 1:4], body[:, 1:4], atol=1e-12)
        self.assertLess(metrics.evaluate(transformed, body)["ape_rotation_deg"]["rmse"], 1e-10)


if __name__ == "__main__":
    unittest.main()
