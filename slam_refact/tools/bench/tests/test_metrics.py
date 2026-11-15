import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import metrics  # noqa: E402


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def traj(n=200, dt=0.05):
    t = np.arange(n) * dt
    p = np.stack([np.sin(t), np.cos(0.7 * t), 0.1 * t], 1) * 2.0
    r = np.array([rot_z(0.3 * x) @ rot_x(0.1 * x) for x in t])
    return {"t": t, "p": p, "R": r}


def transformed(gt, rot, tr, scale=1.0):
    return {"t": gt["t"], "p": scale * (gt["p"] @ rot.T) + tr, "R": np.array([rot @ r for r in gt["R"]])}


class MetricsTest(unittest.TestCase):
    def test_known_se3_noise_free(self):  # criterion 1a
        gt = traj()
        rot, tr = rot_z(0.8) @ rot_x(-0.4), np.array([1.0, -2.0, 0.5])
        est = transformed(gt, rot, tr)
        s = metrics.score_trajectory(est, gt, len(gt["t"]))
        self.assertTrue(s["valid"])
        self.assertLessEqual(s["ate_pos_rmse_m"], 1e-9)
        self.assertLessEqual(s["ate_rot_rmse_deg"], 1e-6)
        self.assertEqual(s["coverage"], 1.0)

    def test_scale_half_is_not_hidden_by_se3(self):  # criterion 1b
        gt = traj()
        est = transformed(gt, rot_z(0.2), np.array([0.3, 0.1, 0.0]), scale=0.5)
        s = metrics.score_trajectory(est, gt, len(gt["t"]))
        self.assertGreater(s["ate_pos_rmse_m"], 0.0)
        self.assertAlmostEqual(s["sim3_diagnostic"]["scale_est_over_gt"], 0.5, delta=1e-9)
        self.assertLessEqual(s["sim3_diagnostic"]["ate_pos_rmse_m"], 1e-9)

    def test_zero_poses_invalid(self):  # criterion 2
        gt = traj()
        est = {"t": np.zeros(0), "p": np.zeros((0, 3)), "R": np.zeros((0, 3, 3))}
        s = metrics.score_trajectory(est, gt, 100)
        self.assertFalse(s["valid"])
        self.assertEqual(s["invalid_reason"], "zero_poses")
        self.assertIsNone(s["ate_pos_rmse_m"])

    def test_zero_associations_invalid(self):  # criterion 2
        gt = traj()
        est = transformed(gt, np.eye(3), np.zeros(3))
        est["t"] = est["t"] + 1000.0
        s = metrics.score_trajectory(est, gt, len(gt["t"]))
        self.assertFalse(s["valid"])
        self.assertEqual(s["invalid_reason"], "zero_associations")
        self.assertEqual(s["coverage"], 0.0)

    def test_all_poses_identical_invalid(self):  # zero-motion pose is never a pass
        gt = traj()
        est = {"t": gt["t"].copy(), "p": np.zeros((len(gt["t"]), 3)), "R": np.array([np.eye(3)] * len(gt["t"]))}
        s = metrics.score_trajectory(est, gt, len(gt["t"]))
        self.assertFalse(s["valid"])
        self.assertEqual(s["invalid_reason"], "zero_motion_pose")

    def test_association_is_one_to_one(self):
        gt_t = np.array([0.0, 1.0, 2.0])
        ei, gi = metrics.associate([0.001, 0.002, 1.0], gt_t, 0.006)
        self.assertEqual(list(gi), sorted(set(gi)))
        self.assertEqual(len(ei), 2)  # two estimates compete for GT row 0; the closer one wins


if __name__ == "__main__":
    unittest.main()
