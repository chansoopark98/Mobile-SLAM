import json
import math
import os
import re
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import metrics  # noqa: E402
import xr_bench  # noqa: E402

def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


MAN = {"R_BFromCam": [[1, 0, 0], [0, 0, 1], [0, -1, 0]], "t_ic": [0, 0, 0]}


def synth(n=120, dt=0.04):
    """GT in body frame plus a trace in the engine format that reproduces it (identity alignment)."""
    gt_t = np.arange(n) * dt
    gt_p = np.stack([np.sin(gt_t), 0.2 * gt_t, np.cos(gt_t)], 1)
    gt_r = np.array([metrics.quat_xyzw_to_matrix(np.array([0.0, math.sin(0.1 * t), 0.0, math.cos(0.1 * t)]))
                     for t in gt_t])
    r_bc = np.array(MAN["R_BFromCam"], float)
    rows = []
    for i, t in enumerate(gt_t):
        r_cv = gt_r[i] @ r_bc
        r_gl = r_cv @ xr_bench.D_GL_CV
        w = float(np.sqrt(max(0.0, 1 + np.trace(r_gl))) / 2)
        q = [w, (r_gl[2, 1] - r_gl[1, 2]) / (4 * w), (r_gl[0, 2] - r_gl[2, 0]) / (4 * w),
             (r_gl[1, 0] - r_gl[0, 1]) / (4 * w)]
        rows.append({"i": i, "t": float(t), "fresh": True, "status": "NORMAL", "srRepeated": False,
                     "p": gt_p[i].tolist(), "q_wxyz": q})
    return rows, {"t": gt_t, "p": gt_p, "R": gt_r}


class Args:
    fixture, arm, env, gt_kind, max_dt = "flip6-arcore-synth", "port-default", "desktop-replay", None, 0.006
    out = ledger = None


class BenchTest(unittest.TestCase):
    def test_trace_roundtrip_exact(self):
        rows, gt = synth()
        core, _ = xr_bench.build_row(Args, rows, MAN, gt, None)
        acc = core["accuracy"]
        self.assertTrue(acc["valid"])
        self.assertLessEqual(acc["ate_pos_rmse_m"], 1e-9)
        self.assertEqual(core["gt_kind"], "reference")
        self.assertIn("not independent", core["gt_note"])

    def test_limited_and_predicted_not_in_coverage(self):  # criterion 5
        rows, gt = synth()
        for r in rows[60:]:
            r["status"] = "LIMITED"
        for r in rows[40:60]:
            r["fresh"] = False  # stale / predicted republication
        core, _ = xr_bench.build_row(Args, rows, MAN, gt, None)
        acc = core["accuracy"]
        self.assertEqual(acc["accepted_poses"], 40)
        self.assertAlmostEqual(acc["coverage"], 40 / 120)
        self.assertEqual(core["robustness"]["limited_ratio"], 60 / 120)
        self.assertEqual(core["robustness"]["stale_not_fresh_poses"], 20)

    def test_only_limited_is_invalid(self):  # criterion 2 via the trace path
        rows, gt = synth()
        for r in rows:
            r["status"] = "LIMITED"
        core, _ = xr_bench.build_row(Args, rows, MAN, gt, None)
        self.assertFalse(core["accuracy"]["valid"])
        self.assertFalse(core["comparable"])

    def test_row_digest_independent_of_timing(self):  # criterion 3 machinery
        rows, gt = synth()
        for r in rows:
            r["srStageMs"] = {"frontend": 1.0, "glue": 0.1}
        a, ta = xr_bench.build_row(Args, rows, MAN, gt, None)
        for r in rows:
            r["srStageMs"] = {"frontend": 9.0, "glue": 0.7}
        b, tb = xr_bench.build_row(Args, rows, MAN, gt, None)
        self.assertEqual(xr_bench.canonical(a), xr_bench.canonical(b))
        self.assertNotEqual(xr_bench.canonical(ta), xr_bench.canonical(tb))

    def test_verdict_is_report_only_and_no_thresholds(self):  # criterion 4
        rows, gt = synth()
        core, _ = xr_bench.build_row(Args, rows, MAN, gt, None)
        self.assertEqual(core["verdict"], "report-only")
        for name in ("xr_bench.py", "metrics.py", "ledger.py"):
            src = _read(os.path.join(HERE, "..", name))
            self.assertNotRegex(src, r"(?i)\b(pass|fail)_?(threshold|limit)|PASS_THRESH|assert_below")
            self.assertNotRegex(src, r'"verdict":\s*"(pass|fail)"', name)

    def test_no_host_paths_in_tracked_files(self):
        pat = re.compile("|".join(["/home/" + "park", "/media/" + "park", "_quaran" + "tine", "park_" + "file"]))
        for name in os.listdir(os.path.join(HERE, "..")):
            full = os.path.join(HERE, "..", name)
            if os.path.isfile(full) and name.endswith(".py"):
                self.assertIsNone(pat.search(_read(full)), name)
        for name in os.listdir(HERE):
            if name.endswith(".py"):
                self.assertIsNone(pat.search(_read(os.path.join(HERE, name))), name)


if __name__ == "__main__":
    unittest.main()
