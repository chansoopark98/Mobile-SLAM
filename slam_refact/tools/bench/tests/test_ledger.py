import json
import os
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import ledger  # noqa: E402


def entry(**kw):
    e = {"ts": "t0", "head_sha": "abc", "head_dirty": False, "fixture": "f", "arm": "port-default",
         "profile_env": "p", "engine_sha256": "e", "bundle_sha256": "b", "scorer_sha256": "s",
         "env_label": "desktop-replay", "gt_kind": "reference", "kind": "bench",
         "metrics": {"accuracy": {"valid": True, "ate_pos_rmse_m": 1.0, "ate_rot_rmse_deg": 2.0, "coverage": 0.5,
                                  "rpe_1s": {"pos_rmse_m": 0.1}}, "robustness": {"normal_ratio": 1.0, "reset_count": 0}}}
    e.update(kw)
    return e


class LedgerTest(unittest.TestCase):
    def test_append_only_two_lines_first_bytes_unchanged(self):  # B-3 1, 2
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "bench-ledger.jsonl")
            ledger.append(path, entry(ts="t1"))
            first = pathlib.Path(path).read_bytes()
            ledger.append(path, entry(ts="t2"))
            data = pathlib.Path(path).read_bytes()
            self.assertEqual(len(data.splitlines()), 2)
            self.assertTrue(data.startswith(first))
        self.assertFalse(any(n in dir(ledger) for n in ("overwrite", "rewrite", "delete", "truncate")))

    def test_missing_field_rejected_nothing_written(self):  # B-3 4
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "l.jsonl")
            for k in ledger.REQUIRED:
                e = entry()
                del e[k]
                with self.assertRaises(ValueError):
                    ledger.append(path, e)
            with self.assertRaises(ValueError):
                ledger.append(path, entry(engine_sha256=""))
            self.assertFalse(os.path.exists(path))

    def test_summary_never_merges_env_labels(self):  # B-3 3
        rows = [entry(ts="1", env_label="desktop-replay"), entry(ts="2", env_label="device")]
        md = ledger.summary_markdown(rows)
        self.assertIn("env=desktop-replay", md)
        self.assertIn("env=device", md)
        self.assertEqual(md.count("| f | port-default |"), 2)

    def test_summary_lists_invalid_outside_table(self):
        bad = entry(metrics={"accuracy": {"valid": False, "invalid_reason": "zero_poses"}, "robustness": {}})
        md = ledger.summary_markdown([bad])
        self.assertIn("INVALID (excluded from comparison)", md)
        self.assertNotIn("| f | port-default |", md)

    def test_verdict_column_is_report_only(self):
        md = ledger.summary_markdown([entry()])
        self.assertIn("| report-only |", md)


if __name__ == "__main__":
    unittest.main()
