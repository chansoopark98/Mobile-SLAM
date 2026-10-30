"""Unit tests of the scenario tool of the image-target scale gate (US-015; its_gate_data.py, it-scale-gate.v1.md section 1).

They fix what the executor must not be able to adjust: the derivation of the registered table from a seed is a
deterministic function (a pinned digest over a seed that is NOT the coordinator seed), every derived value lies in its
registered range, the premises of the gate (camera steps above / below the computeScale baseline threshold, k away from the
public prior 0.5, pair parameters) hold for many seeds, and the variants cannot be mixed up (registered needs a seed file;
dev takes none; dev-seed is labelled dev).

Run (repo root): python3 -I -m unittest discover -s slam_refact/tools/image_target/tests -p 'test_its_gate_data.py' -v
"""
import hashlib
import pathlib
import subprocess
import sys
import tempfile
import unittest

TOOL = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))

import its_gate_data as d  # noqa: E402

COL = {name: i for i, name in enumerate(d.COLUMNS)}
BASELINE_THRESHOLD = 3e-2  # ImageTrackingState::computeScale, public source


def rows_by_id(rows):
    return {r[0]: r for r in rows}


class Derivation(unittest.TestCase):
    def test_deterministic_and_pinned(self):
        # 9101 is an executor seed of the dev range checks (gate file section 1), not the coordinator seed. The digest pins the
        # derivation: a change of any label, range or digit count changes it.
        text = '\n'.join('\t'.join(d._fmt(v) for v in row) for row in d.derive_registered(9101))
        self.assertEqual(text, '\n'.join('\t'.join(d._fmt(v) for v in row) for row in d.derive_registered(9101)))
        self.assertEqual(hashlib.sha256(text.encode()).hexdigest(), PINNED_9101)
        self.assertNotEqual(d.derive_registered(9101), d.derive_registered(9102))

    def test_u_is_sha256_first_eight_bytes(self):
        h = hashlib.sha256(b'9101:main.A.k').digest()
        self.assertEqual(d._seed_u(9101, 'main.A.k'), int.from_bytes(h[:8], 'big') / 2.0 ** 64)

    def test_columns_and_ids(self):
        for seed in (1, 9101, 3018981447):
            rows = d.derive_registered(seed)
            self.assertTrue(all(len(r) == len(d.COLUMNS) for r in rows))
            ids = [r[0] for r in rows]
            self.assertEqual(len(ids), len(set(ids)))
            by = rows_by_id(rows)
            for r in rows:
                if r[COL['pair_of']]:
                    self.assertIn(r[COL['pair_of']], by)
        self.assertTrue(all(len(r) == len(d.COLUMNS) for r in d.DEV_ROWS))
        self.assertEqual(len({r[0] for r in d.DEV_ROWS}), len(d.DEV_ROWS))


class Ranges(unittest.TestCase):
    def test_premises_hold_for_many_seeds(self):
        for seed in range(2000, 2300):
            rows = d.derive_registered(seed)
            by = rows_by_id(rows)
            for tag in 'ABCD':
                r = by['r' + tag]
                k = r[COL['k']]
                lo, hi = d.K_RANGES[tag]
                self.assertTrue(lo <= k <= hi, (seed, tag, k))
                self.assertGreaterEqual(abs(k - 0.5) / k, 0.05)                       # premise P2
                self.assertTrue(0.040 <= r[COL['speed']] <= 0.060)                      # camera step > 0.033 (premise P1)
                self.assertGreater(r[COL['speed']] * 0.94, BASELINE_THRESHOLD * 1.1 - 0.004)
                self.assertGreaterEqual(r[COL['n_frames']] - 60, 40)                  # premise P3
                self.assertTrue(1.15 <= r[COL['rho']] <= 1.45 and 25.0 <= r[COL['span_deg']] <= 40.0)
                self.assertTrue(1000 <= r[COL['target_seed']] <= 999999)
            slow = by['rZ1']
            self.assertTrue(0.008 <= slow[COL['speed']] <= 0.015)
            self.assertLess(slow[COL['speed']] * 1.06, BASELINE_THRESHOLD * 0.9)       # premise P1s (max step with modulation)
            self.assertEqual(by['rZ0'][COL['kind']], 'zero')
            e1, e2, l0 = by['rE1'], by['rE2'], by['rL0']
            self.assertTrue(1.4 <= e1[COL['pair_c']] <= 2.0)
            self.assertTrue(30.0 <= e2[COL['pair_c']] <= 150.0)
            self.assertTrue(60 <= l0[COL['loss_start']] <= 80 and 8 <= l0[COL['loss_len']] <= 12)
            self.assertEqual((by['rN1'][COL['noise_px']], by['rN2'][COL['noise_px']]), (0.5, 1.0))
            for rid in ('rE1', 'rL0'):
                self.assertEqual(by[rid][COL['k']], by['rB'][COL['k']])
            for rid in ('rE2', 'rN1', 'rN2'):
                self.assertEqual(by[rid][COL['k']], by['rC'][COL['k']])

    def test_kinds_present(self):
        kinds = [r[COL['kind']] for r in d.derive_registered(5)]
        for need, n in (('main', 4), ('zero', 1), ('slow', 1), ('scale_pair', 1), ('yaw_pair', 1), ('loss', 1), ('noise', 2)):
            self.assertEqual(kinds.count(need), n, need)
        self.assertFalse([k for k in kinds if k.startswith('null_')])  # null rows are dev only

    def test_dev_rows_cover_the_null_run(self):
        kinds = [r[COL['kind']] for r in d.DEV_ROWS]
        for need in ('null_main', 'null_scale_pair', 'null_yaw_pair'):
            self.assertIn(need, kinds)
        by = rows_by_id(d.DEV_ROWS)
        self.assertEqual(by['dN0'][COL['k']], 0.5)          # truth = the public prior
        self.assertEqual(by['dE1n'][COL['pair_c']], 1.0)    # identity pairs
        self.assertEqual(by['dE2n'][COL['pair_c']], 0.0)


class Variants(unittest.TestCase):
    def run_tool(self, *args):
        return subprocess.run([sys.executable, '-I', str(TOOL / 'its_gate_data.py'), *args], capture_output=True, text=True)

    def test_registered_needs_a_seed_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = self.run_tool('render', '--variant', 'registered', '--out', tmp + '/o')
            self.assertNotEqual(r.returncode, 0)
            self.assertIn('--seed-file', r.stderr + r.stdout)
            r = self.run_tool('render', '--variant', 'registered', '--seed', '5', '--out', tmp + '/o')
            self.assertNotEqual(r.returncode, 0)
            r = self.run_tool('render', '--variant', 'registered', '--seed-file', tmp + '/missing.txt', '--out', tmp + '/o')
            self.assertNotEqual(r.returncode, 0)
            self.assertIn('does not exist', r.stderr + r.stdout)
            seed_file = pathlib.Path(tmp) / 'seed.txt'
            seed_file.write_text('not a number\n')
            r = self.run_tool('render', '--variant', 'registered', '--seed-file', str(seed_file), '--out', tmp + '/o')
            self.assertNotEqual(r.returncode, 0)

    def test_dev_takes_no_seed_and_is_labelled_dev(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertNotEqual(self.run_tool('render', '--variant', 'dev', '--seed', '1', '--out', tmp + '/a').returncode, 0)
            self.assertEqual(self.run_tool('render', '--variant', 'dev', '--out', tmp + '/a').returncode, 0)
            self.assertEqual((pathlib.Path(tmp) / 'a/variant.txt').read_text(), 'dev\n')
            self.assertFalse((pathlib.Path(tmp) / 'a/seed.txt').exists())

    def test_dev_seed_is_labelled_dev_and_registered_is_labelled_registered(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.run_tool('render', '--variant', 'dev-seed', '--seed', '9101', '--out', tmp + '/s').returncode, 0)
            self.assertEqual((pathlib.Path(tmp) / 's/variant.txt').read_text(), 'dev\n')
            seed_file = pathlib.Path(tmp) / 'coordinator-seed.txt'
            seed_file.write_text('9101 2026-10-20T00:00:00+09:00\n')   # a throwaway file for this test, not a coordinator seed
            self.assertEqual(self.run_tool('render', '--variant', 'registered', '--seed-file', str(seed_file), '--out', tmp + '/r').returncode, 0)
            self.assertEqual((pathlib.Path(tmp) / 'r/variant.txt').read_text(), 'registered\n')
            self.assertEqual((pathlib.Path(tmp) / 'r/seed.txt').read_text(), '9101\n')
            # same numbers; only the label (and seed.txt) differ, which is what the held device and the dev guard look at
            self.assertEqual((pathlib.Path(tmp) / 'r/scenarios.tsv').read_text(), (pathlib.Path(tmp) / 's/scenarios.tsv').read_text())

    def test_dev_seed_accepts_only_executor_seeds(self):
        with tempfile.TemporaryDirectory() as tmp:
            for bad in ('5', '3018981447', '9200', '9099'):
                r = self.run_tool('render', '--variant', 'dev-seed', '--seed', bad, '--out', tmp + '/s')
                self.assertNotEqual(r.returncode, 0, bad)
                self.assertIn('executor seeds', r.stderr + r.stdout)
            self.assertEqual(self.run_tool('render', '--variant', 'dev-seed', '--seed', '9199', '--out', tmp + '/s').returncode, 0)

    def test_dev_table_is_byte_stable(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.run_tool('render', '--variant', 'dev', '--out', tmp + '/a').returncode, 0)
            text = (pathlib.Path(tmp) / 'a/scenarios.tsv').read_bytes()
            self.assertEqual(hashlib.sha256(text).hexdigest(), PINNED_DEV)


PINNED_9101 = '56462e2ba6bd9b7572e6972f50ab77bd689fcd64229bc4be118fa5f589060fa4'
PINNED_DEV = 'c468c35f8753234cecdd6b4c21f72c18c3ad61cc5646671ad22245ae1035f330'

if __name__ == '__main__':
    unittest.main()
