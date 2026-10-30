"""Unit tests of the registration helper of the image-target scale gate (its_finalize.py): phase B (seed) on a throwaway
seed and a hand-built manifest. No engine, no coordinator seed, no registered data of the gate.

Run (repo root): python3 -I -m unittest discover -s slam_refact/tools/image_target/tests -p 'test_its_finalize.py' -v
"""
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

TOOL = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))

import its_finalize as fin  # noqa: E402
import curvy_held_tool as ht  # noqa: E402


def manifest():
    return {
        'gate_id': 'it-scale-gate', 'version': 1, 'created_at': '2026-10-20T10:00:00+09:00',
        'coordinator_seed': {'value': None, 'pending': 'x'}, 'held_pins': {}, 'inputs_tree_sha256': {'registered': None},
        'review': {'at': None}, 'first_real_run_at': None, 'revisions': [], 'pending': ['phase B: x', 'other'],
        'tool_sha256': {s: fin.sha_file(fin.ROOT / s) for s in fin.SOURCES},
    }


class PhaseB(unittest.TestCase):
    def run_seed(self, tmp, seed_text, m, extra=()):
        mp = pathlib.Path(tmp) / 'm.json'
        mp.write_text(json.dumps(m))
        sf = pathlib.Path(tmp) / 'seed.txt'
        if seed_text is not None:
            sf.write_text(seed_text)
        r = subprocess.run([sys.executable, '-I', str(TOOL / 'its_finalize.py'), 'seed', '--seed-file', str(sf),
                            '--registered-dir', str(pathlib.Path(tmp) / 'reg'), '--manifest', str(mp), *extra],
                           capture_output=True, text=True)
        return r, json.loads(mp.read_text())

    def test_valid_seed_writes_the_pins_and_nothing_else(self):
        with tempfile.TemporaryDirectory() as tmp:
            r, m = self.run_seed(tmp, '424242 2026-10-20T12:00:00+09:00\n', manifest())
            self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
            self.assertEqual(m['coordinator_seed']['value'], 424242)
            self.assertEqual(m['coordinator_seed']['generated_at'], '2026-10-20T12:00:00+09:00')
            self.assertEqual(m['inputs_tree_sha256']['registered'], ht.tree_sha(str(pathlib.Path(tmp) / 'reg')))
            self.assertEqual((pathlib.Path(tmp) / 'reg/variant.txt').read_text(), 'registered\n')
            self.assertNotEqual(m['created_at'], '2026-10-20T10:00:00+09:00')
            self.assertEqual(m['review'], {'at': None})
            self.assertIsNone(m['first_real_run_at'])
            self.assertEqual(m['pending'], ['other'])

    def test_refusals(self):
        with tempfile.TemporaryDirectory() as tmp:
            r, _ = self.run_seed(tmp, None, manifest())                      # no seed file
            self.assertNotEqual(r.returncode, 0)
            self.assertIn('does not exist', r.stderr + r.stdout)
            r, _ = self.run_seed(tmp, '9150 x\n', manifest())                # an executor seed
            self.assertNotEqual(r.returncode, 0)
            self.assertIn('executor seeds', r.stderr + r.stdout)
            m = manifest()
            m['review']['at'] = '2026-10-20T13:00:00+09:00'
            r, _ = self.run_seed(tmp, '424242\n', m)                         # already reviewed
            self.assertNotEqual(r.returncode, 0)
            m = manifest()
            m['first_real_run_at'] = '2026-10-20T13:00:00+09:00'
            r, _ = self.run_seed(tmp, '424242\n', m)                         # already run
            self.assertNotEqual(r.returncode, 0)
            m = manifest()
            m['tool_sha256'][fin.SOURCES[0]] = '0' * 64                       # a pinned source changed
            r, _ = self.run_seed(tmp, '424242\n', m)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn('sources changed', r.stderr + r.stdout)


if __name__ == '__main__':
    unittest.main()
