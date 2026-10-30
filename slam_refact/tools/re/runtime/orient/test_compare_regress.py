"""Unit tests of compare_regress.py (US-023 parity comparison written to a file).

  python3 -I -m unittest slam_refact/tools/re/runtime/orient/test_compare_regress.py   (from the repo root)
Synthetic traces only; no recording, no engine.
"""
import json
import os
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import compare_regress as cr  # noqa: E402


def write_bundle(root, name, rows, wp=None, stage_ms=1.0):
  d = pathlib.Path(root, name)
  d.mkdir(parents=True)
  with open(d / 'trace.jsonl', 'w') as f:
    for i, p in enumerate(rows):
      f.write(json.dumps({'i': i, 'p': p, 'status': 'NORMAL', 'srStageMs': {'frontend': stage_ms + i}, 'wallSeconds': stage_ms,
                          'srTimingX': stage_ms}) + '\n')
  with open(d / 'worldpoints-sampled.jsonl', 'w') as f:
    f.write(json.dumps({'frameIndex': 0, 'points': wp or []}) + '\n')


class CompareRegressTest(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.root = pathlib.Path(self.tmp.name)
    self.rows = [[0, 2, 0], [0.1, 2, 0.2], [0.3, 2.1, 0.2]]

  def tearDown(self):
    self.tmp.cleanup()

  def make(self, label, bundles, **kw):
    d = self.root / label
    for b in bundles:
      write_bundle(d, b, self.rows, **kw)
    return d

  def test_identical_up_to_timing_fields(self):
    a, b = self.make('a', ['x', 'y']), self.make('b', ['x', 'y'], stage_ms=7.5)
    lines, ok, total = cr.compare_dirs(a, b, expect_bundles=2)
    self.assertTrue(ok, lines)
    self.assertEqual(total, 2 * (3 + 1))
    self.assertEqual(lines[-1], 'ALL IDENTICAL rows 8')
    self.assertEqual(sum('IDENTICAL sha256' in l for l in lines), 4)

  def test_one_changed_value_fails_and_names_the_row(self):
    a, b = self.make('a', ['x', 'y']), self.make('b', ['x'])
    write_bundle(b, 'y', [[0, 2, 0], [0.1, 2, 0.2], [0.3, 2.1, 0.2000001]])
    lines, ok, _ = cr.compare_dirs(a, b, expect_bundles=2)
    self.assertFalse(ok)
    diff = [l for l in lines if 'DIFFERENT' in l]
    self.assertEqual(len(diff), 1)
    self.assertIn('first differing row 2', diff[0])
    self.assertEqual(lines[-1], 'DIFFERENCES (see the lines above)')

  def test_changed_worldpoints_fail(self):
    a, b = self.make('a', ['x']), self.root / 'b'
    write_bundle(b, 'x', self.rows, wp=[[1, 2, 3]])
    _, ok, _ = cr.compare_dirs(a, b, expect_bundles=1)
    self.assertFalse(ok)

  def test_missing_bundle_and_missing_file_fail(self):
    a, b = self.make('a', ['x', 'y']), self.make('b', ['x'])
    lines, ok, _ = cr.compare_dirs(a, b, expect_bundles=1)
    self.assertFalse(ok)
    self.assertTrue(any('MISSING in B' in l for l in lines), lines)
    c = self.make('c', ['x', 'y'])
    (c / 'y' / 'trace.jsonl').unlink()
    lines, ok, _ = cr.compare_dirs(a, c, expect_bundles=2)
    self.assertFalse(ok)
    self.assertTrue(any('MISSING (A present, B absent)' in l for l in lines), lines)

  def test_fewer_bundles_than_expected_fails_even_when_identical(self):
    a, b = self.make('a', ['x']), self.make('b', ['x'])
    lines, ok, _ = cr.compare_dirs(a, b, expect_bundles=14)
    self.assertFalse(ok)
    self.assertTrue(any('FEWER BUNDLES' in l for l in lines))

  def test_empty_directories_do_not_pass(self):
    (self.root / 'a').mkdir()
    (self.root / 'b').mkdir()
    _, ok, _ = cr.compare_dirs(self.root / 'a', self.root / 'b', expect_bundles=14)
    self.assertFalse(ok)

  def test_main_writes_the_file_once_and_sets_the_exit_code(self):
    a, b = self.make('a', ['x']), self.make('b', ['x'])
    out = self.root / 'compare.txt'
    self.assertEqual(cr.main(['--a', str(a), '--b', str(b), '--out', str(out), '--expect-bundles', '1']), 0)
    text = out.read_text()
    self.assertIn('ALL IDENTICAL rows 4', text)
    self.assertIn('# A = ', text)
    with self.assertRaises(FileExistsError):
      cr.main(['--a', str(a), '--b', str(b), '--out', str(out), '--expect-bundles', '1'])
    write_bundle(b, 'z', self.rows)
    out2 = self.root / 'compare2.txt'
    self.assertEqual(cr.main(['--a', str(a), '--b', str(b), '--out', str(out2), '--expect-bundles', '1']), 1)
    self.assertIn('DIFFERENCES', out2.read_text())


if __name__ == '__main__':
  unittest.main()
