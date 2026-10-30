"""Unit tests of the it-world gate tools after the v2 patch (docs/design/it-world-gate-v2-proposed.patch) and the version designation.

  python3 -B -I slam_refact/tools/re/iwt_oracle/test_it_world_tools.py -v      (from the repo root)
Synthetic inputs only: no engine, no original wasm, no node. What is checked:
  F-8   compare.py: a finite/non-finite mismatch is a failure (the old max() dropped a NaN difference)
  R-8   it_world_gate.py keeps the engine's stderr next to engine.txt (it was discarded)
  G-O1  evaluate(): a finite/non-finite class mismatch fails the integer gate (unchanged behaviour, pinned here)
  v<N>  the registered mode reads the files of the DESIGNATED version and creates that version's marker only
"""
import contextlib
import hashlib
import io
import json
import math
import os
import pathlib
import stat
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True
import compare  # noqa: E402
import it_world_gate as g  # noqa: E402

INF, NAN = math.inf, math.nan


def mat(v=0.0):
  return [float(v + i) for i in range(16)]


def oracle_case(name='c1', x=1.0):
  state = {'scale': 1.0, 'world_pose': {'m': mat(), 'inv': mat(1)}, 'rays': [], 'search_pts_in_camera': [], 'world_pts': [],
           'target_pts': [], 'visible': [], 'ids': []}
  return {'name': name, 'error': None, 'out': {'m': mat(x), 'inv': mat(2)}, 'state': state}


def engine_text(name='c1', out_first=1.0):
  out = mat(out_first)
  return '\n'.join(['case ' + name, 'scale 1', 'out ' + ' '.join(repr(v) for v in out + mat(2)),
                    'state_pose ' + ' '.join(repr(v) for v in mat() + mat(1)),
                    'rays 0', 'search 0', 'world 0', 'visible 0', 'ids 0', 'target 0']) + '\n'


class CompareFloatsTest(unittest.TestCase):
  def test_finite_against_nonfinite_is_infinite_difference(self):
    for a, b in ((1.0, NAN), (1.0, INF), (INF, -INF), (INF, 1.0), (NAN, 1.0), (None, 2.0)):
      same, mx, n = compare.floats([a], [b])
      self.assertEqual(same, 0, (a, b))
      self.assertEqual(mx, INF, (a, b))
      self.assertEqual(n, 1)

  def test_matching_values_and_matching_nonfinite_classes_stay_clean(self):
    self.assertEqual(compare.floats([1.5, INF, None], [1.5, INF, NAN])[:2], (3, 0.0))   # null in the oracle JSON = NaN or an infinity
    self.assertEqual(compare.floats([None], [-INF])[:2], (1, 0.0))

  def test_ordinary_difference_is_still_measured(self):
    same, mx, _ = compare.floats([1.0, 2.0], [1.0, 2.5])
    self.assertEqual(same, 1)
    self.assertAlmostEqual(mx, 0.5)

  def test_one_nonfinite_mismatch_is_not_hidden_by_a_later_finite_difference(self):
    self.assertEqual(compare.floats([1.0, 2.0], [NAN, 2.5])[1], INF)


class CompareMainTest(unittest.TestCase):
  def run_compare(self, oracle, engine):
    with tempfile.TemporaryDirectory() as tmp:
      o, e = pathlib.Path(tmp, 'oracle.json'), pathlib.Path(tmp, 'engine.txt')
      o.write_text(json.dumps({'results': oracle}))
      e.write_text(engine)
      proc = subprocess.run([sys.executable, '-B', '-I', str(HERE / 'compare.py'), str(o), str(e)], capture_output=True, text=True)
      return proc.returncode

  def test_null_run_passes(self):
    self.assertEqual(self.run_compare([oracle_case()], engine_text()), 0)

  def test_nan_in_the_engine_output_fails(self):
    self.assertEqual(self.run_compare([oracle_case()], engine_text(out_first=NAN)), 1)

  def test_infinity_in_the_engine_output_fails(self):
    self.assertEqual(self.run_compare([oracle_case()], engine_text(out_first=INF)), 1)

  def test_small_finite_difference_inside_the_tolerance_passes(self):
    self.assertEqual(self.run_compare([oracle_case()], engine_text(out_first=1.000001)), 0)


class EvaluateTest(unittest.TestCase):
  def evaluate(self, oracle, engine):
    with tempfile.TemporaryDirectory() as tmp:
      cases, o, e = pathlib.Path(tmp, 'cases.json'), pathlib.Path(tmp, 'oracle.json'), pathlib.Path(tmp, 'engine.txt')
      cases.write_text('{}')
      o.write_text(json.dumps({'results': oracle}))
      e.write_text(engine)
      return g.evaluate(cases, o, e)

  def test_null_run_passes_both_gates(self):
    r = self.evaluate([oracle_case()], engine_text())
    self.assertTrue(r['G-O1']['pass'])
    self.assertTrue(r['G-O2']['pass'])

  def test_finite_against_nonfinite_fails_the_integer_gate(self):
    r = self.evaluate([oracle_case()], engine_text(out_first=NAN))
    self.assertFalse(r['G-O1']['pass'])
    self.assertEqual(r['G-O1']['failing'], ['c1'])
    self.assertFalse(r['G-O2']['pass'])

  def test_relative_error_above_the_tolerance_fails(self):
    r = self.evaluate([oracle_case()], engine_text(out_first=1.001))
    self.assertTrue(r['G-O1']['pass'])
    self.assertFalse(r['G-O2']['pass'])


class EngineStderrTest(unittest.TestCase):
  """R-8: the dev/registered run keeps the stderr of the engine replay (engine.stderr) instead of discarding it."""

  def test_dev_run_writes_engine_stderr(self):
    with tempfile.TemporaryDirectory() as tmp:
      tmp = pathlib.Path(tmp)
      run_dir = tmp / 'run'  # main() creates it (exist_ok=False) before the first subprocess
      replay = tmp / 'fake_replay.sh'
      replay.write_text('#!/bin/sh\necho "case c1"\necho "diag: the engine said something" >&2\n')
      replay.chmod(replay.stat().st_mode | stat.S_IXUSR)
      real_run, real_eval = g.subprocess.run, g.evaluate

      def fake_run(args, **kw):
        if args[0] == sys.executable:  # make_cases.py: leave the two files it writes
          (run_dir / 'cases.json').write_text('{}')
          (run_dir / 'cases.txt').write_text('iwt-cases 1 0\n')
          return subprocess.CompletedProcess(args, 0)
        if args[0] == 'node':  # run_f994.mjs: an empty oracle
          kw['stdout'].write('{"results": []}')
          return subprocess.CompletedProcess(args, 0)
        return real_run(args, **kw)  # the replay: really executed

      g.subprocess.run = fake_run
      g.evaluate = lambda *a: {'G-O1': {'pass': True}, 'G-O2': {'pass': True}}
      old_argv = sys.argv
      sys.argv = ['it_world_gate.py', 'dev', '--out', str(run_dir), '--n', '1', '--replay', str(replay), '--wasm', str(replay)]
      try:
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as ctx:
          g.main()
      finally:
        g.subprocess.run, g.evaluate, sys.argv = real_run, real_eval, old_argv
      self.assertEqual(ctx.exception.code, 0)
      self.assertEqual((run_dir / 'engine.txt').read_text(), 'case c1\n')
      self.assertIn('diag: the engine said something', (run_dir / 'engine.stderr').read_text())


class GateVersionTest(unittest.TestCase):
  def test_designated_version(self):
    self.assertEqual(g.gate_version('it-world-gate.v1'), 1)
    self.assertEqual(g.gate_version('it-world-gate.v12'), 12)

  def test_anything_else_is_held(self):
    for bad in ('', 'it-world-gate', 'it-world-gate.v', 'it-world-gate.v0', 'it-world-gate.v02', 'it-world-gate.vx', 'other.v2',
                'it-world-gate.v1234567', 'it-world-gate.v2 '):
      with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as ctx:
        g.gate_version(bad)
      self.assertEqual(ctx.exception.code, 3, bad)


class RegisteredModeTest(unittest.TestCase):
  """check_registered reads the manifest, gate md, seed file and marker of the DESIGNATED version."""

  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.root = pathlib.Path(self.tmp.name)
    self.prereg = self.root / 'prereg'
    self.prereg.mkdir()
    self.replay, self.wasm = self.root / 'replay', self.root / 'f994.wasm'
    self.replay.write_bytes(b'replay bytes')
    self.wasm.write_bytes(b'wasm bytes')
    self.saved = (g.PREREG, g.REPLAY, g.WASM)
    g.PREREG, g.REPLAY, g.WASM = self.prereg, self.replay, self.wasm

  def tearDown(self):
    g.PREREG, g.REPLAY, g.WASM = self.saved
    self.tmp.cleanup()

  def sha(self, p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()

  def register(self, version, designation_gate=None, designation_version=None, seed='123456789 coordinator'):
    gate_md = self.prereg / ('it-world-gate.v%d.md' % version)
    gate_md.write_text('gate v%d' % version)
    pinned = {'gate_md_sha256': self.sha(gate_md), 'replay_sha256': self.sha(self.replay), 'wasm_sha256': self.sha(self.wasm),
              'make_cases_sha256': self.sha(HERE / 'make_cases.py'), 'run_f994_sha256': self.sha(HERE / 'run_f994.mjs'),
              'compare_sha256': self.sha(HERE / 'compare.py'), 'runner_sha256': self.sha(HERE / 'it_world_gate.py')}
    designation = {'gate': designation_gate or 'it-world-gate', 'first_real_run_permitted': True, 'pinned': pinned}
    if designation_version is not None:
      designation['version'] = designation_version
    d_path = self.prereg / ('it-world-gate.v%d.review-designation.json' % version)
    d_path.write_text(json.dumps(designation))
    manifest = {'sha256': self.sha(gate_md), 'review': {'at': '2026-10-20T10:00:00+09:00', 'designation_file_sha256': self.sha(d_path)}}
    (self.prereg / ('it-world-gate.v%d.manifest.json' % version)).write_text(json.dumps(manifest))
    (self.prereg / ('it-world-gate.v%d.coordinator-seed.txt' % version)).write_text(seed + '\n')
    return d_path

  def check(self, d_path, version):
    with contextlib.redirect_stdout(io.StringIO()):
      return g.check_registered(str(d_path), version)

  def held(self, d_path, version):
    with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as ctx:
      g.check_registered(str(d_path), version)
    self.assertEqual(ctx.exception.code, 3)

  def test_v2_uses_the_v2_files_and_creates_only_the_v2_marker(self):
    d = self.register(2)
    seed, checks = self.check(d, 2)
    self.assertEqual(seed, 123456789)
    self.assertIn('marker created', checks)
    self.assertTrue((self.prereg / 'it-world-gate.v2.first-run.marker').exists())
    self.assertFalse((self.prereg / 'it-world-gate.v1.first-run.marker').exists())
    self.held(d, 2)  # the marker exists now: a second start is HELD

  def test_a_v1_registration_cannot_start_v2(self):
    d = self.register(1)
    with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(FileNotFoundError):
      g.check_registered(str(d), 2)
    self.assertFalse((self.prereg / 'it-world-gate.v2.first-run.marker').exists())

  def test_designation_naming_another_version_is_held_and_leaves_no_marker(self):
    for kw in ({'designation_version': 1}, {'designation_gate': 'it-world-gate.v1'}):
      d = self.register(2, **kw)
      self.held(d, 2)
      self.assertFalse((self.prereg / 'it-world-gate.v2.first-run.marker').exists(), kw)

  def test_designation_in_the_gate_dot_version_form_is_accepted(self):
    d = self.register(2, designation_gate='it-world-gate.v2')
    self.assertEqual(self.check(d, 2)[0], 123456789)

  def test_changed_replay_binary_is_held_and_leaves_no_marker(self):
    d = self.register(2)
    self.replay.write_bytes(b'rebuilt')
    self.held(d, 2)
    self.assertFalse((self.prereg / 'it-world-gate.v2.first-run.marker').exists())


if __name__ == '__main__':
  unittest.main()
