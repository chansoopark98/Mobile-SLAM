"""Unit tests of rule7.py (OR-4 v6 rule 7): the comparison, the stages and the preconditions, on synthetic traces. The engine is never run.

  python3 -B -I slam_refact/tools/gates/or4-rule7/test_rule7.py -v        (from the repo root)
"""
import contextlib
import hashlib
import io
import json
import os
import pathlib
import sys
import tempfile
import types
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True
import rule7  # noqa: E402


def rows(seed, n=4, stage_ms=1.0, wall=0.5):
  return [{'i': i, 'p': [seed + i, 2.0, 0.5 * i], 'status': 'NORMAL', 'srStageMs': {'frontend': stage_ms + i}, 'wallSeconds': wall,
           'srTimingFoo': stage_ms} for i in range(n)]


def write_trace(directory, rs):
  d = pathlib.Path(directory)
  d.mkdir(parents=True, exist_ok=True)
  with open(d / 'trace.jsonl', 'w') as f:
    for r in rs:
      f.write(json.dumps(r) + '\n')


class Tree:
  """A synthetic pinned-reference tree and a place for the traces of the binary under test."""

  def __init__(self, root):
    self.root = pathlib.Path(root)
    self.or4 = self.root / 'or4'
    self.out = self.root / 'out'
    for k, key in enumerate(rule7.BUNDLE_KEYS):
      write_trace(self.or4 / 'v3/null/N' / key / 's0', rows(10 * k))
      write_trace(self.or4 / 'v3/decomp/D' / key / 'force', rows(10 * k + 1))
      write_trace(self.or4 / 'v6/real/T' / key / 's0', rows(10 * k + 2))

  def new(self, arm_dir, key, name, rs=None, stage_ms=9.0):
    k = rule7.BUNDLE_KEYS.index(key)
    base = {'N': 0, 'D': 1, 'T': 2}[arm_dir]
    write_trace(self.out / arm_dir / key / name, rows(10 * k + base, stage_ms=stage_ms, wall=7.0) if rs is None else rs)

  def fill(self, skip=()):
    for key in rule7.BUNDLE_KEYS:
      for _, arm_dir, name in rule7.ARMS:
        if (arm_dir, key) not in skip:
          self.new(arm_dir, key, name)


class CompareTest(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.t = Tree(self.tmp.name)
    self.saved = rule7.OR4
    rule7.OR4 = self.t.or4

  def tearDown(self):
    rule7.OR4 = self.saved
    self.tmp.cleanup()

  def test_definition_is_14_bundles_times_three_arms(self):
    self.assertEqual(len(rule7.BUNDLE_KEYS), 14)
    self.assertEqual(len(set(rule7.BUNDLE_KEYS)), 14)
    self.assertEqual([a[0] for a in rule7.ARMS], ['N s0', 'D force', 'T s0'])
    self.assertEqual(rule7.EXPECTED_TRACES, 42)

  def test_all_identical_up_to_timing_is_42_of_42(self):
    self.t.fill()
    table = rule7.compare_arms(self.t.out)
    self.assertEqual(len(table), 42)
    self.assertEqual(rule7.verdict(table), (True, '42/42'))

  def test_one_changed_value_in_any_arm_fails(self):
    for arm_dir, name in (('N', 's0'), ('D', 'force'), ('T', 's0')):
      with self.subTest(arm=arm_dir):
        self.t.fill()
        key = rule7.BUNDLE_KEYS[5]
        k = rule7.BUNDLE_KEYS.index(key)
        changed = rows(10 * k + {'N': 0, 'D': 1, 'T': 2}[arm_dir])
        changed[2]['p'][0] += 1e-9
        self.t.new(arm_dir, key, name, changed)
        ok, summary = rule7.verdict(rule7.compare_arms(self.t.out))
        self.assertFalse(ok)
        self.assertEqual(summary, '41/42')

  def test_changed_status_and_extra_row_fail(self):
    self.t.fill()
    key = rule7.BUNDLE_KEYS[0]
    changed = rows(0)
    changed[1]['status'] = 'LIMITED'
    self.t.new('N', key, 's0', changed)
    self.t.new('D', key, 'force', rows(1, n=5))
    self.assertEqual(rule7.verdict(rule7.compare_arms(self.t.out)), (False, '40/42'))

  def test_missing_new_trace_and_missing_reference_fail(self):
    self.t.fill(skip=[('T', rule7.BUNDLE_KEYS[13])])
    table = rule7.compare_arms(self.t.out)
    self.assertEqual(rule7.verdict(table), (False, '41/42'))
    self.assertIn('new trace missing', table[rule7.BUNDLE_KEYS[13] + '/T s0']['note'])
    self.t.new('T', rule7.BUNDLE_KEYS[13], 's0')
    (self.t.or4 / 'v3/null/N' / rule7.BUNDLE_KEYS[3] / 's0' / 'trace.jsonl').unlink()
    table = rule7.compare_arms(self.t.out)
    self.assertEqual(rule7.verdict(table), (False, '41/42'))
    self.assertIn('reference trace missing', table[rule7.BUNDLE_KEYS[3] + '/N s0']['note'])

  def test_fewer_than_42_compared_is_never_ok(self):
    self.t.fill()
    table = rule7.compare_arms(self.t.out, arms=rule7.ARMS[:2])
    self.assertEqual(len(table), 28)
    self.assertEqual(rule7.verdict(table), (False, '28/28'))

  def test_d_reference_falls_back_to_the_v5_directory(self):
    key = rule7.BUNDLE_KEYS[2]
    (self.t.or4 / 'v5/decomp/D' / key).mkdir(parents=True)
    os.rename(self.t.or4 / 'v3/decomp/D' / key / 'force', self.t.or4 / 'v5/decomp/D' / key / 'force')
    self.t.fill()
    self.assertEqual(rule7.verdict(rule7.compare_arms(self.t.out)), (True, '42/42'))

  def test_stripped_sha_matches_the_gate_tools_identical(self):
    sys.path.insert(0, str(HERE.parents[1] / 'image_target'))
    try:
      import or4_world_regression as g
    except Exception as e:  # the gate tool needs numpy
      self.skipTest(f'or4_world_regression not importable: {e}')
    a, b = rows(1), rows(1, stage_ms=99.0, wall=3.0)
    c = rows(1)
    c[3]['p'][2] += 1e-12
    for x, y in ((a, b), (a, c), (b, c)):
      write_trace(self.t.root / 'x', x)
      write_trace(self.t.root / 'y', y)
      same_sha = rule7.stripped_trace_sha(self.t.root / 'x' / 'trace.jsonl')[0] == rule7.stripped_trace_sha(self.t.root / 'y' / 'trace.jsonl')[0]
      self.assertEqual(same_sha, g.identical(x, y)[0])
    self.assertEqual(rule7.TIMING_KEYS, g.TIMING_KEYS)


class FakeGate:
  """What cmd_run needs from or4_world_regression, with a recording runner instead of the engine."""

  def __init__(self, tree, pin_file):
    self.tree = tree
    self.ENGINE_PIN = pin_file
    self.calls = []
    self.break_run = None
    self.change = {}      # (arm_dir, key) -> rows written instead of the identical ones


class RunStagesTest(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.t = Tree(self.tmp.name)
    self.pin = pathlib.Path(self.tmp.name) / 'pin.bin'
    self.pin.write_bytes(b'pinned binary')
    self.binary = pathlib.Path(self.tmp.name) / 'new.bin'
    self.binary.write_bytes(b'binary under test')
    self.bsha = hashlib.sha256(b'binary under test').hexdigest()
    self.saved = (rule7.OR4, rule7.PIN_BINARY_SHA256, rule7.load_gate_tool, rule7.run_arm)
    rule7.OR4 = self.t.or4
    rule7.PIN_BINARY_SHA256 = hashlib.sha256(b'pinned binary').hexdigest()
    self.gate = FakeGate(self.t, self.pin)
    rule7.load_gate_tool = lambda: self.gate

    def fake_run_arm(g, binary, out, arm_dir, workers):
      g.calls.append(arm_dir)
      for key in rule7.BUNDLE_KEYS:
        name = {'N': 's0', 'D': 'force', 'T': 's0'}[arm_dir]
        k = rule7.BUNDLE_KEYS.index(key)
        rs = g.change.get((arm_dir, key), rows(10 * k + {'N': 0, 'D': 1, 'T': 2}[arm_dir], stage_ms=5.0))
        write_trace(pathlib.Path(out) / arm_dir / key / name, rs)
      bad = [{'key': 'x', 'class': 'CRASH', 'rc': -11}] if g.break_run == arm_dir else []
      return [], bad

    rule7.run_arm = fake_run_arm

  def tearDown(self):
    rule7.OR4, rule7.PIN_BINARY_SHA256, rule7.load_gate_tool, rule7.run_arm = self.saved
    self.tmp.cleanup()

  def run_rule(self, out=None, workers=2, sha=None):
    out = out or self.t.out
    argv = ['run', '--binary', str(self.binary), '--binary-sha256', sha or self.bsha, '--out', str(out), '--workers', str(workers)]
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
      return rule7.main(argv)

  def report(self):
    return json.loads((self.t.out / 'compare-rule7.json').read_text())

  def test_all_identical_runs_n_d_then_t_and_succeeds(self):
    self.assertEqual(self.run_rule(), 0)
    self.assertEqual(self.gate.calls, ['N', 'D', 'T'])
    r = self.report()
    self.assertTrue(r['all_identical'])
    self.assertEqual((r['stage'], r['summary']), ('complete', '42/42'))
    self.assertEqual(r['binary_sha256'], self.bsha)
    self.assertEqual(len(r['traces']), 42)

  def test_a_difference_among_n_d_stops_before_t(self):
    k = 4
    self.gate.change[('D', rule7.BUNDLE_KEYS[k])] = rows(999)
    self.assertEqual(self.run_rule(), 1)
    self.assertEqual(self.gate.calls, ['N', 'D'])   # the 14 T runs were never started
    r = self.report()
    self.assertFalse(r['all_identical'])
    self.assertEqual(r['stage'], 'stopped after N-D')
    self.assertEqual(r['summary'], '27/28')
    self.assertFalse((self.t.out / 'T').exists())

  def test_a_difference_in_t_fails_the_whole_rule(self):
    self.gate.change[('T', rule7.BUNDLE_KEYS[13])] = rows(999)
    self.assertEqual(self.run_rule(), 1)
    self.assertEqual(self.gate.calls, ['N', 'D', 'T'])
    r = self.report()
    self.assertEqual((r['all_identical'], r['summary']), (False, '41/42'))

  def test_an_engine_run_that_did_not_finish_fails(self):
    self.gate.break_run = 'N'
    self.assertEqual(self.run_rule(), 1)
    self.assertFalse(self.report()['all_identical'])
    self.assertEqual(self.gate.calls, ['N', 'D'])

  def test_report_is_written_once(self):
    self.assertEqual(self.run_rule(), 0)
    self.assertEqual(self.run_rule(), 2)   # out now holds results: refused before anything runs

  def test_preconditions_run_nothing(self):
    self.assertEqual(self.run_rule(sha='0' * 64), 2)                          # not the designated binary
    self.assertEqual(self.run_rule(workers=3), 2)
    self.assertEqual(self.run_rule(workers=0), 2)
    self.assertEqual(self.gate.calls, [])
    self.pin.write_bytes(b'someone replaced the pinned reference binary')
    self.assertEqual(self.run_rule(), 2)
    self.assertEqual(self.gate.calls, [])
    self.assertFalse(self.t.out.exists())

  def test_an_existing_run_log_does_not_block_the_run(self):
    self.t.out.mkdir()
    (self.t.out / 'run.log').write_text('started\n')
    self.assertEqual(self.run_rule(), 0)

  def test_compare_command_rereads_without_running(self):
    self.assertEqual(self.run_rule(), 0)
    with contextlib.redirect_stdout(io.StringIO()):
      self.assertEqual(rule7.main(['compare', '--out', str(self.t.out)]), 0)
      write_trace(self.t.out / 'N' / rule7.BUNDLE_KEYS[0] / 's0', rows(777))
      self.assertEqual(rule7.main(['compare', '--out', str(self.t.out)]), 1)


if __name__ == '__main__':
  unittest.main()
