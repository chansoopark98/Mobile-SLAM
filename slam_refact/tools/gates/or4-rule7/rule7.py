#!/usr/bin/env python3
"""OR-4 v6 rule 7 (succession condition) as one registered command (US-014; review OR-4 / closure audit B).

Gate: preregistration/or4-it-world-regression.v6.md section 0.2 rule 7:
  the 42 traces of the 14 bundles -- N s0, D force and T s0, timing fields excluded -- that the engine binary under test writes are bit
  identical to the traces the pinned binary b1f4f92a wrote (sha256 of the stripped traces only). One difference anywhere = the result of
  the gate is NOT succeeded to this binary; the gate is registered again with the new binary (R_D measured anew).
  Order: N s0 and D force first, T s0 last. A difference among the 28 N/D traces stops the run: the 14 T runs are not started.

  python3 -I rule7.py run --binary BIN --binary-sha256 SHA --out NEWDIR [--workers 1|2]
  python3 -I rule7.py compare --out DIR            (re-read DIR: same verdict, no engine run)
  python3 -I rule7.py plan --binary BIN            (print the 42 command lines; nothing is executed)

The pinned references are read only: N s0 or4/v3/null/N/<bundle>/s0, D force or4/v3/decomp/D/<bundle>/force (or the v5 directory),
T s0 or4/v6/real/T/<bundle>/s0 under build/slam-refact-full-20261019/. Exit code 0 only for 42 of 42 identical; 1 for any difference, a
failed run or a missing trace; 2 when a precondition is not met (nothing was run). This tool does not run analyze-null / real-*, writes no
first-run marker and no ledger; the engine runs are the ones of or4_world_regression.run_engine (the gate's own runner).
Nothing here is part of the product runtime. Stdlib only (the run/plan commands import tools/image_target/or4_world_regression.py).
"""
import argparse
import hashlib
import json
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
OR4 = REPO / 'build/slam-refact-full-20261019/or4'
PIN_BINARY_SHA256 = 'b1f4f92a010ab413a26057fcfbe7a31be2ebd08270355ed985d83242604e972a'   # gate v6 section 2: or4/v3/bin/sr_or4_replay
TIMING_KEYS = ('srStageMs', 'wallSeconds')   # as tools/image_target/or4_world_regression.py

# The 14 exact-input bundles of gate v6 section 2, in the order of or4_world_regression.BUNDLES.
BUNDLE_KEYS = (
    'flip6-n1__pmr3b', 'flip6-n1__pmr4a', 'flip6-s2__pmr3a', 'flip6-s2__pmr4a', 'flip6-s3-lowtex__pmr3a', 'flip6-s3-lowtex__pmr4a',
    'tum-room1-full__pmr3a', 'tum-room1-full__pmr4a', 'tum-room1-lowtex__pmr3b', 'tum-room1-lowtex__pmr4a',
    'flip6-s2-absolute__pma1', 'flip6-s2-absolute__pma2', 'tum-room1-full-absolute__pma1', 'tum-room1-full-absolute__pma2')
# (arm label, directory name of the new run under --out, run name). N and D first, T last (rule 7).
ARMS = (('N s0', 'N', 's0'), ('D force', 'D', 'force'), ('T s0', 'T', 's0'))
EXPECTED_TRACES = len(BUNDLE_KEYS) * len(ARMS)   # 42


# ------------------------------------------------------------------------------------------------- references and comparison
def reference_dir(arm_dir, key, name, refs=None):
  """Where the pinned binary's trace of one arm lives (read only)."""
  refs = refs or {}
  if arm_dir == 'N':
    return pathlib.Path(refs.get('N', OR4 / 'v3/null/N')) / key / name
  if arm_dir == 'T':
    return pathlib.Path(refs.get('T', OR4 / 'v6/real/T')) / key / name
  for root in refs.get('D', (OR4 / 'v3/decomp/D', OR4 / 'v5/decomp/D')):   # the v3 run 'force' is in the v3 directory
    if (pathlib.Path(root) / key / name).is_dir():
      return pathlib.Path(root) / key / name
  return pathlib.Path(refs.get('D', (OR4 / 'v3/decomp/D',))[0]) / key / name


def strip_timing(o):
  if isinstance(o, dict):
    return {k: strip_timing(v) for k, v in o.items() if k not in TIMING_KEYS and not k.startswith('srTiming')}
  if isinstance(o, list):
    return [strip_timing(v) for v in o]
  return o


def stripped_trace_sha(path):
  """(sha256 of the timing-stripped trace, number of rows); None when the file is missing or unreadable."""
  try:
    with open(path) as f:
      lines = [json.dumps(strip_timing(json.loads(l)), sort_keys=True) for l in f if l.strip()]
  except (OSError, ValueError):
    return None
  return hashlib.sha256('\n'.join(lines).encode()).hexdigest(), len(lines)


def file_sha256(path):
  try:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
      for chunk in iter(lambda: f.read(1 << 20), b''):
        h.update(chunk)
    return h.hexdigest()
  except OSError:
    return None


def compare_arms(out, arms=ARMS, refs=None):
  """Compare the traces under `out` with the pinned references, by sha256 only. Returns the per-trace table."""
  table = {}
  for label, arm_dir, name in arms:
    for key in BUNDLE_KEYS:
      new = pathlib.Path(out) / arm_dir / key / name / 'trace.jsonl'
      ref = reference_dir(arm_dir, key, name, refs) / 'trace.jsonl'
      n, r = stripped_trace_sha(new), stripped_trace_sha(ref)
      entry = {'new_sha256_stripped': n and n[0], 'ref_sha256_stripped': r and r[0], 'new_rows': n and n[1], 'ref_rows': r and r[1],
               'new_trace_file_sha256': file_sha256(new), 'ref_trace_file_sha256': file_sha256(ref)}
      if n is None:
        entry['identical'], entry['note'] = False, 'new trace missing or unreadable'
      elif r is None:
        entry['identical'], entry['note'] = False, 'pinned reference trace missing or unreadable'
      else:
        entry['identical'] = n[0] == r[0]
      table[f'{key}/{label}'] = entry
  return table


def verdict(table, expected=EXPECTED_TRACES):
  """(ok, 'k/n'): ok only when exactly `expected` traces were compared and every one is identical."""
  n = sum(1 for e in table.values() if e['identical'])
  return (len(table) == expected and n == expected), f'{n}/{len(table)}'


def write_report(out, table, stage, extra):
  ok, summary = verdict(table, EXPECTED_TRACES if stage == 'complete' else len(table))
  ok = ok and stage == 'complete'
  doc = dict(extra, rule='or4-it-world-regression.v6 section 0.2 rule 7', stage=stage, summary=summary, all_identical=ok,
             pin_binary_sha256=PIN_BINARY_SHA256, traces=table)
  fd = os.open(pathlib.Path(out) / 'compare-rule7.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
  with os.fdopen(fd, 'w') as f:
    f.write(json.dumps(doc, indent=1, sort_keys=True) + '\n')
  return ok, summary


# ------------------------------------------------------------------------------------------------- engine runs (the gate's own runner)
def load_gate_tool():
  sys.path.insert(0, str(HERE.parents[1] / 'image_target'))
  import or4_world_regression as g   # noqa: E402
  if tuple(k for k, _, _, _ in g.BUNDLES) != BUNDLE_KEYS:
    raise SystemExit('the bundle list of or4_world_regression.BUNDLES differs from the 14 bundles of rule 7 (this tool, gate section 2)')
  return g


def jobs(g, binary, out, arm_dir):
  """The run_engine jobs of one arm directory, as in the dev re-check of rule 7 (same seeds and options)."""
  result = []
  for key, path, _, _ in g.BUNDLES:
    s0 = g.seeds_of(g.bundle_seed(path))[0]
    if arm_dir == 'N':
      result.append(((binary, key, path, out / 'N' / key / 's0'), {'seed': s0, 'tolerate_failure': True}))
    elif arm_dir == 'D':
      result.append(((binary, key, path, out / 'D' / key / 'force'), {'seed': s0, 'force_orb': True, 'tolerate_failure': True}))
    else:
      result.append(((binary, key, path, out / 'T' / key / 's0'), {'seed': s0, 'targets': g.TARGET_DIR, 'tolerate_failure': True}))
  return result


def command_line(g, job):
  args, kw = job
  binary, _, path, out = args
  cmd = [str(binary), '--bundle', str(g.ROOT / path), '--out', str(out)]
  if kw.get('seed') is not None:
    cmd += ['--id-seed', str(kw['seed'])]
  if kw.get('force_orb'):
    cmd += ['--force-orb-gorb', '1']
  if kw.get('targets'):
    cmd += ['--targets', str(kw['targets'])]
  return cmd


def cmd_plan(a):
  g = load_gate_tool()
  for _, arm_dir, _ in ARMS:
    for job in jobs(g, pathlib.Path(a.binary), pathlib.Path('OUT'), arm_dir):
      print(' '.join(command_line(g, job)))
  return 0


def run_arm(g, binary, out, arm_dir, workers):
  statuses = g.run_many(jobs(g, binary, out, arm_dir), workers)
  bad = [s for s in statuses if s['class'] != 'OK']
  for s in bad:
    print(f"RUN NOT OK: {s['key']} class {s['class']} rc {s['rc']}", flush=True)
  return statuses, bad


def cmd_run(a):
  out = pathlib.Path(a.out).resolve()
  binary = pathlib.Path(a.binary).resolve()
  if not binary.is_file():
    print(f'precondition: binary {binary} is not a file', file=sys.stderr)
    return 2
  bsha = file_sha256(binary)
  if bsha != a.binary_sha256:
    print(f'precondition: binary sha256 {bsha} is not the designated {a.binary_sha256}', file=sys.stderr)
    return 2
  if out.exists() and set(os.listdir(out)) - {'run.log'}:
    print(f'precondition: {out} already holds results (a new directory is required)', file=sys.stderr)
    return 2
  g = load_gate_tool()
  pin = g.ENGINE_PIN
  if file_sha256(pin) != PIN_BINARY_SHA256:
    print(f'precondition: the pinned reference binary {pin} does not have sha256 {PIN_BINARY_SHA256}', file=sys.stderr)
    return 2
  if not 1 <= a.workers <= 2:
    print('precondition: --workers must be 1 or 2', file=sys.stderr)
    return 2
  out.mkdir(parents=True, exist_ok=True)
  extra = {'binary': str(binary), 'binary_sha256': bsha, 'workers': a.workers}
  table = {}
  for stage, arms in (('N-D', ARMS[:2]), ('T', ARMS[2:])):
    all_bad = []
    for _, arm_dir, _ in arms:
      _, bad = run_arm(g, binary, out, arm_dir, a.workers)
      all_bad += bad
    table.update(compare_arms(out, arms))
    ok_so_far = not all_bad and all(e['identical'] for e in table.values())
    print(f'stage {stage}: {sum(1 for e in table.values() if e["identical"])}/{len(table)} identical so far', flush=True)
    if not ok_so_far:
      _, summary = write_report(out, table, f'stopped after {stage}', extra)
      print(f'RULE 7 NOT SATISFIED: {summary} identical; the result of the gate is not succeeded to this binary (register the gate again)')
      return 1
  ok, summary = write_report(out, table, 'complete', extra)
  print(f'rule7 {summary} ' + ('ALL IDENTICAL' if ok else 'DIFFERENCES'))
  return 0 if ok else 1


def cmd_compare(a):
  out = pathlib.Path(a.out)
  table = compare_arms(out)
  ok, summary = verdict(table)
  print(json.dumps({'rule7': summary}), 'ALL IDENTICAL' if ok else 'DIFFERENCES')
  return 0 if ok else 1


def main(argv):
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  sub = ap.add_subparsers(dest='cmd', required=True)
  p = sub.add_parser('run')
  p.add_argument('--binary', required=True)
  p.add_argument('--binary-sha256', required=True, help='sha256 of the engine binary under test (the frozen one); anything else is refused')
  p.add_argument('--out', required=True)
  p.add_argument('--workers', type=int, default=2)
  p.set_defaults(fn=cmd_run)
  p = sub.add_parser('compare')
  p.add_argument('--out', required=True)
  p.set_defaults(fn=cmd_compare)
  p = sub.add_parser('plan')
  p.add_argument('--binary', required=True)
  p.set_defaults(fn=cmd_plan)
  a = ap.parse_args(argv)
  return a.fn(a)


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
