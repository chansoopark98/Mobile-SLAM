#!/usr/bin/env python3
"""Default-path parity of two exact-input replay runs, with the comparison written to a file (US-023 criterion 3; review N-3 / C-U23-1).

The portrait default path must not change: the 14 exact-input bundles (regress/run_exact.sh) are replayed with two builds of
slam_refact_replay (A = before, B = after) and every trace is compared with the timing fields removed. The comparison used to be shown
only in the console; this tool writes it to a file that the results document can cite.

  python3 -I compare_regress.py --a DIR_A --b DIR_B --out FILE [--expect-bundles N]

DIR_x holds one directory per bundle: <bundle>/trace.jsonl and <bundle>/worldpoints-sampled.jsonl (slam_refact_replay --out), and
optionally binary.sha256 (the replay binary). Per bundle and file the stripped rows are compared (keys srStageMs, wallSeconds and every
key starting with srTiming are removed, as in tools/image_target/or4_world_regression.py strip_timing; each row is dumped with sorted keys,
the rows are joined by "\\n" and hashed; the 16 hex digits printed are the head of that sha256).
FILE is created exclusively (an existing file is never overwritten). Exit code 0 only when every file of every bundle is identical, both
directories hold the same bundles, and at least --expect-bundles bundles (default 14) were compared; otherwise 1.
Stdlib only.
"""
import argparse
import hashlib
import json
import os
import pathlib
import sys

TIMING_KEYS = ('srStageMs', 'wallSeconds')
FILES = ('trace.jsonl', 'worldpoints-sampled.jsonl')


def strip_timing(o):
  if isinstance(o, dict):
    return {k: strip_timing(v) for k, v in o.items() if k not in TIMING_KEYS and not k.startswith('srTiming')}
  if isinstance(o, list):
    return [strip_timing(v) for v in o]
  return o


def stripped_lines(path):
  with open(path) as f:
    return [json.dumps(strip_timing(json.loads(l)), sort_keys=True) for l in f if l.strip()]


def bundles(root):
  return sorted(p.name for p in pathlib.Path(root).iterdir() if p.is_dir())


def compare_dirs(dir_a, dir_b, expect_bundles=14):
  """Returns (lines, ok, total_rows). lines: the report text, one entry per compared file plus the verdict."""
  a_names, b_names = bundles(dir_a), bundles(dir_b)
  lines, ok, total = [], True, 0
  for name in sorted(set(a_names) | set(b_names)):
    if name not in a_names or name not in b_names:
      lines.append('%-40s %-26s MISSING in %s' % (name, '', 'B' if name in a_names else 'A'))
      ok = False
      continue
    for fname in FILES:
      pa, pb = pathlib.Path(dir_a, name, fname), pathlib.Path(dir_b, name, fname)
      if not pa.is_file() or not pb.is_file():
        lines.append('%-40s %-26s MISSING (A %s, B %s)' % (name, fname, 'present' if pa.is_file() else 'absent', 'present' if pb.is_file() else 'absent'))
        ok = False
        continue
      la, lb = stripped_lines(pa), stripped_lines(pb)
      sha_a = hashlib.sha256('\n'.join(la).encode()).hexdigest()
      sha_b = hashlib.sha256('\n'.join(lb).encode()).hexdigest()
      if la == lb:
        total += len(la)
        lines.append('%-40s %-26s rows=%5d IDENTICAL sha256(stripped)=%s' % (name, fname, len(la), sha_a[:16]))
      else:
        first = next((i for i, (x, y) in enumerate(zip(la, lb)) if x != y), min(len(la), len(lb)))
        lines.append('%-40s %-26s rows=%5d/%5d DIFFERENT first differing row %d sha256(stripped) A=%s B=%s' %
                     (name, fname, len(la), len(lb), first, sha_a[:16], sha_b[:16]))
        ok = False
  compared = len(set(a_names) & set(b_names))
  if compared < expect_bundles:
    lines.append('FEWER BUNDLES than expected: %d compared, %d expected' % (compared, expect_bundles))
    ok = False
  lines.append(('ALL IDENTICAL rows %d' % total) if ok else 'DIFFERENCES (see the lines above)')
  return lines, ok, total


def binary_line(label, root):
  p = pathlib.Path(root, 'binary.sha256')
  return '# %s binary: %s' % (label, p.read_text().strip() if p.is_file() else 'no binary.sha256 in the directory')


def main(argv):
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--a', required=True, help='directory of the earlier build (baseline)')
  ap.add_argument('--b', required=True, help='directory of the build under test')
  ap.add_argument('--out', required=True, help='report file (created exclusively)')
  ap.add_argument('--expect-bundles', type=int, default=14)
  a = ap.parse_args(argv)
  lines, ok, _ = compare_dirs(a.a, a.b, a.expect_bundles)
  header = ['# A = %s' % a.a, binary_line('A', a.a), '# B = %s' % a.b, binary_line('B', a.b),
            '# stripped keys: %s and srTiming*; rows dumped with sorted keys, joined by newline' % ', '.join(TIMING_KEYS)]
  fd = os.open(a.out, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
  with os.fdopen(fd, 'w') as f:
    f.write('\n'.join(header + lines) + '\n')
  print('\n'.join(lines[-1:]))
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
