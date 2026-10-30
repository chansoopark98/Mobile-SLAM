#!/usr/bin/env python3
"""Spec lane S1: Gr8Gl CPU-half parity on captured original pyramids.

For each capture: original binary func917+func918 (binary_gr8_probe.mjs) vs public keypointsForCropJs +
Gr8Gl::retainBest compiled natively (g++/libstdc++) and with em++ (libc++). Compares kept sets and order.

Usage: python3 -I cpu_half_parity.py --capture-dir DIR --src-w W --src-h H --out OUT.json
"""
import argparse
import gzip
import json
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
BIN = os.path.join(ROOT, 'build/slam-refact-20261015/spec/frontend/bin')


def f32(v):
  return struct.unpack('<f', struct.pack('<f', v))[0]


def norm(lst):
  return [tuple(f32(x) if isinstance(x, float) else x for x in r) for r in lst]


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--capture-dir', required=True)
  ap.add_argument('--src-w', type=int, required=True)
  ap.add_argument('--src-h', type=int, required=True)
  ap.add_argument('--out', required=True)
  args = ap.parse_args()
  meta = json.load(open(os.path.join(args.capture_dir, 'meta.json')))
  layout = json.loads(subprocess.run([os.path.join(BIN, 'frontend_probe'), 'layout', str(args.src_w), str(args.src_h)],
                                     check=True, capture_output=True, text=True).stdout)
  levels = json.dumps([{k: l[k] for k in ('r', 'c', 'w', 'h')} for l in layout['levels']])
  rows = meta['rows']
  results = []
  with tempfile.TemporaryDirectory() as tmp:
    for cap in meta['captures']:
      if cap.get('inputFrame') is None:
        continue
      gz = os.path.join(args.capture_dir, 'cap-%03d.rgba.gz' % cap['capture'])
      raw = os.path.join(tmp, 'cap.rgba')
      open(raw, 'wb').write(gzip.open(gz).read())
      bout = os.path.join(tmp, 'bin.json')
      subprocess.run(['node', os.path.join(HERE, 'binary_gr8_probe.mjs'), '--capture', gz, '--rows', str(rows),
                      '--levels', levels, '--out', bout], check=True, capture_output=True, text=True)
      b = json.load(open(bout))
      nat = json.loads(subprocess.run([os.path.join(BIN, 'frontend_probe'), 'crop', raw, str(rows), str(args.src_w),
                                       str(args.src_h)], check=True, capture_output=True, text=True).stdout)
      wasm = json.loads(subprocess.run(['node', os.path.join(BIN, 'frontend_probe_wasm.js'), 'crop', raw, str(rows),
                                        str(args.src_w), str(args.src_h)], check=True, capture_output=True, text=True).stdout)
      per = []
      for lb, ln, lw in zip(b['levels'], nat['levels'], wasm['levels']):
        A, N, W = norm(lb['list']), norm(ln['list']), norm(lw['list'])
        per.append({'level': lb['level'], 'candidates_binary': lb['candidates'], 'candidates_public': ln['candidates'],
                    'kept': len(A), 'native_same_set': sorted(A) == sorted(N), 'native_same_order': A == N,
                    'wasm_same_set': sorted(A) == sorted(W), 'wasm_same_order': A == W, 'retain_best_trimmed': lb['candidates'] > len(A)})
      results.append({'capture': cap['capture'], 'inputFrame': cap['inputFrame'], 'levels': per})
  summary = {k: sum(1 for r in results for l in r['levels'] if l[k]) for k in
             ('native_same_set', 'native_same_order', 'wasm_same_set', 'wasm_same_order', 'retain_best_trimmed')}
  summary['level_cases'] = sum(len(r['levels']) for r in results)
  summary['trimmed_native_same_order'] = sum(1 for r in results for l in r['levels'] if l['retain_best_trimmed'] and l['native_same_order'])
  summary['trimmed_wasm_same_order'] = sum(1 for r in results for l in r['levels'] if l['retain_best_trimmed'] and l['wasm_same_order'])
  doc = {'schema': 'slam-refact-spec-frontend-cpu-half-parity-v1', 'capture_dir': os.path.relpath(args.capture_dir, ROOT),
         'summary': summary, 'results': results}
  os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
  json.dump(doc, open(args.out, 'w'), indent=1)
  print(json.dumps({'out': os.path.relpath(args.out, ROOT), 'summary': summary}))
  return 0


if __name__ == '__main__':
  sys.exit(main())
