#!/usr/bin/env python3
"""Spec lane S1: evaluate tolerances-v2.json gates on one capture directory.

Runs (per capture mapped to an input frame):
  - camera texture readback == input PGM (byte equality, R=G=B, A=255)
  - frontend_probe isolate ... 0 1  (stage2+3 from captured gray)        -> BIT_EXACT gate
  - frontend_probe isolate ... 1 1  (levels 1-3 rebuilt from captured L0) -> near-exact gate
  - compare_captures.py (full CPU pyramid, swiftshader_approx=1)          -> regression gate
  - cpu_half_parity.py (binary func917/918 vs public, libc++ and libstdc++) -> BIT_EXACT gate (wasm)

Usage: python3 -I run_gates.py --capture-dir DIR --src-w W --src-h H --fx-level0 F --out OUT.json
"""
import argparse
import gzip
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
PROBE = os.path.join(ROOT, 'build/slam-refact-20261015/spec/frontend/bin/frontend_probe')


def pgm(path):
  b = open(path, 'rb').read()
  parts = b.split(b'\n', 3)
  w, h = map(int, parts[1].split())
  return w, h, parts[3]


def frames_of(meta):
  ds = meta['input']['dataset']
  if meta['scenario'].startswith('tum'):
    fr = json.load(open(os.path.join(ds, 'input.json')))['frames']
  else:
    fr = json.load(open(os.path.join(ds, 'record.json')))['recording']['frames']
  return ds, fr


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--capture-dir', required=True)
  ap.add_argument('--src-w', type=int, required=True)
  ap.add_argument('--src-h', type=int, required=True)
  ap.add_argument('--fx-level0', type=float, required=True)
  ap.add_argument('--out', required=True)
  args = ap.parse_args()
  tol = json.load(open(os.path.join(HERE, 'tolerances-v2.json')))['gates']
  meta = json.load(open(os.path.join(args.capture_dir, 'meta.json')))
  ds, frames = frames_of(meta)
  rows = meta['rows']
  out = {'schema': 'slam-refact-spec-frontend-gates-v2', 'capture_dir': os.path.relpath(args.capture_dir, ROOT),
         'tolerances': 'slam_refact/tools/re/spec_frontend/tolerances-v2.json'}
  # Camera texture equality.
  cam = []
  for c in meta.get('cameraCaptures', []):
    if c['inputFrame'] is None:
      continue
    w, h, gray = pgm(os.path.join(ds, 'mav0/cam0/data', frames[c['inputFrame']]['file']))
    rgba = gzip.open(os.path.join(args.capture_dir, 'cam-%03d.rgba.gz' % c['capture'])).read()
    ok = len(rgba) == w * h * 4 and all(rgba[4 * i] == gray[i] and rgba[4 * i + 1] == gray[i] and
                                        rgba[4 * i + 2] == gray[i] and rgba[4 * i + 3] == 255 for i in range(w * h))
    cam.append({'capture': c['capture'], 'inputFrame': c['inputFrame'], 'equal_to_pgm': ok})
  out['camera_texture'] = {'checked': len(cam), 'equal': sum(c['equal_to_pgm'] for c in cam)}
  # Isolation tests.
  iso = []
  with tempfile.TemporaryDirectory() as tmp:
    raw = os.path.join(tmp, 'cap.rgba')
    for c in meta['captures']:
      if c.get('inputFrame') is None:
        continue
      open(raw, 'wb').write(gzip.open(os.path.join(args.capture_dir, 'cap-%03d.rgba.gz' % c['capture'])).read())
      rec = {'capture': c['capture'], 'inputFrame': c['inputFrame']}
      for rb in (0, 1):
        r = json.loads(subprocess.run([PROBE, 'isolate', raw, str(rows), str(args.src_w), str(args.src_h), str(rb), '1'],
                                      check=True, capture_output=True, text=True).stdout)
        n = sum(l['pixels'] for l in r['atlas'])
        eq = [sum(l['eq'][ch] for l in r['atlas']) / n for ch in range(4)]
        md = [max(l['maxdiff'][ch] for l in r['atlas']) for ch in range(4)]
        rec['stage23' if rb == 0 else 'rebuild'] = {'identical_fraction': eq, 'maxdiff': md}
      rec['stage23_bit_exact'] = all(e == 1.0 for e in rec['stage23']['identical_fraction'])
      e, m = rec['rebuild']['identical_fraction'], rec['rebuild']['maxdiff']
      rec['rebuild_pass'] = e[0] >= 0.9999 and e[1] >= 0.9999 and m[0] <= 1 and m[1] <= 1 and e[2] >= 0.9999 and e[3] >= 0.9999
      iso.append(rec)
  out['isolation'] = {'captures': len(iso), 'stage23_bit_exact': sum(r['stage23_bit_exact'] for r in iso),
                      'rebuild_pass': sum(r['rebuild_pass'] for r in iso), 'per_capture': iso}
  # Full pyramid comparison and CPU half.
  full_json = os.path.join(os.path.dirname(os.path.abspath(args.out)), 'full-' + os.path.basename(args.out))
  subprocess.run([sys.executable, '-I', os.path.join(HERE, 'compare_captures.py'), '--capture-dir', args.capture_dir,
                  '--fx-level0', repr(args.fx_level0), '--swiftshader-approx', '1', '--binary-forms', '1', '--out', full_json],
                 check=True, capture_output=True, text=True)
  full = json.load(open(full_json))
  g = tol['full_cpu_pyramid_vs_capture']
  fr = []
  for r in full['results']:
    av = r['atlas_verdict']
    cpu, capk = r['raw']['cpu_port']['per_level'], r['raw']['captured']['per_level']
    rel = max(abs(a - b) / max(b, 1) for a, b in zip(cpu, capk))
    ok = (av['identical_fraction'][0] >= g['R_identical_min_fraction'] and av['maxdiff'][0] <= g['R_max_abs_diff'] and
          av['identical_fraction'][1] >= g['G_identical_min_fraction'] and av['maxdiff'][1] <= g['G_max_abs_diff'] and
          av['candidate_jaccard'] >= g['B_candidates_jaccard_min'] and rel <= g['per_level_keypoint_count_rel_diff_max'])
    fr.append({'inputFrame': r['inputFrame'], 'pass': ok, 'R': av['identical_fraction'][0], 'G': av['identical_fraction'][1],
               'maxdiff': av['maxdiff'], 'candidate_jaccard': av['candidate_jaccard'], 'per_level_count_rel_diff_max': rel,
               'keypoint_jaccard': r['keypoint_verdict']['jaccard'],
               'descriptor_identical_fraction': r['keypoint_verdict']['descriptor_identical_fraction']})
  out['full_cpu_pyramid'] = {'file': os.path.relpath(full_json, ROOT), 'captures': len(fr), 'pass': sum(r['pass'] for r in fr),
                             'per_capture': fr}
  half_json = os.path.join(os.path.dirname(os.path.abspath(args.out)), 'cpu-half-' + os.path.basename(args.out))
  subprocess.run([sys.executable, '-I', os.path.join(HERE, 'cpu_half_parity.py'), '--capture-dir', args.capture_dir,
                  '--src-w', str(args.src_w), '--src-h', str(args.src_h), '--out', half_json], check=True, capture_output=True, text=True)
  half = json.load(open(half_json))
  out['cpu_half'] = {'file': os.path.relpath(half_json, ROOT), 'summary': half['summary']}
  s = half['summary']
  out['verdicts'] = {
    'camera_texture_equals_input': out['camera_texture']['equal'] == out['camera_texture']['checked'] > 0,
    'cpu_half_wasm_libcxx_BIT_EXACT': s['wasm_same_set'] == s['wasm_same_order'] == s['level_cases'],
    'cpu_half_native_libstdcxx_BIT_EXACT': s['native_same_set'] == s['native_same_order'] == s['level_cases'],
    'isolated_stage23_BIT_EXACT': out['isolation']['stage23_bit_exact'] == out['isolation']['captures'],
    'isolated_rebuild_PASS': out['isolation']['rebuild_pass'] == out['isolation']['captures'],
    'full_cpu_pyramid_PASS': out['full_cpu_pyramid']['pass'] == out['full_cpu_pyramid']['captures'],
  }
  json.dump(out, open(args.out, 'w'), indent=1)
  print(json.dumps({'out': os.path.relpath(args.out, ROOT), 'verdicts': out['verdicts'],
                    'isolation': {k: out['isolation'][k] for k in ('captures', 'stage23_bit_exact', 'rebuild_pass')},
                    'full_pass': [out['full_cpu_pyramid']['pass'], out['full_cpu_pyramid']['captures']], 'cpu_half': s}))
  return 0


if __name__ == '__main__':
  sys.exit(main())
