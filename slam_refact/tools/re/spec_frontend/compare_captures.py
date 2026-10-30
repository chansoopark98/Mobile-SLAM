#!/usr/bin/env python3
"""Spec lane S1: compare CPU-port pyramids (gr8_gpu_ref) with captured original GPU pyramids.

For every capture in <capture_dir>/meta.json that maps to an input frame, run
  frontend_probe compare <frame.pgm> <capture.rgba> <rows> <fx_level0> <qw qx qy qz> [bits]
and evaluate the verdict against tolerances.json (fixed before the first comparison).

Usage: python3 -I compare_captures.py --capture-dir DIR --fx-level0 F --out OUT.json
                                      [--probe BIN] [--linear-weight-bits N]
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


def verdict_atlas(levels, tol):
  near = tol['pyramid_atlas']['NEAR']
  n = sum(l['pixels'] for l in levels)
  eq = [sum(l['eq'][c] for l in levels) for c in range(4)]
  maxd = [max(l['maxdiff'][c] for l in levels) for c in range(4)]
  ca = sum(l['cand_a'] for l in levels)
  cb = sum(l['cand_b'] for l in levels)
  cboth = sum(l['cand_both'] for l in levels)
  cand_j = cboth / (ca + cb - cboth) if (ca + cb - cboth) else 1.0
  if all(e == n for e in eq):
    v = 'BIT_EXACT'
  elif (eq[0] / n >= near['R_identical_min_fraction'] and maxd[0] <= near['R_max_abs_diff'] and
        eq[1] / n >= near['G_identical_min_fraction'] and maxd[1] <= near['G_max_abs_diff'] and
        cand_j >= near['B_candidates_jaccard_min'] and
        eq[3] / n >= near['A_identical_min_fraction_over_all_pixels']):
    v = 'NEAR'
  else:
    v = 'FAIL'
  return {'verdict': v, 'pixels': n, 'identical_fraction': [e / n for e in eq], 'maxdiff': maxd,
          'candidate_jaccard': cand_j, 'candidates_cpu': ca, 'candidates_captured': cb}


def verdict_keys(k, tol):
  near = tol['keypoints']['NEAR']
  inter = k['intersection']
  pairs = k['orb_pairs'] + k['gorb_pairs']
  ident = k['orb_identical'] + k['gorb_identical']
  ham = (k['orb_mean_hamming'] * k['orb_pairs'] + k['gorb_mean_hamming'] * k['gorb_pairs']) / pairs if pairs else 0.0
  if k['jaccard'] == 1.0 and k['same_response'] == inter and ident == pairs:
    v = 'BIT_EXACT'
  elif (k['jaccard'] >= near['jaccard_min'] and
        (inter == 0 or k['same_response'] / inter >= near['same_response_min_fraction_of_intersection']) and
        (pairs == 0 or ident / pairs >= near['descriptor_identical_min_fraction']) and
        ham <= near['descriptor_mean_hamming_max_bits']):
    v = 'NEAR'
  else:
    v = 'FAIL'
  return {'verdict': v, 'jaccard': k['jaccard'], 'same_response_fraction': k['same_response'] / inter if inter else 1.0,
          'descriptor_identical_fraction': ident / pairs if pairs else 1.0, 'descriptor_mean_hamming': ham}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--capture-dir', required=True)
  ap.add_argument('--fx-level0', type=float, required=True)
  ap.add_argument('--out', required=True)
  ap.add_argument('--probe', default=os.path.join(ROOT, 'build/slam-refact-20261015/spec/frontend/bin/frontend_probe'))
  ap.add_argument('--linear-weight-bits', type=int, default=0)
  ap.add_argument('--swiftshader-approx', type=int, default=0)
  ap.add_argument('--binary-forms', type=int, default=1)
  args = ap.parse_args()
  tol = json.load(open(os.path.join(HERE, 'tolerances.json')))
  meta = json.load(open(os.path.join(args.capture_dir, 'meta.json')))
  dataset = meta['input']['dataset']
  if meta['scenario'].startswith('tum'):
    frames = json.load(open(os.path.join(dataset, 'input.json')))['frames']
  else:
    frames = json.load(open(os.path.join(dataset, 'record.json')))['recording']['frames']
  rows = meta['rows']
  results = []
  with tempfile.TemporaryDirectory() as tmp:
    for cap in meta['captures']:
      if cap.get('inputFrame') is None:
        continue
      raw = os.path.join(tmp, 'cap.rgba')
      with gzip.open(os.path.join(args.capture_dir, 'cap-%03d.rgba.gz' % cap['capture']), 'rb') as f:
        open(raw, 'wb').write(f.read())
      pgm = os.path.join(dataset, 'mav0/cam0/data', frames[cap['inputFrame']]['file'])
      q = cap['stageArgs']['qwxyz']
      cmd = [args.probe, 'compare', pgm, raw, str(rows), repr(args.fx_level0)] + [repr(x) for x in q] + \
            [str(args.linear_weight_bits), str(args.swiftshader_approx), str(args.binary_forms)]
      out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
      r = json.loads(out)
      results.append({'capture': cap['capture'], 'inputFrame': cap['inputFrame'], 'pgm': os.path.relpath(pgm, ROOT),
                      'atlas_verdict': verdict_atlas(r['atlas'], tol), 'keypoint_verdict': verdict_keys(r['keypoints'], tol),
                      'raw': r})
  summary = {}
  for key in ('atlas_verdict', 'keypoint_verdict'):
    summary[key] = {}
    for r in results:
      summary[key][r[key]['verdict']] = summary[key].get(r[key]['verdict'], 0) + 1
  doc = {'schema': 'slam-refact-spec-frontend-compare-v1', 'capture_dir': os.path.relpath(args.capture_dir, ROOT),
         'fx_level0': args.fx_level0, 'linear_weight_bits': args.linear_weight_bits,
         'swiftshader_approx': args.swiftshader_approx, 'binary_expression_forms': args.binary_forms,
         'tolerances': 'slam_refact/tools/re/spec_frontend/tolerances.json', 'summary': summary, 'results': results}
  os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
  json.dump(doc, open(args.out, 'w'), indent=1)
  print(json.dumps({'out': os.path.relpath(args.out, ROOT), 'summary': summary}))
  return 0


if __name__ == '__main__':
  sys.exit(main())
