#!/usr/bin/env python3
"""B6 / RM-2: compare the outputs of the public CLI (runs/orig) with our CLI (runs/pil, runs/libvips)
that run_cli_matrix.sh produced. Read-only on both. Run with `python3 -I`.

Per case: file-set equality, JSON equality (diff_target_json.compare + text mode), and for every
raster file the decoded size and the per-pixel difference (mean / p95 / max / fraction exactly equal)
between the public CLI and ours. Results go to <out>/compare_runs.json and compare_runs.txt.
Numbers are recomputed from the files here; nothing is hand-copied.
"""
import importlib.util
import json
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    REPO, 'build', 'slam-refact-full-c1-net', 'rm2-results')
IMPLS = tuple(i for i in ('pil', 'libvips', 'libvips_lut')
              if os.path.isdir(os.path.join(OUT, 'runs', i)))
KINDS = ('original', 'cropped', 'thumbnail', 'luminance', 'geometry')


def load_diff_module():
  spec = importlib.util.spec_from_file_location(
      'diff_target_json', os.path.join(HERE, '..', 'diff_target_json.py'))
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def pix_stats(a, b):
  d = np.abs(a.astype(np.int16) - b.astype(np.int16)).astype(np.float64)
  return {'mean': float(d.mean()), 'p95': float(np.percentile(d, 95)), 'max': float(d.max()),
          'exact_frac': float((d == 0).mean())}


def load(path):
  with Image.open(path) as im:
    im.load()
    return im.copy()


def main():
  dj = load_diff_module()
  runs = os.path.join(OUT, 'runs')
  cases = sorted(f[:-4] for f in os.listdir(os.path.join(runs, 'orig', 'test')) if f.endswith('.txt'))
  result = {}
  lines = []
  for script in cases:
    # the output name is the last answer of the script (the public scripts' names differ from the file)
    with open(os.path.join(runs, 'orig', 'test', script + '.txt'), encoding='utf-8') as f:
      case = [x for x in f.read().splitlines() if x.strip()][-1].strip()
    entry = {'script': script}
    # file names for exactly this case (prefix match is ambiguous: manual-crop vs manual-crop-rotated)
    def case_files(impl):
      names = [f for f in os.listdir(os.path.join(runs, impl, 'tmp'))
               if f == case + '.json' or any(f == '%s_%s.%s' % (case, k, e)
                                              for k in KINDS for e in ('jpg', 'png', 'pgm'))]
      return sorted(names)
    of = case_files('orig')
    for impl in IMPLS:
      f = case_files(impl)
      e = {'files_equal_excluding_pgm': [x for x in f if not x.endswith('.pgm')] == of}
      with open(os.path.join(runs, 'orig', 'tmp', case + '.json'), encoding='utf-8') as fa, \
           open(os.path.join(runs, impl, 'tmp', case + '.json'), encoding='utf-8') as fb:
        ta, tb = fa.read(), fb.read()
      e['json_parsed_diffs'] = dj.compare(json.loads(ta), json.loads(tb))
      e['json_text_equal_except_time'] = (dj._TIME_LINE.sub(r'\1T\2', ta) == dj._TIME_LINE.sub(r'\1T\2', tb))
      e['rasters'] = {}
      for k in KINDS:
        for ext in ('jpg', 'png'):
          fn = '%s_%s.%s' % (case, k, ext)
          pa = os.path.join(runs, 'orig', 'tmp', fn)
          if not os.path.exists(pa):
            continue
          pb = os.path.join(runs, impl, 'tmp', fn)
          ia, ib = load(pa), load(pb)
          r = {'orig_size': ia.size, 'ours_size': ib.size, 'orig_mode': ia.mode, 'ours_mode': ib.mode}
          if ia.size == ib.size:
            ca, cb = ia.convert('RGBA'), ib.convert('RGBA')
            r['rgba'] = pix_stats(np.asarray(ca), np.asarray(cb))
            if k == 'luminance':
              r['gray'] = pix_stats(np.asarray(ia.convert('L')), np.asarray(ib.convert('L')))
          e['rasters'][k] = r
      entry[impl] = e
    result[script] = entry
    lines.append('== %s (target name %s)' % (script, case))
    for impl in IMPLS:
      e = entry[impl]
      lines.append('  [%s] files_equal=%s json_parsed_diffs=%d json_text_equal=%s' % (
          impl, e['files_equal_excluding_pgm'], len(e['json_parsed_diffs']), e['json_text_equal_except_time']))
      for d in e['json_parsed_diffs'][:6]:
        lines.append('      json: ' + d)
      for k, r in e['rasters'].items():
        s = r.get('gray') or r.get('rgba')
        st = ('mean %.2f p95 %.0f max %.0f exact %.3f' % (s['mean'], s['p95'], s['max'], s['exact_frac'])
              if s else 'SIZE MISMATCH')
        lines.append('      %-9s orig %s ours %s  %s' % (k, r['orig_size'], r['ours_size'], st))
  with open(os.path.join(OUT, 'compare_runs.json'), 'w') as f:
    json.dump(result, f, indent=1)
  with open(os.path.join(OUT, 'compare_runs.txt'), 'w') as f:
    f.write('\n'.join(lines) + '\n')
  print('\n'.join(lines))


if __name__ == '__main__':
  main()
