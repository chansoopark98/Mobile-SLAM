#!/usr/bin/env python3
"""B6 / RM-2: analyse the buffers written by sharp_gray_stages.mjs. Run with `python3 -I`.

Questions answered from data (nothing assumed):
  Q1  Which gray formula does sharp's .grayscale() apply? (D = gray at full resolution, no resampling,
      compared with candidate formulas)
  Q2  In which order does sharp run resize and grayscale? (A = resize().grayscale() vs gray(C) and vs B)
  Q3  How far is our PIL path (default `pil`, and `libvips` mode) from sharp's final luminance A, and
      how much of that is the resampling (PIL Lanczos vs libvips) rather than the gray formula?
Usage: analyze_gray.py <gray-stages-dir> [<out-prefix>]
"""
import io
import json
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
STAGES = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(STAGES, '..', 'gray_analysis')


def rd(dirpath, name, w, h, ch):
  a = np.fromfile(os.path.join(dirpath, name), dtype=np.uint8)
  assert a.size == w * h * ch, (name, a.size, w, h, ch)
  return a.reshape(h, w, ch) if ch > 1 else a.reshape(h, w)


def as_gray(a, what, notes):
  """A gray buffer that sharp returned with 3-4 channels: R=G=B must hold (checked and recorded)."""
  if a.ndim == 2:
    return a
  same = bool((a[..., 0] == a[..., 1]).all() and (a[..., 0] == a[..., 2]).all())
  notes.append('%s: %d channels, R=G=B %s%s' % (what, a.shape[2], same, (
      ', alpha min %d max %d' % (a[..., 3].min(), a[..., 3].max())) if a.shape[2] == 4 else ''))
  return a[..., 0]


def st(a, b):
  d = np.abs(a.astype(np.int16) - b.astype(np.int16)).astype(np.float64)
  return {'mean': round(float(d.mean()), 4), 'p95': float(np.percentile(d, 95)), 'max': float(d.max()),
          'exact': round(float((d == 0).mean()), 5)}


def srgb_to_lin(c):
  return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def lin_to_srgb(y):
  return np.where(y <= 0.0031308, 12.92 * y, 1.055 * np.power(np.maximum(y, 0), 1 / 2.4) - 0.055)


def gray_709_linear(rgb, weights=(0.2126, 0.7152, 0.0722)):
  c = rgb[..., :3].astype(np.float64) / 255.0
  y = srgb_to_lin(c) @ np.array(weights)
  return np.rint(np.clip(lin_to_srgb(y), 0, 1) * 255).astype(np.uint8)


def gray_601_gamma(rgb):  # PIL convert('L')
  return np.asarray(Image.fromarray(np.ascontiguousarray(rgb[..., :3]), 'RGB').convert('L'))


def gray_709_gamma(rgb):
  c = rgb[..., :3].astype(np.float64)
  return np.rint(c @ np.array([0.2126, 0.7152, 0.0722])).astype(np.uint8)


def gray_601_linear(rgb):
  return gray_709_linear(rgb, (0.299, 0.587, 0.114))


_LUT = np.rint(lin_to_srgb(np.arange(256) / 255.0) * 255.0)  # 256-entry table, 8-bit sRGB per linear level


def gray_vips_lut(rgb):
  """Fitted to sharp 0.34.5 / libvips 8.17.3 (rm2/README.md): linearise each 8-bit channel, Y =
  0.2126 R + 0.7152 G + 0.0722 B (linear), look Y*255 up in a 256-entry sRGB table with LINEAR
  INTERPOLATION between entries, round half up."""
  lin = srgb_to_lin(rgb[..., :3].astype(np.float64) / 255.0)
  x = np.clip(lin @ np.array([0.2126, 0.7152, 0.0722]), 0.0, 1.0) * 255.0
  i = np.minimum(np.floor(x).astype(np.int64), 254)
  f = x - i
  v = _LUT[i] * (1.0 - f) + _LUT[i + 1] * f
  return np.floor(v + 0.5).astype(np.uint8)


FORMULAS = {'vips_lut_interp': gray_vips_lut, 'rec709_linear_light': gray_709_linear, 'pil_601_on_gamma': gray_601_gamma,
            'rec709_on_gamma': gray_709_gamma, 'rec601_linear_light': gray_601_linear}


def pil_resize_h(arr, height=640):
  im = Image.fromarray(np.ascontiguousarray(arr))
  w, h = im.size
  return np.asarray(im.resize((max(1, int(np.floor(w * height / h + 0.5))), height), Image.Resampling.LANCZOS))


def main():
  res = {}
  lines = []
  for id_ in sorted(os.listdir(STAGES)):
    d = os.path.join(STAGES, id_)
    if not os.path.exists(os.path.join(d, 'meta.json')):
      continue
    m = json.load(open(os.path.join(d, 'meta.json')))
    ci, Ai, Ci, Di, Bi = m['cropped'], m['A'], m['C'], m['D'], m['B']
    cropped = rd(d, 'cropped_rgb.raw', ci['width'], ci['height'], ci['channels'])
    A = rd(d, 'A_final_gray.raw', Ai['width'], Ai['height'], Ai['channels'])
    C = rd(d, 'C_resized_rgb.raw', Ci['width'], Ci['height'], Ci['channels'])
    D = rd(d, 'D_gray_fullres.raw', Di['width'], Di['height'], Di['channels'])
    B = rd(d, 'B_gray_then_resize.raw', Bi['width'], Bi['height'], Bi['channels'])
    notes = []
    B = as_gray(B, 'B (gray, then resize)', notes)
    r = {'image': os.path.relpath(m['image'], os.path.join(m['image'], '..', '..')) if False else m['image'].split('lfs-files/')[-1],
         'format': m['format'], 'exif_orientation': m['exifOrientation'], 'src_channels': ci['channels'],
         'cropped_size': [ci['width'], ci['height']], 'lum_size': [Ai['width'], Ai['height']],
         'libvips': m['sharp'].get('vips'),
         # validateCrop() of the public CLI rejects crops under 480x640, so smaller ones never occur
         'valid_for_cli': ci['width'] >= 480 and ci['height'] >= 640}
    # Q1: formula at full resolution (no resampling involved)
    r['Q1_full_res_gray_vs_formula'] = {k: st(D, f(cropped)) for k, f in FORMULAS.items()}
    # Q2: order of resize and grayscale in sharp's pipeline
    r['Q2_A_vs_gray_of_resized'] = {k: st(A, f(C)) for k, f in FORMULAS.items()}
    r['Q2_A_vs_B_gray_then_resize'] = st(A, B)
    # Q3: our PIL paths against sharp's final luminance
    pil_rgb = pil_resize_h(np.ascontiguousarray(cropped[..., :3]))
    pil_path = gray_601_gamma(pil_rgb)                                   # --gray-mode pil
    vips_path = pil_resize_h(gray_709_linear(cropped))                    # --gray-mode libvips (gray, then resize)
    vips_path_after = gray_709_linear(pil_rgb)                            # formula after PIL resize
    lut_then_pil = pil_resize_h(gray_vips_lut(cropped))                   # fitted LUT gray, then PIL resize
    r['Q3_A_vs_pil_mode'] = st(A, pil_path)
    r['Q3_A_vs_libvips_mode'] = st(A, vips_path)
    r['Q3_A_vs_libvipsformula_after_pil_resize'] = st(A, vips_path_after)
    r['Q3_A_vs_fitted_lut_gray_then_pil_resize'] = st(A, lut_then_pil)
    # resampling only: PIL Lanczos vs sharp resize on the RGB(A) image
    r['Q3_resample_only_rgb_C_vs_pil'] = st(C[..., :3], pil_rgb)
    # lossy file the CLI writes (JPEG q80) vs raw A
    ext = m['ext']
    f = os.path.join(d, 'A_file.%s.raw' % ext)
    Fi = m['file']
    if os.path.exists(f):
      F = rd(d, 'A_file.%s.raw' % ext, Fi['width'], Fi['height'], Fi['channels'])
      r['A_raw_vs_A_file_decoded'] = st(A, as_gray(F, 'file luminance.%s decoded by sharp' % ext, notes))
      if ext == 'jpg':
        # is the JPEG encoder a source of difference? re-encode sharp's own gray A with PIL q80 (L and RGB
        # source) and compare the decoded Y with what sharp's file decodes to
        b = io.BytesIO()
        Image.fromarray(A, 'L').save(b, format='JPEG', quality=80)
        b.seek(0)
        r['JPEG_PIL_L_q80_vs_sharp_file'] = st(np.asarray(Image.open(b)), F[..., 0] if F.ndim == 3 else F)
        b2 = io.BytesIO()
        Image.fromarray(np.stack([A] * 3, -1), 'RGB').save(b2, format='JPEG', quality=80)
        b2.seek(0)
        r['JPEG_PIL_RGB_q80_vs_sharp_file'] = st(np.asarray(Image.open(b2).convert('L')), F[..., 0] if F.ndim == 3 else F)
    r['notes'] = notes
    res[id_] = r
    lines.append('== %s  (%s, src ch %d, crop %dx%d -> %dx%d, EXIF %s, %s)' % (
        r['image'], r['format'], r['src_channels'], ci['width'], ci['height'], Ai['width'], Ai['height'],
        r['exif_orientation'], 'CLI-valid crop' if r['valid_for_cli'] else 'crop < 480x640: OUTSIDE CLI contract'))
    lines.extend('   note: ' + n for n in notes)
    for k in ('Q1_full_res_gray_vs_formula', 'Q2_A_vs_gray_of_resized'):
      for name, s in r[k].items():
        lines.append('   %-27s %-22s mean %.3f p95 %.0f max %.0f exact %.4f' % (k[:2], name, s['mean'], s['p95'], s['max'], s['exact']))
    for k in ('Q2_A_vs_B_gray_then_resize', 'Q3_A_vs_pil_mode', 'Q3_A_vs_libvips_mode',
              'Q3_A_vs_libvipsformula_after_pil_resize', 'Q3_A_vs_fitted_lut_gray_then_pil_resize',
              'Q3_resample_only_rgb_C_vs_pil',
              'A_raw_vs_A_file_decoded', 'JPEG_PIL_L_q80_vs_sharp_file', 'JPEG_PIL_RGB_q80_vs_sharp_file'):
      if k in r:
        s = r[k]
        lines.append('   %-40s mean %.3f p95 %.0f max %.0f exact %.4f' % (k, s['mean'], s['p95'], s['max'], s['exact']))
  # summary table (tab separated) of the numbers used in README.md
  cols = ['Q1_full_res_gray_vs_formula:vips_lut_interp', 'Q1_full_res_gray_vs_formula:pil_601_on_gamma',
          'Q2_A_vs_B_gray_then_resize', 'Q3_A_vs_pil_mode', 'Q3_A_vs_libvips_mode',
          'Q3_A_vs_fitted_lut_gray_then_pil_resize', 'Q3_resample_only_rgb_C_vs_pil', 'A_raw_vs_A_file_decoded']
  tsv = ['image\tvalid_for_cli\tfmt\t' + '\t'.join(c + '(mean/p95/max/exact)' for c in cols)]
  for id_, r in res.items():
    row = [r['image'], str(r['valid_for_cli']), r['format']]
    for c in cols:
      v = r
      for part in c.split(':'):
        v = v.get(part) if isinstance(v, dict) else None
      row.append('%.3f/%.0f/%.0f/%.4f' % (v['mean'], v['p95'], v['max'], v['exact']) if v else '-')
    tsv.append('\t'.join(row))
  open(OUT + '.tsv', 'w').write('\n'.join(tsv) + '\n')
  os.makedirs(os.path.dirname(os.path.abspath(OUT + '.json')), exist_ok=True)
  json.dump(res, open(OUT + '.json', 'w'), indent=1)
  open(OUT + '.txt', 'w').write('\n'.join(lines) + '\n')
  print('\n'.join(lines))


if __name__ == '__main__':
  main()
