#!/usr/bin/env python3
"""B6 / RM-2: does sharp change pixel values that PIL leaves alone (embedded ICC profiles)? Run with `python3 -I`.
For every LFS image: PIL (EXIF transpose, no colour management) vs sharp(path).autoOrient().raw()
(written by icc_probe.mjs). Prints per image: embedded profile description, and the per-pixel difference.
Usage: icc_probe.py <icc-raw-dir> <out.json>
"""
import io
import json
import os
import sys

import numpy as np
from PIL import Image, ImageCms, ImageOps

raw_dir, out = sys.argv[1:3]
root = os.path.join(raw_dir, '..', '..', 'lfs-files')
meta = json.load(open(os.path.join(raw_dir, 'meta.json')))
rows = []
for m in meta:
  with Image.open(os.path.join(root, m['path'])) as im:
    icc = im.info.get('icc_profile')
    desc = ImageCms.getProfileDescription(ImageCms.ImageCmsProfile(io.BytesIO(icc))).strip() if icc else None
    o = ImageOps.exif_transpose(im)
    a = np.asarray(o.convert({1: 'L', 2: 'LA', 3: 'RGB', 4: 'RGBA'}[m['channels']]))
  b = np.fromfile(os.path.join(raw_dir, m['id'] + '.raw'), dtype=np.uint8)
  b = b.reshape(m['height'], m['width'], m['channels']) if m['channels'] > 1 else b.reshape(m['height'], m['width'])
  if a.shape != b.shape:
    rows.append({'path': m['path'], 'icc': desc, 'shape_mismatch': [list(a.shape), list(b.shape)]})
    continue
  d = np.abs(a.astype(int) - b.astype(int))
  row = {'path': m['path'], 'icc': desc, 'icc_bytes': m['iccBytes'], 'mean': round(float(d.mean()), 4),
         'p95': float(np.percentile(d, 95)), 'max': int(d.max()), 'exact': round(float((d == 0).mean()), 5)}
  if desc and m['channels'] in (3, 4):
    # candidate fix: apply the embedded profile with lcms (PIL ImageCms) -> sRGB, intents tried in order
    # (also run for profiles after which sharp changed nothing, to see whether the fix would be a no-op there)
    with Image.open(os.path.join(root, m['path'])) as im:
      src = ImageCms.ImageCmsProfile(io.BytesIO(im.info['icc_profile']))
      o = ImageOps.exif_transpose(im).convert('RGB')
    row['pil_lcms'] = {}
    for iname, intent in (('perceptual', 0), ('relative', 1), ('saturation', 2), ('absolute', 3)):
      for bpc in (False, True):
        flags = ImageCms.Flags.BLACKPOINTCOMPENSATION if bpc else ImageCms.Flags.NONE
        try:
          c = np.asarray(ImageCms.profileToProfile(o, src, ImageCms.createProfile('sRGB'), renderingIntent=intent,
                                                   outputMode='RGB', flags=flags))
        except ImageCms.PyCMSError as e:  # e.g. a profile lcms cannot build a transform from
          row['pil_lcms']['%s%s' % (iname, '+bpc' if bpc else '')] = {'error': str(e)}
          continue
        dd = np.abs(c.astype(int) - b[..., :3].astype(int))
        row['pil_lcms']['%s%s' % (iname, '+bpc' if bpc else '')] = {
            'mean': round(float(dd.mean()), 4), 'p95': float(np.percentile(dd, 95)), 'max': int(dd.max()),
            'exact': round(float((dd == 0).mean()), 5)}
  rows.append(row)
for r in rows:
  print(r)
json.dump(rows, open(out, 'w'), indent=1)
