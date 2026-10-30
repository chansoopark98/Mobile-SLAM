#!/usr/bin/env python3
"""Synthetic image target for the OR-4 world regression gate (preregistration/or4-it-world-regression.v1.md).

Writes into --out (refuses an existing non-empty directory unless --force):
  or4-flat.json              target JSON in the image-target-cli format (PLANAR, no crop)
  or4-flat_luminance.pgm     luminance, 480x640 binary PGM (read by the engine, sr_or4_replay --targets)
  or4-flat_luminance.png     the same luminance as an 8-bit grey PNG (loaded by the original SDK in a browser)
  SHA256SUMS                 sha256 of the three files

The texture is render_synthetic.make_texture(default_rng(seed)) with the seed of the US-013 gates (13), so the
pixels equal synth-flat_luminance.pgm of those gates; only the target name differs. The target is NOT visible in
any world sequence: OR-4 measures what loading a target does to the world SLAM while the target is never seen.
Development tool (numpy + Pillow); nothing here is part of the product runtime.
"""

import argparse
import hashlib
import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_synthetic as rs  # noqa: E402

NAME = 'or4-flat'


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=13)
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args(argv)
    if os.path.isdir(a.out) and os.listdir(a.out) and not a.force:
        print(f'refusing to write into non-empty {a.out} (use --force)', file=sys.stderr)
        return 2
    os.makedirs(a.out, exist_ok=True)
    texture = rs.make_texture(np.random.default_rng(a.seed))
    rs.write_pgm(os.path.join(a.out, f'{NAME}_luminance.pgm'), texture)
    Image.fromarray(texture, mode='L').save(os.path.join(a.out, f'{NAME}_luminance.png'), optimize=False)
    # Round trip: the PNG must decode to the PGM pixels (8-bit grey, no colour management in the file).
    back = np.asarray(Image.open(os.path.join(a.out, f'{NAME}_luminance.png')).convert('L'))
    if not np.array_equal(back, texture):
        print('PNG round trip differs from the PGM pixels', file=sys.stderr)
        return 3
    target = {
        'type': 'PLANAR',
        'properties': {'left': 0, 'top': 0, 'width': rs.W, 'height': rs.H, 'isRotated': False,
                       'originalWidth': rs.W, 'originalHeight': rs.H},
        'imagePath': f'image-targets/{NAME}_luminance.png',
        'metadata': None,
        'name': NAME,
        'resources': {'luminanceImage': f'{NAME}_luminance.png'},
        'created': 0,
        'updated': 0,
    }
    with open(os.path.join(a.out, f'{NAME}.json'), 'w') as f:
        json.dump(target, f, indent=2)
    lines = []
    for fn in (f'{NAME}.json', f'{NAME}_luminance.pgm', f'{NAME}_luminance.png'):
        with open(os.path.join(a.out, fn), 'rb') as f:
            lines.append(f'{hashlib.sha256(f.read()).hexdigest()}  {fn}')
    with open(os.path.join(a.out, 'SHA256SUMS'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
