#!/usr/bin/env python3
"""Deterministic, feature-rich source images for the phone demo targets (package C, desktop tooling only).

  make_demo_images.py --out DIR     writes demo-flat.png (1200x1600, 3:4 portrait) and demo-cylinder.png (2000x1000, 2:1)

The images are procedural (seeded numpy noise at several scales + polygons + stripes), so they carry no third-party
artwork and have corners at every scale for the engine's ORB detector. Needs numpy and Pillow (development tools only).
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageDraw


def _value_noise(rng, h, w, cell):
    gh, gw = h // cell + 2, w // cell + 2
    grid = rng.random((gh, gw))
    big = np.kron(grid, np.ones((cell, cell)))
    return big[:h, :w]


def _texture(seed, h, w):
    rng = np.random.default_rng(seed)
    base = np.zeros((h, w))
    for cell, amp in ((128, 0.35), (48, 0.3), (16, 0.2), (6, 0.15)):
        base += amp * _value_noise(rng, h, w, cell)
    rgb = np.stack([base, np.roll(base, 37, axis=1) * 0.8 + 0.1, np.roll(base, 71, axis=0) * 0.7 + 0.15], axis=2)
    img = Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8), 'RGB')
    d = ImageDraw.Draw(img)
    for _ in range(90):
        cx, cy = rng.integers(0, w), rng.integers(0, h)
        r = int(rng.integers(20, max(24, min(h, w) // 7)))
        pts = [(cx + r * np.cos(a) * rng.uniform(0.5, 1.0), cy + r * np.sin(a) * rng.uniform(0.5, 1.0))
               for a in np.linspace(0, 2 * np.pi, int(rng.integers(3, 7)), endpoint=False)]
        col = tuple(int(c) for c in rng.integers(0, 256, 3))
        d.polygon(pts, fill=col, outline=(0, 0, 0))
    for i in range(0, w, 97):
        d.line([(i, 0), (i + int(rng.integers(-60, 60)), h)], fill=(0, 0, 0), width=3)
    d.rectangle([4, 4, w - 5, h - 5], outline=(0, 0, 0), width=6)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    _texture(20261022, 1600, 1200).save(os.path.join(args.out, 'demo-flat.png'))
    _texture(20261023, 1000, 2000).save(os.path.join(args.out, 'demo-cylinder.png'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
