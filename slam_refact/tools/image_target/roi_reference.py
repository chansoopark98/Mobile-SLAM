#!/usr/bin/env python3
"""Independent numpy oracle of the CPU ROI draw (RM-6, gate file rm6-it-roi-gate.v1.md).

`prepare` writes, into --out, everything the C++ test needs, and the expected slot images:
  G-ROI-A  a_camera.pgm, a_warps.txt (4 lines of 16 row-major values M(row, col)),
           a_expected_<i>.pgm / a_mask_<i>.pgm (slot size; mask 255 = covered)
  G-ROI-B  b_camera_<i>.pgm, b_pose_<i>.txt (12 values: R row-major, t; camera -> target view),
           b_expected.pgm (the same for every pose), b_mask.pgm

The expected images do not use any engine code. G-ROI-A is the GL semantics written out from the
shader description (gl-reality-frame.cc drawInitialStage, gr8-rotate.vert, gr8-stage1.frag): a slot
pixel centre in NDC is mapped through the inverse of the 3x3 homography of the mvp rows/columns
{0, 1, 3}; the quad (x, y in [-1, 1]) covers the pixel when the point lies inside with clip w > 0;
uv = (p + 1) / 2; the camera texture is sampled bilinearly at uv * size - 0.5 (clamp to edge) and
rounded to 8 bit. G-ROI-B is a geometric rule: a target seen from a known pose fills the middle
90 % x 90 % of the slot (target 3:4 = slot 3:4) upright, so the expected slot is the target texture
resampled that way, independent of any warp matrix.

Only numpy is required (development tool).
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_synthetic as rs  # noqa: E402  (same geometry and PGM writer as the synthetic gate)

CAM_W, CAM_H = 480, 640
K = [506.4104, 506.4104, 239.5, 319.5]
SEED_A = 31
SEED_B = 21


def bilinear_clamp(img, tx, ty, dtype=np.float64):
    """GL LINEAR with CLAMP_TO_EDGE at texel coordinates (tx, ty) (texel centres at integers)."""
    h, w = img.shape
    x0 = np.floor(tx).astype(int)
    y0 = np.floor(ty).astype(int)
    fx = (tx - x0).astype(dtype)
    fy = (ty - y0).astype(dtype)

    def at(yy, xx):
        return img[np.clip(yy, 0, h - 1), np.clip(xx, 0, w - 1)].astype(dtype)

    top = at(y0, x0) * (1 - fx) + at(y0, x0 + 1) * fx
    bot = at(y0 + 1, x0) * (1 - fx) + at(y0 + 1, x0 + 1) * fx
    return top * (1 - fy) + bot * fy


def quantise(v):
    return np.clip(np.floor(v + 0.5), 0, 255).astype(np.uint8)


def camera_a(kind):
    """480x640 camera image for G-ROI-A and the null-runs; every value >= 16."""
    gy, gx = np.mgrid[0:CAM_H, 0:CAM_W].astype(np.float64)
    if kind == 'constant':
        return np.full((CAM_H, CAM_W), 100, dtype=np.uint8)
    if kind == 'ramp':
        return np.clip(np.floor(16 + gx / 4.0 + gy / 8.0), 0, 255).astype(np.uint8)
    rng = np.random.default_rng(SEED_A)
    img = 70.0 + 50.0 * gx / CAM_W + 40.0 * gy / CAM_H
    for _ in range(140):
        w, h = rng.integers(10, 70, size=2)
        x, y = rng.integers(0, CAM_W - 4), rng.integers(0, CAM_H - 4)
        img[y:y + h, x:x + w] = rng.uniform(20, 255)
    img += rng.normal(0.0, 3.0, size=img.shape)
    return np.clip(np.rint(img), 16, 255).astype(np.uint8)


def warps_a():
    c, s = np.cos(np.radians(20.0)), np.sin(np.radians(20.0))
    return [
        np.diag([0.9, 0.9, 1.0, 1.0]),
        np.array([[0.8 * c, -0.8 * s, 0, 0.1], [0.8 * s, 0.8 * c, 0, -0.05], [0, 0, 1, 0],
                  [0, 0, 0, 1]]),
        np.array([[0.9, 0.1, 0, 0.05], [-0.05, 0.85, 0, 0], [0, 0, 1, 0], [0.12, -0.08, 0, 1]]),
        np.diag([1.3, 1.3, 1.0, 1.0]),
    ]


def slot_expected(camera, m, slot_w, slot_h, dtype=np.float64):
    """GL semantics of one planar ROI slot (rotation 0); dtype is the arithmetic precision."""
    m = np.asarray(m, dtype=np.float32).astype(dtype)  # the engine's warp is float32
    h3 = np.array([[m[0, 0], m[0, 1], m[0, 3]], [m[1, 0], m[1, 1], m[1, 3]],
                   [m[3, 0], m[3, 1], m[3, 3]]], dtype=dtype)
    inv = np.linalg.inv(h3)
    ys, xs = np.mgrid[0:slot_h, 0:slot_w].astype(dtype)
    nx = 2.0 * (xs + 0.5) / slot_w - 1.0
    ny = 2.0 * (ys + 0.5) / slot_h - 1.0
    q = np.stack([nx, ny, np.ones_like(nx)], axis=-1) @ inv.T
    with np.errstate(divide='ignore', invalid='ignore'):
        px = q[..., 0] / q[..., 2]
        py = q[..., 1] / q[..., 2]
    clip_w = h3[2, 0] * px + h3[2, 1] * py + h3[2, 2]
    covered = (np.abs(px) <= 1.0) & (np.abs(py) <= 1.0) & (clip_w > 0.0) & np.isfinite(px)
    px = np.where(covered, px, 0.0)
    py = np.where(covered, py, 0.0)
    tx = (px + 1.0) / 2.0 * CAM_W - 0.5
    ty = (py + 1.0) / 2.0 * CAM_H - 0.5
    out = quantise(bilinear_clamp(camera, tx, ty, dtype))
    out = np.where(covered, out, 0).astype(np.uint8)
    return out, np.where(covered, 255, 0).astype(np.uint8)


def smooth_texture():
    """Low-frequency, asymmetric 480x640 target texture (G-ROI-B)."""
    rng = np.random.default_rng(SEED_B)
    coarse = rng.uniform(-110, 110, size=(7, 5))
    gy, gx = np.mgrid[0:CAM_H, 0:CAM_W].astype(np.float64)
    cy = gy / (CAM_H - 1) * (coarse.shape[0] - 1)
    cx = gx / (CAM_W - 1) * (coarse.shape[1] - 1)
    y0 = np.clip(np.floor(cy).astype(int), 0, coarse.shape[0] - 2)
    x0 = np.clip(np.floor(cx).astype(int), 0, coarse.shape[1] - 2)
    fy = cy - y0
    fx = cx - x0
    base = (coarse[y0, x0] * (1 - fx) * (1 - fy) + coarse[y0, x0 + 1] * fx * (1 - fy) +
            coarse[y0 + 1, x0] * (1 - fx) * fy + coarse[y0 + 1, x0 + 1] * fx * fy)
    return np.clip(np.rint(128.0 + base), 16, 255).astype(np.uint8)


class ZeroNoise:
    def normal(self, loc, scale, size):
        return np.zeros(size)


def target_expected(texture, slot_w, slot_h, dtype=np.float64):
    ys, xs = np.mgrid[0:slot_h, 0:slot_w].astype(dtype)
    tu = ((xs + 0.5) / slot_w - 0.05) / 0.9 * CAM_W - 0.5
    tv = ((ys + 0.5) / slot_h - 0.05) / 0.9 * CAM_H - 0.5
    return quantise(bilinear_clamp(texture, tu, tv, dtype))


def write_text_row(f, values):
    f.write(' '.join(repr(float(v)) for v in values) + '\n')


def prepare(a):
    os.makedirs(a.out, exist_ok=True)
    w, h = a.slot_w, a.slot_h
    cam = camera_a(a.camera_kind)
    rs.write_pgm(os.path.join(a.out, 'a_camera.pgm'), cam)
    ws = warps_a()
    with open(os.path.join(a.out, 'a_warps.txt'), 'w') as f:
        for m in ws:
            write_text_row(f, m.reshape(-1))
    for i, m in enumerate(ws):
        exp, mask = slot_expected(cam, m, w, h)
        rs.write_pgm(os.path.join(a.out, f'a_expected_{i}.pgm'), exp)
        rs.write_pgm(os.path.join(a.out, f'a_mask_{i}.pgm'), mask)
    # G-ROI-B: frontal, roll +8 deg, roll -8 deg (centred, distance 2.1).
    tex = smooth_texture()
    rs.write_pgm(os.path.join(a.out, 'b_texture.pgm'), tex)
    expected = target_expected(tex, w, h)
    rs.write_pgm(os.path.join(a.out, 'b_expected.pgm'), expected)
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[int(0.1 * h):int(np.ceil(0.9 * h)), int(0.1 * w):int(np.ceil(0.9 * w))] = 255
    rs.write_pgm(os.path.join(a.out, 'b_mask.pgm'), mask)
    # Poses of G-ROI-B (gate file v2 section 2): frontal, commanded roll +8 / -8 degrees, and the
    # registered trajectory at k = 18, 32, 64 (segment B starts at frame 12).
    poses = []
    for roll in (0.0, 8.0, -8.0):
        poses.append((rs.rot_axis('z', roll), np.array([0.0, 0.0, 1.0 - 2.1])))
    for k in (18, 32, 64):
        poses.append(rs.camera_pose_registered(k, 12, True))
    for i, (r, t) in enumerate(poses):
        img = rs.render(tex, K, K, r, t, True, ZeroNoise())
        rs.write_pgm(os.path.join(a.out, f'b_camera_{i}.pgm'), img)
        with open(os.path.join(a.out, f'b_pose_{i}.txt'), 'w') as f:
            write_text_row(f, list(r.reshape(-1)) + list(t))
    print(f'wrote G-ROI-A/B inputs and expected images to {a.out} (slot {w}x{h}, camera {a.camera_kind})')
    return 0


def selftest(a):
    """N4: numeric noise of this oracle (float32 versus float64 arithmetic)."""
    cam = camera_a('textured')
    worst_max, worst_mean, worst_cov = 0, 0.0, 0.0
    for m in warps_a():
        e64, k64 = slot_expected(cam, m, a.slot_w, a.slot_h, np.float64)
        e32, k32 = slot_expected(cam, m, a.slot_w, a.slot_h, np.float32)
        both = (k64 > 0) & (k32 > 0)
        d = np.abs(e64.astype(int) - e32.astype(int))[both]
        worst_max = max(worst_max, int(d.max()))
        worst_mean = max(worst_mean, float(d.mean()))
        worst_cov = max(worst_cov, float(((k64 > 0) != (k32 > 0)).mean()))
    tex = smooth_texture()
    b64 = target_expected(tex, a.slot_w, a.slot_h, np.float64).astype(np.float64)
    b32 = target_expected(tex, a.slot_w, a.slot_h, np.float32).astype(np.float64)
    h, w = b64.shape
    sl = (slice(int(0.1 * h), int(np.ceil(0.9 * h))), slice(int(0.1 * w), int(np.ceil(0.9 * w))))
    e = float(np.sqrt(np.mean((b32[sl] - b64[sl]) ** 2)) / b64[sl].std())
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'selftest.txt'), 'w') as f:
        f.write(f'a_max={worst_max}\na_mean={worst_mean!r}\na_cov={worst_cov!r}\nb_e={e!r}\n')
    print(f'N4 float32 vs float64: A2 {worst_max}, A3 {worst_mean:.6f}, A1 {worst_cov:.6f}, B1 e {e:.6f}')
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--out', required=True)
    p.add_argument('--slot-w', type=int, required=True)
    p.add_argument('--slot-h', type=int, required=True)
    p.add_argument('--camera-kind', choices=['textured', 'constant', 'ramp'], default='textured')
    t = sub.add_parser('selftest')
    t.add_argument('--out', required=True)
    t.add_argument('--slot-w', type=int, required=True)
    t.add_argument('--slot-h', type=int, required=True)
    a = ap.parse_args(argv)
    return prepare(a) if a.cmd == 'prepare' else selftest(a)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
