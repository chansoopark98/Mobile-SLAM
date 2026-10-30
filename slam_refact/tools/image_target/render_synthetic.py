#!/usr/bin/env python3
"""Synthetic flat image target sequence with ground truth (full-parity PRD US-013).

Writes into --out (refuses an existing non-empty directory unless --force):
  synth-flat.json            target JSON in the image-target-cli format (PLANAR, no crop)
  synth-flat_luminance.pgm   target luminance (480x640, binary PGM)
  frames/NNNN.pgm            camera frames (480x640 gray)
  gt.json                    camera K, target K, segments and per-frame camera pose
  gt.tsv                     the same ground truth as plain text (read by the C++ gate test)

Geometry (slam_refact/docs/design/image-target.md §7.1): coordinates are the target view of the
engine (TrackedImage::pose): x right, y up, z forward. Target pixel (u, v) lies at
((u - cx_t) / fx_t, -(v - cy_t) / fy_t, 1). A camera pose T maps camera coordinates to the target
view, X_tv = R X_cam + t; a camera point projects to u = fx x / z + cx, v = cy - fy y / z
(c8 HMatrixGen::intrinsic, -fy).

Curved targets (US-014, `--shape cylinder|cone`, independent numpy implementation, section
"Curved targets" below): the target is wrapped around a cylinder or a cone frustum and rendered by
casting a ray through every camera pixel to the surface (the inverse of the engine's forward
"lifting"); the ground truth is the camera-to-model transform of the engine's curvy model frame
(axis y up, cropped height 1 centred on the origin, label front at -z, camera x right, y up, z
forward). See preregistration/curvy-it-gate.v1.md.

Curved targets (US-014, `--shape cylinder|cone`, independent numpy implementation, section
"Curved targets" below): the target is wrapped around a cylinder or a cone frustum and rendered by
casting a ray through every camera pixel to the surface (the inverse of the engine's forward
"lifting"); the ground truth is the camera-to-model transform of the engine's curvy model frame
(axis y up, cropped height 1 centred on the origin, label front at -z, camera x right, y up, z
forward). See preregistration/curvy-it-gate.v1.md.

Only numpy is required (development tool; nothing here is part of the product runtime).
"""

import argparse
import hashlib
import json
import math
import os
import re
import sys

import numpy as np

W, H = 480, 640
NAME = 'synth-flat'
# Segments of the sequence (frame index = stage index), image-target.md §7.1.
SEGMENTS = [('A', 0, 12, False), ('B', 12, 72, True), ('C', 72, 96, False), ('D', 96, 132, True)]
FPS = 30.0


def parse_k(text):
    v = [float(x) for x in text.split(',')]
    if len(v) != 4:
        raise argparse.ArgumentTypeError('expected fx,fy,cx,cy')
    return v


def make_texture(rng):
    """Deterministic 480x640 texture with many corners."""
    gy, gx = np.mgrid[0:H, 0:W].astype(np.float64)
    coarse = rng.uniform(-40, 40, size=(9, 7))
    # Bilinear upsampling of the coarse grid.
    cy = gy / (H - 1) * (coarse.shape[0] - 1)
    cx = gx / (W - 1) * (coarse.shape[1] - 1)
    y0 = np.clip(np.floor(cy).astype(int), 0, coarse.shape[0] - 2)
    x0 = np.clip(np.floor(cx).astype(int), 0, coarse.shape[1] - 2)
    fy = cy - y0
    fx = cx - x0
    base = (coarse[y0, x0] * (1 - fx) * (1 - fy) + coarse[y0, x0 + 1] * fx * (1 - fy) +
            coarse[y0 + 1, x0] * (1 - fx) * fy + coarse[y0 + 1, x0 + 1] * fx * fy)
    img = 128.0 + base
    for _ in range(260):
        w, h = rng.integers(8, 61, size=2)
        x, y = rng.integers(0, W - 4), rng.integers(0, H - 4)
        img[y:y + h, x:x + w] = rng.uniform(0, 255)
    for _ in range(120):
        r = rng.uniform(4, 30)
        x, y = rng.uniform(0, W), rng.uniform(0, H)
        img[(gx - x) ** 2 + (gy - y) ** 2 <= r * r] = rng.uniform(0, 255)
    # 3x3 box blur (edge replicate) against aliasing.
    p = np.pad(img, 1, mode='edge')
    img = sum(p[dy:dy + H, dx:dx + W] for dy in range(3) for dx in range(3)) / 9.0
    return np.clip(np.rint(img), 0, 255).astype(np.uint8)


def write_pgm(path, gray):
    h, w = gray.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(np.ascontiguousarray(gray, dtype=np.uint8).tobytes())


def rot_axis(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def camera_pose_narrow(k, segment_start, visible):
    """First-run trajectory (run1/run2): camera-to-target-view rotation R and position t of frame k.

    Narrower than the pre-registered ranges (review M2); kept to reproduce run1/run2 byte for byte.
    """
    tau = (k - segment_start) / FPS
    phi = 0.7 if segment_start >= 96 else 0.0
    centre = np.array([0.0, 0.0, 1.0])
    if visible:
        dist = 2.1 + 0.3 * math.sin(2 * math.pi * tau / 3.1 + phi)
        pos = centre + np.array([0.3 * math.sin(2 * math.pi * tau / 4.3 + phi),
                                 0.2 * math.sin(2 * math.pi * tau / 3.7 + 2 * phi), -dist])
        z = centre - pos
    else:
        # Looking away from the target (about 90 degrees to the side): background only.
        pos = centre + np.array([0.0, 0.0, -2.1])
        z = np.array([1.0, 0.0, 0.05])
    z = z / np.linalg.norm(z)
    up = np.array([0.0, 1.0, 0.0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    r_look = np.stack([x, y, z], axis=1)
    pert = (rot_axis('y', 2.5 * math.sin(2 * math.pi * tau / 2.9)) @
            rot_axis('x', 2.5 * math.sin(2 * math.pi * tau / 3.3 + 1.0)) @
            rot_axis('z', 8.0 * math.sin(2 * math.pi * tau / 5.1)))
    return r_look @ pert, pos


def look_at(pos, centre=(0.0, 0.0, 1.0)):
    """Rotation whose columns are the camera x, y, z axes (target view) looking at `centre`."""
    z = np.asarray(centre) - pos
    z = z / np.linalg.norm(z)
    up = np.array([0.0, 1.0, 0.0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


# Registered trajectory (build/slam-refact-full-20261019/preregistration/rm6-it-roi-gate.v1.md,
# G-TRAJ): period 2.0 s, phase c0 per segment.
TRAJ_PERIOD_S = 2.0
TRAJ_C0 = {12: 0.0, 96: 0.35}


def camera_pose_registered(k, segment_start, visible):
    """Registered trajectory: camera-to-target-view rotation R and position t of frame k."""
    if not visible:
        pos = np.array([0.0, 0.0, 1.0]) + np.array([0.0, 0.0, -2.1])
        z = np.array([1.0, 0.0, 0.05])
        z = z / np.linalg.norm(z)
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
        x /= np.linalg.norm(x)
        return np.stack([x, np.cross(z, x), z], axis=1), pos
    tau = (k - segment_start) / FPS
    c0 = TRAJ_C0[segment_start]

    def s(c):
        return math.sin(2 * math.pi * (tau / TRAJ_PERIOD_S + c0 + c))

    dist = 2.1 + 0.3 * s(0.65)
    pos = np.array([0.3 * s(0.15), 0.3 * s(0.40), 1.0 - dist])
    pert = (rot_axis('y', -4.5 * s(0.15)) @ rot_axis('x', 4.5 * s(0.40)) @ rot_axis('z', 8.5 * s(0.10)))
    return look_at(pos) @ pert, pos


def camera_pose(k, segment_start, visible, trajectory='registered'):
    if trajectory == 'narrow':
        return camera_pose_narrow(k, segment_start, visible)
    return camera_pose_registered(k, segment_start, visible)


def trajectory_quantities(r, t):
    """(x, y, distance, yaw, pitch, roll) of one camera pose, G-TRAJ definitions (degrees)."""
    z = r @ np.array([0.0, 0.0, 1.0])
    x_cam = r @ np.array([1.0, 0.0, 0.0])
    x_ref = np.cross(np.array([0.0, 1.0, 0.0]), z)
    x_ref /= np.linalg.norm(x_ref)
    yaw = math.degrees(math.atan2(z[0], z[2]))
    pitch = math.degrees(math.atan2(z[1], math.hypot(z[0], z[2])))
    roll = math.degrees(math.atan2(float(np.dot(np.cross(x_cam, x_ref), z)), float(np.dot(x_cam, x_ref))))
    return t[0], t[1], 1.0 - t[2], yaw, pitch, roll


# (index, nominal centre, amplitude, hard bound) of trajectory_quantities for the range check.
TRAJ_RANGES = [('x', 0.0, 0.3, 0.31), ('y', 0.0, 0.3, 0.31), ('distance', 2.1, 0.3, None),
               ('yaw', 0.0, 12.0, 14.0), ('pitch', 0.0, 12.0, 14.0), ('roll', 0.0, 8.0, 9.0)]


def check_trajectory_ranges(poses):
    """G-TRAJ range check over (R, t) of the visible frames: returns (report lines, ok)."""
    q = np.array([trajectory_quantities(r, t) for r, t in poses])
    ok = True
    lines = []
    for i, (name, centre, amp, bound) in enumerate(TRAJ_RANGES):
        lo, hi = float(q[:, i].min()), float(q[:, i].max())
        reach = lo <= centre - 0.9 * amp and hi >= centre + 0.9 * amp
        inside = True
        if bound is not None:
            inside = lo >= centre - bound and hi <= centre + bound
        if name == 'distance':
            inside = lo >= 1.79 and hi <= 2.41
        lines.append(f'{name}: min {lo:+.4f} max {hi:+.4f} (need <= {centre - 0.9 * amp:+.3f} and '
                     f'>= {centre + 0.9 * amp:+.3f}) {"OK" if reach and inside else "VIOLATION"}')
        ok = ok and reach and inside
    return lines, ok


def render(texture, cam_k, tgt_k, r, t, visible, rng):
    fx, fy, cx, cy = cam_k
    tfx, tfy, tcx, tcy = tgt_k
    acc = np.zeros((H, W))
    gy, gx = np.mgrid[0:H, 0:W].astype(np.float64)
    background = 90.0 + 30.0 * gx / W + 20.0 * gy / H
    tex = texture.astype(np.float64)
    for oy in (-0.25, 0.25):
        for ox in (-0.25, 0.25):
            u = gx + ox
            v = gy + oy
            sample = background.copy()
            if visible:
                d = np.stack([(u - cx) / fx, -(v - cy) / fy, np.ones_like(u)], axis=-1) @ r.T
                with np.errstate(divide='ignore', invalid='ignore'):
                    s = (1.0 - t[2]) / d[..., 2]
                px = t[0] + s * d[..., 0]
                py = t[1] + s * d[..., 1]
                ut = tfx * px + tcx
                vt = tcy - tfy * py
                inside = (s > 0) & (ut >= 0) & (ut <= W - 1) & (vt >= 0) & (vt <= H - 1)
                ui = np.clip(np.floor(ut), 0, W - 2).astype(int)
                vi = np.clip(np.floor(vt), 0, H - 2).astype(int)
                fu = np.clip(ut - ui, 0, 1)
                fv = np.clip(vt - vi, 0, 1)
                val = (tex[vi, ui] * (1 - fu) * (1 - fv) + tex[vi, ui + 1] * fu * (1 - fv) +
                       tex[vi + 1, ui] * (1 - fu) * fv + tex[vi + 1, ui + 1] * fu * fv)
                sample = np.where(inside, val, sample)
            acc += sample
    img = acc / 4.0 + rng.normal(0.0, 1.5, size=(H, W))
    return np.clip(np.rint(img), 0, 255).astype(np.uint8)


# ------------------------------------------------------------------------------------------------
# Curved targets (US-014). Independent of the engine: the geometry is derived from the physical
# description of the printed object (lengths in mm), the image is rendered by casting a ray through
# every camera pixel to the surface (the engine does the opposite, a forward lift of every ROI
# pixel), and nothing here reads engine code.
#
# Model frame (the frame of TrackedImage::pose, image-target.md section 7.1 convention extended to
# curved targets): the axis of the cylinder / cone is the y axis (up), the cropped label has height
# 1 and is centred on the origin (y in [-0.5, 0.5]), the label front is at -z (angle fraction 0.5).
# A label pixel (col, row) of an image of w0 x h0 pixels lies at the angle fraction
#   f = left + (right - left) * col / (w0 - 1)       (left, right: activation region)
# and at the height y = 0.5 - row / (h0 - 1); the point is
#   (-rho(y) sin(2 pi f), y, rho(y) cos(2 pi f)),  rho(y) = rb + (rt - rb) (y + 0.5).
# A camera pose T maps camera coordinates to the model frame, X_model = R X_cam + t.
# ------------------------------------------------------------------------------------------------
BODY_GRAY = 70.0       # printed object outside the label
INTERIOR_GRAY = 45.0   # inside wall seen through the open top / bottom

# ------------------------------------------------------------------------------------------------
# Data variants (curvy-it-gate, coordinator decision after the v2 review B-1; the same for every later version).
#   dev         the shapes, seeds, trajectory and poses of the exposed v1/v2 data: every engine run before the
#               independent review (null-runs, mutation, development) uses ONLY this variant.
#   registered  derived from the coordinator seed by derive_registered() below, never fed to the engine before
#               the first real run. The executor does not adjust any derived value.
#               The coordinator seed has NO default: it is read from a seed file (--seed-file, or the environment variable
#               SLAM_REFACT_CURVY_SEED_FILE) by use_seed_file(); without one the registered variant does not exist. (The
#               v3 seed used to be a constant here; a later version silently reusing it would have rendered the exposed
#               v3 data. The v3 seed lives only in preregistration/curvy-it-gate.v3.coordinator-seed.txt.)
# The extreme poses of the registered variant are fixed at the trajectory limits (azimuth +-35 deg, height +-0.3,
# distance 1.8 / 2.4, look-at yaw / pitch +-4.5 deg, roll +-8.5 deg), not derived.
# ------------------------------------------------------------------------------------------------
SEED_FILE_ENV = 'SLAM_REFACT_CURVY_SEED_FILE'
COORDINATOR_SEED = None   # set by use_coordinator_seed(); None = no registered variant


def _seed_u(label):
    """Uniform [0, 1) from sha256('<coordinator seed>:<label>') (first 8 bytes, big endian)."""
    if COORDINATOR_SEED is None:
        raise RuntimeError('the registered variant needs a coordinator seed (--seed-file or ' + SEED_FILE_ENV + ')')
    h = hashlib.sha256(f'{COORDINATOR_SEED}:{label}'.encode()).digest()
    return int.from_bytes(h[:8], 'big') / 2.0 ** 64


def _seed_range(label, lo, hi, digits):
    return round(lo + (hi - lo) * _seed_u(label), digits)


def _seed_int(label, lo, hi):
    return lo + int(_seed_u(label) * (hi - lo + 1))


def derive_registered():
    """The registered variant, a deterministic function of COORDINATOR_SEED (procedure in curvy-it-gate.v3.md section 1;
    unchanged for later versions)."""
    shapes = {
        'cylinder': {'name': 'synth-cylinder', 'type': 'CYLINDER',
                     'circumference': _seed_range('cyl.circumference', 150.0, 250.0, 1),
                     'arc': _seed_range('cyl.arc', 0.45, 0.60, 3)},
        'cone': {'name': 'synth-cone', 'type': 'CONICAL', 'circumference_top': 100.0,
                 'base': _seed_range('cone.base', 1.25, 1.60, 3), 'arc': _seed_range('cone.arc', 0.55, 0.70, 3)},
    }
    traj = {'period': 2.0,
            'c0': {12: _seed_range('traj.c0.B', 0.0, 1.0, 3), 96: _seed_range('traj.c0.D', 0.0, 1.0, 3)},
            'phase': {k: _seed_range(f'traj.phase.{k}', 0.0, 1.0, 3)
                      for k in ('azimuth', 'distance', 'height', 'yaw', 'pitch', 'roll')}}
    b_poses = []   # (azimuth deg, y, distance, roll deg, look-at yaw deg, look-at pitch deg)
    for i in range(4):
        b_poses.append(((-1) ** i * _seed_range(f'pose.b{i}.az', 10.0, 30.0, 1), _seed_range(f'pose.b{i}.y', -0.2, 0.2, 3),
                        _seed_range(f'pose.b{i}.d', 1.9, 2.3, 2), _seed_range(f'pose.b{i}.roll', -6.0, 6.0, 1), 0.0, 0.0))
    b_poses.append((35.0, 0.3, 1.8, 8.5, 4.5, 4.5))        # trajectory limit: all maxima at once
    b_poses.append((-35.0, -0.3, 2.4, -8.5, -4.5, -4.5))   # trajectory limit: all minima at once
    a_poses = [((-1) ** i * _seed_range(f'pose.a{i}.az', 5.0, 25.0, 1), _seed_range(f'pose.a{i}.y', -0.2, 0.2, 3),
                _seed_range(f'pose.a{i}.d', 1.95, 2.3, 2), _seed_range(f'pose.a{i}.roll', -8.0, 8.0, 1), 0.0, 0.0)
               for i in range(4)]
    return {'shapes': shapes, 'traj': traj, 'b_poses': b_poses, 'a_poses': a_poses,
            'seeds': {'cylinder': _seed_int('seed.cylinder', 1000, 999999), 'cone': _seed_int('seed.cone', 1000, 999999)},
            'texture_seed_b': _seed_int('seed.texture_b', 1000, 999999),
            'camera_seed_a': _seed_int('seed.camera_a', 1000, 999999)}


VARIANTS = {
    'dev': {
        'shapes': {'cylinder': {'name': 'synth-cylinder', 'type': 'CYLINDER', 'circumference': 185.0, 'arc': 0.52},
                   'cone': {'name': 'synth-cone', 'type': 'CONICAL', 'circumference_top': 100.0, 'base': 1.4,
                            'arc': 0.65}},
        'traj': {'period': 2.0, 'c0': {12: 0.10, 96: 0.45},
                 'phase': {'azimuth': 0.05, 'distance': 0.60, 'height': 0.35, 'yaw': 0.20, 'pitch': 0.35, 'roll': 0.15}},
        'b_poses': [(2.0, 0.04, 2.05, 1.0, 0.0, 0.0), (22.0, -0.05, 2.15, -2.0, 0.0, 0.0), (-27.0, 0.08, 2.0, 3.0, 0.0, 0.0),
                    (33.0, 0.0, 2.2, 7.5, 0.0, 0.0), (-12.0, 0.28, 1.85, -1.0, 0.0, 0.0), (8.0, -0.28, 2.35, -7.0, 0.0, 0.0)],
        'a_poses': [(-4.0, 0.03, 2.05, 0.0, 0.0, 0.0), (28.0, 0.12, 1.95, 9.0, 0.0, 0.0),
                    (-22.0, -0.18, 2.15, -5.0, 0.0, 0.0), (12.0, 0.02, 2.05, 2.0, 0.0, 0.0)],
        'seeds': {'cylinder': 131, 'cone': 232}, 'texture_seed_b': 61, 'camera_seed_a': 33,
    },
}
ACTIVE_VARIANT = 'dev'   # a process works on one variant; the default is the exposed one


def read_seed_file(path):
    """The coordinator seed of a seed file: the first whitespace-separated token, an unsigned decimal integer without leading
    zeros and at most 2**64 - 1 (the rest of the line is a comment). Anything else is an error (no fallback)."""
    with open(path, 'r', encoding='utf-8') as f:
        tokens = f.read().split()
    if not tokens or not re.fullmatch(r'0|[1-9][0-9]{0,19}', tokens[0]) or int(tokens[0]) >= 2 ** 64:
        raise ValueError(f'{path}: the first token is not an unsigned decimal integer below 2**64')
    return int(tokens[0])


def use_coordinator_seed(seed):
    """Create the registered variant from `seed` (a pure function of it)."""
    global COORDINATOR_SEED
    COORDINATOR_SEED = int(seed)
    VARIANTS['registered'] = derive_registered()


def use_seed_file(path=None):
    """Create the registered variant from the seed file `path`, else from $SLAM_REFACT_CURVY_SEED_FILE. There is no default
    seed: with neither, this raises. Returns the sha256 of the seed file (for logs; the seed value is never printed)."""
    path = path or os.environ.get(SEED_FILE_ENV)
    if not path:
        raise ValueError('the registered variant needs the coordinator seed file: --seed-file FILE or ' + SEED_FILE_ENV)
    use_coordinator_seed(read_seed_file(path))
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def set_variant(variant):
    global ACTIVE_VARIANT
    if variant not in VARIANTS:
        if variant == 'registered':
            raise ValueError('the registered variant needs the coordinator seed (use_seed_file / --seed-file / ' + SEED_FILE_ENV + ')')
        raise ValueError(f'unknown variant {variant}')
    ACTIVE_VARIANT = variant


def variant_data(variant=None):
    return VARIANTS[variant or ACTIVE_VARIANT]


def curvy_model(shape, w0=W, h0=H, variant=None):
    """Physical description -> model-frame radii, activation region and the target JSON properties.

    cylinder: circumference C, label width wT = arc * C, label height hT = wT * h0 / w0 (the image
    keeps its aspect ratio on the object). Model unit = hT, so r = C / (2 pi hT).
    cone: top circumference Ct, bottom circumference Cb = Ct / base, label arc fraction `arc` of the
    circumference; the unrolled geometry image keeps the aspect ratio of its wider edge:
    slant S = arc * max(Ct, Cb) * h0 / w0, height hp = sqrt(S^2 - (rT - rB)^2) (rT = Ct / 2 pi, ...),
    model unit = hp.
    """
    spec = variant_data(variant)['shapes'][shape]
    arc = spec['arc']
    if shape == 'cylinder':
        c = spec['circumference']
        wt = arc * c
        ht = wt * h0 / w0
        r = c / (2 * math.pi * ht)
        rt, rb = r, r
        props = {'targetCircumferenceTop': wt, 'cylinderCircumferenceTop': c,
                 'cylinderCircumferenceBottom': c, 'cylinderSideLength': ht,
                 'arcAngle': arc * 360.0, 'coniness': 0, 'inputMode': 'ADVANCED', 'unit': 'mm'}
    else:
        ct = spec['circumference_top']
        cb = ct / spec['base']
        slant = arc * max(ct, cb) * h0 / w0
        rt_p, rb_p = ct / (2 * math.pi), cb / (2 * math.pi)
        hp = math.sqrt(slant * slant - (rt_p - rb_p) ** 2)
        rt, rb = rt_p / hp, rb_p / hp
        props = {'cylinderCircumferenceTop': ct, 'cylinderCircumferenceBottom': cb,
                 'targetCircumferenceTop': ct * arc, 'cylinderSideLength': slant,
                 'coniness': math.log2(spec['base']), 'arcAngle': arc * 360.0,
                 'inputMode': 'ADVANCED', 'unit': 'mm'}
    return {'shape': shape, 'rt': rt, 'rb': rb, 'left': 0.5 - arc / 2.0, 'right': 0.5 + arc / 2.0,
            'w0': w0, 'h0': h0, 'arc': arc, 'props': props}


def curvy_target_json(model, name, type_name, props=None):
    p = {'left': 0, 'top': 0, 'width': model['w0'], 'height': model['h0'], 'isRotated': False,
         'originalWidth': model['w0'], 'originalHeight': model['h0']}
    p.update(props if props is not None else model['props'])
    return {'type': type_name, 'properties': p, 'imagePath': f'image-targets/{name}_luminance.png',
            'metadata': None, 'name': name, 'resources': {'luminanceImage': f'{name}_luminance.png'},
            'created': 0, 'updated': 0}


def curvy_json_mutations(model):
    """Parameter errors a wrong JSON would introduce: id -> properties (the ground truth is unchanged)."""
    p = model['props']
    out = {}
    q = dict(p)  # curvature: both circumferences 1.3x, the printed width unchanged -> arc / 1.3
    q['cylinderCircumferenceTop'] = p['cylinderCircumferenceTop'] * 1.3
    q['cylinderCircumferenceBottom'] = p['cylinderCircumferenceBottom'] * 1.3
    out['circ13'] = q
    q = dict(p)  # printed width 0.8x
    q['targetCircumferenceTop'] = p['targetCircumferenceTop'] * 0.8
    out['arc80'] = q
    if model['shape'] == 'cone':
        q = dict(p)  # top and bottom swapped (the cone upside down)
        q['cylinderCircumferenceTop'], q['cylinderCircumferenceBottom'] = (
            p['cylinderCircumferenceBottom'], p['cylinderCircumferenceTop'])
        out['swapTB'] = q
        q = dict(p)  # cone described as a cylinder
        q['cylinderCircumferenceBottom'] = p['cylinderCircumferenceTop']
        out['cone2cyl'] = q
    else:
        q = dict(p)  # cylinder described as a cone with base 1.5
        q['cylinderCircumferenceBottom'] = p['cylinderCircumferenceTop'] / 1.5
        out['cyl2cone'] = q
    return out


def curvy_expected_geometry(model):
    """The 9 fields of CurvyGeometry the engine reports (DetectionImage::toCurvyGeometry meaning)
    for an uncropped label, in model units (cropped height 1): independent of engine code."""
    rt, rb = model['rt'], model['rb']
    arc = model['right'] - model['left']
    return {
        'curvyCircumferenceTop': 2 * math.pi * rt,
        'curvyCircumferenceBottom': 2 * math.pi * rb,
        'curvySideLength': math.sqrt((rt - rb) ** 2 + 1.0),
        'targetCircumferenceTop': 2 * math.pi * rt * arc,
        'height': 1.0,
        'topRadius': rt,
        'bottomRadius': rb,
        'arcLengthRadians': 2 * math.pi * arc,
        'arcStartRadians': 2 * math.pi * model['left'],
    }


def curvy_crop_case(variant=None):
    """A cropped cylinder target (geometry only, no rendering): the label (the original image,
    800 x 900 px) wraps a cylinder; the tracked image is its 480 x 640 crop at (160, 120). Independent
    of the engine, from the physical statement: the crop is a rectangle of the printed label, so
      model unit = physical height of the crop, r = C / (2 pi h_crop),
      activation region left / right = the crop's columns as a fraction of the label arc,
      the reported CurvyGeometry (DetectionImage::toCurvyGeometry) describes the FULL label:
      height = H_o / h (label height in crop-height units), arc = the full label arc."""
    shape = variant_data(variant)['shapes']['cylinder']
    c, arc = shape['circumference'], shape['arc']
    wo, ho, x, y, w, h = 800, 900, 160, 120, 480, 640
    wt_full = arc * c
    ht_full = wt_full * ho / wo
    h_crop = ht_full * h / ho
    r = c / (2 * math.pi * h_crop)
    left = 0.5 - arc / 2.0 + arc * x / wo
    right = left + arc * w / wo
    props = {'targetCircumferenceTop': wt_full, 'cylinderCircumferenceTop': c,
             'cylinderCircumferenceBottom': c, 'cylinderSideLength': ht_full,
             'arcAngle': arc * 360.0, 'coniness': 0, 'inputMode': 'ADVANCED', 'unit': 'mm'}
    height_out = ho / h
    expected = {
        'curvyCircumferenceTop': 2 * math.pi * r, 'curvyCircumferenceBottom': 2 * math.pi * r,
        'curvySideLength': height_out, 'targetCircumferenceTop': 2 * math.pi * r * arc,
        'height': height_out, 'topRadius': r, 'bottomRadius': r,
        'arcLengthRadians': 2 * math.pi * arc, 'arcStartRadians': 2 * math.pi * (0.5 - arc / 2.0)}
    json_props = {'left': x, 'top': y, 'width': w, 'height': h, 'isRotated': False,
                  'originalWidth': wo, 'originalHeight': ho}
    json_props.update(props)
    return {'json_props': json_props, 'rt': r, 'rb': r, 'left': left, 'right': right, 'expected': expected}


def surface_point(model, col, row):
    """Model-frame point of label pixel (col, row)."""
    f = model['left'] + (model['right'] - model['left']) * np.asarray(col, dtype=np.float64) / (model['w0'] - 1)
    y = 0.5 - np.asarray(row, dtype=np.float64) / (model['h0'] - 1)
    rho = model['rb'] + (model['rt'] - model['rb']) * (y + 0.5)
    ang = 2 * math.pi * f
    return np.stack([-rho * np.sin(ang), y, rho * np.cos(ang)], axis=-1)


def cast_curvy(model, origin, dirs):
    """First intersection of rays origin + s * dirs (model frame, dirs (..., 3)) with the surface.

    Returns (hit, outer, point): hit = the ray meets the surface at |y| <= 0.5, outer = it meets the
    outside (printed side) first. The surface is x^2 + z^2 = rho(y)^2, rho(y) = rho0 + k y.
    """
    k = model['rt'] - model['rb']
    rho0 = 0.5 * (model['rt'] + model['rb'])
    ox, oy, oz = origin
    dx, dy, dz = dirs[..., 0], dirs[..., 1], dirs[..., 2]
    q = rho0 + k * oy
    a = dx * dx + dz * dz - k * k * dy * dy
    b = ox * dx + oz * dz - k * dy * q
    c = ox * ox + oz * oz - q * q
    disc = b * b - a * c
    ok = (disc >= 0) & (np.abs(a) > 1e-12)
    sq = np.sqrt(np.where(ok, disc, 0.0))
    with np.errstate(divide='ignore', invalid='ignore'):
        s1 = (-b - sq) / a
        s2 = (-b + sq) / a
    best = np.full(dx.shape, np.inf)
    for s in (s1, s2):
        y = oy + s * dy
        rho = rho0 + k * y
        valid = ok & (s > 1e-9) & (np.abs(y) <= 0.5 + 1e-9) & (rho > 1e-6)
        best = np.where(valid & (s < best), s, best)
    hit = np.isfinite(best)
    s = np.where(hit, best, 0.0)
    px, py, pz = ox + s * dx, oy + s * dy, oz + s * dz
    rho = rho0 + k * py
    # outward normal = gradient of x^2 + z^2 - rho(y)^2 = (x, -rho k, z); outside hit <=> n . d < 0
    outer = hit & ((px * dx - rho * k * dy + pz * dz) < 0)
    return hit, outer, np.stack([px, py, pz], axis=-1)


def label_coordinates(model, point):
    """(col, row, inside) of a model-frame surface point on the label image."""
    ang = np.arctan2(-point[..., 0], point[..., 2]) % (2 * math.pi)
    f = ang / (2 * math.pi)
    col = (f - model['left']) / (model['right'] - model['left']) * (model['w0'] - 1)
    row = (0.5 - point[..., 1]) * (model['h0'] - 1)
    inside = (col >= 0) & (col <= model['w0'] - 1) & (row >= 0) & (row <= model['h0'] - 1)
    return col, row, inside


def bilinear_texture(tex, col, row):
    h, w = tex.shape
    c = np.clip(col, 0, w - 1)
    r = np.clip(row, 0, h - 1)
    c0 = np.clip(np.floor(c).astype(int), 0, w - 2)
    r0 = np.clip(np.floor(r).astype(int), 0, h - 2)
    fc, fr = c - c0, r - r0
    t = tex.astype(np.float64)
    return (t[r0, c0] * (1 - fc) * (1 - fr) + t[r0, c0 + 1] * fc * (1 - fr) +
            t[r0 + 1, c0] * (1 - fc) * fr + t[r0 + 1, c0 + 1] * fc * fr)


def render_curvy(texture, model, cam_k, r, t, visible, rng, noise=1.5):
    fx, fy, cx, cy = cam_k
    acc = np.zeros((H, W))
    gy, gx = np.mgrid[0:H, 0:W].astype(np.float64)
    background = 90.0 + 30.0 * gx / W + 20.0 * gy / H
    for oy in (-0.25, 0.25):
        for ox in (-0.25, 0.25):
            sample = background.copy()
            if visible:
                u, v = gx + ox, gy + oy
                d = np.stack([(u - cx) / fx, -(v - cy) / fy, np.ones_like(u)], axis=-1) @ r.T
                hit, outer, pt = cast_curvy(model, t, d)
                col, row, inside = label_coordinates(model, pt)
                lab = bilinear_texture(texture, col, row)
                val = np.where(outer & inside, lab, np.where(outer, BODY_GRAY, INTERIOR_GRAY))
                sample = np.where(hit, val, sample)
            acc += sample
    img = acc / 4.0
    if noise > 0:
        img = img + rng.normal(0.0, noise, size=(H, W))
    return np.clip(np.rint(img), 0, 255).astype(np.uint8)


def camera_pose_curvy(k, segment_start, visible, variant=None):
    """Camera-to-model rotation R and position t of frame k (curved target sequences)."""
    if not visible:
        pos = np.array([0.0, 0.0, -2.1])
        z = np.array([1.0, 0.0, 0.05])
        z = z / np.linalg.norm(z)
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
        x /= np.linalg.norm(x)
        return np.stack([x, np.cross(z, x), z], axis=1), pos
    tr = variant_data(variant)['traj']
    ph = tr['phase']
    tau = (k - segment_start) / FPS
    c0 = tr['c0'][segment_start]

    def s(c):
        return math.sin(2 * math.pi * (tau / tr['period'] + c0 + c))

    psi = math.radians(35.0) * s(ph['azimuth'])     # azimuth around the axis, 0 = in front of the label centre
    dist = 2.1 + 0.3 * s(ph['distance'])
    pos = np.array([dist * math.sin(psi), 0.3 * s(ph['height']), -dist * math.cos(psi)])
    pert = (rot_axis('y', -4.5 * s(ph['yaw'])) @ rot_axis('x', 4.5 * s(ph['pitch'])) @ rot_axis('z', 8.5 * s(ph['roll'])))
    return look_at(pos, (0.0, 0.0, 0.0)) @ pert, pos


def curvy_trajectory_quantities(r, t):
    """(azimuth, y, distance, yaw, pitch, roll): azimuth / y / distance of the camera position around the
    axis (degrees, model units); yaw / pitch / roll = the rotation of the camera against the look-at-the-axis
    frame (trajectory_quantities of that relative rotation)."""
    rel = look_at(t, (0.0, 0.0, 0.0)).T @ r
    _, _, _, yaw, pitch, roll = trajectory_quantities(rel, np.zeros(3))
    return (math.degrees(math.atan2(t[0], -t[2])), t[1], math.hypot(t[0], t[2]), yaw, pitch, roll)


# (name, nominal centre, amplitude, hard bound or None) of curvy_trajectory_quantities.
CURVY_TRAJ_RANGES = [('azimuth', 0.0, 35.0, 37.0), ('y', 0.0, 0.3, 0.31), ('distance', 2.1, 0.3, None),
                     ('yaw', 0.0, 4.5, 5.5), ('pitch', 0.0, 4.5, 5.5), ('roll', 0.0, 8.0, 9.0)]


def check_curvy_trajectory_ranges(poses):
    q = np.array([curvy_trajectory_quantities(r, t) for r, t in poses])
    ok = True
    lines = []
    for i, (name, centre, amp, bound) in enumerate(CURVY_TRAJ_RANGES):
        lo, hi = float(q[:, i].min()), float(q[:, i].max())
        reach = lo <= centre - 0.9 * amp and hi >= centre + 0.9 * amp
        inside = True
        if bound is not None:
            inside = lo >= centre - bound and hi <= centre + bound
        if name == 'distance':
            inside = lo >= 1.79 and hi <= 2.41
        lines.append(f'{name}: min {lo:+.4f} max {hi:+.4f} (need <= {centre - 0.9 * amp:+.3f} and '
                     f'>= {centre + 0.9 * amp:+.3f}) {"OK" if reach and inside else "VIOLATION"}')
        ok = ok and reach and inside
    return lines, ok


def main_curvy(a):
    shape = a.shape
    variant = a.variant
    if variant == 'registered':
        try:
            print('coordinator seed file sha256 ' + use_seed_file(a.seed_file))
        except (ValueError, OSError) as e:
            print(f'cannot create the registered variant: {e}', file=sys.stderr)
            return 2
    set_variant(variant)
    spec = variant_data()['shapes'][shape]
    name = spec['name']
    model = curvy_model(shape)
    poses_v = [camera_pose_curvy(k, start, True, variant)
               for _, start, end, visible in SEGMENTS if visible for k in range(start, end)]
    lines, ok = check_curvy_trajectory_ranges(poses_v)
    print('\n'.join(lines))
    print(f'variant {variant}: ' + json.dumps({'shape': spec, 'seed': variant_data()['seeds'][shape]}))
    if a.check_only:
        return 0 if ok else 1
    if not ok:
        print(f'the {variant} trajectory violates the G-TRAJ range check', file=sys.stderr)
        return 3
    if variant == 'registered' and a.seed is not None:
        print('the registered variant takes its seed from the coordinator seed (no --seed)', file=sys.stderr)
        return 2
    if os.path.isdir(a.out) and os.listdir(a.out) and not a.force:
        print(f'refusing to write into non-empty {a.out} (use --force)', file=sys.stderr)
        return 2
    os.makedirs(os.path.join(a.out, 'frames'), exist_ok=True)
    seed = a.seed if a.seed is not None else variant_data()['seeds'][shape]
    rng = np.random.default_rng(seed)
    texture = make_texture(rng)
    write_pgm(os.path.join(a.out, f'{name}_luminance.pgm'), texture)
    with open(os.path.join(a.out, f'{name}.json'), 'w') as f:
        json.dump(curvy_target_json(model, name, spec['type']), f, indent=2)
    for mid, props in curvy_json_mutations(model).items():
        with open(os.path.join(a.out, f'{name}.mut-{mid}.json'), 'w') as f:
            json.dump(curvy_target_json(model, name, spec['type'], props), f, indent=2)
    if shape == 'cylinder':  # geometry-only cropped case (G-CGEO-crop)
        crop = curvy_crop_case()
        with open(os.path.join(a.out, f'{name}-crop.json'), 'w') as f:
            json.dump({'type': spec['type'], 'properties': crop['json_props'],
                       'imagePath': f'image-targets/{name}_luminance.png', 'metadata': None,
                       'name': f'{name}-crop', 'resources': {'luminanceImage': f'{name}_luminance.png'},
                       'created': 0, 'updated': 0}, f, indent=2)
        with open(os.path.join(a.out, 'geometry-crop.json'), 'w') as f:
            json.dump({'variant': variant, 'shape': 'cylinder-crop', 'rt': crop['rt'], 'rb': crop['rb'], 'left': crop['left'],
                       'right': crop['right'], 'w0': 480, 'h0': 640,
                       'expected_curvy_geometry': crop['expected']}, f, indent=1)
    with open(os.path.join(a.out, 'geometry.json'), 'w') as f:
        json.dump({'variant': variant, 'shape': shape, 'rt': model['rt'], 'rb': model['rb'], 'left': model['left'],
                   'right': model['right'], 'w0': model['w0'], 'h0': model['h0'],
                   'expected_curvy_geometry': curvy_expected_geometry(model)}, f, indent=1)
    frames = []
    noise_rng = np.random.default_rng(seed + 1)
    for seg, start, end, visible in SEGMENTS:
        for k in range(start, end):
            r, t = camera_pose_curvy(k, start, visible, variant)
            img = render_curvy(texture, model, a.camera_k, r, t, visible, noise_rng)
            write_pgm(os.path.join(a.out, 'frames', f'{k:04d}.pgm'), img)
            pose = np.eye(4)
            pose[:3, :3] = r
            pose[:3, 3] = t
            frames.append({'index': k, 'segment': seg, 'visible': visible,
                           'time_ns': int(round(1e9 + k * 1e9 / FPS)),
                           'camera_to_model': pose.reshape(-1).tolist()})
    gt = {'name': name, 'shape': shape, 'width': W, 'height': H, 'camera_k': a.camera_k,
          'target_k': a.target_k, 'seed': seed, 'variant': variant,
          'segments': [list(s) for s in SEGMENTS], 'frames': frames}
    with open(os.path.join(a.out, 'gt.json'), 'w') as f:
        json.dump(gt, f, indent=1)
    with open(os.path.join(a.out, 'gt.tsv'), 'w') as f:
        f.write('camera_k\t' + '\t'.join(repr(float(v)) for v in a.camera_k) + '\n')
        f.write('target_k\t' + '\t'.join(repr(float(v)) for v in a.target_k) + '\n')
        f.write('variant\t' + variant + '\n')
        f.write('shape\t' + shape + '\n')
        f.write('model\t' + '\t'.join(repr(float(model[x])) for x in ('rt', 'rb', 'left', 'right')) +
                f"\t{model['w0']}\t{model['h0']}\n")
        for fr in frames:
            f.write('\t'.join([str(fr['index']), str(int(fr['visible'])), str(fr['time_ns'])] +
                              [repr(float(v)) for v in fr['camera_to_model']]) + '\n')
    print(f'wrote {len(frames)} frames to {a.out}')
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--camera-k', type=parse_k, default=[506.4104, 506.4104, 239.5, 319.5])
    ap.add_argument('--target-k', type=parse_k, default=[506.4104, 506.4104, 239.5, 319.5])
    ap.add_argument('--seed', type=int, default=None,
                    help='default: 13 (flat), the variant seed (curved; the registered variant takes no --seed)')
    ap.add_argument('--variant', choices=['dev', 'registered'], default='dev',
                    help='curved targets only: dev (default; the exposed data, the only one an engine may run on before '
                         'the review) or registered (derived from the coordinator seed file, curvy-it-gate)')
    ap.add_argument('--seed-file', default=None,
                    help='curved targets, --variant registered: the coordinator seed file (no default; else the environment '
                         'variable ' + SEED_FILE_ENV + ')')
    ap.add_argument('--shape', choices=['flat', 'cylinder', 'cone'], default='flat',
                    help='flat (default, US-013) or a curved target (US-014)')
    ap.add_argument('--trajectory', choices=['registered', 'narrow'], default='registered',
                    help='flat targets: registered: G-TRAJ ranges (default); narrow: first-run trajectory (run1/run2)')
    ap.add_argument('--check-only', action='store_true',
                    help='only evaluate the G-TRAJ range check of the chosen trajectory (no rendering)')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args(argv)
    if a.shape != 'flat':
        return main_curvy(a)
    if a.seed is None:
        a.seed = 13
    if a.check_only:
        poses = [camera_pose(k, start, True, a.trajectory)
                 for _, start, end, visible in SEGMENTS if visible for k in range(start, end)]
        lines, ok = check_trajectory_ranges(poses)
        print('\n'.join(lines))
        return 0 if ok else 1
    if os.path.isdir(a.out) and os.listdir(a.out) and not a.force:
        print(f'refusing to write into non-empty {a.out} (use --force)', file=sys.stderr)
        return 2
    if a.trajectory == 'registered':
        poses = [camera_pose(k, start, True, a.trajectory)
                 for _, start, end, visible in SEGMENTS if visible for k in range(start, end)]
        lines, ok = check_trajectory_ranges(poses)
        print('\n'.join(lines))
        if not ok:
            print('registered trajectory violates the G-TRAJ range check', file=sys.stderr)
            return 3
    os.makedirs(os.path.join(a.out, 'frames'), exist_ok=True)
    rng = np.random.default_rng(a.seed)
    texture = make_texture(rng)
    write_pgm(os.path.join(a.out, f'{NAME}_luminance.pgm'), texture)
    target = {
        'type': 'PLANAR',
        'properties': {'left': 0, 'top': 0, 'width': W, 'height': H, 'isRotated': False,
                       'originalWidth': W, 'originalHeight': H},
        'imagePath': f'image-targets/{NAME}_luminance.png',
        'metadata': None,
        'name': NAME,
        'resources': {'luminanceImage': f'{NAME}_luminance.png'},
        'created': 0,
        'updated': 0,
    }
    with open(os.path.join(a.out, f'{NAME}.json'), 'w') as f:
        json.dump(target, f, indent=2)
    frames = []
    noise_rng = np.random.default_rng(a.seed + 1)
    for name, start, end, visible in SEGMENTS:
        for k in range(start, end):
            r, t = camera_pose(k, start, visible, a.trajectory)
            img = render(texture, a.camera_k, a.target_k, r, t, visible, noise_rng)
            write_pgm(os.path.join(a.out, 'frames', f'{k:04d}.pgm'), img)
            pose = np.eye(4)
            pose[:3, :3] = r
            pose[:3, 3] = t
            frames.append({'index': k, 'segment': name, 'visible': visible,
                           'time_ns': int(round(1e9 + k * 1e9 / FPS)),
                           'camera_to_target_view': pose.reshape(-1).tolist()})
    gt = {'name': NAME, 'width': W, 'height': H, 'camera_k': a.camera_k, 'target_k': a.target_k,
          'seed': a.seed, 'segments': [list(s) for s in SEGMENTS], 'frames': frames}
    if a.trajectory != 'narrow':
        gt['trajectory'] = a.trajectory  # the narrow (run1/run2) gt.json stays byte-identical
    with open(os.path.join(a.out, 'gt.json'), 'w') as f:
        json.dump(gt, f, indent=1)
    # Same ground truth as plain text for the C++ test: K lines, then one line per frame
    # (index, visible, time_ns, 16 row-major values of camera_to_target_view).
    with open(os.path.join(a.out, 'gt.tsv'), 'w') as f:
        f.write('camera_k\t' + '\t'.join(repr(float(v)) for v in a.camera_k) + '\n')
        f.write('target_k\t' + '\t'.join(repr(float(v)) for v in a.target_k) + '\n')
        for fr in frames:
            f.write('\t'.join([str(fr['index']), str(int(fr['visible'])), str(fr['time_ns'])] +
                              [repr(float(v)) for v in fr['camera_to_target_view']]) + '\n')
    print(f'wrote {len(frames)} frames to {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
