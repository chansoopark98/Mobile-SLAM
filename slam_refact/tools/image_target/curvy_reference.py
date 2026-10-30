#!/usr/bin/env python3
"""Independent numpy oracle of the curvy ROI draw (US-014, gate file curvy-it-gate.v1.md).

The CPU port under test is frontend::DrawRoisIntoAtlas for a CURVY_IMAGE_TARGET ROI (roi_draw.cc,
the curvy branch of gr8-stage1.frag). `prepare` writes, into --out, everything the C++ test needs:

  G-CROI-A  (implementation check against the shader semantics, any pose, any geometry)
            a_camera.pgm (random camera), a_case_<i>.txt (the case: geometry, K, pose, rotation),
            a_expected_<i>.pgm / a_mask_<i>.pgm (slot size; mask 255 = covered)
            The expected image is the GLSL of gr8-stage1.frag getCurvyLiftedUv + main written out in
            numpy (float32, shader operation order); the host-side uniforms (gl-reality-frame.cc
            :397-433) are derived here from the case, not read from the engine.
  G-CROI-B  (semantic check against geometry, independent of the shader formula)
            b_<shape>_geom.txt (rt rb left right w0 h0 of the physical model), b_<shape>_pose_<i>.txt (camera -> model
            R row-major, t), b_<shape>_camera_<i>.pgm (the
            target seen from that pose, rendered by ray casting), b_<shape>_expected_<i>.pgm (the ROI
            slot a correct lift must show: the target texture itself, where the surface point is
            visible and inside the camera image), b_<shape>_mask_<i>.pgm (the part of that mask seen
            within 60 degrees of the surface normal, eroded by 3 px: where the value is compared) and b_<shape>_cover_<i>.pgm (the uneroded mask: coverage)
            Rule (no shader formula): ROI pixel (x, y) shows target pixel ((x + 0.5) / w * roiW,
            (y + 0.5) / h * roiH); that texel sits at a known point of the cylinder / cone
            (render_synthetic.surface_point); the point is drawn iff it faces the camera and projects
            into the camera image; its value is the target texture there. The camera image is NOT
            used for the expectation.

`--variant dev|registered` selects the data (default dev: the exposed data, the only variant an engine may run on before
the independent review; registered: derived from the coordinator seed file, --seed-file or $SLAM_REFACT_CURVY_SEED_FILE, no default);
`variant.txt` records it.
`selftest` writes the numbers of the null-runs that need no engine (N4: float32 versus float64
shader model, N5: shader model on the rendered camera versus the geometric rule).

Only numpy is required (development tool).
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_synthetic as rs  # noqa: E402  (renderer, geometry and PGM writer of the synthetic gate)
import roi_reference as rr  # noqa: E402  (bilinear_clamp, quantise, camera_a of the planar oracle)

CAM_W, CAM_H = 480, 640
K = [506.4104, 506.4104, 239.5, 319.5]
SEED_B = None      # texture seed of G-CROI-B, set by use_variant()
ERODE_PX = 3


# ------------------------------------------------------------------------------------------------
# Shader model (gr8-stage1.frag + gr8-rotate.vert + the host uniforms of gl-reality-frame.cc)
# ------------------------------------------------------------------------------------------------
def host_uniforms(case, slot_w, slot_h):
    """gl-reality-frame.cc:397-433 for the case (geom srcRows / srcCols / left / right ...)."""
    rows, cols = case['src_rows'], case['src_cols']
    rotated = False
    if cols > rows:
        rotated = True
        rows, cols = cols, rows
    sx = (case['right'] - case['left']) / (cols - 1)
    sy = 1.0 / (rows - 1)
    aspect = slot_w / slot_h
    wide = cols / rows > aspect
    roi_rows = cols / aspect if wide else rows
    roi_cols = cols if wide else aspect * rows
    return {'rotation': 90 if rotated else 0, 'post': case['rotation_deg'],
            'target': (float(cols), float(rows)), 'roi': (float(roi_cols), float(roi_rows)),
            'search': (float(CAM_W), float(CAM_H)), 'sx': sx, 'sy': sy, 'shift': case['left'],
            'radius': case['radius'], 'radius_bottom': case['radius_bottom']}


def slot_shader_model(camera, case, slot_w, slot_h, dtype=np.float32):
    """The expected slot of one curvy ROI draw: (uint8 image, uint8 mask 255 = drawn)."""
    f = dtype
    u = host_uniforms(case, slot_w, slot_h)
    ys, xs = np.mgrid[0:slot_h, 0:slot_w]
    uvx = (xs.astype(f) + f(0.5)) / f(slot_w)
    uvy = (ys.astype(f) + f(0.5)) / f(slot_h)
    rot = u['rotation']
    if rot in (-90, 270):
        tx, ty = f(1.0) - uvy, uvx
    elif rot in (180, -180):
        tx, ty = f(1.0) - uvx, f(1.0) - uvy
    elif rot in (-270, 90):
        tx, ty = uvy, f(1.0) - uvx
    else:
        tx, ty = uvx, uvy
    ppx = tx * f(u['roi'][0])
    ppy = ty * f(u['roi'][1])
    inside_target = ~((ppx > f(u['target'][0])) | (ppy > f(u['target'][1])))
    radius_at_y = ty * f(u['radius_bottom']) + f(u['radius']) * (f(1.0) - ty)
    v_x = f(2.0) * f(math.pi) * (f(u['shift']) + f(u['sx']) * ppx)
    sin_v, cos_v = np.sin(v_x), np.cos(v_x)
    # model -> camera: X_cam = R^T (X_model - t)
    r = np.asarray(case['R'], dtype=f).reshape(3, 3)
    t = np.asarray(case['t'], dtype=f)
    nrm = np.stack([-radius_at_y * sin_v,
                    radius_at_y * (f(u['radius_bottom']) - f(u['radius'])),
                    radius_at_y * cos_v], axis=-1)
    tn = nrm @ r  # (R^T n) row-vector form
    pt = np.stack([-radius_at_y * sin_v, -f(u['sy']) * ppy + f(0.5), radius_at_y * cos_v], axis=-1)
    ray3 = (pt - t) @ r
    with np.errstate(divide='ignore', invalid='ignore'):
        one_over_z = f(1.0) / ray3[..., 2]
        ray_x = ray3[..., 0] * one_over_z
        ray_y = ray3[..., 1] * one_over_z
        fx, fy, cx, cy = [f(v) for v in case['K']]
        pix_x = (fx * ray3[..., 0] + cx * ray3[..., 2]) * one_over_z
        pix_y = (-fy * ray3[..., 1] + cy * ray3[..., 2]) * one_over_z
        facing = (tn[..., 0] * ray_x + tn[..., 1] * ray_y + tn[..., 2] * f(1.0)) < 0
        post = u['post']
        sw, sh = f(u['search'][0]), f(u['search'][1])
        if post in (-90, 270):
            lx, ly = f(1.0) - pix_y / sh, pix_x / sw
        elif post in (180, -180):
            lx, ly = f(1.0) - pix_x / sw, f(1.0) - pix_y / sh
        elif post in (-270, 90):
            lx, ly = pix_y / sh, f(1.0) - pix_x / sw
        else:
            lx, ly = pix_x / sw, pix_y / sh
    drawn = (inside_target & facing & np.isfinite(lx) & np.isfinite(ly) &
             (lx >= 0) & (lx <= 1) & (ly >= 0) & (ly <= 1))
    # texture(sampler, liftedUv * pixScale / roiSrc size) with pixScale = roiSrc size: LINEAR, clamp
    tcx = np.where(drawn, lx, f(0.0)).astype(np.float64) * CAM_W - 0.5
    tcy = np.where(drawn, ly, f(0.0)).astype(np.float64) * CAM_H - 0.5
    out = rr.quantise(rr.bilinear_clamp(camera, tcx, tcy, np.float64))
    out = np.where(drawn, out, 0).astype(np.uint8)
    return out, np.where(drawn, 255, 0).astype(np.uint8)


# ------------------------------------------------------------------------------------------------
# Geometric rule (G-CROI-B)
# ------------------------------------------------------------------------------------------------
def erode(mask_bool, n):
    out = mask_bool.copy()
    for _ in range(n):
        p = np.pad(out, 1, mode='constant', constant_values=False)
        out = (p[1:-1, 1:-1] & p[:-2, 1:-1] & p[2:, 1:-1] & p[1:-1, :-2] & p[1:-1, 2:])
    return out


COS_STRONG = 0.5   # the value comparison keeps surface points seen within 60 degrees of their normal


def geometric_expected(texture, model, r, t, slot_w, slot_h):
    """The ROI slot a correct (zero-margin) lift shows: (uint8 image, bool drawn mask, bool strong mask).

    strong = drawn and the view direction is within 60 degrees of the surface normal: the camera
    resolves those texels (a glancing surface compresses many texels into one camera pixel, where any
    sub-pixel convention would dominate the value comparison)."""
    aspect = slot_w / slot_h
    w0, h0 = model['w0'], model['h0']
    wide = w0 / h0 > aspect
    roi_rows = w0 / aspect if wide else h0
    roi_cols = w0 if wide else aspect * h0
    ys, xs = np.mgrid[0:slot_h, 0:slot_w].astype(np.float64)
    px = (xs + 0.5) / slot_w * roi_cols
    py = (ys + 0.5) / slot_h * roi_rows
    in_target = (px <= w0) & (py <= h0)
    col = np.clip(px, 0, w0 - 1)
    row = np.clip(py, 0, h0 - 1)
    p = rs.surface_point(model, col, row)
    k = model['rt'] - model['rb']
    rho = model['rb'] + (model['rt'] - model['rb']) * (p[..., 1] + 0.5)
    n = np.stack([p[..., 0], -rho * k, p[..., 2]], axis=-1)
    to_cam = np.asarray(t) - p
    facing = (n * to_cam).sum(-1) > 0
    with np.errstate(divide='ignore', invalid='ignore'):
        cos_view = (n * to_cam).sum(-1) / (np.linalg.norm(n, axis=-1) * np.linalg.norm(to_cam, axis=-1))
    pc = (p - np.asarray(t)) @ np.asarray(r)  # R^T (P - t)
    with np.errstate(divide='ignore', invalid='ignore'):
        u = K[0] * pc[..., 0] / pc[..., 2] + K[2]
        v = K[3] - K[1] * pc[..., 1] / pc[..., 2]
    inside = (pc[..., 2] > 0) & (u >= 0) & (u <= CAM_W) & (v >= 0) & (v <= CAM_H)
    mask = in_target & facing & inside
    val = rs.bilinear_texture(texture, col, row)
    return np.where(mask, rr.quantise(val), 0).astype(np.uint8), mask, mask & (cos_view >= COS_STRONG)


def smooth_texture_b():
    """Low-frequency asymmetric 480x640 label texture, every value >= 16 (0 means not drawn)."""
    rng = np.random.default_rng(SEED_B)
    coarse = rng.uniform(-110, 110, size=(7, 5))
    gy, gx = np.mgrid[0:CAM_H, 0:CAM_W].astype(np.float64)
    cy = gy / (CAM_H - 1) * (coarse.shape[0] - 1)
    cx = gx / (CAM_W - 1) * (coarse.shape[1] - 1)
    y0 = np.clip(np.floor(cy).astype(int), 0, coarse.shape[0] - 2)
    x0 = np.clip(np.floor(cx).astype(int), 0, coarse.shape[1] - 2)
    fy, fx = cy - y0, cx - x0
    base = (coarse[y0, x0] * (1 - fx) * (1 - fy) + coarse[y0, x0 + 1] * fx * (1 - fy) +
            coarse[y0 + 1, x0] * (1 - fx) * fy + coarse[y0 + 1, x0 + 1] * fx * fy)
    return np.clip(np.rint(128.0 + base), 16, 255).astype(np.uint8)


def pose_from(azimuth_deg, y, dist, roll_deg, yaw_deg=0.0, pitch_deg=0.0):
    """Camera looking at the axis point (0, 0, 0) from azimuth / height / distance, then turned by the look-at yaw /
    pitch (as the trajectory does: Ry(yaw) Rx(pitch) Rz(roll))."""
    psi = math.radians(azimuth_deg)
    pos = np.array([dist * math.sin(psi), y, -dist * math.cos(psi)])
    return rs.look_at(pos, (0.0, 0.0, 0.0)) @ (rs.rot_axis('y', yaw_deg) @ rs.rot_axis('x', pitch_deg) @
                                                rs.rot_axis('z', roll_deg)), pos


def use_variant(variant, seed_file=None):
    """Select the data variant (render_synthetic.VARIANTS): shape, poses, texture seed and random camera seed.
    dev = the exposed data (the only variant an engine may run on before the review); registered = derived from the
    coordinator seed file (seed_file, else $SLAM_REFACT_CURVY_SEED_FILE; no default, curvy-it-gate)."""
    global SEED_B
    if variant == 'registered':
        print('coordinator seed file sha256 ' + rs.use_seed_file(seed_file))
    rs.set_variant(variant)
    v = rs.variant_data(variant)
    SEED_B = v['texture_seed_b']
    rr.SEED_A = v['camera_seed_a']


def b_poses():
    return rs.variant_data()['b_poses']


use_variant('dev')


def cases_a():
    """Arbitrary-geometry cases of G-CROI-A (not tied to a rendered scene)."""
    cyl = rs.curvy_model('cylinder')
    cone = rs.curvy_model('cone')
    (r0, p0), (r1, p1), (r2, p2), (r3, p3) = [pose_from(*q) for q in rs.variant_data()['a_poses']]
    return [
        # cylinder, portrait target, no rotation
        dict(radius=cyl['rt'], radius_bottom=cyl['rb'], src_rows=640, src_cols=480, left=cyl['left'],
             right=cyl['right'], K=K, R=r0.reshape(-1).tolist(), t=p0.tolist(), rotation_deg=0),
        # cone, rolled, off axis
        dict(radius=cone['rt'], radius_bottom=cone['rb'], src_rows=640, src_cols=480,
             left=cone['left'], right=cone['right'], K=K, R=r1.reshape(-1).tolist(), t=p1.tolist(),
             rotation_deg=0),
        # landscape target (geometry rows < cols): the swap and the vertex rotation 90
        dict(radius=cyl['rt'], radius_bottom=cyl['rb'], src_rows=480, src_cols=640, left=cyl['left'],
             right=cyl['right'], K=K, R=r2.reshape(-1).tolist(), t=p2.tolist(), rotation_deg=0),
        # post rotation 90 (engine rotation_deg = 90)
        dict(radius=cone['rt'], radius_bottom=cone['rb'], src_rows=640, src_cols=480,
             left=cone['left'], right=cone['right'], K=K, R=r3.reshape(-1).tolist(), t=p3.tolist(),
             rotation_deg=90),
    ]


def write_case(path, case):
    with open(path, 'w') as f:
        f.write(' '.join(repr(float(v)) for v in [case['radius'], case['radius_bottom'], case['src_rows'],
                                                  case['src_cols'], case['left'], case['right'],
                                                  case['rotation_deg']]) + '\n')
        f.write(' '.join(repr(float(v)) for v in case['K']) + '\n')
        f.write(' '.join(repr(float(v)) for v in list(case['R']) + list(case['t'])) + '\n')


def prepare(out, slot_w, slot_h, camera_kind):
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'variant.txt'), 'w') as f:  # the C++ tests assert it (dev vs registered separation)
        f.write(rs.ACTIVE_VARIANT + '\n')
    cam = rr.camera_a(camera_kind)
    rs.write_pgm(os.path.join(out, 'a_camera.pgm'), cam)
    for i, case in enumerate(cases_a()):
        write_case(os.path.join(out, f'a_case_{i}.txt'), case)
        exp, mask = slot_shader_model(cam, case, slot_w, slot_h, np.float32)
        rs.write_pgm(os.path.join(out, f'a_expected_{i}.pgm'), exp)
        rs.write_pgm(os.path.join(out, f'a_mask_{i}.pgm'), mask)
    if camera_kind != 'textured':
        return
    texture = smooth_texture_b()
    rs.write_pgm(os.path.join(out, 'b_texture.pgm'), texture)
    for shape in ('cylinder', 'cone'):
        model = rs.curvy_model(shape)
        with open(os.path.join(out, f'b_{shape}_geom.txt'), 'w') as f:  # rt rb left right w0 h0
            f.write(' '.join(repr(float(model[x])) for x in ('rt', 'rb', 'left', 'right', 'w0', 'h0')) + '\n')
        for i, q in enumerate(b_poses()):
            r, t = pose_from(*q)
            with open(os.path.join(out, f'b_{shape}_pose_{i}.txt'), 'w') as f:
                f.write(' '.join(repr(float(v)) for v in list(r.reshape(-1)) + list(t)) + '\n')
            camera = rs.render_curvy(texture, model, K, r, t, True, np.random.default_rng(0), noise=0.0)
            rs.write_pgm(os.path.join(out, f'b_{shape}_camera_{i}.pgm'), camera)
            exp, mask, strong = geometric_expected(texture, model, r, t, slot_w, slot_h)
            rs.write_pgm(os.path.join(out, f'b_{shape}_expected_{i}.pgm'), exp)
            rs.write_pgm(os.path.join(out, f'b_{shape}_cover_{i}.pgm'), np.where(mask, 255, 0).astype(np.uint8))
            rs.write_pgm(os.path.join(out, f'b_{shape}_mask_{i}.pgm'),
                         np.where(erode(strong, ERODE_PX), 255, 0).astype(np.uint8))


def normalised_rms(actual, expected, mask):
    m = mask > 0
    e = expected[m].astype(np.float64)
    d = actual[m].astype(np.float64) - e
    return float(math.sqrt((d * d).mean()) / math.sqrt(max(e.var(), 1e-12)))


def selftest(out, slot_w, slot_h):
    """N4 (float32 vs float64 shader model, G-CROI-A cases) and N5 (shader model on the rendered camera
    vs the geometric rule, G-CROI-B poses): numbers only, no engine."""
    os.makedirs(out, exist_ok=True)
    cam = rr.camera_a('textured')
    a_max, a_mean, a_cov = 0, 0.0, 0.0
    for case in cases_a():
        e32, m32 = slot_shader_model(cam, case, slot_w, slot_h, np.float32)
        e64, m64 = slot_shader_model(cam, case, slot_w, slot_h, np.float64)
        both = (m32 > 0) & (m64 > 0)
        diff = np.abs(e32.astype(int) - e64.astype(int))[both]
        a_max = max(a_max, int(diff.max()) if diff.size else 0)
        a_mean = max(a_mean, float(diff.mean()) if diff.size else 0.0)
        a_cov = max(a_cov, float(((m32 > 0) != (m64 > 0)).mean()))
    texture = smooth_texture_b()
    lines = [f'a_max={a_max}', f'a_mean={a_mean:.6f}', f'a_cov={a_cov:.6f}']
    worst_e, worst_cov = 0.0, 0.0
    for shape in ('cylinder', 'cone'):
        model = rs.curvy_model(shape)
        for i, q in enumerate(b_poses()):
            r, t = pose_from(*q)
            camera = rs.render_curvy(texture, model, K, r, t, True, np.random.default_rng(0), noise=0.0)
            exp, mask, strong = geometric_expected(texture, model, r, t, slot_w, slot_h)
            case = dict(radius=model['rt'], radius_bottom=model['rb'], src_rows=model['h0'],
                        src_cols=model['w0'], left=model['left'], right=model['right'], K=K,
                        R=r.reshape(-1).tolist(), t=t.tolist(), rotation_deg=0)
            sh, shm = slot_shader_model(camera, case, slot_w, slot_h, np.float32)
            er = erode(strong, ERODE_PX)
            e = normalised_rms(sh, exp, np.where(er, 255, 0).astype(np.uint8))
            cov = float(((shm > 0) != mask).mean())
            lines.append(f'n5_{shape}_{i}_e={e:.6f} n5_{shape}_{i}_cov={cov:.6f} area={float(mask.mean()):.4f}')
            worst_e, worst_cov = max(worst_e, e), max(worst_cov, cov)
    lines.append(f'b_e={worst_e:.6f}')
    lines.append(f'b_cov={worst_cov:.6f}')
    text = '\n'.join(lines) + '\n'
    with open(os.path.join(out, 'selftest.txt'), 'w') as f:
        f.write(text)
    print(text)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--out', required=True)
    p.add_argument('--slot-w', type=int, required=True)
    p.add_argument('--slot-h', type=int, required=True)
    p.add_argument('--camera-kind', choices=['textured', 'constant', 'ramp'], default='textured')
    p.add_argument('--variant', choices=['dev', 'registered'], default='dev',
                   help='dev (default; the exposed data, the only variant an engine may run on before the review) or '
                        'registered (derived from the coordinator seed file; curvy-it-gate)')
    p.add_argument('--seed-file', default=None,
                   help='registered: the coordinator seed file (no default; else $SLAM_REFACT_CURVY_SEED_FILE)')
    s = sub.add_parser('selftest')
    s.add_argument('--variant', choices=['dev', 'registered'], default='dev')
    s.add_argument('--seed-file', default=None)
    s.add_argument('--out', required=True)
    s.add_argument('--slot-w', type=int, required=True)
    s.add_argument('--slot-h', type=int, required=True)
    a = ap.parse_args(argv)
    try:
        use_variant(a.variant, a.seed_file)
    except (ValueError, OSError) as e:
        print(f'cannot create the registered variant: {e}', file=sys.stderr)
        return 2
    if a.cmd == 'prepare':
        prepare(a.out, a.slot_w, a.slot_h, a.camera_kind)
    else:
        selftest(a.out, a.slot_w, a.slot_h)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
