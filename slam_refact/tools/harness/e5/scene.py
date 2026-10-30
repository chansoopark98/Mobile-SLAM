#!/usr/bin/env python3
"""E5 synthetic sequence: a textured room corner (front wall carrying the image target, left wall, table) rendered
from a smooth handheld-like camera trajectory, plus an IMU stream derived analytically from the same trajectory.

Package H (US-015 criterion 2'). Development tool; numpy + Pillow only. Nothing here is product runtime.

  python3 -I scene.py --out <new dir> --seed <int> [--frames N] [--variant dev|reg]

Writes (never into an existing non-empty directory):
  input.json                 fixture 'mobile-slam-synthetic-vio-v1' (what tools/re/runtime run-reference reads: tum kind)
  mav0/cam0/data/NNNNN.pgm   480x640 gray frames
  target/e5-target.json, e5-target_luminance.{pgm,png}   the image target (PLANAR, the embedded pixels)
  gt.json / gt.tsv           ground truth (camera pose in the room frame, target visibility per frame)

Conventions
  room frame: X right, Y into the front wall, Z up; gravity (0,0,-9.81).
  camera frame: x right, y down, z forward (neutral camera: x_c=+X, y_c=-Z, z_c=+Y), pinhole fx=fy=506.41 (the intrinsics
  the original SDK reports for a 480x640 capture, build/.../flip6-s2/r1/meta.json), cx=239.5, cy=319.5.
  IMU body frame = the TUM fixture convention: b = R_ic c with R_ic=[1,0,0;0,0,1;0,-1,0] (sensor_adapter.mjs
  R_DEVICE_FROM_BODY then gives the W3C device frame). accel = specific force (includes gravity), gyro rad/s.
"""
import argparse, hashlib, json, math, os, sys
import numpy as np
from PIL import Image

W, H = 480, 640
FX = FY = 506.4103889465332
CX, CY = 239.5, 319.5
G = 9.81
FPS = 30.0
IMU_HZ = 100.0
PPM = 800.0                      # texture pixels per metre (target 0.6 x 0.8 m = 480 x 640 px, 1:1)
D_WALL, X_LEFT, Z_TABLE = 1.1, -1.6, -0.3
R_IC = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
R_NEUTRAL = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)  # columns x_c=(1,0,0), y_c=(0,0,-1), z_c=(0,1,0)


def smooth(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * x * (x * (6 * x - 15) + 10)


def pulse(t, a0, a1, b0, b1):
    """0 -> 1 on [a0,a1], 1 on [a1,b0], 1 -> 0 on [b0,b1] (smootherstep)."""
    return smooth((t - a0) / (a1 - a0)) * (1.0 - smooth((t - b0) / (b1 - b0)))


def make_texture(rng, h, w, scale):
    """Feature-rich gray texture; `scale` stretches the blob sizes (pixels)."""
    gy, gx = np.mgrid[0:h, 0:w].astype(np.float32)
    coarse = rng.uniform(-45, 45, size=(h // 160 + 2, w // 160 + 2)).astype(np.float32)
    cy = gy / 160.0; cx = gx / 160.0
    y0 = np.clip(cy.astype(int), 0, coarse.shape[0] - 2); x0 = np.clip(cx.astype(int), 0, coarse.shape[1] - 2)
    fy, fx = cy - y0, cx - x0
    img = 128.0 + (coarse[y0, x0] * (1 - fx) * (1 - fy) + coarse[y0, x0 + 1] * fx * (1 - fy) +
                   coarse[y0 + 1, x0] * (1 - fx) * fy + coarse[y0 + 1, x0 + 1] * fx * fy)
    n_rect = int(h * w / 8000)
    for _ in range(n_rect):
        rw, rh = rng.integers(int(8 * scale), int(70 * scale), size=2)
        x, y = rng.integers(0, max(1, w - rw)), rng.integers(0, max(1, h - rh))
        img[y:y + rh, x:x + rw] = rng.uniform(10, 245)
    for _ in range(n_rect // 2):
        r = rng.uniform(4 * scale, 30 * scale)
        x, y = rng.uniform(0, w), rng.uniform(0, h)
        x0_, x1_, y0_, y1_ = int(max(0, x - r)), int(min(w, x + r + 1)), int(max(0, y - r)), int(min(h, y + r + 1))
        sub = (gx[y0_:y1_, x0_:x1_] - x) ** 2 + (gy[y0_:y1_, x0_:x1_] - y) ** 2 <= r * r
        img[y0_:y1_, x0_:x1_][sub] = rng.uniform(10, 245)
    p = np.pad(img, 1, mode='edge')
    img = sum(p[dy:dy + h, dx:dx + w] for dy in range(3) for dx in range(3)) / 9.0
    return np.clip(img, 0, 255).astype(np.float32)


def make_target(seed):
    """The image target texture: 480x640, same recipe family as render_synthetic.make_texture but own seed."""
    rng = np.random.default_rng(seed)
    return np.clip(np.rint(make_texture(rng, H, W, 1.0)), 0, 255).astype(np.uint8)


# ----------------------------------------------------------------------------- trajectory
class Trajectory:
    """Camera pose p(t), R_wc(t) in the room frame. Segments (seconds) are fixed by this file, not by a seed except for phases."""
    def __init__(self, rng, duration):
        self.dur = duration
        self.ph = rng.uniform(0, 2 * math.pi, size=8)
        # speed regimes: fast [0,9.5] and [21.5,end], slow [11.5,18]
        self.env = lambda t: 0.2 + 0.8 * (1.0 - pulse(t, 9.5, 11.0, 18.0, 20.0))
        # look-away windows: short (target out of view ~12-20 frames) and long (> 100 frames)
        self.away = lambda t: -math.radians(62.0) * (pulse(t, 8.3, 8.65, 8.85, 9.2) + pulse(t, 17.0, 17.9, 22.0, 22.9))
        self.reentry = None

    def pos(self, t):
        e = self.env(t); ph = self.ph
        return np.array([0.16 * e * math.sin(2 * math.pi * 1.10 * t + ph[0]),
                         0.06 * e * math.sin(2 * math.pi * 0.50 * t + ph[1]),
                         0.05 * e * math.sin(2 * math.pi * 0.65 * t + ph[2])])

    def rot(self, t):
        e = self.env(t); ph = self.ph
        yaw = self.away(t) + math.radians(4.0) * e * math.sin(2 * math.pi * 0.35 * t + ph[3])
        pitch = math.radians(3.0) * e * math.sin(2 * math.pi * 0.45 * t + ph[4])
        roll = math.radians(2.0) * e * math.sin(2 * math.pi * 0.55 * t + ph[5])
        cz, sz = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        cp, sp = math.cos(pitch), math.sin(pitch)
        Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
        cr, sr = math.cos(roll), math.sin(roll)
        Rr = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
        return Rz @ R_NEUTRAL @ Rx @ Rr

    def imu(self, t, h=1e-3):
        p0, p1, p2 = self.pos(t - h), self.pos(t), self.pos(t + h)
        acc_w = (p2 - 2 * p1 + p0) / (h * h)
        R = self.rot(t)
        dR = (self.rot(t + h) - self.rot(t - h)) / (2 * h)
        S = R.T @ dR
        w_c = np.array([S[2, 1], S[0, 2], S[1, 0]])
        f_c = R.T @ (acc_w + np.array([0.0, 0.0, G]))
        return R_IC @ f_c, R_IC @ w_c


# ----------------------------------------------------------------------------- rendering
class Room:
    def __init__(self, seed, target):
        rng = np.random.default_rng(seed)
        self.front = make_texture(rng, round(2.3 * PPM), round(4.6 * PPM), 1.5)     # X in [-1.6,3.0], Z in [-0.3,2.0]
        self.left = make_texture(rng, round(2.3 * PPM), round(2.1 * PPM), 1.5)      # Y in [-1.0,1.1], Z in [-0.3,2.0]
        self.table = make_texture(rng, round(2.1 * PPM), round(4.6 * PPM), 1.5)     # X in [-1.6,3.0], Y in [-1.0,1.1]
        # target 0.6 x 0.8 m centred at X=0, Z=0.1 on the front wall: X in [-0.3,0.3], Z in [-0.3,0.5]
        c0 = int(round((-0.3 - -1.6) * PPM)); r0 = int(round((2.0 - 0.5) * PPM))
        self.front[r0:r0 + H, c0:c0 + W] = target.astype(np.float32)
        self.target_box = (-0.3, 0.3, -0.3, 0.5)

    @staticmethod
    def sample(tex, u, v):
        h, w = tex.shape
        u = np.clip(u, 0, w - 1.001); v = np.clip(v, 0, h - 1.001)
        u0 = u.astype(np.int32); v0 = v.astype(np.int32)
        fu = u - u0; fv = v - v0
        return (tex[v0, u0] * (1 - fu) * (1 - fv) + tex[v0, u0 + 1] * fu * (1 - fv) +
                tex[v0 + 1, u0] * (1 - fu) * fv + tex[v0 + 1, u0 + 1] * fu * fv)

    def render(self, p, R, rng):
        gy, gx = np.mgrid[0:H, 0:W].astype(np.float32)
        acc = np.zeros((H, W), np.float32)
        for oy in (-0.25, 0.25):
            for ox in (-0.25, 0.25):
                d = np.stack([(gx + ox - CX) / FX, (gy + oy - CY) / FY, np.ones_like(gx)], -1) @ R.T.astype(np.float32)
                best = np.full((H, W), np.inf, np.float32); val = np.full((H, W), 100.0, np.float32)
                with np.errstate(divide='ignore', invalid='ignore'):
                    for name in ('front', 'left', 'table'):
                        if name == 'front':
                            s = (D_WALL - p[1]) / d[..., 1]
                            X = p[0] + s * d[..., 0]; Z = p[2] + s * d[..., 2]
                            ok = (s > 0) & (X >= -1.6) & (X <= 3.0) & (Z >= Z_TABLE) & (Z <= 2.0)
                            col = (X + 1.6) * PPM; row = (2.0 - Z) * PPM; tex = self.front
                        elif name == 'left':
                            s = (X_LEFT - p[0]) / d[..., 0]
                            Y = p[1] + s * d[..., 1]; Z = p[2] + s * d[..., 2]
                            ok = (s > 0) & (Y >= -1.0) & (Y <= D_WALL) & (Z >= Z_TABLE) & (Z <= 2.0)
                            col = (Y + 1.0) * PPM; row = (2.0 - Z) * PPM; tex = self.left
                        else:
                            s = (Z_TABLE - p[2]) / d[..., 2]
                            X = p[0] + s * d[..., 0]; Y = p[1] + s * d[..., 1]
                            ok = (s > 0) & (X >= -1.6) & (X <= 3.0) & (Y >= -1.0) & (Y <= D_WALL)
                            col = (X + 1.6) * PPM; row = (D_WALL - Y) * PPM; tex = self.table
                        ok &= s < best
                        if ok.any():
                            v = self.sample(tex, np.where(ok, col, 0), np.where(ok, row, 0))
                            val = np.where(ok, v, val); best = np.where(ok, s, best)
                acc += val
        img = acc / 4.0 + rng.normal(0.0, 1.5, size=(H, W)).astype(np.float32)
        return np.clip(np.rint(img), 0, 255).astype(np.uint8)


def target_visibility(p, R):
    x0, x1, z0, z1 = -0.3, 0.3, -0.3, 0.5
    xs = np.linspace(x0, x1, 13); zs = np.linspace(z0, z1, 17)
    X, Z = np.meshgrid(xs, zs)
    P = np.stack([X.ravel(), np.full(X.size, D_WALL), Z.ravel()], -1)
    c = (P - p) @ R              # row vectors: R^T (P - p)
    ok = c[:, 2] > 1e-6
    u = FX * c[:, 0] / np.where(ok, c[:, 2], 1) + CX
    v = FY * c[:, 1] / np.where(ok, c[:, 2], 1) + CY
    inside = ok & (u >= 0) & (u <= W - 1) & (v >= 0) & (v <= H - 1)
    cen = (np.array([0.0, D_WALL, 0.1]) - p) @ R
    cu = FX * cen[0] / cen[2] + CX if cen[2] > 0 else float('nan')
    cv = FY * cen[1] / cen[2] + CY if cen[2] > 0 else float('nan')
    return float(inside.mean()), cu, cv, float(cen[2])


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, required=True, help='room/target/phase/noise seed')
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--duration', type=float, default=28.0, help='seconds')
    ap.add_argument('--imu-noise', type=int, default=1, help='1 = add white noise + constant bias (deterministic from the seed)')
    a = ap.parse_args(argv)
    if os.path.isdir(a.out) and os.listdir(a.out):
        print('refusing non-empty ' + a.out, file=sys.stderr); return 2
    os.makedirs(os.path.join(a.out, 'mav0/cam0/data'), exist_ok=True)
    os.makedirs(os.path.join(a.out, 'target'), exist_ok=True)
    rng = np.random.default_rng(a.seed)
    target = make_target(int(rng.integers(1, 2**31 - 1)))
    room = Room(int(rng.integers(1, 2**31 - 1)), target)
    a.frames = int(round(a.duration * a.fps))
    traj = Trajectory(rng, a.duration)
    nrng = np.random.default_rng(int(rng.integers(1, 2**31 - 1)))
    # target files
    Image.fromarray(target, mode='L').save(os.path.join(a.out, 'target/e5-target_luminance.png'), optimize=False)
    with open(os.path.join(a.out, 'target/e5-target_luminance.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H)); f.write(target.tobytes())
    json.dump({'type': 'PLANAR', 'properties': {'left': 0, 'top': 0, 'width': W, 'height': H, 'isRotated': False,
                                                'originalWidth': W, 'originalHeight': H},
               'imagePath': 'image-targets/e5-target_luminance.png', 'metadata': None, 'name': 'e5-target',
               'resources': {'luminanceImage': 'e5-target_luminance.png'}, 'created': 0, 'updated': 0},
              open(os.path.join(a.out, 'target/e5-target.json'), 'w'), indent=2)
    frames, gt = [], []
    t_first = 0.004
    for k in range(a.frames):
        t = t_first + k / a.fps
        p, R = traj.pos(t), traj.rot(t)
        img = room.render(p, R, nrng)
        fn = '%05d.pgm' % k
        payload = img.tobytes()
        with open(os.path.join(a.out, 'mav0/cam0/data', fn), 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (W, H)); f.write(payload)
        frames.append({'file': fn, 'index': k, 'physicalTimestamp': t, 'sha256': hashlib.sha256(payload).hexdigest(), 'timestamp': t})
        vis, cu, cv, depth = target_visibility(p, R)
        gt.append({'index': k, 't': t, 'p': p.tolist(), 'R_wc': R.tolist(), 'target_visible_fraction': vis, 'target_center_px': [cu, cv], 'target_depth': depth})
    n_imu = int((a.frames / a.fps + 0.2) * IMU_HZ)
    imu = []
    bias_a = nrng.normal(0, 0.05, 3) if a.imu_noise else np.zeros(3)
    bias_g = nrng.normal(0, 0.002, 3) if a.imu_noise else np.zeros(3)
    for i in range(n_imu):
        t = i / IMU_HZ
        f_b, w_b = traj.imu(t)
        if a.imu_noise:
            f_b = f_b + bias_a + nrng.normal(0, 0.03, 3); w_b = w_b + bias_g + nrng.normal(0, 0.002, 3)
        imu.append([t, *map(float, f_b), *map(float, w_b)])
    cfg = {'cx': CX, 'cy': CY, 'fx': FX, 'fy': FY, 'height': H, 'width': W, 'modelType': 2, 'k2': 0.0, 'k3': 0.0, 'k4': 0.0, 'k5': 0.0,
           'r_ic': R_IC.ravel().astype(int).tolist(), 't_ic': [0.0, 0.0, -0.02], 'g_norm': G}
    json.dump({'schema': 'mobile-slam-synthetic-vio-v1', 'config': cfg, 'frames': frames, 'imu': imu,
               'scope': 'E5 synthetic room (package H); desktop synthetic only', 'seed': a.seed, 'generator': 'slam_refact/tools/harness/e5/scene.py'},
              open(os.path.join(a.out, 'input.json'), 'w'))
    json.dump({'schema': 'e5-gt-v1', 'seed': a.seed, 'frames': gt, 'target_box_xz': list(room.target_box), 'wall_y': D_WALL,
               'note': 'room frame X right, Y into wall, Z up; R_wc columns = camera x,y,z axes in the room frame'},
              open(os.path.join(a.out, 'gt.json'), 'w'))
    with open(os.path.join(a.out, 'gt.tsv'), 'w') as f:
        f.write('index\tt\tvisible_fraction\tcenter_u\tcenter_v\tdepth\tpx\tpy\tpz\n')
        for g in gt:
            f.write('%d\t%.6f\t%.4f\t%.2f\t%.2f\t%.4f\t%.5f\t%.5f\t%.5f\n' % (g['index'], g['t'], g['target_visible_fraction'],
                    g['target_center_px'][0], g['target_center_px'][1], g['target_depth'], *g['p']))
    print('wrote', a.out, a.frames, 'frames', len(imu), 'imu rows')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
