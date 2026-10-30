#!/usr/bin/env python3
"""US-025 LC-6b synthetic out-and-back scene (desktop synthetic only; development tool, not product code).

Reuses slam_refact/tools/harness/e5/scene.py (room renderer, texture generator, camera model, IMU frame convention) unchanged and replaces only
the trajectory: the camera slides along the textured front wall away from the start and returns to the start, with a small hand-held wobble so
the original SDK sees IMU excitation. The IMU stream is the analytic specific force / gyro of the same trajectory (scene.py Trajectory.imu).

  python3 -I scene_oab.py --out <NEW dir> --seed <int> [--kind line|loop|ret] [--amp 1.8] [--pitch-down 0] [--duration 52] [--fps 15] [--jobs 6]

kind=line  pure lateral out-and-back along the wall:  x(t) = A*(smooth((t-5)/20) - smooth((t-27)/20)), return to x=0 at t ~ 47, hold to the end.
kind=ret   like line, but from t=27 s the camera is flown 0.45 m farther from the wall and yawed 12 deg (new viewpoint on the return leg, so the
           original creates keyframes there; line revisits the start views and creates none after t~22 s: lc6b doc).
kind=loop  same lateral out-and-back plus a depth excursion (away from the wall and back) and a yaw sweep; start and end pose coincide.
Writes input.json (fixture 'mobile-slam-synthetic-vio-v1', the 'tum' scenario kind of tools/re/runtime), mav0/cam0/data/*.pgm, gt.json/gt.tsv.
Never writes into an existing non-empty directory.
"""
import argparse, hashlib, importlib.util, json, math, os, pathlib, sys
import numpy as np
from multiprocessing import Pool

E5 = pathlib.Path(__file__).resolve().parents[2] / 'e5' / 'scene.py'
spec = importlib.util.spec_from_file_location('e5_scene', E5)
S = importlib.util.module_from_spec(spec)
spec.loader.exec_module(S)


def make_dense_texture(rng, h, w, scale):
    """scene.py make_texture recipe with 5x the shape density plus fine grain (many FAST corners on the surface a downward-looking camera sees)."""
    gy, gx = np.mgrid[0:h, 0:w].astype(np.float32)
    img = np.full((h, w), 128.0, np.float32) + rng.normal(0.0, 10.0, size=(h, w)).astype(np.float32)
    for _ in range(int(h * w / 1600)):
        rw, rh = rng.integers(int(6 * scale) + 1, int(45 * scale) + 2, size=2)
        x, y = rng.integers(0, max(1, w - rw)), rng.integers(0, max(1, h - rh))
        img[y:y + rh, x:x + rw] = rng.uniform(10, 245)
    for _ in range(int(h * w / 3200)):
        r = rng.uniform(3 * scale, 18 * scale)
        x, y = rng.uniform(0, w), rng.uniform(0, h)
        x0_, x1_, y0_, y1_ = int(max(0, x - r)), int(min(w, x + r + 1)), int(max(0, y - r)), int(min(h, y + r + 1))
        sub = (gx[y0_:y1_, x0_:x1_] - x) ** 2 + (gy[y0_:y1_, x0_:x1_] - y) ** 2 <= r * r
        img[y0_:y1_, x0_:x1_][sub] = rng.uniform(10, 245)
    p = np.pad(img, 1, mode='edge')
    img = sum(p[dy:dy + h, dx:dx + w] for dy in range(3) for dx in range(3)) / 9.0
    return np.clip(img, 0, 255).astype(np.float32)


class OutAndBack(S.Trajectory):
    def __init__(self, rng, duration, kind, amp, pitch_down):
        self.dur, self.kind, self.amp = duration, kind, amp
        self.pitch0 = -math.radians(pitch_down)   # camera x axis rotation; negative = looking down (see the doc string of main: ORB only below the horizon)
        self.ph = rng.uniform(0, 2 * math.pi, size=10)

    def base(self, t):
        a = self.amp
        x = a * (S.smooth((t - 5.0) / 20.0) - S.smooth((t - 27.0) / 20.0))
        y = 0.0
        yaw = 0.0
        if self.kind == 'ret':   # the return leg is flown farther from the wall (+0.45 m) and yawed 12 deg: a viewpoint that is new relative to the start
            y = -0.45 * S.smooth((t - 27.0) / 10.0)
            yaw = math.radians(12.0) * S.smooth((t - 27.0) / 10.0)
        if self.kind == 'loop':
            y = -0.35 * (S.smooth((t - 8.0) / 14.0) - S.smooth((t - 30.0) / 14.0))   # away from the wall (y is the wall normal; wall at y=1.1)
            yaw = math.radians(14.0) * (S.smooth((t - 6.0) / 18.0) - S.smooth((t - 28.0) / 18.0))
        return x, y, yaw

    def pos(self, t):
        ph = self.ph
        x, y, _ = self.base(t)
        return np.array([x + 0.05 * math.sin(2 * math.pi * 0.80 * t + ph[0]),
                         y + 0.04 * math.sin(2 * math.pi * 0.55 * t + ph[1]),
                         0.04 * math.sin(2 * math.pi * 0.65 * t + ph[2])])

    def rot(self, t):
        ph = self.ph
        _, _, ybase = self.base(t)
        yaw = ybase + math.radians(3.0) * math.sin(2 * math.pi * 0.35 * t + ph[3])
        pitch = self.pitch0 + math.radians(2.0) * math.sin(2 * math.pi * 0.45 * t + ph[4])
        roll = math.radians(1.5) * math.sin(2 * math.pi * 0.55 * t + ph[5])
        cz, sz = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        cp, sp = math.cos(pitch), math.sin(pitch)
        Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
        cr, sr = math.cos(roll), math.sin(roll)
        Rr = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
        return Rz @ S.R_NEUTRAL @ Rx @ Rr


_G = {}


def render_one(k):
    t = _G['t0'] + k / _G['fps']
    p, R = _G['traj'].pos(t), _G['traj'].rot(t)
    img = _G['room'].render(p, R, np.random.default_rng([_G['seed'], 7, k]))
    return k, t, p, R, img


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--kind', choices=['line', 'loop', 'ret'], default='line')
    ap.add_argument('--amp', type=float, default=1.8)
    ap.add_argument('--pitch-down', type=float, default=0.0, help='degrees the camera looks down from horizontal (the original adds ORB features only in the lower gravity portions, spec/01-frontend.md 3.5)')
    ap.add_argument('--table-scale', type=float, default=1.5, help='size scale of the DENSE table texture (make_dense_texture); 1.5 = scene.py Room table unchanged')
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--duration', type=float, default=52.0)
    ap.add_argument('--jobs', type=int, default=6)
    a = ap.parse_args(argv)
    out = pathlib.Path(a.out)
    if out.is_dir() and any(out.iterdir()):
        print('refusing non-empty ' + a.out, file=sys.stderr); return 2
    (out / 'mav0/cam0/data').mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    target = S.make_target(int(rng.integers(1, 2**31 - 1)))
    room = S.Room(int(rng.integers(1, 2**31 - 1)), target)
    if a.table_scale != 1.5:   # finer table texture: more corners on the surface the downward-looking camera sees (the original keeps ORB only below the horizon)
        room.table = make_dense_texture(np.random.default_rng(int(rng.integers(1, 2**31 - 1))), round(2.1 * S.PPM), round(4.6 * S.PPM), a.table_scale)
    traj = OutAndBack(rng, a.duration, a.kind, a.amp, a.pitch_down)
    nrng = np.random.default_rng(int(rng.integers(1, 2**31 - 1)))
    nframes = int(round(a.duration * a.fps))
    _G.update(t0=0.004, fps=a.fps, traj=traj, room=room, seed=a.seed)
    with Pool(a.jobs) as pool:
        results = sorted(pool.map(render_one, range(nframes), chunksize=4), key=lambda r: r[0])
    frames, gt = [], []
    for k, t, p, R, img in results:
        fn = '%05d.pgm' % k
        payload = img.tobytes()
        (out / 'mav0/cam0/data' / fn).write_bytes(b'P5\n%d %d\n255\n' % (S.W, S.H) + payload)
        frames.append({'file': fn, 'index': k, 'physicalTimestamp': t, 'sha256': hashlib.sha256(payload).hexdigest(), 'timestamp': t})
        gt.append({'index': k, 't': t, 'p': p.tolist(), 'R_wc': R.tolist()})
    n_imu = int((nframes / a.fps + 0.2) * S.IMU_HZ)
    bias_a, bias_g = nrng.normal(0, 0.05, 3), nrng.normal(0, 0.002, 3)
    imu = []
    for i in range(n_imu):
        t = i / S.IMU_HZ
        f_b, w_b = traj.imu(t)
        f_b = f_b + bias_a + nrng.normal(0, 0.03, 3); w_b = w_b + bias_g + nrng.normal(0, 0.002, 3)
        imu.append([t, *map(float, f_b), *map(float, w_b)])
    cfg = {'cx': S.CX, 'cy': S.CY, 'fx': S.FX, 'fy': S.FY, 'height': S.H, 'width': S.W, 'modelType': 2, 'k2': 0.0, 'k3': 0.0, 'k4': 0.0, 'k5': 0.0,
           'r_ic': S.R_IC.ravel().astype(int).tolist(), 't_ic': [0.0, 0.0, -0.02], 'g_norm': S.G}
    (out / 'input.json').write_text(json.dumps({'schema': 'mobile-slam-synthetic-vio-v1', 'config': cfg, 'frames': frames, 'imu': imu,
        'scope': 'US-025 LC-6b synthetic out-and-back (%s); desktop synthetic only' % a.kind, 'seed': a.seed,
        'generator': 'slam_refact/tools/harness/us025/scene/scene_oab.py'}))
    (out / 'gt.json').write_text(json.dumps({'schema': 'lc6b-gt-v1', 'seed': a.seed, 'kind': a.kind, 'amp': a.amp, 'pitch_down_deg': a.pitch_down, 'table_scale': a.table_scale, 'frames': gt,
        'note': 'room frame X right, Y into wall, Z up; R_wc columns = camera axes in the room frame'}))
    with open(out / 'gt.tsv', 'w') as f:
        f.write('index\tt\tpx\tpy\tpz\n')
        for g in gt:
            f.write('%d\t%.6f\t%.5f\t%.5f\t%.5f\n' % (g['index'], g['t'], *g['p']))
    print('wrote', a.out, nframes, 'frames', len(imu), 'imu rows')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
