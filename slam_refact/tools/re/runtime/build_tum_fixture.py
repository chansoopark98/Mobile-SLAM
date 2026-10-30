#!/usr/bin/env python3
"""Build a TUM VI room1 fixture for the headless reference runtime reference runs (lane C, US-004).

Same conversion as build/rea-analysis-20261015/side-by-side/tools/build_fixture.py (16-bit >> 8, cv2.fisheye
rectification to a 320x240 pinhole with K = reference runtime's 320x240 intrinsics, IMU rotated into a phone-style body
frame, optional low-texture blend with the same seed), plus one change: --orientation portrait rectifies the
fisheye directly into a 240x320 (portrait) pinhole with the same focal length and R=I, so the image x axis stays
parallel to the W3C device x axis of an upright portrait phone and the IMU mapping is unchanged.  The prior
landscape fixture paired this portrait-relation IMU with a 320x240 video in a landscape viewport (screen
orientation 90), which reference runtime interprets as a device physically rotated by 90 degrees.

Output schema is mobile-slam-synthetic-vio-v1 (input.json + mav0/cam0/data/*.pgm), groundtruth-body.csv is
written separately and is never served to the engine.
"""
import argparse
import hashlib
import json
import pathlib

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path(__file__).resolve().parents[4]
DATA = ROOT / 'assets/datasets/tum/dataset-room1_512_16'
# R_Bc (camera -> phone body B), phone convention used by the existing sensor adapter (body -> W3C device).
R_PHONE = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
# TUM VI room1 Kalibr calibration (cam0 / imu0), copied from the prior side-by-side builder.
T_CAM_IMU = np.array([
    [-0.9995250378696743, 0.029615343885863205, -0.008522328211654736, 0.04727988224914392],
    [0.0075019185074052044, -0.03439736061393144, -0.9993800792498829, -0.047443232143367084],
    [-0.02989013031643309, -0.998969345370175, 0.03415885127385616, -0.0681999605066297],
    [0, 0, 0, 1]])
K_SRC = np.array([[190.97847715128717, 0, 254.93170605935475], [0, 190.9733070521226, 256.8974428996504], [0, 0, 1]])
D_SRC = np.array([0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182])


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', required=True, type=pathlib.Path)
    ap.add_argument('--dataset', type=pathlib.Path, default=DATA,
                    help='TUM VI 512_16 sequence directory (same cam0/imu0 calibration for every sequence)')
    ap.add_argument('--start-frame', type=int, default=0)
    ap.add_argument('--frames', type=int, default=0, help='0 = all remaining frames')
    ap.add_argument('--orientation', choices=['portrait', 'landscape'], default='portrait')
    ap.add_argument('--fx', type=float, default=260.9, help='rectified focal length (pixels)')
    ap.add_argument('--lowtex-start', type=float, default=None, help='fixture seconds')
    ap.add_argument('--lowtex-end', type=float, default=None)
    ap.add_argument('--ramp', type=float, default=0.25)
    a = ap.parse_args()
    data = a.dataset if a.dataset.is_absolute() else ROOT / a.dataset
    out = a.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'mav0/cam0/data').mkdir(parents=True)

    cam = [line.strip().split(',') for line in open(data / 'mav0/cam0/data.csv') if not line.startswith('#')]
    cam = cam[a.start_frame:]
    if a.frames:
        cam = cam[:a.frames]
    imu = np.loadtxt(data / 'mav0/imu0/data.csv', delimiter=',', comments='#')
    gt = np.loadtxt(data / 'mav0/mocap0/data.csv', delimiter=',', comments='#')
    cam_ns = np.array([int(c[0]) for c in cam], dtype=np.int64)
    imu_ns = imu[:, 0].astype(np.int64)
    first = np.searchsorted(imu_ns, cam_ns[0], side='right') - 1
    if first < 0:
        raise SystemExit('first frame precedes IMU')
    t0_ns = int(imu_ns[first])
    last = np.searchsorted(imu_ns, cam_ns[-1] + 100_000_000)

    portrait = a.orientation == 'portrait'
    out_w, out_h = (240, 320) if portrait else (320, 240)
    R_cI = T_CAM_IMU[:3, :3]
    p_c_in_I = -R_cI.T @ T_CAM_IMU[:3, 3]
    R_BI = R_PHONE @ R_cI                            # IMU -> body B (R_Bc = R_PHONE; rectification R = I)
    t_ic = (R_BI @ p_c_in_I).tolist()
    assert np.allclose(R_BI @ R_BI.T, np.eye(3), atol=1e-9)

    rows = []
    for r in imu[first:last]:
        t = (int(r[0]) - t0_ns) * 1e-9
        g = R_BI @ r[1:4]
        acc = R_BI @ r[4:7]
        rows.append([round(t, 9), *acc.tolist(), *g.tolist()])

    fx = a.fx
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(
        K_SRC, D_SRC, np.eye(3), np.array([[fx, 0, (out_w - 1) / 2], [0, fx, (out_h - 1) / 2], [0, 0, 1]]),
        (out_w, out_h), cv2.CV_32FC1)
    rng = np.random.default_rng(20261015)
    frames = []
    for i, (ns, c) in enumerate(zip(cam_ns, cam)):
        src = cv2.imread(str(data / 'mav0/cam0/data' / c[1].strip()), cv2.IMREAD_UNCHANGED)
        g8 = (src >> 8).astype(np.uint8)
        img = cv2.remap(g8, m1, m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).astype(np.float64)
        t = (int(ns) - t0_ns) * 1e-9
        alpha = 0.0
        if a.lowtex_start is not None:
            s, e, rp = a.lowtex_start, a.lowtex_end, a.ramp
            if s <= t < e:
                alpha = min(1.0, (t - s) / rp, (e - t) / rp)
        if alpha > 0:
            mean = img.mean()
            flat = mean + 0.04 * (cv2.GaussianBlur(img, (0, 0), 20) - mean) + rng.normal(0, 1.0, img.shape)
            img = (1 - alpha) * img + alpha * flat
        pix = np.clip(np.rint(img), 0, 255).astype(np.uint8)
        assert pix.shape == (out_h, out_w)
        name = f'{i:05d}.pgm'
        (out / 'mav0/cam0/data' / name).write_bytes(f'P5\n{out_w} {out_h}\n255\n'.encode() + pix.tobytes())
        grad = np.sqrt(np.mean(np.square(cv2.Sobel(pix, cv2.CV_64F, 1, 0)) + np.square(cv2.Sobel(pix, cv2.CV_64F, 0, 1))))
        frames.append({'file': name, 'index': i, 'physicalTimestamp': round(t, 9), 'timestamp': round(t, 9),
                       'sha256': sha(pix.tobytes()), 'sourceNs': int(ns), 'lowTextureAlpha': round(alpha, 4),
                       'gradientRms': float(grad)})
    cx, cy = (out_w - 1) / 2, (out_h - 1) / 2
    spec = {'source': 'TUM VI %s cam0+imu0' % data.name, 'startFrame': a.start_frame, 'frames': len(frames),
            'orientation': a.orientation, 't0_ns': t0_ns, 'K': [fx, fx, cx, cy],
            'lowtex': [a.lowtex_start, a.lowtex_end, a.ramp] if a.lowtex_start is not None else None}
    spec_bytes = json.dumps(spec, sort_keys=True).encode()
    config = {'cx': cx, 'cy': cy, 'fx': fx, 'fy': fx, 'height': out_h, 'width': out_w, 'modelType': 2,
              'k2': 0.0, 'k3': 0.0, 'k4': 0.0, 'k5': 0.0, 'r_ic': R_PHONE.flatten().astype(int).tolist(), 't_ic': t_ic,
              'acc_n': 0.5, 'acc_w': 0.003, 'gyr_n': 0.2, 'gyr_w': 0.0002, 'g_norm': 9.81}
    inp = {'config': config,
           'frames': [{k: f[k] for k in ('file', 'index', 'physicalTimestamp', 'sha256', 'timestamp')} for f in frames],
           'imu': rows, 'schema': 'mobile-slam-synthetic-vio-v1',
           'scope': 'TUM VI ' + data.name + ' real pixels (rectified) and real IMU (rotated to phone body frame); no GT in input',
           'spec_sha256': sha(spec_bytes)}
    (out / 'input.json').write_text(json.dumps(inp))
    (out / 'spec.json').write_bytes(spec_bytes)
    # GT: body B pose in the mocap world on the fixture time base (B origin = IMU origin). Never served.
    sel = (gt[:, 0] >= cam_ns[0] - 50_000_000) & (gt[:, 0] <= cam_ns[-1] + 50_000_000)
    R_WI = Rotation.from_quat(gt[sel][:, [5, 6, 7, 4]]).as_matrix()
    R_WB = R_WI @ R_BI.T
    q = Rotation.from_matrix(R_WB).as_quat()
    tgt = (gt[sel, 0].astype(np.int64) - t0_ns) * 1e-9
    np.savetxt(out / 'groundtruth-body.csv', np.column_stack([tgt, gt[sel, 1:4], q]), delimiter=',',
               header='t_fixture_s,px,py,pz,qx,qy,qz,qw (body B, mocap world; B origin = IMU)', fmt='%.9f')
    manifest = {'spec': spec, 'R_BFromImu': R_BI.tolist(), 'R_BFromCam': R_PHONE.tolist(), 't_ic': t_ic,
                'imuRows': len(rows), 'frames': len(frames), 'durationS': frames[-1]['physicalTimestamp'],
                'lowTextureFrames': [f['index'] for f in frames if f['lowTextureAlpha'] > 0],
                'gradientRms': [round(f['gradientRms'], 3) for f in frames],
                'inputSha256': sha((out / 'input.json').read_bytes()), 'groundTruthProvidedToEngine': False,
                'notes': ['16-bit PNG >>8 to 8-bit', f'cv2.fisheye rectification R=I INTER_LINEAR to {out_w}x{out_h} pinhole',
                          'low-texture: blend to mean+4% sigma20 blur + N(0,1) noise, seed 20261015']}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=1))
    print(json.dumps({'out': str(out), 'frames': len(frames), 'imu': len(rows), 'duration': frames[-1]['physicalTimestamp'],
                      'size': [out_w, out_h], 'lowtexFrames': len(manifest['lowTextureFrames'])}))


if __name__ == '__main__':
    main()
