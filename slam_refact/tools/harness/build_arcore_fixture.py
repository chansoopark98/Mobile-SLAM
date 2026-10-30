#!/usr/bin/env python3
"""ARCore phone recording -> replay fixture with an ARCore pseudo ground truth (accuracy development).

Input: one ARCore reference run pulled from the phone (build/flip6-20261015/A-*/pulled/files/reference-runs/<id>/):
  recording.mp4  ARCore session recording, 640x480 camera stream (~30 fps, H.264)
  images/*.pgm   the CPU images ARCore tracked (10 Hz), Y plane 640x480, with poses.jsonl imageTimestampNs
  poses.jsonl    ARCore camera pose per tracked frame (T_W_C, OpenGL camera, metres, gravity-aligned world)
  imu.jsonl      Android SensorManager accelerometer (type 1) and calibrated gyroscope (type 4), device frame

Output: a fixture in the TUM schema (mobile-slam-synthetic-vio-v1) so the existing tools work unchanged:
  * frames: every mp4 frame, gray (cv2 BT.601), rotated 90 deg clockwise to the 480x640 portrait image an
    upright phone shows (the back camera sensor is landscape); the mp4 presentation time is mapped to the
    ARCore clock by matching mp4 frames against the tracked CPU images (median offset, residual reported);
  * imu rows [t, acc(3), gyro(3)] in the phone body frame B of the sensor adapter (body = R_BODY_FROM_DEVICE *
    device). Chrome-like DeviceMotion is emulated (--chrome, default): ~60 Hz nearest-sample decimation,
    acceleration quantized to 0.1 m/s^2 and rotation rate to 0.1 deg/s, as the Flip6 Chrome records show;
  * groundtruth-body.csv: ARCore TRACKING poses as body poses (R_W_B = R_W_Ccv(portrait) * R_PHONE^T,
    camera centre as body origin, t_ic = 0). ARCore is a reference VIO, not an independent ground truth:
    reports must call it ARCore-referenced;
  * manifest.json: K of the portrait image from ARCore's calibration, offsets, residuals.

Usage (repo root):
  python3 -I slam_refact/tools/harness/build_arcore_fixture.py --run build/flip6-20261015/A-S2-loop/pulled/files/reference-runs/flip6-20261015-a-s2 \
      --output build/slam-refact-20261015/reference/fixtures/flip6-arcore-s2
"""
import argparse
import hashlib
import json
import pathlib

import cv2
import numpy as np

R_PHONE = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)      # camera(cv) -> body B, sensor adapter convention
R_BODY_FROM_DEVICE = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
D_GL_CV = np.diag([1.0, -1.0, -1.0])
# Clockwise image rotation: u' = H-1-v, v' = u  =>  portrait camera axes in the landscape camera (cv) frame.
R_LAND_FROM_PORTRAIT = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)


def sha(b):
    return hashlib.sha256(b).hexdigest()


def quat_xyzw_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat_xyzw(r):
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q


def read_pgm(path):
    data = path.read_bytes()
    parts = data.split(maxsplit=3)
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[len(data) - w * h:], np.uint8).reshape(h, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=pathlib.Path)
    ap.add_argument('--output', required=True, type=pathlib.Path)
    ap.add_argument('--imu-hz', type=float, default=60.0)
    ap.add_argument('--raw-imu', action='store_true', help='keep full-rate unquantized IMU (not Chrome-like)')
    ap.add_argument('--imu-delay-s', type=float, default=0.0,
                    help='stamp every IMU sample this much later than its sensor time (emulates event delivery lag)')
    a = ap.parse_args()
    run, out = a.run, a.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'mav0/cam0/data').mkdir(parents=True)

    poses = [json.loads(line) for line in open(run / 'poses.jsonl')]
    frames_ar = [p for p in poses if p.get('event') == 'frame']
    # Saved CPU images are numbered from 1 in ARCore frame order of frames with a payload.
    with_image = [p for p in frames_ar if p.get('imagePayloadSha256')]
    pgms = sorted((run / 'images').glob('*.pgm'))
    if len(pgms) != len(with_image):
        raise SystemExit('images %d != frames with payload %d' % (len(pgms), len(with_image)))

    # Decode the mp4 (gray, landscape) with presentation times.
    cap = cv2.VideoCapture(str(run / 'recording.mp4'))
    mp4, pts = [], []
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        pts.append(cap.get(cv2.CAP_PROP_POS_MSEC) * 1e-3)
        mp4.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    pts = np.asarray(pts)
    if len(mp4) < 10:
        raise SystemExit('mp4 decode failed')

    # Clock offset: for each tracked image find the best mp4 frame (mean abs diff on a 4x down-sample).
    small = [cv2.resize(m, (160, 120), interpolation=cv2.INTER_AREA).astype(np.float32) for m in mp4]
    offsets, residual = [], []
    for p, path in zip(with_image, pgms):
        img = cv2.resize(read_pgm(path), (160, 120), interpolation=cv2.INTER_AREA).astype(np.float32)
        d = np.array([np.mean(np.abs(img - s)) for s in small])
        k = int(np.argmin(d))
        offsets.append(int(p['imageTimestampNs']) * 1e-9 - pts[k])
        residual.append(float(d[k]))
    offsets = np.asarray(offsets)
    off = float(np.median(offsets))
    spread = np.abs(offsets - off)
    ts = pts + off                                            # ARCore clock seconds, per mp4 frame

    imu = [json.loads(line) for line in open(run / 'imu.jsonl')]
    acc = np.array([[int(r['timestampNs']) * 1e-9, *r['values']] for r in imu if r['sensorType'] == 1])
    gyr = np.array([[int(r['timestampNs']) * 1e-9, *r['values']] for r in imu if r['sensorType'] == 4])
    acc = acc[np.argsort(acc[:, 0])]
    gyr = gyr[np.argsort(gyr[:, 0])]
    acc[:, 0] += a.imu_delay_s
    gyr[:, 0] += a.imu_delay_s
    if a.raw_imu:
        grid = acc[:, 0]
    else:
        grid = np.arange(max(acc[0, 0], gyr[0, 0]), min(acc[-1, 0], gyr[-1, 0]), 1.0 / a.imu_hz)

    def nearest(table, t):
        i = np.clip(np.searchsorted(table[:, 0], t), 1, len(table) - 1)
        i = np.where(np.abs(table[i - 1, 0] - t) < np.abs(table[i, 0] - t), i - 1, i)
        return table[i, 1:4]

    dev_acc, dev_gyr = nearest(acc, grid), nearest(gyr, grid)
    if not a.raw_imu:
        dev_acc = np.round(dev_acc / 0.1) * 0.1
        q = np.deg2rad(0.1)
        dev_gyr = np.round(dev_gyr / q) * q
    body_acc = dev_acc @ R_BODY_FROM_DEVICE.T
    body_gyr = dev_gyr @ R_BODY_FROM_DEVICE.T

    keep = (ts >= grid[0] + 0.05) & (ts <= grid[-1] - 0.1)
    idx = np.nonzero(keep)[0]
    first = int(np.searchsorted(grid, ts[idx[0]], side='right') - 1)
    t0 = float(grid[first])
    rows = [[round(t - t0, 9), *ba.tolist(), *bg.tolist()]
            for t, ba, bg in zip(grid[first:], body_acc[first:], body_gyr[first:]) if t <= ts[idx[-1]] + 0.1]

    frames = []
    for n, k in enumerate(idx):
        pix = cv2.rotate(mp4[k], cv2.ROTATE_90_CLOCKWISE)
        h, w = pix.shape
        name = '%05d.pgm' % n
        (out / 'mav0/cam0/data' / name).write_bytes(b'P5\n%d %d\n255\n' % (w, h) + pix.tobytes())
        t = round(float(ts[k]) - t0, 9)
        frames.append({'file': name, 'index': n, 'physicalTimestamp': t, 'timestamp': t, 'sha256': sha(pix.tobytes()),
                       'mp4Frame': int(k)})

    fx_l, fy_l = frames_ar[-1]['focalLengthPx']
    cx_l, cy_l = frames_ar[-1]['principalPointPx']
    lw, lh = frames_ar[-1]['cpuImageDimensions']
    K = [fy_l, fx_l, (lh - 1) - cy_l, cx_l]            # portrait fx, fy, cx, cy (u' = H-1-v, v' = u)

    gt_rows = []
    for p in frames_ar:
        if p['trackingState'] != 'TRACKING':
            continue
        m = np.array(p['matrixColumnMajorT_W_C_OpenGL'], float).reshape(4, 4).T
        r_w_cv = m[:3, :3] @ D_GL_CV @ R_LAND_FROM_PORTRAIT
        r_w_b = r_w_cv @ R_PHONE.T
        q = matrix_to_quat_xyzw(r_w_b)
        gt_rows.append([int(p['timestampNs']) * 1e-9 - t0, *m[:3, 3].tolist(), *q.tolist()])
    np.savetxt(out / 'groundtruth-body.csv', np.asarray(gt_rows), delimiter=',', fmt='%.9f',
               header='t_fixture_s,px,py,pz,qx,qy,qz,qw (ARCore-referenced: portrait camera centre, body B axes, ARCore world)')

    # Sanity: gravity in the first IMU row (body) must match ARCore world -y expressed in body at that time.
    r0 = np.array(gt_rows[0][4:8])
    g_body_gt = quat_xyzw_to_matrix(r0).T @ np.array([0.0, 1.0, 0.0])
    a_body = np.mean(body_acc[(grid >= gt_rows[0][0] + t0 - 0.2) & (grid <= gt_rows[0][0] + t0 + 0.2)], axis=0)
    gravity_angle = float(np.degrees(np.arccos(np.clip(np.dot(g_body_gt, a_body / np.linalg.norm(a_body)), -1, 1))))

    config = {'cx': K[2], 'cy': K[3], 'fx': K[0], 'fy': K[1], 'height': 640, 'width': 480, 'modelType': 2,
              'k2': 0.0, 'k3': 0.0, 'k4': 0.0, 'k5': 0.0, 'r_ic': R_PHONE.flatten().astype(int).tolist(), 't_ic': [0, 0, 0],
              'acc_n': 0.5, 'acc_w': 0.003, 'gyr_n': 0.2, 'gyr_w': 0.0002, 'g_norm': 9.81}
    spec = {'source': str(run), 'frames': len(frames), 'K': K, 'chromeLikeImu': not a.raw_imu, 'imuHz': a.imu_hz,
            'imuDelayS': a.imu_delay_s}
    spec_bytes = json.dumps(spec, sort_keys=True).encode()
    inp = {'config': config, 'frames': [{k: f[k] for k in ('file', 'index', 'physicalTimestamp', 'sha256', 'timestamp')} for f in frames],
           'imu': rows, 'schema': 'mobile-slam-synthetic-vio-v1',
           'scope': 'Galaxy Z Flip6 ARCore recording: mp4 camera frames (portrait) and SensorManager IMU in phone body frame; no GT in input',
           'spec_sha256': sha(spec_bytes)}
    (out / 'input.json').write_text(json.dumps(inp))
    (out / 'spec.json').write_bytes(spec_bytes)
    manifest = {'spec': spec, 't_ic': [0.0, 0.0, 0.0], 'R_BFromCam': R_PHONE.tolist(), 'portraitK': K,
                'landscapeK': [fx_l, fy_l, cx_l, cy_l], 'landscapeSize': [lw, lh],
                'clock': {'offsetS': off, 'offsetSpreadP95S': float(np.percentile(spread, 95)), 'matches': len(offsets),
                          'matchResidualP50': float(np.median(residual)), 'matchResidualP95': float(np.percentile(residual, 95))},
                'mp4Frames': len(mp4), 'framesKept': len(frames), 'imuRows': len(rows), 'gtRows': len(gt_rows),
                'firstFrameGravityAngleDeg': gravity_angle,
                'groundTruthProvidedToEngine': False, 'groundTruthKind': 'ARCore-referenced (reference VIO, not independent)',
                'inputSha256': sha((out / 'input.json').read_bytes())}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=1))
    print(json.dumps({k: manifest[k] for k in ('clock', 'mp4Frames', 'framesKept', 'imuRows', 'gtRows', 'firstFrameGravityAngleDeg', 'portraitK')}))


if __name__ == '__main__':
    main()
