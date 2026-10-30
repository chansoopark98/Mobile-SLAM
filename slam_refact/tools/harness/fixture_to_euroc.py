#!/usr/bin/env python3
"""Run the existing VINS-Mono engine (native_replay) on slam_refact fixtures.

Subcommands
  convert FIXTURE_DIR OUT_DATASET --noise {mobile,tum}
      input.json (schema mobile-slam-synthetic-vio-v1) -> EuRoC-style mav0 tree + config.yaml.
  trace   FIXTURE_DIR NATIVE_OUT_DIR TRACE_JSONL
      native_replay results.json -> reference runtime-reference-schema trace.jsonl.

Conventions (verified against code, not guessed)
  input.json imu rows: [t, ax, ay, az, gx, gy, gz]  (phone body frame B, accel includes gravity).
  EuRoC imu0/data.csv read by src/utility/measurement_processor.cpp and tests/audit_dataset_replay.cpp:
      ts_ns, gyro xyz, acc xyz   (values[1..3] = gyro, values[4..6] = acc)  -> columns are reordered here.
  config r_ic / t_ic: src/utility/config.cpp loads extrinsicRotation/Translation into camera.r_ic / t_ic and
      VIOEngine::publishPose emits T_W_C = (R_WB * r_ic, p_WB + R_WB * t_ic) => r_ic = R_B_cam (camera->body),
      t_ic = camera origin in B. This equals input.json r_ic/t_ic (= R_PHONE of eval/accuracy.py).
  native pose (results.json, 4x4 row-major) is T_W_C with an OpenCV camera (x right, y down, z forward);
      trace wants an OpenGL camera: R_W_gl = R_W_cv * diag(1,-1,-1) (the evaluator applies the same diag back).
"""
import argparse
import json
import math
import os
import pathlib
import sys

import numpy as np

NOISE = {
    # repo mobile_default preset (scripts/dev/generate-synthetic-vio.py)
    'mobile': dict(acc_n=0.5, acc_w=0.003, gyr_n=0.2, gyr_w=0.0002, max_cnt=120),
    # config/tum_vi_room1.yaml
    'tum': dict(acc_n=0.04, acc_w=0.0004, gyr_n=0.004, gyr_w=2.0e-5, max_cnt=150),
}
SOLVER = dict(max_solver_time=0.06, max_num_iterations=10)  # VIOEngine mobile default (vio_engine.cpp:183)


def yaml_text(dataset, cfg, noise):
    return ('%YAML:1.0\n---\n'
            f'dataset_path: {dataset}\nframe_skip: 0\nstart_frame: 0\nend_frame: -1\n'
            'model_type: PINHOLE\ncamera_name: fixture\n'
            f'image_width: {cfg["width"]}\nimage_height: {cfg["height"]}\n'
            f'projection_parameters: {{fx: {cfg["fx"]}, fy: {cfg["fy"]}, cx: {cfg["cx"]}, cy: {cfg["cy"]}}}\n'
            'distortion_parameters: {k1: 0.0, k2: 0.0, p1: 0.0, p2: 0.0}\n'
            f'extrinsicRotation: !!opencv-matrix\n  rows: 3\n  cols: 3\n  dt: d\n  data: {json.dumps(cfg["r_ic"])}\n'
            f'extrinsicTranslation: !!opencv-matrix\n  rows: 3\n  cols: 1\n  dt: d\n  data: {json.dumps(cfg["t_ic"])}\n'
            f'max_cnt: {noise["max_cnt"]}\nmin_dist: 20\nF_threshold: 1.0\nshow_track: 0\nequalize: 1\nfisheye: 0\n'
            f'max_solver_time: {SOLVER["max_solver_time"]}\nmax_num_iterations: {SOLVER["max_num_iterations"]}\nkeyframe_parallax: 10.0\n'
            'lk_window_size: 21\nlk_pyramid_levels: 3\nlk_iterations: 20\nlk_eps: 0.03\n'
            f'acc_n: {noise["acc_n"]}\nacc_w: {noise["acc_w"]}\ngyr_n: {noise["gyr_n"]}\ngyr_w: {noise["gyr_w"]}\ng_norm: 9.81\n')


def convert(fixture, out, noise_name):
    fixture = pathlib.Path(fixture).resolve()
    out = pathlib.Path(out).resolve()
    if 'build' not in out.parts:
        raise SystemExit('outputs must live under build/')
    data = json.loads((fixture / 'input.json').read_text())
    cfg = data['config']
    (out / 'mav0/cam0/data').mkdir(parents=True, exist_ok=False)
    (out / 'mav0/imu0').mkdir(parents=True)
    cam = ['#timestamp [ns],filename']
    last = -1
    for fr in data['frames']:
        ns = round(fr['physicalTimestamp'] * 1e9)
        if ns <= last:
            raise SystemExit('non-increasing frame timestamp')
        last = ns
        os.symlink(fixture / 'mav0/cam0/data' / fr['file'], out / 'mav0/cam0/data' / fr['file'])
        cam.append(f'{ns},{fr["file"]}')
    (out / 'mav0/cam0/data.csv').write_text('\n'.join(cam) + '\n')
    imu = ['#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z']
    last = -1
    for row in data['imu']:
        t, ax, ay, az, gx, gy, gz = row[:7]
        ns = round(t * 1e9)
        if ns <= last:
            continue  # strictly increasing required by the loaders
        last = ns
        imu.append(f'{ns},{gx!r},{gy!r},{gz!r},{ax!r},{ay!r},{az!r}')
    (out / 'mav0/imu0/data.csv').write_text('\n'.join(imu) + '\n')
    (out / 'config.yaml').write_text(yaml_text(out, cfg, NOISE[noise_name]))
    print(f'{fixture.name}: {len(data["frames"])} frames, {len(imu) - 1} imu rows -> {out}')


def mat_to_quat_wxyz(r):
    m = np.asarray(r, float)
    tr = np.trace(m)
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return q.tolist()


def trace(fixture, native_out, trace_path):
    fixture = pathlib.Path(fixture)
    frames = json.loads((fixture / 'input.json').read_text())['frames']
    res = json.loads((pathlib.Path(native_out) / 'results.json').read_text())['frames']
    if len(res) != len(frames):
        raise SystemExit(f'frame count mismatch input={len(frames)} results={len(res)}')
    d = np.diag([1.0, -1.0, -1.0])
    rows = []
    for fr, r in zip(frames, res):
        row = {'i': fr['index'], 't': fr['physicalTimestamp'], 'status': 'LIMITED', 'fresh': False,
               'nativeStatus': r['statusLabel']}
        pose = r.get('pose')
        if r.get('poseValid') and r.get('poseFresh') and pose:
            m = np.array(pose, float).reshape(4, 4)
            row['fresh'] = True
            row['status'] = 'NORMAL'
            row['p'] = m[:3, 3].tolist()
            row['q_wxyz'] = mat_to_quat_wxyz(m[:3, :3] @ d)
        rows.append(row)
    pathlib.Path(trace_path).parent.mkdir(parents=True, exist_ok=True)
    with open(trace_path, 'w') as f:
        for row in rows:
            f.write(json.dumps(row) + '\n')
    n = sum(1 for r in rows if r['fresh'])
    print(f'{trace_path}: {n}/{len(rows)} frames with fresh pose')


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('convert')
    c.add_argument('fixture'); c.add_argument('out'); c.add_argument('--noise', choices=NOISE, required=True)
    t = sub.add_parser('trace')
    t.add_argument('fixture'); t.add_argument('native_out'); t.add_argument('trace')
    a = ap.parse_args()
    if a.cmd == 'convert':
        convert(a.fixture, a.out, a.noise)
    else:
        trace(a.fixture, a.native_out, a.trace)


if __name__ == '__main__':
    sys.exit(main())
