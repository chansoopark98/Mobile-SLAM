#!/usr/bin/env python3
"""Fixture of roi_draw_golden_test.cc (contract change US015-CC-1): the ORIGINAL's GPU readback of planar ROI slots.

  gen_roi_draw_golden.py --glcap <run>/runs/e5-seq-a/o1/glcap --camera-dir <seq-a>/mav0/cam0/data --out roi-draw-golden.bin.gz

Source: the it-scale-v4 ROI pixel diagnosis (build/slam-refact-full-20261019/it-scale-v4/diag/orig-gl2, original xr-slam.wasm with
--gl-capture, README in slam_refact/tools/harness/e5/diag). Capture k = stage call S<k+1>; its camera image is PGM k-1. Per case
(capture frame): the 2 non-identity `mvp` uniforms of the ROI slot draws at viewport row 650 (slot 0 IMAGE_TARGET, slot 1 HIRES_SCAN),
and per window of a slot the original's R bytes plus the camera pixels the window samples (a crop with margin; the test rebuilds a zero
camera and pastes the crops). File = gzip of: "SRROI1\\n", one JSON line, then the blobs the JSON lists in order."""
import argparse, gzip, json, struct
import numpy as np

CASES = [  # (capture frame, [(slot, x, y, w, h)]) : f10 left slanted edge of slot 0, f31 right edge, HIRES interior
    (10, [(0, 0, 0, 64, 336), (1, 88, 100, 96, 136)]),
    (31, [(0, 188, 0, 64, 336)]),
]
CAM_W, CAM_H, MARGIN = 480, 640, 6


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        return np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)


def slot_mvps(ops, frame):
    stage, cur, found = f'S{frame + 1}', None, []
    for o in ops:
        if o[0] != stage:
            continue
        if o[1] == 'viewport':
            cur = o[2:6]
        if o[1] == 'uniformMatrix4fv' and o[4] != [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1] and cur and cur[1] == 650:
            found.append((cur, o[4]))
    return found


def window_crop(mvp, vw, vh, x, y, w, h):
    """Camera pixel box (x0, y0, x1, y1 exclusive) that the window pixels sample (float64 homography of mvp columns {0, 1, 3})."""
    m = np.array(mvp, dtype=np.float64).reshape(4, 4).T  # row-major view of the column-major uniform
    hm = m[[0, 1, 3]][:, [0, 1, 3]]
    inv = np.linalg.inv(hm)
    xs, ys = np.meshgrid(np.arange(x, x + w), np.arange(y, y + h))
    p = inv @ np.stack([2 * ((xs.ravel() + 0.5) / vw) - 1, 2 * ((ys.ravel() + 0.5) / vh) - 1, np.ones(xs.size)])
    px, py = p[0] / p[2], p[1] / p[2]
    ok = (np.abs(px) <= 1) & (np.abs(py) <= 1)
    cx, cy = (px[ok] + 1) / 2 * CAM_W - 0.5, (py[ok] + 1) / 2 * CAM_H - 0.5
    return (max(int(np.floor(cx.min())) - MARGIN, 0), max(int(np.floor(cy.min())) - MARGIN, 0),
            min(int(np.ceil(cx.max())) + MARGIN + 1, CAM_W), min(int(np.ceil(cy.max())) + MARGIN + 1, CAM_H))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glcap', required=True)
    ap.add_argument('--camera-dir', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    ops = json.load(open(f'{a.glcap}/glcap.json'))['ops']
    cases, blobs = [], []
    for frame, windows in CASES:
        cap = np.frombuffer(gzip.open(f'{a.glcap}/cap-{frame:03d}.rgba.gz').read(), dtype=np.uint8).reshape(1024, 1024, 4)
        cam = read_pgm(f'{a.camera_dir}/{frame - 1:05d}.pgm')
        assert cam.shape == (CAM_H, CAM_W)
        slots = slot_mvps(ops, frame)
        rois = [{'source': 2 + si, 'mvp': [float('%.9g' % v) for v in m]} for si, (vp, m) in enumerate(slots[:2])]
        for r in rois:  # the test parses doubles and rounds to float: it must give the uniform's float back
            assert all(float(np.float32(v)) == v or np.float32(v) == np.float32(float('%.9g' % v)) for v in r['mvp'])
        entry = {'frame': frame, 'rois': rois, 'windows': []}
        for slot, x, y, w, h in windows:
            vp, m = slots[slot]
            c, r, vw, vh = vp
            x0, y0, x1, y1 = window_crop(m, vw, vh, x, y, w, h)
            entry['windows'].append({'slot': slot, 'x': x, 'y': y, 'w': w, 'h': h, 'crop': [x0, y0, x1 - x0, y1 - y0]})
            blobs.append(np.ascontiguousarray(cam[y0:y1, x0:x1]).tobytes())
            blobs.append(np.ascontiguousarray(cap[r + y:r + y + h, c + x:c + x + w, 0]).tobytes())
        cases.append(entry)
    head = json.dumps({'camera': [CAM_W, CAM_H], 'cases': cases}, separators=(',', ':'))
    with gzip.GzipFile(a.out, 'wb', compresslevel=9, mtime=0) as f:
        f.write(b'SRROI1\n' + head.encode() + b'\n' + b''.join(blobs))
    print('wrote', a.out)


if __name__ == '__main__':
    main()
