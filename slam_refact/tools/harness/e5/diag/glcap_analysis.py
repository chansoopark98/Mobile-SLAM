#!/usr/bin/env python3
"""Pixel-level comparison of the ORIGINAL's pyramid readback (glcap/cap-NNN.rgba.gz, run_original_itprobe.mjs --gl-capture) with the ENGINE's CPU atlas
(sr_e5_atlas dumps, ITPROBE_ATLAS_DIR) of the same exact-input run.
  glcap_analysis.py compare --glcap <run>/glcap --atlas <engine atlas dir> [--json out.json] [--inject-dir <new dir>]
    association capture k -> engine frame label by the level-0 R plane (rows/cols of levels[0]); per matched frame: differing pixels per channel in the level
    region (rows above the ROI slots) and per ROI slot (count, max |d|, mean |d|, coverage), plus the GL ops summary of the original (viewports, mvp uniforms).
    --inject-dir writes cap-boot.rgba / cap-fNNNN.rgba (the original's atlas bytes, 1024 rows) for ITPROBE_INJECT_DIR."""
import argparse, glob, gzip, json, os, sys
import numpy as np


def load_caps(d):
    caps = []
    for f in sorted(glob.glob(os.path.join(d, 'cap-*.rgba.gz'))):
        caps.append(np.frombuffer(gzip.open(f).read(), dtype=np.uint8).reshape(-1, 1024, 4))
    return caps


def load_atlas(d):
    out = {}
    for f in sorted(glob.glob(os.path.join(d, 'f*.rgba'))):
        n = int(os.path.basename(f)[1:5])
        j = json.load(open(f[:-5] + '.json'))
        out[n] = (np.fromfile(f, dtype=np.uint8).reshape(j['rows'], 1024, 4), j)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('compare'); c.add_argument('--glcap', required=True); c.add_argument('--atlas', required=True); c.add_argument('--json'); c.add_argument('--inject-dir')
    c.add_argument('--max-frame', type=int, default=60)
    a = ap.parse_args()
    caps, eng = load_caps(a.glcap), load_atlas(a.atlas)
    res = {'captures': len(caps), 'frames': {}}
    assoc = {}
    for k, cap in enumerate(caps):
        best = None
        for n, (e, j) in eng.items():
            r, c0, w, h, _ = j['levels'][0]
            dd = int((cap[r:r + h, c0:c0 + w, 0] != e[r:r + h, c0:c0 + w, 0]).sum())
            if best is None or dd < best[1]:
                best = (n, dd)
        assoc[k] = best
    res['association(capture -> frame, level-0 R differing px)'] = {k: v for k, v in assoc.items() if k < 12}
    used = set()
    for k, (n, dd) in sorted(assoc.items()):
        if dd != 0 or n in used or n > a.max_frame:
            continue
        used.add(n)
        e, j = eng[n]
        cap = caps[k]
        rows = min(e.shape[0], cap.shape[0])
        slot_top = min(s[0] for s in j['roi_slots'])
        rec = {'capture': k, 'levels': {}, 'slots': []}
        for ch, name in enumerate('RGBA'):
            d = cap[:slot_top, :, ch].astype(int) - e[:slot_top, :, ch].astype(int)
            nz = np.argwhere(d != 0)
            rec['levels'][name] = {'differing_px': int(len(nz)), 'max_abs': int(np.abs(d).max()), 'rows': sorted(set(int(v) for v in nz[:, 0]))[:8]}
        for si, (r, c0, w, h, _) in enumerate(j['roi_slots']):
            if si >= len(j['rois']) or e.shape[0] < r + h:
                continue
            src = j['rois'][si]['source'] if si < len(j['rois']) else None
            entry = {'slot': si, 'source': src}
            for ch, name in enumerate('RGBA'):
                d = cap[r:r + h, c0:c0 + w, ch].astype(int) - e[r:r + h, c0:c0 + w, ch].astype(int)
                entry[name] = {'differing_px': int(np.count_nonzero(d)), 'max_abs': int(np.abs(d).max()), 'mean_abs': round(float(np.abs(d).mean()), 4)}
            rd = cap[r:r + h, c0:c0 + w, 0].astype(int) - e[r:r + h, c0:c0 + w, 0].astype(int)
            entry['R_coverage'] = {'orig_nonzero': int((cap[r:r + h, c0:c0 + w, 0] != 0).sum()), 'engine_nonzero': int((e[r:r + h, c0:c0 + w, 0] != 0).sum())}
            entry['R_pattern'] = {'signed_hist': {int(v): int((rd == v).sum()) for v in range(-3, 4)}}
            rec['slots'].append(entry)
        res['frames'][n] = rec
    summ = {'levels_R_B_A_identical_frames': sum(1 for r in res['frames'].values() if all(r['levels'][c]['differing_px'] == 0 for c in 'RBA')),
            'frames_compared': len(res['frames'])}
    for src, name in ((3, 'HIRES_SCAN'), (2, 'IMAGE_TARGET')):
        px = [s['R']['differing_px'] for r in res['frames'].values() for s in r['slots'] if s['source'] == src]
        mx = [s['R']['max_abs'] for r in res['frames'].values() for s in r['slots'] if s['source'] == src]
        summ[name] = {'slots': len(px), 'slots_byte_identical': sum(1 for v in px if v == 0), 'differing_px_total': sum(px), 'max_abs': max(mx) if mx else None}
    res['summary'] = summ
    if a.inject_dir:
        os.makedirs(a.inject_dir, exist_ok=False)
        caps[0].tofile(os.path.join(a.inject_dir, 'cap-boot.rgba'))
        for k, (n, dd) in assoc.items():
            if dd == 0 and n <= a.max_frame:
                caps[k].tofile(os.path.join(a.inject_dir, f'cap-f{n:04d}.rgba'))
        res['injected'] = sorted(os.listdir(a.inject_dir))[:3]
    print(json.dumps(res['summary'], indent=1))
    if a.json:
        open(a.json, 'w').write(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
