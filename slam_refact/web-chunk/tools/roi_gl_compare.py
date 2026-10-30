#!/usr/bin/env python3
"""GL ROI slot pixels versus the independent numpy oracle (RM-4c/RM-4d/RM-4e gate rm4c-chunk-gate.v4.md, G-GL: GL-A1..A4, GL-E1..E6).

The oracle is `slam_refact/tools/image_target/roi_reference.py` (RM-6, not written by the RM-4c executor): GL semantics of
one planar ROI slot written out in float64 (inverse of the 3x3 homography of the mvp rows / columns {0, 1, 3}, quad
coverage |p| <= 1 with clip w > 0, bilinear camera sample at uv * size - 0.5, rounded to 8 bit). Only numpy is required.

  prepare --out DIR --kind gate|smoke|const|ramp [--mutate MF|MZ|MH]   camera.pgm + job.json (the warps the GL page draws) +
                                                                       expected.json (the warps the oracle uses)
  compare --dir DIR [--check]                                           reads gl_result.json (written by run-chunk-gl.mjs),
                                                                       writes metrics.json; --check exits 1 on a violation
  target-alignment --dir DIR --texture PGM                              GL-E4 (REPORT): e of IMAGE_TARGET slots of roiCheck samples
  it-compare --dir DIR [--check] [--no-targets]                         GL-E1..E3, E5, E6 (and the no-target null-run N-E) from it_result.json

Kinds: gate = SEED_GL 59 camera (v3: a seed no GL run had used; the first real run, held until a reviewer is recorded),
smoke = SEED_SMOKE 77 (device check of the harness; also the textured null-run), const (camera 100) and ramp = the null-run
inputs N-GL1 / N-GL2. Mutations act on the warps the GL page receives; the oracle keeps the original warps (the draw has the
bug, the expectation does not). v3: mutations are refused for kind gate (they run on smoke data only).
"""
import argparse
import base64
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../tools/image_target'))
import render_synthetic as rs  # noqa: E402
import roi_reference as rr  # noqa: E402

SEED_GL = 59  # v3 (v2 registered 41; its camera was used only as the input of mutant runs, never as a gate run)
SEED_SMOKE = 77
# gate section 1. v1 registered A2 (absolute max) <= 6; the smoke / null-run showed that sharp edges move by up to 16 levels with
# sub-texel sampling differences while smooth content differs by 1, so v2 registers A2n (contrast-normalised) instead.
# v3: A3 (mean absolute level) is REPORT; the gated mean is A3n (contrast-normalised mean excess), see the gate file section 2.
# v4 (RM-4e, review G-1): the registered A3n tolerance is 0.03 (v3 md and manifest); this file said 0.05 in v3, so a gate run would have judged
# against a looser value than the registered one. `registered_values()` below is compared with the manifest by a unit test.
TOL = {'A1': 0.005, 'A2n': 3, 'A3n': 0.03, 'A4': 0}
EDGE_FRACTION = 0.25  # an edge pixel may differ by up to this fraction of the local 3x3 range (a quarter texel of position error)


def local_range(img):
    """max - min over the 3x3 neighbourhood of every pixel (edge-replicated)."""
    f = np.pad(img.astype(np.int32), 1, mode='edge')
    stack = np.stack([f[dy:dy + img.shape[0], dx:dx + img.shape[1]] for dy in range(3) for dx in range(3)])
    return stack.max(axis=0) - stack.min(axis=0)


def camera_gl(seed):
    """Same formula as roi_reference.camera_a('textured') with another seed (gate: the oracle's SEED_A data was not reused)."""
    gy, gx = np.mgrid[0:rr.CAM_H, 0:rr.CAM_W].astype(np.float64)
    rng = np.random.default_rng(seed)
    img = 70.0 + 50.0 * gx / rr.CAM_W + 40.0 * gy / rr.CAM_H
    for _ in range(140):
        w, h = rng.integers(10, 70, size=2)
        x, y = rng.integers(0, rr.CAM_W - 4), rng.integers(0, rr.CAM_H - 4)
        img[y:y + h, x:x + w] = rng.uniform(20, 255)
    img += rng.normal(0.0, 3.0, size=img.shape)
    return np.clip(np.rint(img), 16, 255).astype(np.uint8)


def camera_for(kind):
    if kind == 'gate':
        return camera_gl(SEED_GL)
    if kind == 'smoke':
        return camera_gl(SEED_SMOKE)
    return rr.camera_a('constant' if kind == 'const' else 'ramp')


def mutated(name, m):
    """Warp the GL page draws (row-major 4x4), applied to the oracle's warp (gate section 3)."""
    if name is None:
        return m
    if name == 'MF':
        return np.diag([1.0, -1.0, 1.0, 1.0]) @ m
    if name == 'MZ':
        return np.diag([1.15, 1.15, 1.0, 1.0]) @ m
    if name == 'MH':
        t = np.eye(4)
        t[0, 3] = 0.06
        return t @ m
    raise SystemExit(f'unknown mutation {name}')


def col_major(m):
    return [float(v) for v in np.asarray(m, dtype=np.float32).T.reshape(-1)]


def prepare(a):
    os.makedirs(a.out, exist_ok=True)
    cam = camera_for(a.kind)
    rs.write_pgm(os.path.join(a.out, 'camera.pgm'), cam)
    warps = rr.warps_a()
    job = {'kind': a.kind, 'mutation': a.mutate, 'width': rr.CAM_W, 'height': rr.CAM_H,
           'camera_b64': base64.b64encode(cam.tobytes()).decode(),
           'warps': [col_major(mutated(a.mutate, m)) for m in warps]}
    json.dump(job, open(os.path.join(a.out, 'job.json'), 'w'))
    json.dump({'kind': a.kind, 'mutation': a.mutate, 'warps_row_major': [m.reshape(-1).tolist() for m in warps]},
              open(os.path.join(a.out, 'expected.json'), 'w'))
    print(f'wrote {a.out} (kind {a.kind}, mutation {a.mutate})')
    return 0


def plane(entry):
    return np.frombuffer(base64.b64decode(entry['r_b64']), dtype=np.uint8).reshape(entry['h'], entry['w'])


def a4_pixels(entries):
    """GL-A4: the number of non-zero pixels left in slots that must have been cleared (integer count, tolerance 0)."""
    return sum(int(plane(e).astype(bool).sum()) for e in entries)


def metrics_for(cam, m, entry):
    actual = plane(entry)
    exp, mask = rr.slot_expected(cam, m, entry['w'], entry['h'])
    cov_a, cov_e = actual > 0, mask > 0
    both = cov_a & cov_e
    diff = np.abs(actual.astype(int) - exp.astype(int))
    d = diff[both]
    excess = (diff - EDGE_FRACTION * local_range(exp))[both]
    return {'A1': float((cov_a != cov_e).mean()), 'A2n': float(excess.max()) if d.size else None,
            'A3n': float(np.maximum(excess, 0).mean()) if d.size else None,
            'A3_mean_abs_REPORT': float(d.mean()) if d.size else None,
            'A2_abs_max_REPORT': int(d.max()) if d.size else None,
            'covered_actual': int(cov_a.sum()), 'covered_expected': int(cov_e.sum()), 'both': int(both.sum())}


def compare(a):
    job = json.load(open(os.path.join(a.dir, 'job.json')))
    exp = json.load(open(os.path.join(a.dir, 'expected.json')))
    res = json.load(open(os.path.join(a.dir, 'gl_result.json')))
    cam = np.frombuffer(base64.b64decode(job['camera_b64']), dtype=np.uint8).reshape(job['height'], job['width'])
    rows = [metrics_for(cam, np.array(m).reshape(4, 4), res['a'][i]) for i, m in enumerate(exp['warps_row_major'])]
    a4 = a4_pixels(res['a4'])
    worst = {'A1': max(r['A1'] for r in rows), 'A2n': max((r['A2n'] for r in rows if r['A2n'] is not None), default=None),
             'A3n': max((r['A3n'] for r in rows if r['A3n'] is not None), default=None), 'A4': a4,
             'A3_mean_abs_REPORT': max((r['A3_mean_abs_REPORT'] for r in rows if r['A3_mean_abs_REPORT'] is not None), default=None),
             'A2_abs_max_REPORT': max((r['A2_abs_max_REPORT'] for r in rows if r['A2_abs_max_REPORT'] is not None), default=None)}
    violations = [k for k in ('A1', 'A2n', 'A3n', 'A4') if worst[k] is None or worst[k] > TOL[k]]
    out = {'kind': job['kind'], 'mutation': job['mutation'], 'renderer': res.get('renderer'), 'tolerance': TOL,
           'gate_metrics': ['A1', 'A2n', 'A3n', 'A4'], 'report_metrics': ['A2_abs_max_REPORT', 'A3_mean_abs_REPORT'], 'per_warp': rows, 'worst': worst,
           'violations': violations, 'verdict': 'PASS' if not violations else 'FAIL'}
    json.dump(out, open(os.path.join(a.dir, 'metrics.json'), 'w'), indent=1)
    print(json.dumps({'kind': job['kind'], 'mutation': job['mutation'], 'worst': worst, 'violations': violations, 'verdict': out['verdict']}))
    return 1 if (a.check and violations) else 0


def target_alignment(a):
    """GL-E4 (REPORT): RMS(GL slot - upright target) / std over the middle 80 % of IMAGE_TARGET slots (RM-6 B1 form)."""
    tex = np.frombuffer(open(a.texture, 'rb').read().split(b'255\n', 1)[1], dtype=np.uint8).reshape(rr.CAM_H, rr.CAM_W)
    res = json.load(open(os.path.join(a.dir, 'it_result.json')))
    es = []
    for sample in (res.get('roiCheck') or res.get('chunk', {}).get('roiCheck', [])):
        for roi, slot in zip(sample['rois'], sample['slots']):
            if roi['source'] != 'IMAGE_TARGET':
                continue
            act = np.frombuffer(base64.b64decode(slot['r_b64']), dtype=np.uint8).reshape(slot['h'], slot['w']).astype(np.float64)
            e = rr.target_expected(tex, slot['w'], slot['h']).astype(np.float64)
            h, w = e.shape
            sl = (slice(int(0.1 * h), int(np.ceil(0.9 * h))), slice(int(0.1 * w), int(np.ceil(0.9 * w))))
            es.append(float(np.sqrt(np.mean((act[sl] - e[sl]) ** 2)) / e[sl].std()))
    out = {'samples_with_image_target_roi': len(es), 'e_p50': float(np.median(es)) if es else None, 'e_max': max(es) if es else None,
           'e_min': min(es) if es else None, 'class': 'REPORT'}
    json.dump(out, open(os.path.join(a.dir, 'alignment.json'), 'w'), indent=1)
    print(json.dumps(out))
    return 0


E1_WINDOWS = {'found1': (12, 30), 'lost': (72, 90), 'found2': (96, 114)}  # gate v1 section 2 (stage / source frame)
E2_MAX_DELTA = 4
# v3 GL-E5: the detected target position (m) of the chunk path (GL ROI draw) against the engine's CPU path (CPU ROI draw) for the same
# source frame. Sensitive to the IMAGE_TARGET slot: a chunk without the ROI warp draw moves it by up to 0.104 m (RM-4c review, smoke seed 14).
E5_MAX_POSITION_DELTA_M = 0.01
# v4 (review G-2): E5 sees the IMAGE_TARGET slot (the tracking ROI, the first of the engine's list) only; the scan ROIs of the HIRES slots are
# not seen (a stage that drew only the first ROI passed E1/E2/E3/E5). It also needs a minimum number of sources detected by both paths
# (null-run seed 14: 88), otherwise a single matched frame would pass.
E5_MIN_MATCHED_SOURCES = 80
# v4 GL-E6: the ROI wiring of the chunk path in integers (see e6_wiring). Zero mismatches; at least this many stages with ROIs (null-run 129).
E6_MAX_MISMATCHES = 0
E6_MIN_ROI_STAGES = 100
# Quads of one Gr8ComputePyramid stage that do not depend on the ROIs: input blur 1, level 0 1, 3 x (blur + decimate) 6, stage 2 and 3 2.
E6_FIXED_QUADS = 10


def registered_values():
    """The tolerances this file applies, in the key space of the manifest's `tolerance` (a unit test compares it with the v4 manifest)."""
    return {'GL_A1_coverage_fraction': TOL['A1'], 'GL_A2n_level': TOL['A2n'], 'GL_A2n_edge_fraction': EDGE_FRACTION, 'GL_A3n_mean_level': TOL['A3n'],
            'GL_A4_nonzero_cleared_pixels': TOL['A4'], 'GL_E1_windows_source_frame': {k: list(v) for k, v in E1_WINDOWS.items()},
            'GL_E3_event_and_error_mismatches': 0, 'GL_E5_max_position_delta_m': E5_MAX_POSITION_DELTA_M,
            'GL_E5_min_matched_sources': E5_MIN_MATCHED_SOURCES, 'GL_E6_wiring_mismatches': E6_MAX_MISMATCHES, 'GL_E6_min_roi_stages': E6_MIN_ROI_STAGES}


def e5_position(chunk, cpu):
    """chunk: it_result['chunk']['answers'] (per rAF, with the source frame of the response), cpu: it_result['cpu']['detections']."""
    by_chunk = {}
    for ans in chunk:
        if ans and ans.get('source') is not None and ans['source'] not in by_chunk:
            by_chunk[ans['source']] = ans.get('images') or []
    by_cpu = {d['source']: d['images'] for d in cpu}
    both, only_chunk, only_cpu, deltas = 0, 0, 0, []
    for src in sorted(set(by_chunk) | set(by_cpu)):
        c, g = by_chunk.get(src) or [], by_cpu.get(src) or []
        if c and g:
            both += 1
            a, b = c[0]['position'], g[0]['position']
            deltas.append(float(np.sqrt((a['x'] - b['x']) ** 2 + (a['y'] - b['y']) ** 2 + (a['z'] - b['z']) ** 2)))
        elif c:
            only_chunk += 1
        elif g:
            only_cpu += 1
    return {'matched_sources': both, 'only_chunk_REPORT': only_chunk, 'only_cpu_REPORT': only_cpu,
            'max_delta_m': max(deltas) if deltas else None, 'mean_delta_m_REPORT': float(np.mean(deltas)) if deltas else None,
            'tolerance_m': E5_MAX_POSITION_DELTA_M, 'min_matched_sources': E5_MIN_MATCHED_SOURCES, 'scope': 'IMAGE_TARGET slot',
            'pass': both >= E5_MIN_MATCHED_SOURCES and max(deltas) <= E5_MAX_POSITION_DELTA_M}


def e6_pass(e6):
    """GATE verdict of GL-E6: the probe saw the stages, no mismatch, and enough stages carried ROIs."""
    return bool(e6['instrumented'] and e6['mismatches'] <= E6_MAX_MISMATCHES and e6['roi_stages'] >= E6_MIN_ROI_STAGES)


def e6_wiring(stages, n_slots, readback):
    """GL-E6 (v4): the ROI wiring of the chunk path in integers. `stages` = it_result['chunk']['stages'], one entry per Gr8ComputePyramid.stage:
    asked = the list the engine's getRequestedRois() returned immediately before, passed = the list stage() received, draws = drawElements calls
    of the compute context during stage(). `readback` = the pyramid's own counters at the end. Expected per stage (pyramid code, drawInitialStage):
    every ROI on a valid slot not yet used is cleared and, unless it is CURVY_IMAGE_TARGET, drawn; the other slots are cleared. One ROI = one
    counter (roiDraws or roiSkipped), so the totals follow from the engine's lists alone."""
    list_mism = draw_mism = 0
    exp_draws = exp_skipped = roi_stages = 0
    for st in stages:
        asked, passed = st['asked'], st['passed']
        if asked != passed:
            list_mism += 1
        asked = asked or []  # a stage without an ask of its own (None) is a list mismatch above and makes `instrumented` false below
        seen, nd, ns = set(), 0, 0
        for r in asked:
            if not (0 <= r['slot'] < n_slots) or r['slot'] in seen:
                ns += 1
                continue
            seen.add(r['slot'])
            if r['source'] == 'CURVY_IMAGE_TARGET':
                ns += 1
            else:
                nd += 1
        if st['draws'] != E6_FIXED_QUADS + n_slots + nd:
            draw_mism += 1
        exp_draws += nd
        exp_skipped += ns
        roi_stages += 1 if asked else 0
    counter_mism = int(readback.get('roiDraws') != exp_draws) + int(readback.get('roiSkipped') != exp_skipped) + int(readback.get('roiStages') != roi_stages)
    total = list_mism + draw_mism + counter_mism
    return {'stages': len(stages), 'roi_stages': roi_stages, 'n_slots': n_slots, 'list_mismatches': list_mism, 'draw_mismatches': draw_mism,
            'counter_mismatches': counter_mism, 'mismatches': total, 'expected_roi_draws': exp_draws, 'expected_roi_skipped': exp_skipped,
            'observed_roi_draws': readback.get('roiDraws'), 'observed_roi_skipped': readback.get('roiSkipped'),
            'min_roi_stages': E6_MIN_ROI_STAGES, 'tolerance_mismatches': E6_MAX_MISMATCHES,
            'instrumented': bool(stages) and all(st.get('draws') is not None and st.get('asked') is not None for st in stages)}


def it_compare(a):
    res = json.load(open(os.path.join(a.dir, 'it_result.json')))
    chunk = res['chunk']
    ev = [e for e in chunk['events'] if e['name'] in ('imagefound', 'imageupdated', 'imagelost')]
    fl = [e for e in ev if e['name'] != 'imageupdated']
    out = {'chunk_events': [[e['name'], e['source'], e['k']] for e in ev][:200], 'cpu_events': res['cpu']['events'][:200],
           'png_round_trip': res.get('pngRoundTrip'), 'renderer': res.get('renderer'), 'errors': chunk['errors'], 'warnings': chunk['warnings'],
           'engine_image_events_ignored': chunk['stats'].get('engineImageEventsIgnored'), 'readback': chunk.get('readback')}
    if a.no_targets:
        out['N_E'] = {'image_events': len(ev), 'detections': sum(len((x or {}).get('images', [])) for x in chunk['answers']),
                      'roi_stages': (chunk.get('readback') or {}).get('roiStages'), 'readback_bytes': (chunk.get('readback') or {}).get('readbackBytes'),
                      'layout_rows_bytes': None}
        # v4: E6 on a sequence without targets (no ROI is asked for): the fixed quad count per stage is checked here against the engine's empty lists
        ne6 = e6_wiring(chunk.get('stages') or [], chunk.get('nSlots') or 0, chunk.get('readback') or {})
        out['N_E']['E6'] = ne6
        out['verdict'] = ('PASS' if out['N_E']['image_events'] == 0 and out['N_E']['detections'] == 0 and not out['N_E']['roi_stages']
                          and ne6['instrumented'] and ne6['mismatches'] == 0 and ne6['roi_stages'] == 0 else 'FAIL')
    else:
        names = [e['name'] for e in fl]
        e1 = {'names': names, 'ok_names': names == ['imagefound', 'imagelost', 'imagefound']}
        if e1['ok_names']:
            f1, lo, f2 = (e['source'] for e in fl)
            e1['frames'] = {'found1': f1, 'lost': lo, 'found2': f2}
            e1['in_windows'] = all(E1_WINDOWS[k][0] <= v <= E1_WINDOWS[k][1] for k, v in e1['frames'].items())
        e1['pass'] = bool(e1.get('ok_names') and e1.get('in_windows'))
        cpu = [e for e in res['cpu']['events'] if e['name'] != 'imageupdated']
        e2 = {'cpu': [[e['name'], e['source']] for e in cpu], 'chunk': [[e['name'], e['source']] for e in fl]}
        e2['pass'] = [x[0] for x in e2['cpu']] == [x[0] for x in e2['chunk']] and all(abs(x[1] - y[1]) <= E2_MAX_DELTA for x, y in zip(e2['cpu'], e2['chunk']))
        # E3: one event per change of the answers the app saw (independent recomputation from the per-call summaries)
        seen, expected = {}, []
        for ans in chunk['answers']:
            names_now = set()
            for img in (ans or {}).get('images', []):
                names_now.add(img['name'])
                if img['name'] not in seen: expected.append('imagefound')
                elif json.dumps(seen[img['name']], sort_keys=True) != json.dumps(img, sort_keys=True): expected.append('imageupdated')
                seen[img['name']] = img
            for k in [k for k in seen if k not in names_now]:
                expected.append('imagelost'); del seen[k]
        e3 = {'events': len(ev), 'recomputed': len(expected), 'pass': [e['name'] for e in ev] == expected}
        e5 = e5_position(chunk['answers'], res['cpu'].get('detections') or [])
        e6 = e6_wiring(chunk.get('stages') or [], chunk.get('nSlots') or 0, chunk.get('readback') or {})
        e6['pass'] = e6_pass(e6)
        out.update({'E1': e1, 'E2': e2, 'E3': e3, 'E5': e5, 'E6': e6, 'png_round_trip_equal': bool((res.get('pngRoundTrip') or {}).get('equal'))})
        # v4 classes: GATE = E1 (wiring, known answer), E3 (wiring), E5 (IMAGE_TARGET slot), E6 (ROI wiring, integers), no errorCallback;
        # REPORT = E2, E4, the png round trip
        out['gate_metrics'] = ['E1', 'E3', 'E5', 'E6', 'errors']
        out['report_metrics'] = ['E2', 'E4', 'png_round_trip_equal']
        out['verdict'] = 'PASS' if e1['pass'] and e3['pass'] and e5['pass'] and e6['pass'] and not chunk['errors'] else 'FAIL'
    json.dump(out, open(os.path.join(a.dir, 'it_metrics.json'), 'w'), indent=1)
    print(json.dumps({k: out.get(k) for k in ('E1', 'E2', 'E3', 'E5', 'E6', 'N_E', 'png_round_trip_equal', 'verdict')}))
    return 1 if (a.check and out['verdict'] != 'PASS') else 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--out', required=True)
    p.add_argument('--kind', choices=['gate', 'smoke', 'const', 'ramp'], required=True)
    p.add_argument('--mutate', choices=['MF', 'MZ', 'MH'])
    c = sub.add_parser('compare')
    c.add_argument('--dir', required=True)
    c.add_argument('--check', action='store_true')
    t = sub.add_parser('target-alignment')
    t.add_argument('--dir', required=True)
    t.add_argument('--texture', required=True)
    i = sub.add_parser('it-compare')
    i.add_argument('--dir', required=True)
    i.add_argument('--check', action='store_true')
    i.add_argument('--no-targets', action='store_true')
    a = ap.parse_args(argv)
    return {'prepare': prepare, 'compare': compare, 'target-alignment': target_alignment, 'it-compare': it_compare}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
