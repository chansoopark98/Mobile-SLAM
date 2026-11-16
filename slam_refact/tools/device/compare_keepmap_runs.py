#!/usr/bin/env python3
"""Compare phone metrics JSONs (poseLog schema v4, page opened with ?record=1&diag=1) of default / K1 (?keepmap=1) / K1F (?keepmap=fast) runs.

usage: compare_keepmap_runs.py <metrics.json> [<metrics.json> ...] [--json out.json]

Definitions (map discards and the gyro gate follow build/xr-tracker-diag-20261116/tools/analyze_tracker_diag.py):
  frame            one row per (tracker.session, tracker.sourceFrameIndex), first row wins; time = row.tPerfMs.
  first map        first frame with mapTrackerState == 1. "Lost" frames are frames after it with mapTrackerState != 1
                   (frames of a rebuilt session before its map is tracked again count as lost; the reference
                   analyzer's engine counter totalMapTrackerNotOk also counts the frames before the first map, so it is larger).
  lost episode     maximal run of consecutive lost frames inside one session. Seconds = from the first lost frame to the
                   next frame after the episode (the recovery frame); the last frame's own time if the run ends the log.
  map discard      mapReset after the map had been tracked in that session (the first map of a session is not counted).
                   Gyro gate: peak gyroPeakDegS over the 10 frames up to the loss onset > 80 deg/s.
  extension kf     keyframeCreated and not mapReset and mapTrackerState != 1 (K1/K1F add a keyframe on a lost frame).
  latency          engineReturnedAtMs - sourceStageTimeNanos / 1e6 of the row's source, split by the frame's map state.
  same-view error  reference frames = map-tracked frames with a position within 2 s after the first map is tracked.
                   A later map-tracked frame b (after that window) is a pair with the reference frame a whose attitude is
                   closest and within 5 deg of b's (angle = 2*acos|q_a . q_b|), provided a lost frame lies strictly between
                   a and b (>= 1 lost episode since a). Error = |P_b - P_a| in page-relative units (NOT metres; no ground
                   truth). One pair per frame b. Thirds are by time of b over the run (first map tracked .. end).
  path / net       over consecutive frame pairs after the first map (same session, positions present), attributed to the
                   state of the later frame: path = sum |dP|, net = |sum dP|.
Numbers are measurements of one run each; they carry no verdict."""
import argparse
import json
import math
import statistics
import sys

GYRO_KF, LOOKBACK, REF_WINDOW_S, VIEW_DEG = 80.0, 10, 2.0, 5.0
SCHEMA = 'slam-refact-xr-poselog-v4'
NOT_RECORDED = 'default (not recorded)'


class LogError(Exception):
    pass


def pct(v, p):
    """Linear-interpolated percentile (p in 0..100) of a list; None when empty."""
    if not v:
        return None
    s = sorted(v)
    k = (len(s) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def vec(p):
    return None if not p else (p['x'], p['y'], p['z'])


def quat(r):
    return None if not r else (r['x'], r['y'], r['z'], r['w'])


def load_frames(doc):
    log = doc.get('poseLog') or {}
    if log.get('schema') != SCHEMA:
        raise LogError(f'not a v4 pose log (schema {log.get("schema")!r}): open the page with ?record=1&diag=1')
    frames, seen = [], set()
    for r in log.get('rows') or []:
        t = r.get('tracker')
        if not t or t.get('sourceFrameIndex') is None:
            continue
        key = (t.get('session'), t['sourceFrameIndex'])
        if key in seen:
            continue
        seen.add(key)
        src = r.get('source') or {}
        lat = None
        if src.get('engineReturnedAtMs') is not None and src.get('sourceStageTimeNanos') is not None:
            lat = src['engineReturnedAtMs'] - src['sourceStageTimeNanos'] / 1e6
        frames.append({**t, 't': r['tPerfMs'] / 1e3, 'pos': vec(r.get('position')), 'q': quat(r.get('rotation')), 'lat': lat})
    if len(frames) < 2:
        raise LogError(f'{len(frames)} tracker rows: nothing to analyse (open the page with ?record=1&diag=1)')
    return frames, log


def readback_summary(doc):
    """(active flag names or None, warning or None)."""
    mode = doc.get('engineMode') or NOT_RECORDED
    rb = doc.get('engineReadback')
    active = None
    if isinstance(rb, dict) and rb.get('available'):
        active = sorted(k for k, f in (rb.get('flags') or {}).items() if f.get('effective') and not f.get('ignored'))
        if rb.get('k1KeepMapActive'):
            active = sorted(set(active) | {'k1KeepMap'})
    warning = None
    if mode in ('K1', 'K1F'):
        if not isinstance(rb, dict) or not rb.get('available'):
            warning = f'mode {mode} requested but the engine read-back is not available: cannot confirm the flags are active'
        else:
            bad = sorted(k for k, f in (rb.get('flags') or {}).items() if not f.get('effective') or f.get('ignored'))
            if not rb.get('k1KeepMapActive'):
                bad = sorted(set(bad) | {'k1KeepMap'})
            if rb.get('allProductFlagsActive') is False and not bad:
                bad = ['(allProductFlagsActive false)']
            if bad:
                warning = f'mode {mode} requested but the engine does not report these flags effective: {", ".join(bad)}'
    return mode, active, warning


def episodes(fr, first_map):
    """Lost episodes as (start_idx, end_idx_inclusive); frames after first_map with state != 1, split at session changes."""
    out, i, n = [], first_map + 1, len(fr)
    while i < n:
        if fr[i]['mapTrackerState'] != 1:
            j = i
            while j + 1 < n and fr[j + 1]['mapTrackerState'] != 1 and fr[j + 1].get('session') == fr[i].get('session'):
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def discards(fr):
    """Same walk as analyze_tracker_diag.py; returns list of (frame index, gyro_gated)."""
    out = []
    for i, f in enumerate(fr):
        if not (f['mapReset'] and i > 0):
            continue
        onset = i
        while (onset > 0 and fr[onset - 1]['mapTrackerState'] != 1 and not fr[onset - 1]['mapReset']
               and fr[onset - 1].get('session') == f.get('session')):
            onset -= 1
        if onset == 0 or fr[onset - 1].get('session') != f.get('session'):
            continue
        peaks = [g['gyroPeakDegS'] for g in fr[max(onset - LOOKBACK + 1, 0):onset + 1] if g['gyroPeakDegS'] is not None]
        out.append((i, bool(peaks) and max(peaks) > GYRO_KF))
    return out


def qangle_deg(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2 * math.degrees(math.acos(min(d, 1.0)))


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def stats3(v):
    return {'n': len(v), 'median': pct(v, 50), 'p90': pct(v, 90), 'max': max(v) if v else None}


def same_view(fr, first_map, lost_prefix):
    t0 = fr[first_map]['t']
    ref = [i for i in range(first_map, len(fr)) if fr[i]['t'] - t0 <= REF_WINDOW_S and fr[i]['mapTrackerState'] == 1
           and fr[i]['pos'] and fr[i]['q'] and fr[i].get('session') == fr[first_map].get('session')]
    start = (ref[-1] + 1) if ref else len(fr)
    span = max(fr[-1]['t'] - t0, 1e-9)
    errs, thirds = [], [[], [], []]
    for b in range(start, len(fr)):
        fb = fr[b]
        if fb['mapTrackerState'] != 1 or not fb['pos'] or not fb['q']:
            continue
        best = None
        for a in ref:
            if lost_prefix[b] - lost_prefix[a + 1] <= 0:  # no lost frame strictly between a and b
                continue
            ang = qangle_deg(fr[a]['q'], fb['q'])
            if ang <= VIEW_DEG and (best is None or ang < best[0]):
                best = (ang, a)
        if best is None:
            continue
        e = dist(fb['pos'], fr[best[1]]['pos'])
        errs.append(e)
        thirds[min(int(3 * (fb['t'] - t0) / span), 2)].append(e)
    return {'reference_frames': len(ref), 'all': stats3(errs), 'thirds': [stats3(x) for x in thirds]}


def path_net(fr, first_map, lost_flags):
    acc = {'lost': [0, 0.0, [0.0, 0.0, 0.0], 0.0], 'tracked': [0, 0.0, [0.0, 0.0, 0.0], 0.0]}  # frames, path, net vec, seconds
    for i in range(first_map + 1, len(fr)):
        a, b = fr[i - 1], fr[i]
        if a.get('session') != b.get('session') or not a['pos'] or not b['pos']:
            continue
        k = 'lost' if lost_flags[i] else 'tracked'
        d = [y - x for x, y in zip(a['pos'], b['pos'])]
        acc[k][0] += 1
        acc[k][1] += math.sqrt(sum(x * x for x in d))
        acc[k][2] = [x + y for x, y in zip(acc[k][2], d)]
        acc[k][3] += b['t'] - a['t']
    return {k: {'frames': v[0], 'seconds': v[3], 'path': v[1], 'net': math.sqrt(sum(x * x for x in v[2])),
                'path_per_frame': (v[1] / v[0]) if v[0] else None} for k, v in acc.items()}


def analyse(path):
    with open(path) as fh:
        doc = json.load(fh)
    fr, log = load_frames(doc)
    n = len(fr)
    mode, active, warning = readback_summary(doc)
    first_map = next((i for i, f in enumerate(fr) if f['mapTrackerState'] == 1), None)
    dur = fr[-1]['t'] - fr[0]['t']
    disc = discards(fr)
    ext = [bool(f['keyframeCreated']) and not f['mapReset'] and f['mapTrackerState'] != 1 for f in fr]
    run = best = 0
    for e in ext:
        run = run + 1 if e else 0
        best = max(best, run)
    gyro = [f['gyroPeakDegS'] for f in fr if f['motionEvents'] and f['gyroPeakDegS'] is not None]
    res = {
        'file': path, 'engineMode': mode, 'activeFlags': active, 'warning': warning,
        'durationS': dur, 'trackedFrames': n, 'sessions': sorted({f.get('session') for f in fr}, key=str),
        'mapDiscards': len(disc), 'discardsAfterGyroOver80': sum(1 for _, g in disc if g),
        'extensionKeyframes': sum(ext), 'longestExtensionRun': best,
        'keyframesMax': max(f['numKeyframes'] for f in fr),
        'gyroPeakDegS': {'p50': pct(gyro, 50), 'p90': pct(gyro, 90), 'max': max(gyro) if gyro else None, 'frames': len(gyro)},
        'noMotionEventPct': 100.0 * sum(1 for f in fr if f['motionEvents'] == 0) / n,
        'firstMapFrame': first_map,
    }
    if first_map is None:
        res.update(mapLostFrames=None, lostEpisodes=None, note='the map was never tracked in this run')
        return res
    lost = [i > first_map and f['mapTrackerState'] != 1 for i, f in enumerate(fr)]
    eps = episodes(fr, first_map)
    ep_frames = [j - i + 1 for i, j in eps]
    ep_s = [(fr[j + 1]['t'] if j + 1 < n else fr[j]['t']) - fr[i]['t'] for i, j in eps]
    res.update(mapLostFrames=sum(lost), lostEpisodes=len(eps),
               episodeFrames={'median': pct(ep_frames, 50), 'max': max(ep_frames) if ep_frames else None},
               episodeSeconds={'median': pct(ep_s, 50), 'max': max(ep_s) if ep_s else None})
    lat = {'tracked': [f['lat'] for i, f in enumerate(fr) if i >= first_map and not lost[i] and f['lat'] is not None],
           'lost': [f['lat'] for i, f in enumerate(fr) if lost[i] and f['lat'] is not None]}
    res['latencyMs'] = {k: {'n': len(v), 'p50': pct(v, 50), 'p95': pct(v, 95), 'max': max(v) if v else None} for k, v in lat.items()}
    pre = [0]
    for x in lost:
        pre.append(pre[-1] + (1 if x else 0))  # pre[i] = lost frames among frames < i
    res['sameView'] = same_view(fr, first_map, pre)
    res['pathNet'] = path_net(fr, first_map, lost)
    return res


def f(x, d=1, unit=''):
    return 'n/a' if x is None else f'{x:.{d}f}{unit}'


def rows_of(r):
    lat = r.get('latencyMs')
    sv = r.get('sameView')
    pn = r.get('pathNet')
    out = [('engine mode', r['engineMode']),
           ('flags active', ','.join(r['activeFlags']) if r['activeFlags'] else ('none' if r['activeFlags'] == [] else 'no read-back')),
           ('duration s / tracked frames / sessions', f'{r["durationS"]:.1f} / {r["trackedFrames"]} / {len(r["sessions"])}'),
           ('map discards', r['mapDiscards']),
           ('  preceded by gyro > 80 deg/s', f'{r["discardsAfterGyroOver80"]}/{r["mapDiscards"]}'),
           ('extension keyframes', r['extensionKeyframes']),
           ('  longest consecutive run', r['longestExtensionRun'])]
    if r['mapLostFrames'] is None:
        out.append(('map-lost frames', 'n/a (map never tracked)'))
    else:
        out += [('map-lost frames', r['mapLostFrames']), ('lost episodes', r['lostEpisodes']),
                ('  episode frames median / max', f'{f(r["episodeFrames"]["median"])} / {f(r["episodeFrames"]["max"], 0)}'),
                ('  episode seconds median / max', f'{f(r["episodeSeconds"]["median"], 2)} / {f(r["episodeSeconds"]["max"], 2)}')]
    out += [('keyframes max', r['keyframesMax']),
            ('gyro peak p50 / p90 / max deg/s', f'{f(r["gyroPeakDegS"]["p50"], 0)} / {f(r["gyroPeakDegS"]["p90"], 0)} / {f(r["gyroPeakDegS"]["max"], 0)}'),
            ('frames without DeviceMotion %', f'{r["noMotionEventPct"]:.0f}')]
    if lat:
        for k, label in (('tracked', 'map-tracked'), ('lost', 'map-lost')):
            v = lat[k]
            out.append((f'latency ms {label} p50/p95/max (n)', f'{f(v["p50"])} / {f(v["p95"])} / {f(v["max"])} ({v["n"]})'))
    if sv:
        a = sv['all']
        out.append(('same-view pairs (n) median/p90/max', f'{a["n"]}: {f(a["median"], 2)} / {f(a["p90"], 2)} / {f(a["max"], 2)}' if a['n'] else '0 (none measured)'))
        for i, t in enumerate(sv['thirds']):
            out.append((f'  third {i + 1} n / median / max', f'{t["n"]} / {f(t["median"], 2)} / {f(t["max"], 2)}' if t['n'] else '0'))
    if pn:
        for k, label in (('lost', 'map-lost'), ('tracked', 'map-tracked')):
            v = pn[k]
            out.append((f'path / net {label} frames', f'{v["frames"]} fr: path {v["path"]:.2f}, net {v["net"]:.2f}'))
    return out


def render(results):
    lines = []
    labels = [r['file'].split('/')[-1] for r in results]
    rows = [rows_of(r) for r in results]
    names = [k for k, _ in max(rows, key=len)]
    w0 = max(len(k) for k in names + ['file'])
    wc = [min(max(len(l), 24), 44) for l in labels]
    lines.append('file'.ljust(w0) + '  ' + '  '.join(l[:wc[i]].ljust(wc[i]) for i, l in enumerate(labels)))
    for k in names:
        cells = []
        for i, rr in enumerate(rows):
            d = dict(rr)
            cells.append(str(d.get(k, '')).ljust(wc[i]))
        lines.append(k.ljust(w0) + '  ' + '  '.join(cells))
    for r in results:
        if r['warning']:
            lines.append(f'WARNING {r["file"].split("/")[-1]}: {r["warning"]}')
    lines.append('units: same-view and path/net are page-relative units (not metres); latency = engineReturnedAtMs - sourceStageTimeNanos')
    return lines


def compare_block(results):
    lines = ['', 'compare']
    metrics = [('map discards', lambda r: r['mapDiscards']), ('extension keyframes', lambda r: r['extensionKeyframes']),
               ('map-lost frames', lambda r: r['mapLostFrames']),
               ('same-view median', lambda r: (r.get('sameView') or {}).get('all', {}).get('median'))]
    for name, g in metrics:
        vals = [(r['file'].split('/')[-1], g(r)) for r in results]
        lines.append(f'  {name}: ' + ', '.join(f'{l} {f(v, 2) if isinstance(v, float) else ("n/a" if v is None else v)}' for l, v in vals))
        have = [(l, v) for l, v in vals if v is not None]
        if len(have) < 2:
            lines.append('    not comparable (missing in a run)')
        else:
            lo = min(v for _, v in have)
            low = [l for l, v in have if v == lo]
            lines.append('    ' + (f'{" and ".join(low)} has the lowest value ({f(lo, 2) if isinstance(lo, float) else lo})' if len(low) < len(have) else 'all runs have the same value'))
    lines += ['', 'NOTE: no ground truth; one run per mode is not statistics; compare runs made with the same motion.']
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('files', nargs='+')
    ap.add_argument('--json', dest='json_out')
    a = ap.parse_args(argv)
    results, failed = [], False
    for p in a.files:
        try:
            results.append(analyse(p))
        except (LogError, OSError, ValueError, KeyError) as e:
            print(f'ERROR {p}: {e}', file=sys.stderr)
            failed = True
    if results:
        print('\n'.join(render(results)))
        if len(results) >= 2:
            print('\n'.join(compare_block(results)))
        else:
            print('\nNOTE: no ground truth; one run per mode is not statistics; compare runs made with the same motion.')
    if a.json_out and results:
        with open(a.json_out, 'w') as fh:
            json.dump(results, fh, indent=1)
    return 1 if failed or not results else 0


if __name__ == '__main__':
    sys.exit(main())
