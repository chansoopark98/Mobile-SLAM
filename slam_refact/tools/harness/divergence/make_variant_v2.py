#!/usr/bin/env python3
"""Divergence lane (v2 of make_variant.py): derive a DIAGNOSTIC replay bundle from an existing prepare_inputs.py bundle.

make_variant.py is pinned by several gates and is not modified; this file is its copy with two changes (review
opus-architecture-review-gate-docs-2026-10-29 section 3). With neither of them taking effect the output is byte for byte the output of
make_variant.py, except that bundle.json's recorded argv names this script:

  (1) fail-closed monotonic check (always on): after EVERY re-timing, immediately before anything is written, frames.tsv time_ns must be
      strictly increasing over all rows in file order (bootstrap included; the replay's bundle_reader.cc rule) and the 'run' rows of
      sensors.tsv non-decreasing. On a violation no bundle is written (the --out directory is not created), a JSON description of the
      first violations goes to stderr and the exit code is EXIT_NONMONOTONIC = 21 (distinct from 0, from argparse 2 and from the SystemExit
      messages of the parent code, which exit 1). Cause it guards: with --capture-binding the frames after the last staged frame keep their
      BASE time (the capture run ended); --capture-wallclock re-times the frames up to it into the capture run's wall clock, so a slow
      original ends after the base time of the first tail frame and the frame times decrease.
  (2) --tail-shift (optional, default OFF): with --capture-wallclock, delta = max(0, times[last] + 1 - base_ns(last + 1)); only if delta > 0 every
      tail frame (index > last staged) and every sensor row after the last bound row are moved by the same delta, so their spacing is kept
      (the parent compresses all tail rows below times[last] to times[last] + 1). delta == 0: output unchanged. Only if delta > 0 does
      bundle.json get the new key diagnosticVariant.tailShift. Why it is off by default: compare_us026.compare compares the fresh set of
      EVERY frame in both directions ('the engine processes exactly the frames the original processed') and the tail frames of the bundle
      keep the base drop mask (processed), so the comparator does look at the tail frames; shifting them changes their time and so
      the engine input of frames that are compared. A gate that wants it needs its own equivalence evidence first.

The base bundle is never modified; the variant goes to a new directory. Variants are sensitivity probes

The base bundle is never modified; the variant goes to a new directory. Variants are sensitivity probes
(which input class moves the first divergence), not gate inputs. Options:

  --max-frame N          keep frames with index <= N (shorter replay; earlier frames unchanged)
  --frame-lag-ms L       frame time_ns += L ms for every frame (video_time_ns unchanged). With the
                         bundle binding rule (rows with receive_time_ns <= time_ns(k)), this binds the
                         sensor rows the original would bind if its rAF ran L ms after the video time.
  --capture DIR          capture_window.mjs / capture_pyramid.mjs output: (a) inject its pyramids
                         (meta captures[].inputFrame -> frames.tsv pyramid column), and with
                         --capture-binding also (b) for every frame the capture run staged, move
                         time_ns(k) minimally so that the bundle rule binds exactly the sensor row
                         whose (alpha, beta, gamma) that run staged (sensor times unchanged; only the
                         frame time moves, as the original's rAF timeNanos does), and (c) use the
                         capture run's processed-frame set (stageCalls) as the drop mask.
                         (A first attempt used the run's timeNanos lag directly; it did not
                         reproduce the staged orientation for 112/167 frames and was replaced.)

  --capture may also be a run-reference.mjs --record-inputs run dir (recorded-inputs.jsonl.gz; no pyramids, use
  --pyramids-from, repeatable, first source holding a frame wins). With such a run:
  --queue-times          (needs --capture-binding --capture-wallclock) re-time every run sensor row that the original
                         handed over in a c8EmAsm_setEventQueue to its recorded handler time (timestampNanos, the time
                         the engine contract uses), clamped into (time of the previous staged frame, time of the frame it
                         was staged with], so the bundle rule binds exactly the recorded queue. Rows are matched to
                         queue events by eventTimestampNanos (= epoch + t). Verified per frame (report queueBinding).
  --queue-linacc         also write the LINEAR_ACCELERATION (kind 4) values of the recorded queue into the lin columns
                         (headless Chrome hands the original (0, 0, 0) when acceleration is null; prepare_inputs.py
                         writes '-' = no event). Diagnostic only.

  python3 -I make_variant_v2.py --base <bundle dir> --out <new bundle dir> [options]
"""
import argparse
import bisect
import gzip
import hashlib
import json
import math
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / 'scripts/dev'))
from reference_metadata import matches_reference_schema

EXIT_NONMONOTONIC = 21   # (1): the re-timed frames/sensors are not monotonic; no bundle written


def monotonic_violations(frames, sensors, limit=20):
    """Violations of the replay's input rules (bundle_reader.cc): frames.tsv time_ns strictly increasing over all rows in file order, the
    'run' rows of sensors.tsv non-decreasing."""
    bad_frames = [i for i in range(1, len(frames)) if int(frames[i][2]) <= int(frames[i - 1][2])]
    run_t = [(i, int(s[1])) for i, s in enumerate(sensors) if s[0] == 'run']
    bad_sensors = [run_t[q][0] for q in range(1, len(run_t)) if run_t[q][1] < run_t[q - 1][1]]
    return {'framesNotIncreasing': {'count': len(bad_frames), 'lines': [i + 1 for i in bad_frames[:limit]]},
            'sensorRunRowsDecreasing': {'count': len(bad_sensors), 'lines': [i + 1 for i in bad_sensors[:limit]]}}


def decode_event_queue(raw):
    """Python twin of tools/re/runtime/capnp-event-queue.mjs decodeEventQueue (single-segment messages)."""
    import base64
    import struct
    b = base64.b64decode(raw)
    nseg = struct.unpack_from('<I', b, 0)[0] + 1
    if nseg != 1:
        raise ValueError('multi-segment event queue')
    seg = 8

    def ptr(p):
        lo, hi = struct.unpack_from('<II', b, p)
        if lo == 0 and hi == 0:
            return None
        off = (lo if lo < 2**31 else lo - 2**32) >> 2
        return lo & 3, p + 8 + off * 8, hi
    kind, at, hi = ptr(seg)
    dw, np_ = hi & 0xffff, hi >> 16
    lst = None
    for k in range(np_):
        lst = ptr(at + (dw + k) * 8)
        if lst:
            break
    if not lst:
        return []
    _, lat, lhi = lst
    tlo, thi = struct.unpack_from('<II', b, lat)
    n, edw, ep = tlo >> 2, thi & 0xffff, thi >> 16
    out = []
    for e in range(n):
        base = lat + 8 + e * (edw + ep) * 8
        v = ptr(base + edw * 8)
        x, y, z = struct.unpack_from('<fff', b, v[1]) if v else (0.0, 0.0, 0.0)
        out.append({'kind': struct.unpack_from('<H', b, base)[0], 'timestampNanos': struct.unpack_from('<q', b, base + 8)[0],
                    'eventTimestampNanos': struct.unpack_from('<q', b, base + 16)[0],
                    'intervalNanos': struct.unpack_from('<q', b, base + 24)[0], 'x': x, 'y': y, 'z': z})
    return out


def load_capture_meta(d):
    """capture_window.mjs meta.json, or the equivalent subset built from a run-reference.mjs recorded-inputs file."""
    rec = d / 'recorded-inputs.jsonl.gz'
    if not rec.exists():
        return json.loads((d / 'meta.json').read_text()), None
    lines = [json.loads(l) for l in gzip.decompress(rec.read_bytes()).decode().splitlines() if l.strip()]
    head, calls = lines[0], lines[1:]
    if not matches_reference_schema(head.get('schema'), 'slam-refact-xr_reference-recorded-inputs-v1', REPO) or not head.get('wrapped') or head.get('droppedCalls'):
        raise SystemExit('unusable recorded-inputs header in ' + str(rec))
    queues = {c['seq']: c for c in calls if c['fn'] == 'setEventQueue'}
    for q in queues.values():  # always decode the raw bytes here; cross-check the page-side decode when present
        ev = decode_event_queue(q['b64']) if q.get('b64') else None
        if q.get('events') and isinstance(q['events'][0], dict) and 'kind' in q['events'][0] and q['events'] != ev:
            raise SystemExit('event queue decode mismatch at call %d' % q['seq'])
        q['events'] = ev
    stage = [{'call': c['seq'], 'args': c['args'], 'inputFrame': c['i'], 'queue': queues.get(c['queueSeq'])}
             for c in calls if c['fn'] == 'stageFrame']
    return {'probe': {'epochWallMs': head['epochWallMs'], 'frames': head['publications']}, 'stageCalls': stage,
            'captures': []}, head


def queue_retime(a, meta, run_rows, times, k_sorted, jmap, epoch, t0_ns):
    """Exact sensor binding from the recorded c8EmAsm_setEventQueue messages (see --queue-times)."""
    row_t = [int(r[1]) for r in run_rows]  # current (wallclock-retimed) times; original video t from the column order
    # Row identity: run rows are in dataset order; their dataset t (s) is recovered from the base order via the
    # eventTimestampNanos of the queue (= epoch + t * 1000 ms). Index rows by dataset t rounded to 0.1 ms.
    base_t = [None] * len(run_rows)
    stage_by_frame = {sc['inputFrame']: sc for sc in meta['stageCalls'] if sc['inputFrame'] is not None}
    qrows = {}       # frame -> sorted row indices handed over with it
    unmatched, lin_rows = 0, {}
    orig_t = [float(r[0]) for r in a._orig_run_t]
    order = sorted(range(len(orig_t)), key=lambda j: orig_t[j])
    keys = [orig_t[j] for j in order]
    for k in k_sorted:
        sc = stage_by_frame.get(k)
        if sc is None or not sc.get('queue') or sc['queue'].get('events') is None:
            continue
        rows = set()
        for ev in sc['queue']['events']:
            t = (ev['eventTimestampNanos'] / 1e6 - epoch) / 1000.0  # dataset seconds
            pos = bisect.bisect_left(keys, t - 6e-4)
            best = None
            for q in range(max(0, pos - 1), min(len(keys), pos + 3)):
                if abs(keys[q] - t) <= 6e-4 and (best is None or abs(keys[q] - t) < abs(keys[best] - t)):
                    best = q
            if best is None:
                unmatched += 1
                continue
            j = order[best]
            rows.add(j)
            hn = t0_ns + int(round((ev['timestampNanos'] / 1e6 - epoch) * 1e6))
            base_t[j] = hn if base_t[j] is None else min(base_t[j], hn)
            if ev['kind'] == 4:
                lin_rows[j] = (ev['x'], ev['y'], ev['z'])
        qrows[k] = sorted(rows)
    report = {'framesWithQueue': len(qrows), 'eventsUnmatched': unmatched, 'rowsQueued': sum(len(v) for v in qrows.values())}
    jm = [k for k in qrows if qrows[k] and k in jmap and max(qrows[k]) != jmap[k]]
    report['orientationBindingDisagrees'] = {'count': len(jm), 'frames': jm[:20]}
    if a.queue_times:
        prev = None
        clamped = 0
        for k in k_sorted:
            if k not in jmap:
                continue
            for j in qrows.get(k, []):
                lo = (prev + 1) if prev is not None else None
                t = base_t[j]
                tc = min(max(t, lo) if lo is not None else t, times[k])
                clamped += tc != t
                row_t[j] = tc
            prev = times[k]
        # Rows never handed over (after the last staged frame, or skipped by the glue) keep their order.
        last_j = max((j for v in qrows.values() for j in v), default=-1)
        queued = {j for v in qrows.values() for j in v}
        for j in range(len(run_rows)):
            if j > last_j:
                row_t[j] = max(row_t[j], (prev or 0) + 1)
        nonmono = [j for j in range(1, len(row_t)) if row_t[j] < row_t[j - 1]]
        report['notQueuedBeforeLast'] = sorted(j for j in range(last_j + 1) if j not in queued)[:20]
        report['clampedToInterval'] = clamped
        report['nonMonotonicRows'] = {'count': len(nonmono), 'rows': nonmono[:20]}
        for j, r in enumerate(run_rows):
            r[1] = str(row_t[j])
        # Check: bundle rule (rows with time <= time_k, after the previous staged frame) == recorded queue.
        rt = sorted((row_t[j], j) for j in range(len(row_t)))
        ts = [x[0] for x in rt]
        bad, prev = [], None
        for k in k_sorted:
            if k not in jmap:
                continue
            lo = bisect.bisect_right(ts, prev) if prev is not None else 0
            hi = bisect.bisect_right(ts, times[k])
            got = sorted(x[1] for x in rt[lo:hi])
            if k in qrows and got != qrows[k]:
                bad.append(k)
            prev = times[k]
        report['bindingCheckFail'] = {'count': len(bad), 'frames': bad[:20]}
    if a.queue_linacc:
        for j, v in lin_rows.items():
            run_rows[j][5:8] = [repr(float(x)) for x in v]
        report['linaccRowsWritten'] = len(lin_rows)
    return report


def id_seed_from_capture(capture):
    """Seed of the original's id generator (G641: libc++ mt19937 seeded with one 32-bit getentropy word). memprobe
    --memprobe-detail records mtTrack.idRng = [index, x[index], x[index + 1]]; libc++ overwrites x[i] in place per
    draw, so x[index] still holds the seeding value x_k = 1812433253 * (x_{k-1} ^ (x_{k-1} >> 30)) + k, which is
    inverted down to x[0] = seed. The last record before the first id draw of the final generator instance is used
    (the engine object is re-created at the start recenter, which reseeds)."""
    if capture is None:
        raise SystemExit('--capture-id-seed needs --capture')
    inv = pow(1812433253, -1, 1 << 32)

    def unxorshift30(y):
        return y ^ (y >> 30)  # x ^ (x >> 30) is its own inverse for a 32-bit word (shift >= 16)

    with gzip.open(capture / 'memprobe.jsonl.gz', 'rt') as f:
        recs = [json.loads(line) for line in list(f)[1:]]
    rngs = [(r['seq'], (r.get('mtTrack') or {}).get('idRng')) for r in recs]
    rngs = [(q, g) for q, g in rngs if g is not None and g[1] is not None]
    if not rngs:
        raise SystemExit('no memprobe record with mtTrack.idRng in ' + str(capture))
    seeds = []
    for q, (index, xk, xk1) in rngs:
        if (1812433253 * (xk ^ (xk >> 30)) + index + 1) & 0xffffffff != xk1:
            continue  # x[index] already regenerated (not a fresh generator)
        x = xk
        for k in range(index, 0, -1):
            x = unxorshift30(((x - k) * inv) & 0xffffffff)
        seeds.append((q, x))
    if not seeds:
        raise SystemExit('no fresh idRng record (seeding values) in ' + str(capture))
    return seeds[-1][1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--max-frame', type=int, default=-1)
    ap.add_argument('--frame-lag-ms', type=float, default=0.0)
    ap.add_argument('--capture', type=pathlib.Path)
    ap.add_argument('--capture-binding', action='store_true')
    ap.add_argument('--capture-pyramids', action='store_true')
    ap.add_argument('--capture-wallclock', action='store_true',
                    help='with --capture-binding: frame time_ns = the capture run timeNanos (rAF wall clock), sensor rows re-timed into the bound interval; '
                         'adds column 9 frame_time_ns = trunc(frameTimeSeconds * 1e9) (the 647 tracking time) in the same time base')
    ap.add_argument('--pyramids-from', type=pathlib.Path, action='append',
                    help='take the injected pyramids from this capture dir instead of --capture (SwiftShader readback is deterministic); repeatable, first source wins')
    ap.add_argument('--tail-shift', action='store_true',
                    help='with --capture-wallclock: move the frames after the last staged frame and the sensor rows after the last bound row by '
                         'delta = max(0, times[last] + 1 - base_ns(last + 1)) (spacing kept). Default off: the comparator compares the tail frames.')
    ap.add_argument('--queue-times', action='store_true')
    ap.add_argument('--queue-linacc', action='store_true')
    ap.add_argument('--inject-from', type=int, default=0, help='inject captured pyramids only for frames >= this index')
    ap.add_argument('--capture-id-seed', action='store_true',
                    help='write id_generator_seed (engine-config.tsv) = the capture run\'s id generator seed, read by memprobe '
                         '--memprobe-detail (mtTrack.idRng = MapBuilder mt19937 [x[0], index, x[1]], G641) before its first twist')
    ap.add_argument('--id-seed', type=int, help='write this id_generator_seed (e.g. one recovered offline for a run without idRng)')
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    frames = [l.rstrip('\n').split('\t') for l in (a.base / 'frames.tsv').read_text().splitlines()]
    sensors = [l.rstrip('\n').split('\t') for l in (a.base / 'sensors.tsv').read_text().splitlines()]
    cfg = (a.base / 'engine-config.tsv').read_text()
    # dataset t of every run row (t0_s = 1.0 in prepare_inputs.py): kept before any re-timing for --queue-times
    a._orig_run_t = [(int(s[1]) / 1e9 - 1.0,) for s in sensors if s[0] == 'run']
    report = {'base': str(a.base), 'argv': sys.argv}

    if a.max_frame >= 0:
        frames = [f for f in frames if f[0] == 'bootstrap' or int(f[1]) <= a.max_frame]
    lag_ns = int(round(a.frame_lag_ms * 1e6))
    if lag_ns:
        for f in frames:
            if f[0] == 'frame':
                f[2] = str(int(f[2]) + lag_ns)

    if a.tail_shift and not (a.capture and a.capture_binding and a.capture_wallclock):
        raise SystemExit('--tail-shift needs --capture --capture-binding --capture-wallclock')
    tail_report = None
    if a.capture:
        meta, rec_head = load_capture_meta(a.capture)
        if (a.queue_times or a.queue_linacc) and rec_head is None:
            raise SystemExit('--queue-times/--queue-linacc need a run-reference --record-inputs run as --capture')
        epoch = meta['probe']['epochWallMs']
        pub = {fr[0]: fr for fr in meta['probe']['frames']}
        by_index = {int(f[1]): f for f in frames if f[0] == 'frame'}
        if a.capture_pyramids:
            injected = []
            srcs = a.pyramids_from or [a.capture]
            pcaps = []
            for pdir in srcs:
                for c in json.loads((pdir / 'meta.json').read_text())['captures']:
                    pcaps.append((pdir, c))
            done = set()
            for pdir, c in pcaps:
                k = c['inputFrame']
                if k is None and a.inject_from == 0:  # bootstrap stage (prepare_inputs.py rule)
                    p = pdir / ('cap-%03d.rgba.gz' % c['capture'])
                    for f in frames:
                        if f[0] == 'bootstrap':
                            f[7] = str(p)
                    continue
                if k is None or k < a.inject_from or k not in by_index or k in done:
                    continue
                done.add(k)
                p = pdir / ('cap-%03d.rgba.gz' % c['capture'])
                raw = gzip.decompress(p.read_bytes())
                if hashlib.sha256(raw).hexdigest() != c['rawSha256']:
                    raise SystemExit('capture sha mismatch ' + str(p))
                by_index[k][7] = str(p)
                injected.append(k)
            report['injectedFrames'] = [min(injected), max(injected), len(injected)] if injected else None
            report['pyramidSources'] = [str(p) for p in srcs]
            staged_set = {sc['inputFrame'] for sc in meta['stageCalls'] if sc['inputFrame'] is not None}
            missing = sorted(k for k in staged_set if k not in done and k >= a.inject_from)
            report['stagedWithoutPyramid'] = {'count': len(missing), 'frames': missing[:50]}
            cfg = cfg.replace('pyramid_injection\tnone', 'pyramid_injection\t' + ';'.join(str(p) for p in srcs))
        if a.capture_binding:
            staged = {}
            for sc in meta['stageCalls']:
                if sc['inputFrame'] is not None:
                    staged[sc['inputFrame']] = sc['args']
            run_t = [int(s[1]) for s in sensors if s[0] == 'run']
            run_or = [tuple(float(v) for v in s[-3:]) for s in sensors if s[0] == 'run']
            run_or_index = {o: jj for jj, o in reversed(list(enumerate(run_or)))}
            lags, mism, checked = [], [], 0
            jmap = {}
            last = max(staged)
            for k, f in by_index.items():
                if k > last:
                    continue  # capture run ended; keep base binding and mask
                if k in staged:
                    # Orientation-matched binding: j = the run sensor row whose (alpha, beta, gamma) the
                    # capture run staged for frame k (bit match, 1e-9 fallback). time_ns(k) is moved
                    # the least amount so that the bundle rule binds exactly rows <= j:
                    # clamp(ns(t_k), ns(t_j), ns(t_{j+1}) - 1). Sensor times are unchanged.
                    want = tuple(staged[k][4:7])
                    j = run_or_index.get(want)
                    if j is None:
                        for jj, o in enumerate(run_or):
                            if all(abs(x - y) < 1e-9 for x, y in zip(o, want)):
                                j = jj
                                break
                    checked += 1
                    if j is None:
                        mism.append(k)
                        continue
                    jmap[k] = j
                    t_ns = int(f[3])
                    lo = run_t[j]
                    hi = run_t[j + 1] - 1 if j + 1 < len(run_t) else t_ns
                    new_ns = min(max(t_ns, lo), hi)
                    lags.append(round((new_ns - t_ns) / 1e6, 3))
                    f[2] = str(new_ns)
                    f[5] = '1'
                else:
                    f[5] = '0'
            # The replay needs strictly increasing frame times. A frame the capture run staged late
            # (bound rows past the next frame's t) pushes later frames forward; a pushed frame that
            # then binds more rows than the capture run did is reported as a conflict.
            prev_ns = None
            conflicts = []
            for k in sorted(by_index):
                f = by_index[k]
                cur = int(f[2])
                if prev_ns is not None and cur <= prev_ns:
                    f[2] = str(prev_ns + 1)
                    if k in staged and k <= last:
                        j_new = bisect.bisect_right(run_t, prev_ns + 1) - 1
                        want = tuple(staged[k][4:7])
                        if any(abs(x - y) > 1e-9 for x, y in zip(run_or[j_new], want)):
                            conflicts.append(k)
                prev_ns = int(f[2])
            report['bindingMonotonicConflicts'] = conflicts
            report['binding'] = {'framesStaged': len(lags), 'frameTimeShiftMs': [min(lags), max(lags)] if lags else None,
                                 'orientationChecked': checked, 'orientationMismatch': mism}
            cfg = cfg.replace('drop_mode\t', 'drop_mode\tcapture-stage-set:%s;was:' % a.capture)
            if a.capture_wallclock:
                # Original glue: timeNanos = performance.now() at the rAF that staged the frame; sensor
                # event times = performance.now() in the handler (not recorded). Model: frame k time =
                # T0 + (timeNanos_k/1e6 - epochWallMs) ms; every sensor row bound to frame k (j in
                # (j_prev, j_k]) is re-timed into (time_prev, time_k] (kept if already inside), later
                # rows stay after time_k. Dropped (unstaged) frames get time_prev + 1 ns.
                t0_ns = int(round(1.0e9))
                k_sorted = [k for k in sorted(by_index) if k <= last]
                times = {}
                prev = None
                for k in k_sorted:
                    if k in jmap:
                        tn = t0_ns + int(round((staged[k][13] / 1e6 - epoch) * 1e6))
                        if prev is not None and tn <= prev:
                            tn = prev + 1
                    else:
                        tn = (prev if prev is not None else int(by_index[k][2]) - 2) + 1
                    times[k] = tn
                    by_index[k][2] = str(tn)
                    prev = tn
                tail_d, tail_ks = 0, []
                if a.tail_shift and k_sorted:
                    tail_ks = [k for k in sorted(by_index) if k > last]
                    if tail_ks:
                        tail_d = max(0, times[k_sorted[-1]] + 1 - int(by_index[tail_ks[0]][2]))
                        if tail_d:
                            for k in tail_ks:
                                by_index[k][2] = str(int(by_index[k][2]) + tail_d)
                run_rows = [s for s in sensors if s[0] == 'run']
                orig_run_t = [int(s[1]) for s in run_rows]
                bound_by = {}
                prev_j, prev_t = -1, None
                for k in k_sorted:
                    if k not in jmap:
                        continue
                    for jj in range(prev_j + 1, jmap[k] + 1):
                        bound_by[jj] = (prev_t, times[k])
                    if jmap[k] > prev_j:
                        prev_j = jmap[k]
                    prev_t = times[k]
                moved, run_max = 0, None
                for jj, srow in enumerate(run_rows):
                    t_old = int(srow[1])
                    if jj in bound_by:
                        lo_t, hi_t = bound_by[jj]
                        t_new = min(max(t_old, (lo_t + 1) if lo_t is not None else t_old), hi_t)
                    elif prev_t is not None and jj > prev_j:
                        t_new = max(t_old + tail_d, prev_t + 1)
                    else:
                        t_new = t_old
                    if run_max is not None and t_new < run_max:
                        t_new = run_max
                    run_max = t_new
                    moved += t_new != t_old
                    srow[1] = str(t_new)
                if tail_d:
                    tail_report = {'deltaNs': tail_d, 'tailFramesShifted': len(tail_ks),
                                   'tailRowsShifted': sum(1 for jj in range(prev_j + 1, len(run_rows))),
                                   'tailRowsRaisedAboveShift': sum(1 for jj in range(prev_j + 1, len(run_rows))
                                                                   if int(run_rows[jj][1]) != orig_run_t[jj] + tail_d)}
                bad = []
                rt = [int(s[1]) for s in run_rows]
                for k in k_sorted:
                    if k in jmap:
                        jb = bisect.bisect_right(rt, times[k]) - 1
                        if jb != jmap[k]:
                            bad.append(k)
                # 647 prepareTrackingSensorFrame: TrackingSensorFrame.timeNanos = frameTimestampNanos when it is
                # non-zero, else timestampNanos. c8EmAsm_stageFrame stores trunc(frameTimeSeconds * 1e9) (f534 wat,
                # PipelineData +440) and the web glue always passes a frame time, so tracking runs on the frame
                # time, not on timeNanos (VERIFIED: memprobe VioTracker hist times = trunc(frameTime * 1e9)).
                # Column 9 (frame_time_ns) = that value shifted into the bundle time base by the same integer
                # offset for every frame (frame-to-frame differences stay exact); 0 = use time_ns.
                off = t0_ns - int(round(epoch * 1e6))
                for f in frames:
                    k = int(f[1]) if f[0] == 'frame' else None
                    ft = 0
                    if k is not None and k in jmap:
                        ft = math.trunc(staged[k][12] * 1e9) + off
                    f.append(str(ft))
                report['frameTime'] = {'column': 'frame_time_ns', 'offsetNs': off,
                                       'framesTimed': sum(1 for f in frames if f[-1] != '0')}
                report['wallclock'] = {'framesTimed': len(times), 'sensorRowsRetimed': moved, 'bindingCheckFail': bad,
                                       'frameMinusVideoMs': [round((times[k] - int(by_index[k][3])) / 1e6, 3) for k in k_sorted if k in jmap][:5]}
                if a.queue_times or a.queue_linacc:
                    report['queueBinding'] = queue_retime(a, meta, run_rows, times, k_sorted, jmap, epoch, t0_ns)

    if a.capture_id_seed or a.id_seed is not None:
        seed = a.id_seed
        if seed is None:
            seed = id_seed_from_capture(a.capture)
        cfg = cfg.rstrip('\n') + '\nid_generator_seed\t%d\n' % seed
        report['idGeneratorSeed'] = seed

    if tail_report:
        report['tailShift'] = tail_report
    viol = monotonic_violations(frames, sensors)
    if viol['framesNotIncreasing']['count'] or viol['sensorRunRowsDecreasing']['count']:
        print(json.dumps({'error': 'non-monotonic re-timed bundle; nothing written', 'out': str(a.out), **viol}), file=sys.stderr)
        sys.exit(EXIT_NONMONOTONIC)
    a.out.mkdir(parents=True)
    (a.out / 'frames.tsv').write_text(''.join('\t'.join(f) + '\n' for f in frames))
    (a.out / 'sensors.tsv').write_text(''.join('\t'.join(s) + '\n' for s in sensors))
    (a.out / 'engine-config.tsv').write_text(cfg)
    base_json = json.loads((a.base / 'bundle.json').read_text())
    if base_json.get('diagnosticVariant'):
        report['baseVariant'] = base_json['diagnosticVariant']  # the base was itself a variant (exact_input_report)
    base_json['diagnosticVariant'] = report
    (a.out / 'bundle.json').write_text(json.dumps(base_json, indent=1) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
