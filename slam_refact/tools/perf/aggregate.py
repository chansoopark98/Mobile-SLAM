#!/usr/bin/env python3
"""Aggregates the xr-perf runs into REPORT.json sections. Nearest-rank percentiles; first WARMUP processed frames of every run excluded;
pre/post init split at the first frame with keyframes > 0 (native/WASM) or worldPoints > 0 (8th Wall reference traces, browser).
usage: aggregate.py <raw dir> <out json> [--reference-runs <dir>]"""
import glob, json, math, os, re, sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
WARMUP = 3
STAGES = ['frontend', 'mapTracker', 'mapBuilder', 'metricScale', 'glue']

def pct(v, p):
    if not v: return None
    s = sorted(v); return s[max(0, math.ceil(p / 100 * len(s)) - 1)]
def st(v):
    v = [x for x in v if x is not None]
    return {'n': len(v), 'p50': pct(v, 50), 'p95': pct(v, 95), 'p99': pct(v, 99), 'max': max(v) if v else None} if v else {'n': 0}
def r3(o):
    if isinstance(o, dict): return {k: r3(v) for k, v in o.items()}
    if isinstance(o, float): return round(o, 3)
    return o
def med(xs):
    xs = sorted(x for x in xs if x is not None); return xs[(len(xs) - 1) // 2] if xs else None

PHASES = (('pre_init', 'post', False), ('post_init', 'post', True), ('pre_wp', 'post_wp', False), ('post_wp', 'post_wp', True))
def summarize_rows(rows):
    """rows: list of dicts {post (first keyframe), post_wp (first world point, optional), m{metric: value}} (warm-up already removed).
    pre_init/post_init split at the first keyframe; pre_wp/post_wp at the first published world point."""
    out = {}
    for name, key, want in PHASES:
        if rows and key not in rows[0]: continue
        rs = [r for r in rows if r[key] == want]
        d = {'frames': len(rs)}
        for k in rs[0]['m'] if rs else []:
            d[k] = st([r['m'].get(k) for r in rs])
        out[name] = d
    return out

def across_repeats(per_run):
    """median over repeats of each p50/p95/p99 (per-run values kept alongside)"""
    agg = {}
    for ph in per_run[0]:
        agg[ph] = {}
        keys = set(k for r in per_run for k in r[ph] if k != 'frames')
        for k in sorted(keys):
            agg[ph][k] = {q: med([r[ph].get(k, {}).get(q) for r in per_run]) for q in ('p50', 'p95', 'p99', 'max')}
        agg[ph]['frames_per_run'] = [r[ph]['frames'] for r in per_run]
    return agg

def native_run(d):
    rows = [json.loads(l) for l in open(os.path.join(d, 'trace.jsonl')) if l.strip()]
    proc = [r for r in rows if r.get('fresh') and not r.get('srRepeated') and r.get('srStageMs')]
    init = next((r['i'] for r in proc if (r.get('srKeyframes') or 0) > 0), None)
    wp = next((r['i'] for r in proc if (r.get('worldPoints') or 0) > 0), None)
    meta = {'rows': len(rows), 'processed': len(proc), 'dropped': sum(1 for r in rows if not r.get('fresh')), 'repeated': sum(1 for r in rows if r.get('srRepeated')),
            'map_resets': sum(1 for r in proc if r.get('srMapReset')), 'init_frame': init, 'first_worldpoints_frame': wp}
    use = []
    for r in proc[WARMUP:]:
        m = dict(r['srStageMs']); m['total'] = sum(m[s] for s in STAGES)
        use.append({'post': init is not None and r['i'] >= init, 'post_wp': wp is not None and r['i'] >= wp, 'm': m})
    return meta, summarize_rows(use)

def wasm_run(f):
    j = json.load(open(f)); rows = j['rows']
    proc = [r for r in rows if r.get('fresh') and not r.get('repeated') and r.get('stageMs')]
    init = next((r['i'] for r in proc if (r.get('numKeyframes') or 0) > 0), None)
    wp = next((r['i'] for r in proc if (r.get('worldPoints') or 0) > 0), None)
    meta = {'rows': len(rows), 'processed': len(proc), 'dropped': sum(1 for r in rows if not r.get('fresh')), 'map_resets': sum(1 for r in proc if r.get('mapReset')), 'init_frame': init,
            'first_worldpoints_frame': wp, 'node': j['node']}
    use = []
    for r in proc[WARMUP:]:
        m = dict(r['stageMs']); m['total'] = sum(m[s] for s in STAGES); m['engineWallMs'] = r['engineMs']; m['pyramidCpuPortMs'] = r['pyramidCpuMs']
        m['marshalMs'] = r['engineMs'] - m['total']
        use.append({'post': init is not None and r['i'] >= init, 'post_wp': wp is not None and r['i'] >= wp, 'm': m})
    return meta, summarize_rows(use)

def ref_run(d):
    rows = [json.loads(l) for l in open(os.path.join(d, 'trace.jsonl')) if l.strip()]
    fresh = [r for r in rows if r.get('fresh') and r.get('xrCpuStageMs') is not None]
    init = next((r['i'] for r in fresh if (r.get('worldPoints') or 0) > 0), None)
    meta = {'rows': len(rows), 'fresh': len(fresh), 'no_fresh_callback': sum(1 for r in rows if not r.get('fresh')), 'first_worldpoints_frame': init}
    use = [{'post': init is not None and r['i'] >= init, 'post_wp': init is not None and r['i'] >= init, 'm': {'xrCpuStageMs': r['xrCpuStageMs'], 'xrGpuStageMs': r.get('xrGpuStageMs'), 'latencyMs': r.get('latencyMs')}} for r in fresh[WARMUP:]]
    return meta, summarize_rows(use)

def main():
    args = sys.argv[1:]
    R = os.path.join(REPO_ROOT, 'build/slam-refact-20261015/reference/runs')
    if '--reference-runs' in args:
        i = args.index('--reference-runs'); R = args[i + 1]; del args[i:i + 2]
    raw, out = args[:2]
    rep = {'schema': 'xr-perf-report-v1', 'warmup_frames_excluded': WARMUP, 'percentile': 'nearest-rank'}
    nat = {}
    for n in ('s2', 's3', 'l1'):
        runs = sorted(glob.glob(os.path.join(raw, f'native-{n}-r*')))
        if not runs: continue
        res = [native_run(d) for d in runs]
        nat[n] = {'runs': [{'dir': os.path.basename(d), 'meta': m, 'stats': s} for d, (m, s) in zip(runs, res)], 'median_of_repeats': across_repeats([s for _, s in res])}
    rep['native_replay'] = nat
    w = {}
    for n in ('s2', 's3', 'l1'):
        fs = sorted(glob.glob(os.path.join(raw, f'wasm-{n}-r*.json')))
        if not fs: continue
        res = [wasm_run(f) for f in fs]
        w[n] = {'runs': [{'file': os.path.basename(f), 'meta': m, 'stats': s} for f, (m, s) in zip(fs, res)], 'median_of_repeats': across_repeats([s for _, s in res])}
    rep['wasm_node'] = w
    ref = {}
    for sc in ('flip6-s2', 'flip6-n1', 'flip6-s3-lowtex', 'tum-room1-full'):
        ds = [d for d in sorted(glob.glob(f'{R}/{sc}/*')) if re.search(r'/(r\d|pmr\d)$', d) and os.path.exists(d + '/trace.jsonl')]
        if not ds: continue
        res = [ref_run(d) for d in ds]
        ref[sc] = {'runs': [{'dir': os.path.basename(d), 'meta': m, 'stats': s} for d, (m, s) in zip(ds, res)], 'median_of_repeats': across_repeats([s for _, s in res])}
    rep['reference_8thwall_desktop_swiftshader'] = ref
    br = sorted(glob.glob(os.path.join(raw, 'browser', 'br-r*', 'meta.json')))
    if br:
        runs = [json.load(open(f)) for f in br]
        def agg_phase(ph):
            o = {}
            for k in ('stageMs', 'readMs', 'engineMs', 'mainThreadChunkMs', 'xrGpuMs', 'xrCpuMs'):
                o[k] = {q: med([(r['timing'][ph] or {}).get(k, {}).get(q) for r in runs]) for q in ('p50', 'p95', 'p99', 'max')}
            o['freshUpdates_per_run'] = [(r['timing'][ph] or {}).get('freshUpdates') for r in runs]
            o['raf_over25_per_run'] = [(r['timing'][ph] or {}).get('raf', {}).get('over25') for r in runs]
            o['longtask_totalMs_per_run'] = [(r['timing'][ph] or {}).get('longtasks', {}).get('totalMs') for r in runs]
            return o
        rep['browser_desktop_swiftshader'] = {
            'note': 'deployed web/slam-refact-xr chunk + WASM, system Chrome headless + SwiftShader, recorded flip6-s2 first 300 frames, real-time playback; the sample page also renders three.js in software, so rAF/long task are NOT engine costs',
            'runs': [{'label': r['label'], 'status': r['status'], 'loadAvgAtStart': r['loadAvgAtStart'], 'browserVersion': r.get('browserVersion'), 'window_s': r['timing']['window']['seconds'],
                      'drops': r['timing']['drops']} for r in runs],
            'pre_init': agg_phase('pre'), 'post_init': agg_phase('post'), 'served': runs[0]['served']}
    json.dump(r3(rep), open(out, 'w'), indent=1)
    print('wrote', out)
main()
