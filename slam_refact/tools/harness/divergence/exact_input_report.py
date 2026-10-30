#!/usr/bin/env python3
"""Exact-input diagnostic (thresholds-registration.md Addendum 6 item 5): REPORT only, no pass/fail, not a gate.

For every exact-input bundle (make_variant.py from a run-reference.mjs --record-inputs run), replay the engine binary
(optional, --run) and compare the engine trace with that SAME reference run's own trace.jsonl and memprobe.jsonl.gz.
The reference run is read from the bundle's bundle.json diagnosticVariant argv (--capture).

Per case (frames compared = input index i present in both traces with the reference row fresh):
  * first frame with |dpos| > 1e-3 m, rotation difference > 0.05 deg, status differs, worldPoints differs;
  * first bitwise pose difference (p and q_wxyz exactly equal as printed doubles);
  * per point type 1..5: engine srPointsBySource[k-1] vs probe pts<k>[2] (live, state != retired) on the probe
    record associated with the same input frame; first mismatch frame per type and over all types;
  * whole-run summaries: fraction of compared frames within each threshold / equal, max differences.
Bundles named <run>-linacc0 are also compared engine-vs-engine with their <run> twin (first frame the two
engine traces differ bitwise in pose, status or worldPoints).

  python3 -I exact_input_report.py --bundles <list file: one bundle dir per line> --replays <dir> --out <dir>
      [--run --binary <slam_refact_replay> --jobs N(<=3)] [--label TEXT]
Replay outputs go to <replays>/<scenario>/<run>/ (existing trace.jsonl is reused, never overwritten).
Writes <out>/report.json and <out>/report.md (refuses to overwrite them).
"""
import argparse
import concurrent.futures
import gzip
import hashlib
import json
import math
import pathlib
import subprocess
import sys

POS_THR_M = 1e-3
ROT_THR_DEG = 0.05
N_TYPES = 5


def load_jsonl(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def load_probe(run_dir):
    p = pathlib.Path(run_dir) / 'memprobe.jsonl.gz'
    if not p.exists():
        return None
    with gzip.open(p, 'rt') as f:
        lines = [json.loads(line) for line in f if line.strip()]
    return {r['i']: r for r in lines[1:] if r.get('i') is not None}


def rot_deg(qa, qb):
    na = math.sqrt(sum(x * x for x in qa))
    nb = math.sqrt(sum(x * x for x in qb))
    d = abs(sum(x * y for x, y in zip(qa, qb))) / (na * nb)
    return math.degrees(2.0 * math.acos(min(1.0, d)))


def first(lst):
    return lst[0] if lst else None


def compare(eng_rows, ref_rows, probe=None):
    """Engine trace vs reference trace (+ optional probe {i: record}). Pure function; see module docstring."""
    eng = {r['i']: r for r in eng_rows if r.get('i') is not None}
    ref = {r['i']: r for r in ref_rows if r.get('i') is not None}
    frames = sorted(i for i, r in ref.items() if r.get('fresh') and i in eng)
    pos_bad, rot_bad, st_bad, wp_bad, bit_bad, eng_repeat = [], [], [], [], [], []
    max_dp = max_rot = max_dwp = 0.0
    max_dp_i = max_rot_i = None
    for i in frames:
        a, b = eng[i], ref[i]
        if a.get('srRepeated'):
            eng_repeat.append(i)
        dp = math.sqrt(sum((x - y) ** 2 for x, y in zip(a['p'], b['p'])))
        dr = rot_deg(a['q_wxyz'], b['q_wxyz'])
        if dp > max_dp:
            max_dp, max_dp_i = dp, i
        if dr > max_rot:
            max_rot, max_rot_i = dr, i
        max_dwp = max(max_dwp, abs((a.get('worldPoints') or 0) - (b.get('worldPoints') or 0)))
        if dp > POS_THR_M:
            pos_bad.append(i)
        if dr > ROT_THR_DEG:
            rot_bad.append(i)
        if a['status'] != b['status']:
            st_bad.append(i)
        if a.get('worldPoints') != b.get('worldPoints'):
            wp_bad.append(i)
        if list(a['p']) != list(b['p']) or list(a['q_wxyz']) != list(b['q_wxyz']):
            bit_bad.append(i)
    n = len(frames)
    frac = (lambda bad: (n - len(bad)) / n if n else None)
    out = {
        'frames_compared': n, 'ref_fresh_frames': sum(1 for r in ref.values() if r.get('fresh')),
        'ref_fresh_missing_in_engine': sum(1 for i, r in ref.items() if r.get('fresh') and i not in eng),
        'engine_repeated_on_compared': {'count': len(eng_repeat), 'first': first(eng_repeat)},
        'first': {'pos_gt_1e-3': first(pos_bad), 'rot_gt_0.05deg': first(rot_bad), 'status_differs': first(st_bad),
                  'world_points_differs': first(wp_bad), 'bitwise_pose_differs': first(bit_bad)},
        'fraction_within': {'pos_le_1e-3': frac(pos_bad), 'rot_le_0.05deg': frac(rot_bad), 'status_equal': frac(st_bad),
                            'world_points_equal': frac(wp_bad), 'pose_bitwise_equal': frac(bit_bad)},
        'max': {'dpos_m': max_dp, 'dpos_i': max_dp_i, 'rot_deg': max_rot, 'rot_i': max_rot_i, 'abs_dworld_points': max_dwp},
    }
    if probe is not None:
        pf = [i for i in frames if i in probe and probe[i].get('pts1') is not None and eng[i].get('srPointsBySource')]
        per = {}
        any_bad = []
        for k in range(1, N_TYPES + 1):
            bad = [i for i in pf if eng[i]['srPointsBySource'][k - 1] != probe[i]['pts%d' % k][2]]
            mx = max((abs(eng[i]['srPointsBySource'][k - 1] - probe[i]['pts%d' % k][2]) for i in pf), default=0)
            per['type%d' % k] = {'first_mismatch': first(bad), 'mismatch_frames': len(bad),
                                 'fraction_equal': (len(pf) - len(bad)) / len(pf) if pf else None, 'max_abs_diff': mx}
            any_bad += bad
        out['probe_live_points'] = {'frames_compared': len(pf), 'first_mismatch_any_type': min(any_bad) if any_bad else None,
                                    'per_type': per}
    return out


def compare_engines(rows_a, rows_b):
    """Engine-vs-engine: first frame where two engine traces differ bitwise (pose, status, worldPoints)."""
    a = {r['i']: r for r in rows_a}
    b = {r['i']: r for r in rows_b}
    common = sorted(set(a) & set(b))
    keys = ('p', 'q_wxyz', 'status', 'worldPoints', 'srPointsBySource')
    diff = {k: [i for i in common if a[i].get(k) != b[i].get(k)] for k in keys}
    anyd = sorted({i for v in diff.values() for i in v})
    return {'frames_common': len(common), 'identical': not anyd and len(a) == len(b),
            'first_any': first(anyd), 'frames_differing': len(anyd), 'first_by_key': {k: first(v) for k, v in diff.items()}}


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def case_of(bundle):
    bundle = pathlib.Path(bundle)
    variant = json.loads((bundle / 'bundle.json').read_text())['diagnosticVariant']
    # A variant of a variant (e.g. make_variant --id-seed on an exact-input bundle) keeps the reference run of
    # the bundle it was derived from in baseVariant.
    while '--capture' not in variant['argv'] and variant.get('baseVariant'):
        variant = variant['baseVariant']
    argv = variant['argv']
    ref = pathlib.Path(argv[argv.index('--capture') + 1])
    cfg = dict(l.split('\t', 1) for l in (bundle / 'engine-config.tsv').read_text().splitlines() if '\t' in l)
    frames = [l.split('\t') for l in (bundle / 'frames.tsv').read_text().splitlines()]
    return {'scenario': bundle.parent.name, 'run': bundle.name, 'bundle': str(bundle), 'reference_run': str(ref),
            'linacc0': bundle.name.endswith('-linacc0'), 'frame_time_column': all(len(f) == 9 for f in frames),
            'pyramid_injection': cfg.get('pyramid_injection', '?') != 'none', 'pyramid_model': cfg.get('pyramid_model'),
            'frames_without_pyramid': sum(1 for f in frames if f[0] == 'frame' and f[7] == '-')}


def run_replay(binary, bundle, out_dir):
    if (out_dir / 'trace.jsonl').exists():
        return 'reused'
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    with open(str(out_dir) + '.log', 'w') as log:
        rc = subprocess.call([str(binary), '--bundle', str(bundle), '--out', str(out_dir)], stdout=log, stderr=subprocess.STDOUT)
    return 'rc=%d' % rc


def fmt(v):
    if v is None:
        return '-'
    if isinstance(v, float):
        return '%.4g' % v
    return str(v)


def write_md(rep, path):
    L = ['# Exact-input diagnostic (Addendum 6 item 5) — REPORT only, no pass/fail', '',
         'Binary: `%s` sha256 `%s`' % (rep['binary'], rep['binary_sha256']), '',
         'Label: %s' % rep.get('label', ''), '',
         'Thresholds: |dpos| > %g m, rotation > %g deg. Frames compared: input index present in both traces, reference row fresh.'
         % (POS_THR_M, ROT_THR_DEG), '',
         '## First differing frame vs the run\'s own trace / memprobe', '',
         '| scenario | run | cmp | bitwise | pos | rot | status | wp | probe live pts (any type) | pyramids |',
         '|---|---|---|---|---|---|---|---|---|---|']
    for c in rep['cases']:
        r = c.get('result')
        if r is None:
            L.append('| %s | %s | BLOCKED: %s |' % (c['scenario'], c['run'], c.get('error')))
            continue
        f = r['first']
        pl = r.get('probe_live_points', {})
        pyr = 'captured' if c['pyramid_injection'] else 'CPU %s' % c['pyramid_model']
        if c['pyramid_injection'] and c['frames_without_pyramid']:
            pyr += ' (+%d CPU)' % c['frames_without_pyramid']
        L.append('| %s | %s | %d | %s | %s | %s | %s | %s | %s | %s |' % (
            c['scenario'], c['run'], r['frames_compared'], fmt(f['bitwise_pose_differs']), fmt(f['pos_gt_1e-3']),
            fmt(f['rot_gt_0.05deg']), fmt(f['status_differs']), fmt(f['world_points_differs']),
            fmt(pl.get('first_mismatch_any_type')), pyr))
    L += ['', '## Whole-run summaries (fraction of compared frames within / equal; max diffs)', '',
          '| scenario | run | pos<=1e-3 | rot<=0.05 | status= | wp= | bitwise= | max dpos m (i) | max rot deg (i) | max dwp | probe types 1..5 first mismatch / frac equal |',
          '|---|---|---|---|---|---|---|---|---|---|---|']
    for c in rep['cases']:
        r = c.get('result')
        if r is None:
            continue
        w, m = r['fraction_within'], r['max']
        pl = r.get('probe_live_points')
        pt = ' '.join('%s/%s' % (fmt(v['first_mismatch']), fmt(v['fraction_equal'])) for v in pl['per_type'].values()) if pl else '-'
        L.append('| %s | %s | %s | %s | %s | %s | %s | %s (%s) | %s (%s) | %s | %s |' % (
            c['scenario'], c['run'], fmt(w['pos_le_1e-3']), fmt(w['rot_le_0.05deg']), fmt(w['status_equal']),
            fmt(w['world_points_equal']), fmt(w['pose_bitwise_equal']), fmt(m['dpos_m']), fmt(m['dpos_i']),
            fmt(m['rot_deg']), fmt(m['rot_i']), fmt(m['abs_dworld_points']), pt))
    if rep['linacc0_pairs']:
        L += ['', '## -linacc0 twin vs base bundle (engine vs engine)', '',
              '| scenario | run | identical | first differing frame | frames differing | first by key |', '|---|---|---|---|---|---|']
        for p in rep['linacc0_pairs']:
            L.append('| %s | %s | %s | %s | %s | %s |' % (p['scenario'], p['run'], p['identical'], fmt(p['first_any']),
                                                         p['frames_differing'], json.dumps(p['first_by_key'])))
    path.write_text('\n'.join(L) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bundles', type=pathlib.Path, required=True, help='file with one bundle dir per line')
    ap.add_argument('--replays', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--binary', type=pathlib.Path)
    ap.add_argument('--run', action='store_true', help='replay bundles whose trace.jsonl is missing')
    ap.add_argument('--jobs', type=int, default=3)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    if not 1 <= a.jobs <= 3:
        raise SystemExit('--jobs must be 1..3')
    if a.run and not a.binary:
        raise SystemExit('--run needs --binary')
    for n in ('report.json', 'report.md'):
        if (a.out / n).exists():
            raise SystemExit('refusing to overwrite ' + str(a.out / n))
    bundles = [l.strip() for l in a.bundles.read_text().splitlines() if l.strip() and not l.startswith('#')]
    cases = [case_of(b) for b in bundles]
    for c in cases:
        c['replay'] = str(a.replays / c['scenario'] / c['run'])
    if a.run:
        with concurrent.futures.ThreadPoolExecutor(a.jobs) as ex:
            futs = {ex.submit(run_replay, a.binary, c['bundle'], pathlib.Path(c['replay'])): c for c in cases}
            for f in concurrent.futures.as_completed(futs):
                futs[f]['replay_status'] = f.result()
    traces = {}
    for c in cases:
        tp = pathlib.Path(c['replay']) / 'trace.jsonl'
        if not tp.exists():
            c['result'], c['error'] = None, 'no engine trace at ' + str(tp)
            continue
        try:
            eng = load_jsonl(tp)
        except json.JSONDecodeError as e:  # replay still running or killed mid-write
            c['result'], c['error'] = None, 'incomplete engine trace %s (%s)' % (tp, e)
            continue
        traces[(c['scenario'], c['run'])] = eng
        ref = load_jsonl(pathlib.Path(c['reference_run']) / 'trace.jsonl')
        c['result'] = compare(eng, ref, load_probe(c['reference_run']))
    pairs = []
    for c in cases:
        if c['linacc0']:
            base = c['run'][:-len('-linacc0')]
            ka, kb = (c['scenario'], base), (c['scenario'], c['run'])
            if ka in traces and kb in traces:
                pairs.append(dict(scenario=c['scenario'], run=base, **compare_engines(traces[ka], traces[kb])))
    rep = {'schema': 'slam-refact-exact-input-report-v1', 'note': 'REPORT only (Addendum 6 item 5); no pass/fail, not a gate',
           'label': a.label, 'binary': str(a.binary) if a.binary else None,
           'binary_sha256': sha256(a.binary) if a.binary else None,
           'thresholds': {'pos_m': POS_THR_M, 'rot_deg': ROT_THR_DEG}, 'argv': sys.argv, 'cases': cases, 'linacc0_pairs': pairs}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'report.json').write_text(json.dumps(rep, indent=1) + '\n')
    write_md(rep, a.out / 'report.md')
    print(json.dumps({'cases': len(cases), 'blocked': sum(1 for c in cases if c.get('result') is None), 'pairs': len(pairs)}))


if __name__ == '__main__':
    main()
