#!/usr/bin/env python3
"""US-026 e2e gate mutation evidence (design §3.7, §4.6; protocol §4, §11): DEV scenario outputs of gate_e2e.mjs only.

Every mutant is an INPUT VARIANT or a COPY (bundle copies with another params line or one sensor value moved by one float32 ulp,
or another replay put in the place of the engine arm). The gate decision code is compare_us026.py, unchanged; a mutant is
DETECTED when the scenario decision is FAIL (or, for the null-control mutant, INVALID).

  python3 -I mutants_e2e.py --gate ba  --dev <gate_e2e dev output dir of flip6-n1> --binary <pinned replay> --out <new dir> [--jobs 2]
  python3 -I mutants_e2e.py --gate mse --dev <gate_e2e dev output dir of flip6-arcore-s3-absolute> --binary <pinned replay> --out <new dir>

BA mutants: M-BA-1 (BA-off replay in the engine arm), M-BA-2 (numGlobalIterativeIterations=i:9), M-BA-3
(useFastPointAdjustmentInGlobalIterative=b:1), M-BA-4 (sub-resolution: one accelerometer value of the last sensor row before the
frame preceding the first BA effect moved by one float32 ulp, BA on).
MSE mutants: M-MSE-1 (method-1 replay in the method-N engine arm, N = 2, 3, 4; method 5 outside the device table is the identity
M-ID-5 and not a mutant), M-MSE-2 (sub-resolution: one linear-acceleration value of the sensor row nearest to 10 frames before the
original latch moved by one float32 ulp, method 2 on).
"""
import argparse
import concurrent.futures as cf
import copy
import gzip
import json
import math
import pathlib
import shutil
import struct
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import compare_us026 as C  # noqa: E402

ROOT = HERE.parents[3]
DEV = {'ba': 'flip6-n1', 'mse': 'flip6-arcore-s3-absolute'}
GATE_ID = {'ba': 'us026-ba-e2e-gate', 'mse': 'us026-mse-e2e-gate'}


def next_f32(v):
    """The next float32 above float32(v) in magnitude, as the exact double (repr round-trips)."""
    f = struct.unpack('<f', struct.pack('<f', v))[0]
    b = struct.unpack('<I', struct.pack('<f', abs(f)))[0] + 1
    return math.copysign(struct.unpack('<f', struct.pack('<I', b))[0], f if f != 0 else 1.0)


def copy_bundle(src, dst, params=None):
    dst.mkdir(parents=True)
    for n in ('frames.tsv', 'sensors.tsv', 'bundle.json'):
        shutil.copyfile(src / n, dst / n)
    cfg = (src / 'engine-config.tsv').read_text().splitlines()
    if params is not None:
        cfg = [x for x in cfg if x.split('\t', 1)[0] != 'params'] + ['params\t' + params]
    (dst / 'engine-config.tsv').write_text('\n'.join(cfg) + '\n')
    meta = json.loads((dst / 'bundle.json').read_text())
    meta['us026Mutant'] = {'source': str(src), 'params': params}
    (dst / 'bundle.json').write_text(json.dumps(meta, indent=1) + '\n')


def nudge_sensor(bundle, before_ns, column, label):
    """Moves column `column` of the last run row with receive_time_ns <= before_ns by one float32 ulp; returns the record."""
    lines = (bundle / 'sensors.tsv').read_text().splitlines()
    idx = max(k for k, l in enumerate(lines) if l.startswith('run\t') and int(l.split('\t')[1]) <= before_ns)
    cols = lines[idx].split('\t')
    old = float(cols[column])
    new = next_f32(old)
    cols[column] = repr(new)
    lines[idx] = '\t'.join(cols)
    (bundle / 'sensors.tsv').write_text('\n'.join(lines) + '\n')
    return {'mutant': label, 'sensorsLine': idx + 1, 'column': column, 'old': old, 'new': new, 'beforeNs': before_ns,
            'float32Old': struct.unpack('<f', struct.pack('<f', old))[0], 'float32New': struct.unpack('<f', struct.pack('<f', new))[0]}


def frame_time_ns(bundle, frame):
    for l in (bundle / 'frames.tsv').read_text().splitlines():
        c = l.split('\t')
        if c[0] == 'frame' and int(c[1]) == frame:
            return int(c[2])
    raise SystemExit('frame %d not in %s' % (frame, bundle))


def replay(binary, bundle, out):
    t0 = time.time()
    with open(str(out) + '.log', 'x') as log:
        rc = subprocess.call([str(binary), '--bundle', str(bundle), '--out', str(out)], stdout=log, stderr=subprocess.STDOUT, cwd=ROOT)
    return rc, round(time.time() - t0, 1)


def decide(g, spec):
    return C.ba_scenario(spec) if g == 'ba' else C.mse_scenario(spec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gate', choices=('ba', 'mse'), required=True)
    ap.add_argument('--dev', type=pathlib.Path, required=True, help='gate_e2e.mjs dev output: <out-root>/<gate id>/<dev scenario>')
    ap.add_argument('--binary', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--jobs', type=int, default=2)
    a = ap.parse_args()
    g = a.gate
    rep = json.loads((a.dev / 'report.json').read_text())
    if rep.get('kind') != 'dev' or rep.get('scenario') != DEV[g] or rep['gate']['id'] != GATE_ID[g]:
        raise SystemExit('refused: %s is not a %s dev report of %s' % (a.dev, g, DEV[g]))
    if a.out.exists():
        raise SystemExit('refusing to reuse ' + str(a.out))
    if not 1 <= a.jobs <= 4:
        raise SystemExit('--jobs 1..4')
    spec0 = json.loads((a.dev / 'spec.json').read_text())
    a.out.mkdir(parents=True)
    bundles = {k: ROOT / v['dir'] for k, v in rep['bundles'].items()}
    cases = []   # (name, description, bundle dir to replay or None, spec transform)
    records = {}
    if g == 'ba':
        orig = C.load_original(ROOT / rep['runs']['poke']['dir'])
        eff = [r['i'] for r in orig['poke'] if (r.get('kfPoseChanged') or 0) > 0 and r.get('i') is not None]
        f1 = min(eff) if eff else None
        cases.append(('M-BA-1', 'BA-off replay of the poke bundle in the engine (BA-on) arm', None, ('on', 'off')))
        for name, extra in (('M-BA-2', 'BundleAdjustment.numGlobalIterativeIterations=i:9'), ('M-BA-3', 'BundleAdjustment.useFastPointAdjustmentInGlobalIterative=b:1')):
            d = a.out / 'bundles' / name
            copy_bundle(bundles['off'], d, rep['bundles']['on']['params'] + ';' + extra)
            cases.append((name, 'BA on + ' + extra, d, ('on', name)))
        d = a.out / 'bundles' / 'M-BA-4'
        copy_bundle(bundles['on'], d)
        records['M-BA-4'] = nudge_sensor(d, frame_time_ns(d, f1 - 1), 2, 'M-BA-4') if f1 else {'error': 'no BA effect frame'}
        records['M-BA-4']['firstBaEffectFrame'] = f1
        cases.append(('M-BA-4', 'BA on, one accelerometer x value before frame f1-1 moved by one float32 ulp (sub-resolution)', d, ('on', 'M-BA-4')))
    else:
        for n in (2, 3, 4):
            cases.append(('M-MSE-1-m%d' % n, 'method-1 replay of the method-%d bundle in the method-%d engine arm' % (n, n), None, ('m%d-on' % n, 'm%d-m1' % n)))
        orig = C.load_original(ROOT / rep['runs']['m2']['dir'])
        lf, _ = C.latch_original(orig)
        d = a.out / 'bundles' / 'M-MSE-2'
        copy_bundle(bundles['m2-on'], d)
        records['M-MSE-2'] = nudge_sensor(d, frame_time_ns(d, lf - 10), 5, 'M-MSE-2') if lf else {'error': 'no latch in the method-2 original'}
        records['M-MSE-2']['originalLatchFrame'] = lf
        cases.append(('M-MSE-2', 'method 2 on, one linear-acceleration x value 10 frames before the latch moved by one float32 ulp (sub-resolution)', d, ('m2-on', 'M-MSE-2')))
    todo = [(c[0], c[2]) for c in cases if c[2] is not None]
    (a.out / 'replays').mkdir()
    results = {}
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        futs = {ex.submit(replay, a.binary.resolve(), ROOT / b if not b.is_absolute() else b, (a.out / 'replays' / n).resolve()): n for n, b in todo}
        for f in cf.as_completed(futs):
            results[futs[f]] = f.result()
    out = []
    for name, desc, b, (arm, src) in cases:
        spec = copy.deepcopy(spec0)
        if b is None:
            spec['replays'][arm] = dict(spec0['replays'][src])
        else:
            spec['replays'][arm] = {'dir': str(a.out / 'replays' / name), 'exit': results[name][0]}
        for v in spec['replays'].values():
            v['dir'] = str(ROOT / v['dir']) if not pathlib.Path(v['dir']).is_absolute() else v['dir']
        for v in spec['runs'].values():
            v['dir'] = str(ROOT / v['dir'])
        r = decide(g, spec)
        if g == 'ba':
            decision = r.get('verdict') or r['class']
            tiers = r.get('tiers')
        else:
            p = (r.get('pairs') or {}).get(arm.split('-')[0]) or {}
            decision = p.get('verdict') or p.get('class') or r['class']
            tiers = p.get('tiers')
        c_on = r.get('compare', {}).get(arm, {})
        out.append({'mutant': name, 'description': desc, 'engineArm': arm, 'replayExit': results.get(name, (0,))[0] if b is not None else 'reused',
                    'replaySeconds': results.get(name, (None, None))[1] if b is not None else None, 'decision': decision, 'detected': decision == 'FAIL',
                    'tiers': tiers, 'firstIntegerMismatch': {k: v for k, v in ((c_on.get('D') or {}).get('firstMismatch') or {}).items() if v is not None},
                    'poseFirstViolation': (c_on.get('S') or {}).get('poseFirstViolation'), 'poseMaxUlps': (c_on.get('S') or {}).get('poseMaxUlps'),
                    'latch': c_on.get('latch'), 'record': records.get(name)})
    summary = {'schema': 'slam-refact-us026-e2e-mutants-v1', 'gate': g, 'dev': str(a.dev), 'devReportSha256': C_sha(a.dev / 'report.json'),
               'binary': {'path': str(a.binary), 'sha256': C_sha(a.binary)}, 'comparator': {'path': str(HERE / 'compare_us026.py'), 'sha256': C_sha(HERE / 'compare_us026.py')},
               'mutants': out, 'detected': sum(1 for m in out if m['detected']), 'total': len(out),
               'undetected': [m['mutant'] for m in out if not m['detected']]}
    (a.out / 'mutants.json').write_text(json.dumps(summary, indent=1) + '\n')
    print(json.dumps({'detected': summary['detected'], 'total': summary['total'], 'undetected': summary['undetected'],
                      'decisions': {m['mutant']: m['decision'] for m in out}}))
    return 0


def C_sha(p):
    import hashlib
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


if __name__ == '__main__':
    sys.exit(main())
