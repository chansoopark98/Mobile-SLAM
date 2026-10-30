#!/usr/bin/env python3
"""US-026 comparator (design us026-design.md §3.5, §3.8, §4.4): ORIGINAL poke run vs the exact-input replay of the SAME
run. Two uses:

1. DRAFT mode (lane A, unchanged behaviour): numbers only, no verdict.
     python3 -I compare_us026.py --original <run dir> --engine <replay dir> [--engine-off <replay dir>] --out <new json>
2. GATE mode (us026-ba-e2e-gate.v1, us026-mse-e2e-gate.v1): one scenario report with the pre-declared verdict rules.
     python3 -I compare_us026.py ba-scenario  --spec <spec.json> --out <new json>
     python3 -I compare_us026.py mse-scenario --spec <spec.json> --out <new json>
     python3 -I compare_us026.py nr1 --original <run dir> --out <new json>      (comparator self vs self, NR-BA-1 / NR-MSE-1)
     python3 -I compare_us026.py print-constants                                 (the decision constants; the manifest pins them)
   spec.json (written by gate_e2e.mjs): {"gate": "ba"|"mse", "scenario": ..., "runs": {<arm>: {"dir": ..., "poke": {...},
   "runnerExit": int, "prerun": {...}}}, "replays": {<bundle arm>: {"dir": ..., "exit": int, "run": <arm>, "params": ...}},
   "sensors": {"expectedSha256Uncompressed": ...}}

Frames compared: every input index i whose ORIGINAL trace row is fresh and which the engine trace has
(exact_input_report.py rule); the fresh sets themselves are compared (integer tier).

Tiers (the values are the constants below; the gate documents and manifests state the same values):
  D (integer, exact-class)  per frame: fresh, status, reason, worldPoints, wpAdded, wpRemoved; from memprobe of the
                            same call: mapReset, keyframeCreated, live keyframes, live points per type 1..5; the
                            published world point id list on sampled frames; MSE gate also: latched flag per frame
                            (engine srScaleLatched vs original VioTracker latch u8 @b+2840), latch frame, number of
                            map resets.
  S (continuous)            BA: pose p[3], q_wxyz[4] per component |a-b| <= ULP_FACTOR * ulp32(max(|a|,|b|)), every frame.
                            MSE: latched scale (engine srLatchedScale vs original f32 @b+2844): methods 1/2/5 within
                            ULP_FACTOR ulp; methods 3/4 relative |a-b|/max(|a|,|b|) <= MSE_REL_TOL.
  P (MSE, continuous)       pose as BA S; methods 3/4: frames at or after the latch frame are REPORT when the latched
                            scales are not bit-identical (pre-declared, gate document §5).
World point POSITIONS are printed at toFixed(5) on both sides (50-frame samples): not measurable at 3 ulp, REPORT only.
"""
import argparse
import gzip
import json
import math
import pathlib
import struct
import sys

# ---- decision constants (us026-ba-e2e-gate.v1 / us026-mse-e2e-gate.v1; pinned in the manifests) -------------------------
ULP_FACTOR = 3                 # continuous tiers: 3 x float32 ulp of max(|a|,|b|) (protocol §3: >= 3 x resolution 1 ulp)
INT_TOLERANCE = 0              # integer tiers: equal (exact-class, protocol §11)
MSE_REL_TOL = 1e-6             # methods 3/4 latched scale, relative (resolution: float32 relative spacing 2^-23 = 1.1920929e-07)
MSE_ULP_METHODS = (1, 2, 5)    # latched scale within ULP_FACTOR ulp
MSE_REL_METHODS = (3, 4)       # latched scale within MSE_REL_TOL relative
MSE_SUM_GATE = 600.0           # E-MSE-1: an estimate is attempted when the MSE rolling sum exceeds this (design §4.4)
WASM_SHA256 = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422'
WP_PRINTED_DECIMALS = 5        # worldpoints-sampled.jsonl position precision on both sides (REPORT only)
CONSTANTS = {'ULP_FACTOR': ULP_FACTOR, 'INT_TOLERANCE': INT_TOLERANCE, 'MSE_REL_TOL': MSE_REL_TOL,
             'MSE_ULP_METHODS': list(MSE_ULP_METHODS), 'MSE_REL_METHODS': list(MSE_REL_METHODS), 'MSE_SUM_GATE': MSE_SUM_GATE,
             'WASM_SHA256': WASM_SHA256, 'WP_PRINTED_DECIMALS': WP_PRINTED_DECIMALS}
SCHEMA = 'slam-refact-us026-compare-v2'


def ulp32(x):
    """Spacing of float32 at |x| (next representable float32 above |x| minus |x|); 2^-149 at 0."""
    x = abs(float(x))
    if math.isinf(x) or math.isnan(x):
        return math.inf
    b = struct.unpack('<I', struct.pack('<f', x))[0]
    lo = struct.unpack('<f', struct.pack('<I', b))[0]
    hi = struct.unpack('<f', struct.pack('<I', b + 1))[0]
    return hi - lo


def ulps(x, y):
    """|x - y| in float32 ulps of max(|x|, |y|) (0 when equal, inf for non-finite differences)."""
    if x == y:
        return 0.0
    if x is None or y is None:
        return math.inf
    u = ulp32(max(abs(x), abs(y)))
    return abs(x - y) / u if u > 0 and math.isfinite(u) else math.inf


def jsonl(p, gz=False):
    with (gzip.open(p, 'rt') if gz else open(p)) as f:
        return [json.loads(line) for line in f if line.strip()]


def first(xs):
    return xs[0] if xs else None


def load_original(d):
    d = pathlib.Path(d)
    o = {'dir': str(d), 'trace': {r['i']: r for r in jsonl(d / 'trace.jsonl')}}
    mp = jsonl(d / 'memprobe.jsonl.gz', gz=True)
    o['memprobeHeader'], o['memprobe'] = mp[0], mp[1:]
    o['probeByI'] = {}
    for r in o['memprobe']:
        if r.get('i') is not None and r['i'] not in o['probeByI']:
            o['probeByI'][r['i']] = r
    pk = jsonl(d / 'poke.jsonl.gz', gz=True) if (d / 'poke.jsonl.gz').exists() else [None]
    o['pokeHeader'], o['poke'] = pk[0], pk[1:]
    o['pokeByI'] = {}
    for r in o['poke']:
        if r.get('i') is not None and r['i'] not in o['pokeByI']:
            o['pokeByI'][r['i']] = r
    w = d / 'worldpoints-sampled.jsonl'
    o['wp'] = {r['frameIndex']: r['points'] for r in jsonl(w)} if w.exists() else {}
    o['meta'] = json.loads((d / 'meta.json').read_text()) if (d / 'meta.json').exists() else {}
    return o


def load_engine(d):
    d = pathlib.Path(d)
    e = {'dir': str(d), 'trace': {r['i']: r for r in jsonl(d / 'trace.jsonl') if r.get('i') is not None}}
    w = d / 'worldpoints-sampled.jsonl'
    e['wp'] = {r['frameIndex']: r['points'] for r in jsonl(w)} if w.exists() else {}
    return e


def latch_original(o):
    xs = [r['i'] for r in o['poke'] if r.get('i') is not None and r.get('vtLatch') == 1]
    i = min(xs) if xs else None
    return i, (o['pokeByI'][i].get('vtLatchedScale') if i is not None else None)


def latch_engine(e, frames):
    xs = [i for i in frames if e['trace'][i].get('srScaleLatched')]
    i = min(xs) if xs else None
    return i, (e['trace'][i].get('srLatchedScale') if i is not None else None)


def compare(o, e, mse=False, method=None):
    """All per-frame comparisons of one original run and one engine replay. Numbers, plus the tier results."""
    ref, eng = o['trace'], e['trace']
    frames = sorted(i for i, r in ref.items() if r.get('fresh') and i in eng)
    keys = ['fresh', 'status', 'reason', 'worldPoints', 'wpAdded', 'wpRemoved', 'mapReset', 'keyframeCreated', 'kfLive',
            'pts1', 'pts2', 'pts3', 'pts4', 'pts5'] + (['scaleLatched'] if mse else [])
    d_bad = {k: [] for k in keys}
    # fresh sets in both directions (the engine processes exactly the frames the original processed)
    for i in sorted(set(ref) | set(eng)):
        if bool(ref.get(i, {}).get('fresh')) != bool(eng.get(i, {}).get('fresh')):
            d_bad['fresh'].append(i)
    probe_frames = 0
    pose_ulps = {}
    for i in frames:
        a, b = eng[i], ref[i]
        for k in ('status', 'reason', 'worldPoints', 'wpAdded', 'wpRemoved'):
            if a.get(k) != b.get(k):
                d_bad[k].append(i)
        m = o['probeByI'].get(i)
        if m is not None and m.get('pts1') is not None and a.get('srPointsBySource') is not None:
            probe_frames += 1
            if bool(a['srMapReset']) != bool(m['outMapReset']):
                d_bad['mapReset'].append(i)
            if bool(a['srKeyframeCreated']) != bool(m['outKeyframeCreated']):
                d_bad['keyframeCreated'].append(i)
            if abs(a['srKeyframes'] - m['kfLive']) > INT_TOLERANCE:
                d_bad['kfLive'].append(i)
            for k in range(1, 6):
                if abs(a['srPointsBySource'][k - 1] - m['pts%d' % k][2]) > INT_TOLERANCE:
                    d_bad['pts%d' % k].append(i)
        if mse:
            pr = o['pokeByI'].get(i)
            if pr is not None and pr.get('vtLatch') is not None and bool(a.get('srScaleLatched')) != (pr['vtLatch'] == 1):
                d_bad['scaleLatched'].append(i)
        ca = (a.get('p') or []) + (a.get('q_wxyz') or [])
        cb = (b.get('p') or []) + (b.get('q_wxyz') or [])
        pose_ulps[i] = math.inf if len(ca) != len(cb) else max([ulps(x, y) for x, y in zip(ca, cb)] or [0.0])
    wp_frames = sorted(set(o['wp']) & set(e['wp']) & set(frames))
    id_bad, pos_bad = [], []
    q = 10 ** WP_PRINTED_DECIMALS
    for i in wp_frames:
        po, pe = o['wp'][i], e['wp'][i]
        if [p[0] for p in po] != [p[0] for p in pe]:
            id_bad.append(i)
            continue
        if any(round(x * q) != round(y * q) for a_, b_ in zip(po, pe) for x, y in zip(a_[2:], b_[2:])):
            pos_bad.append(i)
    out = {'framesCompared': len(frames), 'origFresh': sum(1 for r in ref.values() if r.get('fresh')),
           'engineFresh': sum(1 for r in eng.values() if r.get('fresh')), 'memprobeFramesCompared': probe_frames,
           'firstFrame': first(frames), 'lastFrame': frames[-1] if frames else None}
    resets_o = sum(1 for i in frames if (o['probeByI'].get(i) or {}).get('outMapReset'))
    resets_e = sum(1 for i in frames if eng[i].get('srMapReset'))
    int_mismatch = {k: len(v) for k, v in d_bad.items()}
    D = {'mismatchFrames': int_mismatch, 'firstMismatch': {k: first(v) for k, v in d_bad.items()},
         'wpIdListFramesCompared': len(wp_frames), 'wpIdListMismatchFrames': len(id_bad), 'wpIdListFirstMismatch': first(id_bad),
         'mapResets': {'original': resets_o, 'engine': resets_e}, 'integerTolerance': INT_TOLERANCE}
    any_d = any(int_mismatch.values()) or bool(id_bad) or resets_o != resets_e
    viol = sorted(i for i, u in pose_ulps.items() if u > ULP_FACTOR)
    nonzero = sorted(i for i, u in pose_ulps.items() if u > 0)
    S = {'ulpFactor': ULP_FACTOR, 'poseMaxUlps': max(pose_ulps.values(), default=0.0), 'poseViolationFrames': len(viol),
         'poseFirstViolation': first(viol), 'poseNonzeroFrames': len(nonzero), 'poseFirstNonzero': first(nonzero),
         'wpPositionResolution': 10.0 ** -WP_PRINTED_DECIMALS, 'wpPositionPrintedMismatchFrames': len(pos_bad),
         'wpPositionFirstMismatch': first(pos_bad), 'wpPosition': 'REPORT (printed toFixed(5); not a tier)'}
    out['D'], out['S'] = D, S
    out['bitIdentical'] = not any_d and not nonzero
    if mse:
        lo, lso = latch_original(o)
        le, lse = latch_engine(e, frames)
        lat = {'originalFrame': lo, 'engineFrame': le, 'originalScale': lso, 'engineScale': lse}
        if lo != le:
            D['latchFrameMismatch'] = True
            any_d = True
        else:
            D['latchFrameMismatch'] = False
        if lso is not None and lse is not None:
            lat['scaleUlps'] = ulps(lso, lse)
            mx = max(abs(lso), abs(lse))
            lat['scaleRel'] = 0.0 if lso == lse else (abs(lso - lse) / mx if mx > 0 else math.inf)
            lat['scaleBitIdentical'] = lso == lse
        out['latch'] = lat
        out['bitIdentical'] = out['bitIdentical'] and not D['latchFrameMismatch'] and lat.get('scaleBitIdentical', lso is None and lse is None)
    out['anyIntegerMismatch'] = any_d
    diff_frames = [v[0] for v in d_bad.values() if v] + ([id_bad[0]] if id_bad else []) + ([nonzero[0]] if nonzero else [])
    if mse and D.get('latchFrameMismatch'):
        diff_frames.append(min(x for x in (out['latch']['originalFrame'], out['latch']['engineFrame']) if x is not None))
    elif mse and out['latch'].get('scaleBitIdentical') is False:
        diff_frames.append(out['latch']['originalFrame'])
    out['firstDifferenceFrame'] = min(diff_frames) if diff_frames else None
    out['_poseUlps'] = pose_ulps
    return out


ENGINE_FIELDS = ('fresh', 'status', 'reason', 'p', 'q_wxyz', 'worldPoints', 'wpAdded', 'wpRemoved', 'srMapReset', 'srKeyframeCreated', 'srKeyframes',
                 'srPointsBySource', 'srScaleLatched', 'srLatchedScale')


def engine_first_difference(e1, e2):
    """First input frame at which two engine replays differ in any compared field (bits of the printed float32 values) or in the
    sampled world point lists; None when they never differ."""
    out = []
    for i in sorted(set(e1['trace']) | set(e2['trace'])):
        a, b = e1['trace'].get(i), e2['trace'].get(i)
        if a is None or b is None or any(a.get(k) != b.get(k) for k in ENGINE_FIELDS):
            out.append(i)
            break
    for i in sorted(set(e1['wp']) | set(e2['wp'])):
        if e1['wp'].get(i) != e2['wp'].get(i):
            out.append(i)
            break
    return min(out) if out else None


def setting_frame(o, gate):
    """First input frame at which the ORIGINAL's poked setting can act on its output: BA = the first call that ran the BA loop
    (baLatch 1), MSE = the first call whose rolling sum exceeds MSE_SUM_GATE (no estimate, hence no method path, before it).
    A call without a frame index (bootstrap/repeat) maps to the next call that has one."""
    hit = False
    for r in o['poke']:
        hit = hit or (r.get('baLatch') == 1 if gate == 'ba' else (r.get('mseOk') and (r.get('mseSum') or 0) > MSE_SUM_GATE))
        if hit and r.get('i') is not None:
            return r['i']
    return None


def nondeterminism(c, f_eff, f_set):
    """Pre-declared rule (gate documents §5): the original run differs from its exact-input replay at frame f_d before BOTH
    the first frame at which the engine's setting arm differs from the engine's control arm (f_eff; the two engine arms are
    bit-identical before it) AND the first frame at which the original's setting can act (f_set). Such a difference does not
    depend on the setting under test: the scenario/pair is INVALID (original nondeterminism or harness), not FAIL."""
    f_d = c.get('firstDifferenceFrame')
    lim = min(x for x in (f_eff, f_set, math.inf) if x is not None)
    return {'firstDifferenceFrame': f_d, 'engineSettingEffectFrame': f_eff, 'originalSettingFrame': f_set,
            'invalid': f_d is not None and f_d < lim}


def strip(c):
    return {k: v for k, v in c.items() if not k.startswith('_')}


# ---- validity (V-BA-0 / V-MSE-0, pre-declared) -------------------------------------------------------------------------
def run_validity(o, expected_keys, rule, runner_exit=None, prerun=None, expected_sensors=None):
    """rule 'ba': no gap on any call with memprobe outKeyframeCreated = 1 and baLatch reached 1; rule 'all': no gap on any
    call with seq >= 2; rule 'mse': as 'all' and mseOk on every call with a root."""
    reasons = []
    meta = o['meta']
    if runner_exit not in (None, 0):
        reasons.append('run-reference.mjs exit %s' % runner_exit)
    if meta.get('status') != 'COMPLETED':
        reasons.append('meta.status %s' % meta.get('status'))
    s = meta.get('summary') or {}
    for k in ('blockedExternal', 'pixelHashMismatches', 'pageErrors', 'errors'):
        if s.get(k):
            reasons.append('meta.summary.%s = %s' % (k, s.get(k)))
    ri = meta.get('recordedInputs') or {}
    if not ri.get('wrapped') or ri.get('error') or ri.get('droppedCalls'):
        reasons.append('recorded inputs not complete (wrapped %s, error %s, dropped %s)' % (ri.get('wrapped'), ri.get('error'), ri.get('droppedCalls')))
    mh, ph = o['memprobeHeader'], o['pokeHeader']
    if mh.get('error') or mh.get('wasmSha256') != WASM_SHA256:
        reasons.append('memprobe header error %s / wasm %s' % (mh.get('error'), mh.get('wasmSha256')))
    if ph is None:
        reasons.append('no poke.jsonl.gz')
        return {'valid': False, 'reasons': reasons}
    if ph.get('error') or not ph.get('wrapped') or ph.get('wasmSha256') != WASM_SHA256:
        reasons.append('poke header error %s / wrapped %s / wasm %s' % (ph.get('error'), ph.get('wrapped'), ph.get('wasmSha256')))
    if ph.get('keys') != expected_keys:
        reasons.append('poke keys %s, expected %s' % (ph.get('keys'), expected_keys))
    recs = o['poke']
    mp = {r['seq']: r for r in o['memprobe']}
    if rule == 'ba':
        check = [r for r in recs if (mp.get(r['seq']) or {}).get('outKeyframeCreated') == 1]
    else:
        check = [r for r in recs if r['seq'] >= 2]
    gaps = [r['seq'] for r in check if r.get('gap')]
    if gaps:
        reasons.append('gap on %d checked calls (first seq %d)' % (len(gaps), gaps[0]))
    if not check:
        reasons.append('no call to check for gaps')
    latch = first([r['seq'] for r in recs if r.get('baLatch') == 1])
    if rule == 'ba' and latch is None:
        reasons.append('baLatch never 1 (the BA loop never ran)')
    if rule == 'mse':
        bad = [r['seq'] for r in recs if r.get('root') and not r.get('mseOk')]
        if bad:
            reasons.append('mseOk false on %d calls with a root (first seq %d)' % (len(bad), bad[0]))
    if prerun is not None and not prerun.get('ok'):
        reasons.append('pre-run check not recorded as passed')
    sens = (meta.get('input') or {}).get('engineInputSensors') or {}
    if expected_sensors and sens.get('sha256Uncompressed') != expected_sensors:
        reasons.append('engine-input sensors %s differ from the base bundle sensors %s' % (sens.get('sha256Uncompressed'), expected_sensors))
    return {'valid': not reasons, 'reasons': reasons, 'checkedCalls': len(check), 'gapCalls': len(gaps), 'baLatchFirstSeq': latch,
            'pvValues': sorted({r['post'][0] for r in recs if r.get('post')}), 'records': len(recs),
            'kfPoseChangedSum': sum(r.get('kfPoseChanged') or 0 for r in recs),
            'loadAvgAtStart': meta.get('loadAvgAtStart'), 'prerun': prerun}


def mse_path(o, method):
    """E-MSE-1: the method's estimate path ran (rolling sum above MSE_SUM_GATE; methods 3/4 also residual == -1)."""
    recs = [r for r in o['poke'] if r.get('mseOk')]
    over = [r['seq'] for r in recs if (r.get('mseSum') or 0) > MSE_SUM_GATE]
    neg = [r['seq'] for r in recs if r.get('mseResidual') == -1]
    ok = bool(over) and (method not in MSE_REL_METHODS or bool(neg))
    return {'met': ok, 'callsSumOverGate': len(over), 'firstSumOverGate': first(over), 'callsResidualMinus1': len(neg),
            'maxSum': max([r.get('mseSum') or 0 for r in recs], default=None)}


def identical(c):
    return c['bitIdentical']


# ---- BA scenario (us026-ba-e2e-gate.v1 §5) -----------------------------------------------------------------------------
def ba_scenario(spec):
    runs, reps = spec['runs'], spec['replays']
    sens = (spec.get('sensors') or {}).get('expectedSha256Uncompressed')
    out = {'schema': SCHEMA, 'gate': 'ba', 'scenario': spec['scenario'], 'kind': spec.get('kind'), 'constants': CONSTANTS}
    loaded, val = {}, {}
    for arm, rule, keys in (('poke', 'ba', {'ba': 1}), ('null', 'all', {'ba': 0, 'mse': 1})):
        r = runs.get(arm) or {}
        try:
            loaded[arm] = load_original(r['dir'])
            val[arm] = run_validity(loaded[arm], keys, rule, r.get('runnerExit'), r.get('prerun'), sens)
        except (OSError, KeyError, ValueError, IndexError) as error:
            val[arm] = {'valid': False, 'reasons': ['unreadable run: %s' % error]}
    out['validity'] = val
    eng, rexit = {}, {}
    for b in ('on', 'off', 'null'):
        r = reps.get(b) or {}
        rexit[b] = r.get('exit')
        try:
            if r.get('exit') == 0:
                eng[b] = load_engine(r['dir'])
        except (OSError, KeyError, ValueError) as error:
            rexit[b] = 'unreadable: %s' % error
    out['replayExit'] = rexit
    cmp = {}
    if 'null' in loaded and 'null' in eng:
        cmp['null'] = compare(loaded['null'], eng['null'])
    if 'poke' in loaded:
        for b in ('on', 'off'):
            if b in eng:
                cmp[b] = compare(loaded['poke'], eng[b])
    out['compare'] = {k: strip(v) for k, v in cmp.items()}
    # classification, in the pre-declared order (gate document §5.3)
    why = []
    poke_status, null_status = (loaded.get('poke') or {}).get('meta', {}).get('status'), (loaded.get('null') or {}).get('meta', {}).get('status')
    if null_status == 'COMPLETED' and poke_status not in (None, 'COMPLETED') and val.get('null', {}).get('valid'):
        out['class'] = 'BLOCKED'
        out['classReason'] = 'original BA-poke run did not complete while the null-poke run did (original behaviour, design §3.9): %s' % poke_status
        return out
    for arm in ('poke', 'null'):
        if not val.get(arm, {}).get('valid'):
            why.append('%s run invalid: %s' % (arm, '; '.join(val.get(arm, {}).get('reasons', []))))
    for b in ('on', 'off', 'null'):
        if rexit.get(b) != 0:
            why.append('replay %s exit %s' % (b, rexit.get(b)))
    if 'null' in cmp and not identical(cmp['null']):
        why.append('null-poke control is not bit-identical (integer mismatches %s, pose nonzero frames %d)' % (
            {k: v for k, v in cmp['null']['D']['mismatchFrames'].items() if v}, cmp['null']['S']['poseNonzeroFrames']))
    for b in ('null', 'on', 'off'):
        if b in cmp and cmp[b]['framesCompared'] == 0:
            why.append('no frame compared (%s)' % b)
    if why:
        out['class'] = 'INVALID'
        out['classReason'] = why
        return out
    on, off = cmp['on'], cmp['off']
    nd = nondeterminism(on, engine_first_difference(eng['on'], eng['off']), setting_frame(loaded['poke'], 'ba'))
    out['nondeterminism'] = nd
    if nd['invalid']:
        out['class'] = 'INVALID'
        out['classReason'] = ['original BA-poke run differs from its replay at frame %s, before the BA setting could act (%s)' % (nd['firstDifferenceFrame'], nd)]
        return out
    off_fails = off['anyIntegerMismatch'] or off['S']['poseViolationFrames'] > 0
    e1 = {'kfPoseChangedSum': val['poke']['kfPoseChangedSum'], 'offArmFailsTheGate': off_fails}
    e1['met'] = e1['kfPoseChangedSum'] > 0 and off_fails
    out['E-BA-1'] = e1
    gd = 'PASS' if not on['anyIntegerMismatch'] else 'FAIL'
    gs = 'PASS' if on['S']['poseViolationFrames'] == 0 else 'FAIL'
    out['tiers'] = {'G-BA-D': gd, 'G-BA-S': gs}
    if gd == gs == 'PASS' and not e1['met']:
        # a PASS without evidence that BA acted is not counted; a FAIL is counted whether or not E-BA-1 holds (gate document §5.2)
        out['class'] = 'BLOCKED'
        out['classReason'] = 'E-BA-1 not met (BA without effect) and the on arm passes: %s' % e1
        return out
    out['class'] = 'EVALUATED'
    out['verdict'] = 'PASS' if gd == gs == 'PASS' else 'FAIL'
    out['report'] = {'R-BA-1': None if out['verdict'] == 'PASS' else {'firstIntegerMismatch': {k: v for k, v in on['D']['firstMismatch'].items() if v is not None},
                                                                       'firstPoseViolation': on['S']['poseFirstViolation']},
                     'R-BA-2_offArm': {'firstIntegerMismatch': {k: v for k, v in off['D']['firstMismatch'].items() if v is not None},
                                       'firstPoseViolation': off['S']['poseFirstViolation'], 'poseMaxUlps': off['S']['poseMaxUlps']},
                     'wpPositionPrintedMismatchFrames_on': on['S']['wpPositionPrintedMismatchFrames']}
    return out


# ---- MSE scenario (us026-mse-e2e-gate.v1 §5) ---------------------------------------------------------------------------
def mse_tiers(c, method):
    """G-MSE-D / G-MSE-S / G-MSE-P for one (original method N run, engine replay) comparison."""
    lat = c['latch']
    gd = 'PASS' if not c['anyIntegerMismatch'] else 'FAIL'
    if lat['originalScale'] is None and lat['engineScale'] is None:
        gs, s_detail = 'PASS', 'neither side latched (latch frame equality is in G-MSE-D)'
    elif lat['originalScale'] is None or lat['engineScale'] is None:
        gs, s_detail = 'FAIL', 'one side latched'
    elif method in MSE_REL_METHODS:
        gs, s_detail = ('PASS' if lat['scaleRel'] <= MSE_REL_TOL else 'FAIL'), 'relative %r <= %r' % (lat['scaleRel'], MSE_REL_TOL)
    else:
        gs, s_detail = ('PASS' if lat['scaleUlps'] <= ULP_FACTOR else 'FAIL'), 'ulps %r <= %r' % (lat['scaleUlps'], ULP_FACTOR)
    pu = c['_poseUlps']
    report_from = None
    if method in MSE_REL_METHODS and lat['originalFrame'] is not None and not lat.get('scaleBitIdentical', False):
        report_from = lat['originalFrame']      # pre-declared: post-latch pose is REPORT when the 3/4 scales differ
    judged = {i: u for i, u in pu.items() if report_from is None or i < report_from}
    viol = sorted(i for i, u in judged.items() if u > ULP_FACTOR)
    gp = 'PASS' if not viol else 'FAIL'
    return {'G-MSE-D': gd, 'G-MSE-S': gs, 'G-MSE-S_detail': s_detail, 'G-MSE-P': gp,
            'G-MSE-P_detail': {'framesJudged': len(judged), 'reportFromFrame': report_from, 'violations': len(viol), 'firstViolation': first(viol),
                               'maxUlpsJudged': max(judged.values(), default=0.0),
                               'reportOnlyFramesMaxUlps': max([u for i, u in pu.items() if i not in judged], default=None)}}


def mse_scenario(spec):
    runs, reps = spec['runs'], spec['replays']
    sens = (spec.get('sensors') or {}).get('expectedSha256Uncompressed')
    out = {'schema': SCHEMA, 'gate': 'mse', 'scenario': spec['scenario'], 'kind': spec.get('kind'), 'constants': CONSTANTS}
    loaded, val = {}, {}
    arms = {'null': ({'ba': 0, 'mse': 1}, 'all')}
    for n in (2, 3, 4, 5):
        arms['m%d' % n] = ({'mse': n}, 'mse')
    for arm, (keys, rule) in arms.items():
        r = runs.get(arm) or {}
        try:
            loaded[arm] = load_original(r['dir'])
            val[arm] = run_validity(loaded[arm], keys, rule, r.get('runnerExit'), r.get('prerun'), sens)
        except (OSError, KeyError, ValueError, IndexError) as error:
            val[arm] = {'valid': False, 'reasons': ['unreadable run: %s' % error]}
    out['validity'] = val
    eng, rexit = {}, {}
    for b, r in reps.items():
        rexit[b] = r.get('exit')
        try:
            if r.get('exit') == 0:
                eng[b] = load_engine(r['dir'])
        except (OSError, KeyError, ValueError) as error:
            rexit[b] = 'unreadable: %s' % error
    out['replayExit'] = rexit
    cmp = {}
    if 'null' in loaded and 'null' in eng:
        cmp['null'] = compare(loaded['null'], eng['null'], mse=True, method=1)
    for n in (2, 3, 4, 5):
        arm = 'm%d' % n
        if arm in loaded:
            for b in ('%s-on' % arm, '%s-m1' % arm):
                if b in eng:
                    cmp[b] = compare(loaded[arm], eng[b], mse=True, method=n if b.endswith('-on') else 1)
    out['compare'] = {k: strip(v) for k, v in cmp.items()}
    # scenario-level INVALID: the null-poke control
    why = []
    if not val['null'].get('valid'):
        why.append('null run invalid: %s' % '; '.join(val['null'].get('reasons', [])))
    if rexit.get('null') != 0:
        why.append('replay null exit %s' % rexit.get('null'))
    if 'null' in cmp and not identical(cmp['null']):
        why.append('null-poke control is not bit-identical (integer mismatches %s, pose nonzero frames %d, latch %s)' % (
            {k: v for k, v in cmp['null']['D']['mismatchFrames'].items() if v}, cmp['null']['S']['poseNonzeroFrames'], cmp['null'].get('latch')))
    if 'null' in cmp and cmp['null']['framesCompared'] == 0:
        why.append('no frame compared (null)')
    if why:
        out['class'] = 'INVALID'
        out['classReason'] = why
        return out
    out['class'] = 'EVALUATED'
    pairs = {}
    for n in (2, 3, 4, 5):
        arm = 'm%d' % n
        p = {'method': n}
        bad = []
        if not val[arm].get('valid'):
            bad.append('run invalid: %s' % '; '.join(val[arm].get('reasons', [])))
        if rexit.get('%s-on' % arm) != 0:
            bad.append('replay %s-on exit %s' % (arm, rexit.get('%s-on' % arm)))
        c = cmp.get('%s-on' % arm)
        if c is not None and c['framesCompared'] == 0:
            bad.append('no frame compared')
        if bad:
            p['class'], p['classReason'] = 'INVALID', bad
            pairs[arm] = p
            continue
        m1e = eng.get('%s-m1' % arm)
        nd = nondeterminism(c, engine_first_difference(eng['%s-on' % arm], m1e) if m1e is not None else None, setting_frame(loaded[arm], 'mse'))
        p['nondeterminism'] = nd
        if m1e is None or rexit.get('%s-m1' % arm) != 0:
            p['class'], p['classReason'] = 'INVALID', ['replay %s-m1 missing (exit %s): the control arm of the nondeterminism rule' % (arm, rexit.get('%s-m1' % arm))]
            pairs[arm] = p
            continue
        if nd['invalid']:
            p['class'], p['classReason'] = 'INVALID', ['original method-%d run differs from its replay at frame %s, before the method could act (%s)' % (n, nd['firstDifferenceFrame'], nd)]
            pairs[arm] = p
            continue
        p['E-MSE-1'] = mse_path(loaded[arm], n)
        p['tiers'] = mse_tiers(c, n)
        ok = all(p['tiers'][k] == 'PASS' for k in ('G-MSE-D', 'G-MSE-S', 'G-MSE-P'))
        m1 = cmp.get('%s-m1' % arm)
        t1 = mse_tiers(m1, n)
        p['E-MSE-2'] = {'met': not all(t1[k] == 'PASS' for k in ('G-MSE-D', 'G-MSE-S', 'G-MSE-P')), 'method1ArmTiers': t1}
        p['R-MSE-2_method1Arm'] = {'bitIdentical': m1['bitIdentical'], 'latch': m1['latch'], 'poseFirstNonzero': m1['S']['poseFirstNonzero'],
                                   'anyIntegerMismatch': m1['anyIntegerMismatch']}
        # a FAIL counts whatever E-MSE-1 / E-MSE-2 say; a PASS counts only when the method path ran (E-MSE-1) and the method
        # mattered (E-MSE-2: the engine method-1 arm fails the same tiers); gate document §5.2
        if ok and not p['E-MSE-1']['met']:
            p['class'], p['classReason'] = 'BLOCKED', 'E-MSE-1 not met (method %d path not executed) and the on arm passes: %s' % (n, p['E-MSE-1'])
        elif ok and not p['E-MSE-2']['met']:
            p['class'], p['classReason'] = 'BLOCKED', 'E-MSE-2 not met (method %d without effect: the engine method-1 arm passes too) and the on arm passes' % n
        else:
            p['class'] = 'EVALUATED'
            p['verdict'] = 'PASS' if ok else 'FAIL'
        pairs[arm] = p
    out['pairs'] = pairs
    # M-ID-5: original method 5 run vs the method-1 replay of the same bundle (device outside the lever-arm table)
    mid = {'evaluated': False}
    c5 = cmp.get('m5-m1')
    nd5 = nondeterminism(c5, None, setting_frame(loaded['m5'], 'mse')) if c5 is not None and 'm5' in loaded else None
    if nd5 is not None and nd5['invalid']:
        mid = {'evaluated': False, 'nondeterminism': nd5, 'reason': 'original method-5 run differs from the method-1 replay before any estimate (frame %s)' % nd5['firstDifferenceFrame']}
    elif val['m5'].get('valid') and rexit.get('m5-m1') == 0 and c5 is not None and c5['framesCompared'] > 0:
        t = mse_tiers(c5, 1)
        mid = {'evaluated': True, 'tiers': t, 'bitIdentical': c5['bitIdentical'], 'nondeterminism': nd5,
               'verdict': 'PASS' if all(t[k] == 'PASS' for k in ('G-MSE-D', 'G-MSE-S', 'G-MSE-P')) else 'FAIL'}
    elif not mid.get('reason'):
        mid['reason'] = 'm5 run invalid or m5-m1 replay missing'
    out['M-ID-5'] = mid
    return out


def nr1(orig_dir):
    """Comparator self vs self: an engine-format trace built from the original (memprobe and poke fields mapped to the
    engine names), compared with the same original. Every count must be 0 and every ulp 0."""
    o = load_original(orig_dir)
    eng = {}
    for i, r in o['trace'].items():
        m = o['probeByI'].get(i) or {}
        pr = o['pokeByI'].get(i) or {}
        row = dict(r)
        if m.get('pts1') is not None:
            row.update({'srMapReset': bool(m['outMapReset']), 'srKeyframeCreated': bool(m['outKeyframeCreated']), 'srKeyframes': m['kfLive'],
                        'srPointsBySource': [m['pts%d' % k][2] for k in range(1, 6)]})
        row['srScaleLatched'] = pr.get('vtLatch') == 1
        row['srLatchedScale'] = pr.get('vtLatchedScale') if pr.get('vtLatch') == 1 else 0
        eng[i] = row
    e = {'dir': 'self:' + str(orig_dir), 'trace': eng, 'wp': o['wp']}
    c = strip(compare(o, e, mse=True, method=1))
    return {'schema': SCHEMA, 'nr1': True, 'original': str(orig_dir), 'compare': c, 'zero': c['bitIdentical']}


def draft(a):
    """Lane A draft output (unchanged numbers: on / off comparisons and the poke validity inputs)."""
    o = load_original(a.original)
    rep = {'schema': 'slam-refact-us026-compare-draft-v1', 'note': 'DRAFT comparator, numbers only; tolerances are candidates (gate file decides)',
           'original': str(a.original), 'engine': str(a.engine), 'validity': run_validity(o, o['pokeHeader'].get('keys') if o['pokeHeader'] else None, 'ba'),
           'on': strip(compare(o, load_engine(a.engine)))}
    if a.engine_off:
        off = strip(compare(o, load_engine(a.engine_off)))
        rep['off'] = off
        rep['E-BA-1_inputs'] = {'kfPoseChangedSum': rep['validity'].get('kfPoseChangedSum'),
                                'offArmDiffers': off['anyIntegerMismatch'] or off['S']['poseViolationFrames'] > 0}
    return rep


def write(path, obj):
    path = pathlib.Path(path)
    if path.exists():
        raise SystemExit('refusing to overwrite ' + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'x') as f:
        f.write(json.dumps(obj, indent=1, allow_nan=True) + '\n')


def main(argv):
    if argv and argv[0] == 'print-constants':
        print(json.dumps(CONSTANTS, sort_keys=True))
        return 0
    if argv and argv[0] in ('ba-scenario', 'mse-scenario', 'nr1'):
        ap = argparse.ArgumentParser(prog='compare_us026.py ' + argv[0])
        ap.add_argument('--spec', type=pathlib.Path)
        ap.add_argument('--original', type=pathlib.Path)
        ap.add_argument('--out', type=pathlib.Path, required=True)
        a = ap.parse_args(argv[1:])
        if argv[0] == 'nr1':
            rep = nr1(a.original)
            write(a.out, rep)
            print(json.dumps({'nr1': rep['zero'], 'framesCompared': rep['compare']['framesCompared']}))
            return 0 if rep['zero'] else 1
        spec = json.loads(a.spec.read_text())
        rep = ba_scenario(spec) if argv[0] == 'ba-scenario' else mse_scenario(spec)
        rep['spec'] = spec
        write(a.out, rep)
        print(json.dumps({'scenario': rep['scenario'], 'class': rep['class'], 'verdict': rep.get('verdict'),
                          'pairs': {k: (v.get('class'), v.get('verdict')) for k, v in (rep.get('pairs') or {}).items()},
                          'M-ID-5': (rep.get('M-ID-5') or {}).get('verdict')}))
        return 0
    ap = argparse.ArgumentParser()
    ap.add_argument('--original', type=pathlib.Path, required=True)
    ap.add_argument('--engine', type=pathlib.Path, required=True)
    ap.add_argument('--engine-off', type=pathlib.Path, help='replay of the same bundle with BA off (M-BA-1 arm)')
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args(argv)
    rep = draft(a)
    write(a.out, rep)
    print(json.dumps({'validity': rep['validity'].get('valid'), 'framesCompared': rep['on']['framesCompared'],
                      'G-BA-D_any': rep['on']['anyIntegerMismatch'], 'G-BA-S_viol': rep['on']['S']['poseViolationFrames'],
                      'E-BA-1': rep.get('E-BA-1_inputs')}))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
