#!/usr/bin/env python3
"""US-025 LC-6a comparator (dev/REPORT, not a registered gate): original reference runtime wasm with loop close ON/OFF versus the engine's exact-input replay.

  python3 -B -I compare_us025.py on   --original <lc=1 run dir> --engine <replay dir, loop close ON>  --out <new json>
  python3 -B -I compare_us025.py off  --original <lc=1 run dir> --engine <replay dir, loop close OFF> --out <new json>
  python3 -B -I compare_us025.py null --original <lc=0 run dir> --engine <replay dir, loop close OFF> --out <new json>
  python3 -B -I compare_us025.py suite --spec <spec.json> --out <new json>        (all arms, aggregate verdict; spec = lc6a/make_spec.py)
  python3 -B -I compare_us025.py constants

<original run dir> comes from the LC-G golden runner (run_lcgold.mjs, `lcgold/` directory next to the trace) or, for the null arm, from run_lc6a.mjs /
run-reference.mjs (`--record-inputs --memprobe --memprobe-detail 12 --poke '{"lc":0}'`). <replay dir> comes from slam_refact_replay_lc6a
(lc6a-attempts.jsonl: one line per call of map_builder::LoopClose; trace.jsonl and worldpoints-sampled.jsonl as slam_refact_replay writes them).

What is compared (everything is an integer or a bit pattern; no tolerance is chosen from results):

  ON arm   original lc=1 versus engine lc=1, up to the FIRST loop close success F of the original (N2, docs/spec/12 §10: the original leaves the success call
           through a crash caused by freeing a std::set node it still walks, so its state after F is undefined and the engine's after-F behaviour is a
           declared deviation; nothing after F is compared and the output says so).
    (a) attempt frame set (every keyframe creation with loop close on) and per attempt slot count / live / free / newest handle;
    (b) ring state after the call (last newest index+id, ring[3], cursor) and the candidate id ring[(cursor+2)%3];
    (c) threshold decisions: the stage at which each attempt ended (<5 slots, no candidate, invalid handle, matches < 75, correspondences < 35, PnP failed,
        applied) and, where the original's hook run observed them, the match count (1128) and the correspondence count (1146 input);
    (d) at F: out+0x1c, correction matrix C (128 bytes, bit and ulp), newest keyframe pose after the call (PnP output; declared tolerance 3 ulp, spec 12 §8
        G-E), hook runs also P_old; the trigger frame set (engine success frames up to F must be exactly {F});
    (e) frames < F: the trace and world point output bit for bit (compare_us026.compare, every field and every pose component, 0 ulp, plus the memprobe
        integers);
    (f) the agate keyframe global descriptor of every slot at every attempt (spec 12 §2.5), live slots and free slots counted separately.
  OFF arm  original lc=1 versus engine lc=0: frames < F bit-identical (failed attempts leave no state behind) and the first differing frame is F, i.e. the
           arm reacts to the trigger (detection power of the comparison).
  NULL arm original lc=0 versus engine lc=0 over the whole run: the measurement floor (the engine replay of an original run without loop close).

Declared before any result (docs/design/us025-lc3-impl.md §4/§5 and us025-lcg-golden.md §5, written by other tasks): the original's candidate list tail is
uninitialized heap (U2); an original pick whose id is not a live keyframe of that attempt is such heap content, cannot be reproduced and is counted as
DECLARED_U2, not as match and not as mismatch. Every other difference is a MISMATCH with the first differing value and its position.

Verdicts per run/arm: PASS, PASS_DECLARED (pass except declared U2 picks), FAIL, INVALID (the inputs cannot support a comparison), BLOCKED (the original
never triggered, or the arm cannot detect), TOOL_ERROR (this script crashed; never FAIL). Aggregate: FAIL if any FAIL; else INVALID if any INVALID (an
INVALID run is never silently dropped); else BLOCKED when nothing was evaluated; else INSUFFICIENT when fewer than K_MIN runs passed; else PASS.
Exit codes: PASS/PASS_DECLARED 0, TOOL_ERROR 4, FAIL 10, INVALID 11, INSUFFICIENT 12, BLOCKED 13. An unhandled Python exception exits 1, which is not a
verdict.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import pathlib
import struct
import sys
import traceback

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent

# ---- decision constants (declared before the first result) --------------------------------------------------------------
SCHEMA = 'slam-refact-us025-lc6a-compare-v1'
PNP_ULP = 3                  # docs/spec/12 §8 G-E: PnP output pose within 3 float32 ulp of the original (every other value is bit/integer exact)
MIN_SLOTS = 5                # spec 12 S1
MATCHES_REQUIRED = 75        # spec 12 S9 (WorldMaps.loopClosingMatchesRequired)
CORR_REQUIRED = 35           # spec 12 S11
K_MIN = 3                    # fewer evaluated ON runs than this is INSUFFICIENT (ba-e2e v2 rule: k <= 2 is a thin sample)
WASM_SHA256 = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422'
MATRIX_BYTES = 128           # forward float[16] + inverse float[16]; the two flag bytes are not exposed by the engine's HMatrix (N5)
EXIT = {'PASS': 0, 'PASS_DECLARED': 0, 'TOOL_ERROR': 4, 'FAIL': 10, 'INVALID': 11, 'INSUFFICIENT': 12, 'BLOCKED': 13}
CONSTANTS = {'PNP_ULP': PNP_ULP, 'MIN_SLOTS': MIN_SLOTS, 'MATCHES_REQUIRED': MATCHES_REQUIRED, 'CORR_REQUIRED': CORR_REQUIRED, 'K_MIN': K_MIN,
             'WASM_SHA256': WASM_SHA256, 'MATRIX_BYTES': MATRIX_BYTES, 'EXIT': EXIT}
# engine LoopCloseOutcome (loop_close.h) -> stage name
OUTCOME = {0: 'TOO_FEW_SLOTS', 1: 'NO_CANDIDATE', 2: 'INVALID_HANDLE', 3: 'TOO_FEW_MATCHES', 4: 'TOO_FEW_CORRESPONDENCES', 5: 'PNP_FAILED', 6: 'APPLIED'}
COARSE_NOT_APPLIED = ('INVALID_HANDLE', 'TOO_FEW_MATCHES', 'TOO_FEW_CORRESPONDENCES', 'PNP_FAILED')


def _load_us026():
    spec = importlib.util.spec_from_file_location('compare_us026', HERE.parent / 'us026' / 'compare_us026.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


U26 = _load_us026()
ulps = U26.ulps


class Invalid(Exception):
    """The inputs cannot support a comparison (never a FAIL)."""


# ---- small helpers ------------------------------------------------------------------------------------------------------
def jsonl(p, gz=False):
    with (gzip.open(p, 'rt') if gz else open(p)) as f:
        return [json.loads(line) for line in f if line.strip()]


def floats(b):
    return list(struct.unpack('<%df' % (len(b) // 4), b))


def hex_bytes(h):
    return bytes.fromhex(h) if h else b''


def matrix_diff(orig, eng):
    """Bit/ulp comparison of two 128-byte matrices (32 float32). Returns bitIdentical, maxUlps, first differing component."""
    if len(orig) < MATRIX_BYTES or len(eng) < MATRIX_BYTES:
        return {'bitIdentical': False, 'maxUlps': math.inf, 'first': {'component': None, 'note': 'missing (%d / %d bytes)' % (len(orig), len(eng))}}
    a, b = floats(orig[:MATRIX_BYTES]), floats(eng[:MATRIX_BYTES])
    first_diff, mx = None, 0.0
    for k, (x, y) in enumerate(zip(a, b)):
        if orig[4 * k:4 * k + 4] != eng[4 * k:4 * k + 4]:
            u = ulps(x, y)
            if first_diff is None:
                first_diff = {'component': k, 'original': x, 'engine': y, 'originalHex': orig[4 * k:4 * k + 4].hex(), 'engineHex': eng[4 * k:4 * k + 4].hex(), 'ulps': u}
            mx = max(mx, u)
    return {'bitIdentical': first_diff is None, 'maxUlps': mx, 'first': first_diff}


def check(name, compared=0, match=0, mismatches=None, declared=None, note=None):
    mm = mismatches or []
    out = {'name': name, 'compared': compared, 'match': match, 'mismatch': len(mm), 'declaredU2': len(declared or []),
           'first': mm[0] if mm else None}
    if declared:
        out['firstDeclared'] = declared[0]
    if note:
        out['note'] = note
    return out


# ---- original side -------------------------------------------------------------------------------------------------------
def read_blob(blobs, ref):
    b = blobs[ref['off']:ref['off'] + ref['len']]
    if len(b) != ref['len'] or hashlib.sha256(b).hexdigest() != ref['sha256']:
        raise Invalid('blob sha256 mismatch at offset %d' % ref['off'])
    return b


def load_original_lc(run_dir, expect_lc):
    """The loop close attempts of an LC-G run (lcgold/ directory) with the frame each belongs to. expect_lc is the poke value the run must carry (0 or 1)."""
    d = pathlib.Path(run_dir)
    reasons = []
    meta = json.loads((d / 'meta.json').read_text())
    poke = meta.get('poke') or {}
    if (poke.get('keys') or {}) != {'lc': expect_lc}:
        reasons.append('poke keys %s, expected {"lc": %d}' % (poke.get('keys'), expect_lc))
    if poke.get('error') or poke.get('callsWithGap'):
        reasons.append('poke error %s / calls with gap %s' % (poke.get('error'), poke.get('callsWithGap')))
    if poke.get('wasmSha256') != WASM_SHA256:
        reasons.append('wasm sha256 %s' % poke.get('wasmSha256'))
    ri = meta.get('recordedInputs') or {}
    if not ri.get('wrapped') or ri.get('error') or ri.get('droppedCalls'):
        reasons.append('recorded inputs incomplete (wrapped %s, error %s, dropped %s)' % (ri.get('wrapped'), ri.get('error'), ri.get('droppedCalls')))
    if reasons:
        raise Invalid('; '.join(reasons))
    rec = jsonl(d / 'recorded-inputs.jsonl.gz', gz=True)[1:]
    seq2i = {x['probeSeq']: x['i'] for x in rec if x.get('fn') == 'processStagedFrame' and x.get('probeSeq') is not None}
    out = {'dir': str(d), 'meta': meta, 'seq2i': seq2i, 'attempts': [], 'hooked': False, 'firstSuccess': None, 'restaged': {}}
    lg = d / 'lcgold'
    if expect_lc == 0 or not lg.is_dir():
        return out
    header = json.loads((lg / 'header.json').read_text())
    if header.get('wasmSha256') != WASM_SHA256:
        raise Invalid('lcgold header wasm sha256 %s' % header.get('wasmSha256'))
    blobs = (lg / 'blobs.bin').read_bytes()
    events = jsonl(lg / 'events.jsonl')
    hooks = {}
    if (lg / 'hooks.jsonl').exists():
        for h in jsonl(lg / 'hooks.jsonl'):
            hooks.setdefault(h['seq'], {}).setdefault(h['id'], []).append(h)
    mp = jsonl(d / 'memprobe.jsonl.gz', gz=True)[1:]
    mp_ok = {r['seq']: r.get('outLcOk') for r in mp}
    seen_stage = {}
    for ev in events:
        if 'skipped' in ev:
            continue
        seq = ev['seq']
        if seq not in seq2i or seq2i[seq] is None:
            raise Invalid('event seq %d has no processStagedFrame input frame' % seq)
        if not ev.get('lcEnabled'):
            raise Invalid('event seq %d: loop close not enabled in the original run' % seq)
        if mp_ok.get(seq) is not None and bool(mp_ok[seq]) != bool(ev['outLcOk']):
            raise Invalid('event seq %d: outLcOk %s differs from memprobe %s' % (seq, ev['outLcOk'], mp_ok[seq]))
        stage = seq2i[seq]
        if stage in seen_stage:       # the SDK re-stages a frame after an exception: only the first call of a frame is an attempt
            seen_stage[stage].append(seq)
            continue
        seen_stage[stage] = [seq]
        live_ids = {s['id'] for s in ev['slots'] if s['id'] != 0}
        ring_post = ev['ringPost']
        ring = ring_post[2:5]
        a = {'seq': seq, 'frame': stage, 'nSlots': ev['nSlots'], 'nLive': ev['nLive'], 'free': ev['nSlots'] - ev['nLive'], 'newest': ev['newest'],
             'ringPre': ev['ringPre'], 'ringPost': ring_post, 'pick': ring[(ring_post[5] + 2) % 3], 'liveIds': sorted(live_ids), 'outLcOk': ev['outLcOk'],
             'desc': read_blob(blobs, ev['desc']) if ev.get('desc') else b'',
             'outLcMat': read_blob(blobs, ev['outLcMat']) if ev.get('outLcMat') else b'',
             'newestExtrinsic': read_blob(blobs, ev['newestExtrinsic']) if ev.get('newestExtrinsic') else b'', 'hooks': {}}
        hk = hooks.get(seq)
        if hk:
            h = a['hooks']
            h['u2Found'] = (hk.get('u2-result') or [{}])[0].get('found')
            if '1128-out' in hk:
                h['matches'] = hk['1128-out'][0]['nMatches']
            if '1146-in' in hk:
                h['corr'] = hk['1146-in'][0]['nObs']
                h['pnpPoseIn'] = read_blob(blobs, hk['1146-in'][0]['pose'])
            if '1146-out' in hk:
                h['pnpRet'] = hk['1146-out'][0]['ret']
                h['pnpPoseOut'] = read_blob(blobs, hk['1146-out'][0]['pose'])
            if '660-in' in hk:
                h['poseOld'] = read_blob(blobs, hk['660-in'][0]['A'])
            if '660-out' in hk:
                h['C'] = read_blob(blobs, hk['660-out'][0]['C'])
        out['attempts'].append(a)
    out['hooked'] = bool(header.get('hookMode'))
    out['restaged'] = {str(k): v for k, v in seen_stage.items() if len(v) > 1}
    out['header'] = {'hookMode': header.get('hookMode'), 'aborts': len(header.get('aborts') or []), 'throws': len(header.get('throws') or [])}
    succ = [a for a in out['attempts'] if a['outLcOk']]
    out['firstSuccess'] = succ[0] if succ else None
    return out


def orig_stage(a, hooked):
    """The stage at which the original's attempt ended, from what the run observed."""
    if a['nSlots'] < MIN_SLOTS:
        return 'TOO_FEW_SLOTS'
    if a['pick'] == 0:
        return 'NO_CANDIDATE'
    if a['outLcOk']:
        return 'APPLIED'
    if not hooked:
        return 'PICK_NOT_APPLIED'
    h = a['hooks']
    if 'matches' not in h:
        return 'INVALID_HANDLE'
    if h['matches'] < MATCHES_REQUIRED:
        return 'TOO_FEW_MATCHES'
    if 'corr' not in h:
        return 'TOO_FEW_CORRESPONDENCES'
    return 'PNP_FAILED' if not h.get('pnpRet') else 'UNEXPECTED'


def engine_stage(e, hooked):
    s = OUTCOME[e['outcome']]
    if not hooked and s in COARSE_NOT_APPLIED:
        return 'PICK_NOT_APPLIED'
    return s


# ---- engine side ---------------------------------------------------------------------------------------------------------
def load_engine_lc(run_dir):
    d = pathlib.Path(run_dir)
    if not (d / 'meta.json').exists():
        raise Invalid('engine replay %s has no meta.json (replay did not finish)' % d)
    meta = json.loads((d / 'meta.json').read_text())
    p = d / 'lc6a-attempts.jsonl'
    if not p.exists():
        raise Invalid('%s has no lc6a-attempts.jsonl (not a slam_refact_replay_lc6a run)' % d)
    att = {}
    for r in jsonl(p):
        if r['frame'] in att:
            raise Invalid('engine: two LoopClose calls for frame %d' % r['frame'])
        att[r['frame']] = r
    cfg = meta.get('engineConfig') or {}
    return {'dir': str(d), 'meta': meta, 'attempts': att, 'params': cfg.get('params') or cfg.get('engine_params')}


# ---- ON arm: attempt-level judgement (pure function over loaded data) ------------------------------------------------------------
def judge_attempts(orig, eng):
    """Checks (a)-(d),(f) of the ON arm. `orig` from load_original_lc, `eng` from load_engine_lc."""
    first = orig['firstSuccess']
    if first is None:
        raise Invalid('no original success')   # the caller turns this into BLOCKED before getting here
    F = first['frame']
    hooked = orig['hooked']
    oa = [a for a in orig['attempts'] if a['frame'] <= F]
    ea = {f: r for f, r in eng['attempts'].items() if 0 <= f <= F}
    of = [a['frame'] for a in oa]
    # (a) attempt frame set
    miss = sorted(set(of) - set(ea))
    extra = sorted(set(ea) - set(of))
    set_mm = [{'frame': f, 'original': 'attempt', 'engine': 'none'} for f in miss] + [{'frame': f, 'original': 'none', 'engine': 'attempt'} for f in extra]
    set_mm.sort(key=lambda x: x['frame'])
    checks = [check('attemptFrameSet', compared=len(set(of) | set(ea)), match=len(set(of) & set(ea)), mismatches=set_mm)]
    common = [a for a in oa if a['frame'] in ea]
    slots_mm, newest_mm, live_mm = [], [], []
    ring_mm, pick_mm, pick_decl = [], [], []
    stage_mm, stage_decl = [], []
    mt_mm, mt_cmp, co_mm, co_cmp = [], 0, [], 0
    desc_live_mm, desc_free_mm, desc_live_n, desc_free_n = [], [], 0, 0
    for a in common:
        e = ea[a['frame']]
        f = a['frame']
        if (e['nSlots'], e['nLive'], e['nSlots'] - e['nLive']) != (a['nSlots'], a['nLive'], a['free']):
            slots_mm.append({'frame': f, 'original': {'slots': a['nSlots'], 'live': a['nLive'], 'free': a['free']},
                             'engine': {'slots': e['nSlots'], 'live': e['nLive'], 'free': e['nSlots'] - e['nLive']}})
        if list(e['newest']) != list(a['newest']):
            newest_mm.append({'frame': f, 'original': a['newest'], 'engine': e['newest']})
        # (f) descriptors, slot by slot
        ed = [bytes.fromhex(x) for x in e['desc']]
        for s in range(a['nSlots']):
            ob = a['desc'][32 * s:32 * s + 32]
            is_live = s < len(e['slots']) and e['slots'][s] != 0
            if is_live:
                desc_live_n += 1
            else:
                desc_free_n += 1
            if s >= len(ed) or ob != ed[s]:
                (desc_live_mm if is_live else desc_free_mm).append({'frame': f, 'slot': s, 'original': ob.hex(), 'engine': ed[s].hex() if s < len(ed) else None})
        # (b) ring and candidate
        if list(e['ringPost']) != list(a['ringPost']):
            ring_mm.append({'frame': f, 'original': a['ringPost'], 'engine': e['ringPost']})
        if a['nSlots'] >= MIN_SLOTS:
            e_pick = e['ringPost'][2:5][(e['ringPost'][5] + 2) % 3]
            if e_pick != a['pick']:
                rec = {'frame': f, 'original': a['pick'], 'engine': e_pick}
                (pick_mm if (a['pick'] == 0 or a['pick'] in a['liveIds']) else pick_decl).append(rec)
        # (c) stage reached, match and correspondence counts
        os_, es = orig_stage(a, hooked), engine_stage(e, hooked)
        declared = a['nSlots'] >= MIN_SLOTS and a['pick'] != 0 and a['pick'] not in a['liveIds']
        if os_ != es:
            (stage_decl if declared else stage_mm).append({'frame': f, 'original': os_, 'engine': es})
        elif not declared:
            h = a['hooks']
            if 'matches' in h and e['outcome'] >= 3:
                mt_cmp += 1
                if e['nMatches'] != h['matches']:
                    mt_mm.append({'frame': f, 'original': h['matches'], 'engine': e['nMatches']})
            if 'corr' in h and e['outcome'] >= 4:
                co_cmp += 1
                if e['nCorr'] != h['corr']:
                    co_mm.append({'frame': f, 'original': h['corr'], 'engine': e['nCorr']})
    n_ok_slots = len(common) - len(slots_mm)
    checks.append(check('slotLiveFree', compared=len(common), match=n_ok_slots, mismatches=slots_mm))
    checks.append(check('newestHandle', compared=len(common), match=len(common) - len(newest_mm), mismatches=newest_mm))
    n_s1 = sum(1 for a in common if a['nSlots'] >= MIN_SLOTS)
    checks.append(check('candidateId', compared=n_s1, match=n_s1 - len(pick_mm) - len(pick_decl), mismatches=pick_mm, declared=pick_decl,
                        note='attempts with >= %d slots; DECLARED_U2 = original pick id is not a live keyframe (uninitialized list tail)' % MIN_SLOTS))
    checks.append(check('ringState', compared=len(common), match=len(common) - len(ring_mm),
                        mismatches=[m for m in ring_mm if not any(d['frame'] == m['frame'] for d in pick_decl)],
                        declared=[m for m in ring_mm if any(d['frame'] == m['frame'] for d in pick_decl)],
                        note='[last newest index, id, ring0..2, cursor] after the call'))
    checks.append(check('thresholdStage', compared=len(common), match=len(common) - len(stage_mm) - len(stage_decl), mismatches=stage_mm, declared=stage_decl,
                        note='stage at which the attempt ended; %s' % ('hook run: INVALID_HANDLE..PNP_FAILED are distinguished' if hooked else 'plain run: those four are one class PICK_NOT_APPLIED')))
    if hooked:
        checks.append(check('matchCount', compared=mt_cmp, match=mt_cmp - len(mt_mm), mismatches=mt_mm, note='1128 output size (hook run)'))
        checks.append(check('correspondenceCount', compared=co_cmp, match=co_cmp - len(co_mm), mismatches=co_mm, note='1146 input count (hook run)'))
    else:
        checks.append({'name': 'matchCount', 'status': 'UNOBSERVED', 'note': 'plain (uninstrumented) original run: the match count is not recorded'})
        checks.append({'name': 'correspondenceCount', 'status': 'UNOBSERVED', 'note': 'plain (uninstrumented) original run: the correspondence count is not recorded'})
    checks.append(check('globalDescriptorLiveSlots', compared=desc_live_n, match=desc_live_n - len(desc_live_mm), mismatches=desc_live_mm))
    checks.append(check('globalDescriptorFreeSlots', compared=desc_free_n, match=desc_free_n - len(desc_free_mm), mismatches=desc_free_mm,
                        note='free-slot entries keep stale bytes in both (not erased on removal); reported, same rule'))
    # (d) the success call
    es_frames = sorted(f for f, r in ea.items() if r['applied'])
    trig = [] if es_frames == [F] else [{'frame': es_frames[0] if es_frames else F, 'original': [F], 'engine': es_frames}]
    checks.append(check('triggerFrameSet', compared=1, match=1 - len(trig), mismatches=trig, note='engine loop close successes at frames <= F'))
    e = ea.get(F)
    succ = []
    if e is None or not e['applied']:
        succ.append({'item': 'applied', 'original': True, 'engine': False})
        c_res = pose_res = None
    else:
        if e['outLcOk'] != 1 or first['outLcOk'] != 1:
            succ.append({'item': 'out+0x1c', 'original': first['outLcOk'], 'engine': e['outLcOk']})
        c_res = matrix_diff(first['outLcMat'], hex_bytes(e['C']))
        pose_res = matrix_diff(first['newestExtrinsic'], hex_bytes(e['poseAfter']))
        if not c_res['bitIdentical']:
            succ.append({'item': 'correctionC', 'bitIdentical': False, 'maxUlps': c_res['maxUlps'], 'first': c_res['first']})
        if not pose_res['bitIdentical'] and pose_res['maxUlps'] > PNP_ULP:
            succ.append({'item': 'newestPoseAfter', 'maxUlps': pose_res['maxUlps'], 'first': pose_res['first'], 'tolerance': PNP_ULP})
        h = first['hooks']
        if 'poseOld' in h:
            po = matrix_diff(h['poseOld'], hex_bytes(e['poseBefore']))
            if not po['bitIdentical']:
                succ.append({'item': 'poseOld', 'maxUlps': po['maxUlps'], 'first': po['first']})
    checks.append(check('successCall', compared=1, match=1 - len(succ), mismatches=succ,
                        note='frame %d: out+0x1c, C 128 bytes bit exact (spec G-F), newest pose after PnP within %d ulp (spec G-E)%s'
                        % (F, PNP_ULP, ', P_old bit exact' if 'poseOld' in first['hooks'] else '')))
    detail = {'successFrame': F, 'successSeq': first['seq'], 'selfCandidate': first['pick'] == first['newest'][1],
              'correctionC': c_res, 'newestPoseAfter': pose_res, 'attemptsCompared': len(common), 'attemptsInWindow': len(set(of) | set(ea)),
              'engineAttemptsAfterSuccess(not compared, N2)': sum(1 for f in eng['attempts'] if f > F)}
    return checks, detail


def verdict_of(checks):
    bad = [c for c in checks if c.get('mismatch')]
    declared = [c for c in checks if c.get('declaredU2')]
    empty = [c['name'] for c in checks if 'compared' in c and c['name'] in ('attemptFrameSet', 'successCall') and c['compared'] == 0]
    if bad:
        return 'FAIL'
    if empty:
        return 'INVALID'
    return 'PASS_DECLARED' if declared else 'PASS'


# ---- trace-level comparison through compare_us026 -------------------------------------------------------------------------------
def restricted(o, e, lo, hi):
    """The same original/engine data limited to input frames lo <= i < hi."""
    keep = lambda i: (lo is None or i >= lo) and (hi is None or i < hi)
    o2 = dict(o)
    o2['trace'] = {i: r for i, r in o['trace'].items() if keep(i)}
    o2['probeByI'] = {i: r for i, r in o['probeByI'].items() if keep(i)}
    o2['wp'] = {i: r for i, r in o['wp'].items() if keep(i)}
    e2 = dict(e)
    e2['trace'] = {i: r for i, r in e['trace'].items() if keep(i)}
    e2['wp'] = {i: r for i, r in e['wp'].items() if keep(i)}
    return o2, e2


def first_difference_detail(c, o, e):
    """The first differing value of a compare_us026.compare result: frame, field, original and engine value."""
    f = c['firstDifferenceFrame']
    if f is None:
        return None
    ro, re_ = o['trace'].get(f, {}), e['trace'].get(f, {})
    d = {'frame': f}
    for k in ('fresh', 'status', 'reason', 'worldPoints', 'wpAdded', 'wpRemoved'):
        if ro.get(k) != re_.get(k):
            d.update({'field': k, 'original': ro.get(k), 'engine': re_.get(k)})
            return d
    ca = (re_.get('p') or []) + (re_.get('q_wxyz') or [])
    cb = (ro.get('p') or []) + (ro.get('q_wxyz') or [])
    for k, (x, y) in enumerate(zip(ca, cb)):
        if x != y:
            d.update({'field': 'pose[%d] (p0..2, q_wxyz)' % k, 'original': y, 'engine': x, 'ulps': ulps(x, y)})
            return d
    d['field'] = 'integer tier (see D.firstMismatch) or world point id list'
    return d


def trace_check(o, e, lo, hi, name):
    o2, e2 = restricted(o, e, lo, hi)
    c = U26.compare(o2, e2)
    n = c['framesCompared']
    mism = c['firstDifferenceFrame'] is not None or not c['bitIdentical']
    out = {'name': name, 'compared': n, 'frameRange': [lo, hi], 'memprobeFramesCompared': c['memprobeFramesCompared'], 'bitIdentical': c['bitIdentical'],
           'mismatch': 1 if mism else 0, 'declaredU2': 0, 'D': c['D'], 'S': {k: c['S'][k] for k in ('poseMaxUlps', 'poseNonzeroFrames', 'poseFirstNonzero',
                                                                                  'wpPositionPrintedMismatchFrames', 'wpPositionFirstMismatch')},
           'firstDifferenceFrame': c['firstDifferenceFrame'], 'firstDifference': first_difference_detail(c, o2, e2),
           'wpPositions': 'REPORT (printed toFixed(5), not a tier)'}
    out['first'] = out['firstDifference'] if mism else None
    return out, c


# ---- arms ----------------------------------------------------------------------------------------------------------------------------
def tool_info():
    return {'schema': SCHEMA, 'constants': CONSTANTS}


def arm_on(original, engine):
    out = {'arm': 'on', 'original': str(original), 'engine': str(engine), 'compareRange': 'frames up to and including the first original success F; frames after F are '
           'NOT compared (N2: the original leaves the success call through a crash, docs/spec/12 §10)'}
    try:
        orig = load_original_lc(original, 1)
        eng = load_engine_lc(engine)
        o = U26.load_original(original)
        e = U26.load_engine(engine)
    except Invalid as ex:
        out.update({'verdict': 'INVALID', 'reason': str(ex)})
        return out
    out['originalRun'] = {'success': orig['firstSuccess'] is not None, 'attempts': len(orig['attempts']), 'hookMode': orig['hooked'], 'restaged': orig.get('restaged'),
                          'loadAvgAtStart': orig['meta'].get('loadAvgAtStart'), 'status': orig['meta'].get('status')}
    out['engineAttemptsTotal'] = len(eng['attempts'])
    if orig['firstSuccess'] is None:
        out.update({'verdict': 'BLOCKED', 'reason': 'the original run never succeeded in loop close (no trigger); nothing to compare'})
        return out
    if not eng['attempts']:
        out.update({'verdict': 'INVALID', 'reason': 'engine made no LoopClose call (loop close off in this replay?)'})
        return out
    F = orig['firstSuccess']['frame']
    pre = {k: v for k, v in orig['restaged'].items() if int(k) < F}
    if pre:
        out.update({'verdict': 'INVALID', 'reason': 'a frame before F was staged twice in the original (exception before the first success): %s' % pre})
        return out
    try:
        checks, detail = judge_attempts(orig, eng)
    except Invalid as ex:
        out.update({'verdict': 'INVALID', 'reason': str(ex)})
        return out
    tr, _ = trace_check(o, e, None, F, 'traceAndWorldPointsBeforeSuccess')
    checks.append(tr)
    out.update({'checks': checks, 'detail': detail, 'verdict': verdict_of(checks)})
    if tr['compared'] == 0:
        out['verdict'] = 'INVALID'
        out['reason'] = 'no frame compared before F'
    return out


def arm_off(original, engine):
    out = {'arm': 'off', 'original': str(original), 'engine': str(engine)}
    try:
        orig = load_original_lc(original, 1)
        eng = load_engine_lc(engine)
        o = U26.load_original(original)
        e = U26.load_engine(engine)
    except Invalid as ex:
        out.update({'verdict': 'INVALID', 'reason': str(ex)})
        return out
    if eng['attempts']:
        out.update({'verdict': 'INVALID', 'reason': 'engine made %d LoopClose calls in an OFF replay' % len(eng['attempts'])})
        return out
    if orig['firstSuccess'] is None:
        out.update({'verdict': 'BLOCKED', 'reason': 'the original run never succeeded in loop close (no trigger): the off arm cannot show a reaction'})
        return out
    F = orig['firstSuccess']['frame']
    pre, _ = trace_check(o, e, None, F, 'traceAndWorldPointsBeforeSuccess')
    allr, c = trace_check(o, e, None, None, 'traceAndWorldPointsWholeRun')
    fd = c['firstDifferenceFrame']
    reacts = fd == F
    out.update({'successFrame': F, 'checks': [pre, allr],
                'firstDifferenceFrame': fd,
                'note': 'expected: bit-identical before F (failed attempts leave no state) and the first difference exactly at F (the arm reacts to the trigger)'})
    if pre['compared'] == 0:
        out['verdict'] = 'INVALID'
        out['reason'] = 'no frame compared before F'
    elif pre['mismatch']:
        out['verdict'] = 'FAIL'
        out['reason'] = 'engine OFF differs from the original lc=1 run before F (first difference %s)' % (pre['firstDifference'],)
    elif fd is None:
        out['verdict'] = 'BLOCKED'
        out['reason'] = 'engine OFF is bit-identical to the original lc=1 run over the whole run: the arm cannot detect the trigger'
    elif not reacts:
        out['verdict'] = 'FAIL'
        out['reason'] = 'first difference at frame %s, expected F = %s' % (fd, F)
    else:
        out['verdict'] = 'PASS'
    return out


def arm_null(original, engine):
    out = {'arm': 'null', 'original': str(original), 'engine': str(engine)}
    try:
        load_original_lc(original, 0)
        eng = load_engine_lc(engine)
        o = U26.load_original(original)
        e = U26.load_engine(engine)
    except Invalid as ex:
        out.update({'verdict': 'INVALID', 'reason': str(ex)})
        return out
    if eng['attempts']:
        out.update({'verdict': 'INVALID', 'reason': 'engine made %d LoopClose calls in an OFF replay' % len(eng['attempts'])})
        return out
    status = o['meta'].get('status')
    if status != 'COMPLETED':
        out.update({'verdict': 'INVALID', 'reason': 'original lc=0 run status %s' % status})
        return out
    allr, _ = trace_check(o, e, None, None, 'traceAndWorldPointsWholeRun')
    out.update({'checks': [allr], 'loadAvgAtStart': o['meta'].get('loadAvgAtStart')})
    if allr['compared'] == 0:
        out['verdict'] = 'INVALID'
        out['reason'] = 'no frame compared'
    else:
        out['verdict'] = 'FAIL' if allr['mismatch'] else 'PASS'
    return out


# ---- aggregate ----------------------------------------------------------------------------------------------------------------------
def aggregate(results):
    """results: list of per-run/arm dicts with 'arm' and 'verdict'. FAIL-first; INVALID is never dropped; PASS needs >= K_MIN evaluated ON runs."""
    by = {}
    for r in results:
        by.setdefault(r['arm'], []).append(r['verdict'])
    counts = {arm: {v: vs.count(v) for v in set(vs)} for arm, vs in by.items()}
    allv = [v for vs in by.values() for v in vs]
    if 'TOOL_ERROR' in allv:
        v = 'TOOL_ERROR'
    elif 'FAIL' in allv:
        v = 'FAIL'
    elif 'INVALID' in allv:
        v = 'INVALID'
    else:
        evaluated_on = sum(1 for x in by.get('on', []) if x in ('PASS', 'PASS_DECLARED'))
        if evaluated_on == 0:
            v = 'BLOCKED'
        elif evaluated_on < K_MIN:
            v = 'INSUFFICIENT'
        elif any(x == 'BLOCKED' for x in allv):
            v = 'BLOCKED' if all(x == 'BLOCKED' for x in by.get('on', [])) else 'PASS_DECLARED' if any(x == 'PASS_DECLARED' for x in allv) else 'PASS'
        else:
            v = 'PASS_DECLARED' if any(x == 'PASS_DECLARED' for x in allv) else 'PASS'
    return {'verdict': v, 'counts': counts, 'k_evaluated_on': sum(1 for x in by.get('on', []) if x in ('PASS', 'PASS_DECLARED')), 'K_MIN': K_MIN}


def run_suite(spec):
    results = []
    for r in spec['runs']:
        fn = {'on': arm_on, 'off': arm_off, 'null': arm_null}[r['arm']]
        res = fn(r['original'], r['engine'])
        res['name'] = r['name']
        res['scenario'] = r.get('scenario')
        results.append(res)
    return {'schema': SCHEMA, 'constants': CONSTANTS, 'results': results, 'aggregate': aggregate(results)}


def summarize(res):
    lines = []
    for r in res['results']:
        extra = ''
        if r.get('checks'):
            # off arm: the whole-run difference after F is the expected reaction, not a mismatch (verdict rules in arm_off)
            bad = [c['name'] + '=%d' % c['mismatch'] for c in r['checks'] if c.get('mismatch') and not (r['arm'] == 'off' and c['name'].endswith('WholeRun'))]
            decl = [c['name'] + '=%d' % c['declaredU2'] for c in r['checks'] if c.get('declaredU2')]
            extra = ' mismatches[%s] declaredU2[%s]' % (','.join(bad), ','.join(decl))
        lines.append('%-5s %-18s %-14s%s%s' % (r['arm'], r['name'], r['verdict'], extra, (' reason: ' + str(r['reason'])) if r.get('reason') else ''))
    a = res['aggregate']
    lines.append('AGGREGATE %s (k_evaluated_on=%d, K_MIN=%d) %s' % (a['verdict'], a['k_evaluated_on'], a['K_MIN'], json.dumps(a['counts'], sort_keys=True)))
    return '\n'.join(lines)


def write_new(path, obj):
    p = pathlib.Path(path)
    if p.exists():
        raise SystemExit('refusing to overwrite ' + str(p))
    p.parent.mkdir(parents=True, exist_ok=True)

    def fix(x):   # JSON has no inf/NaN
        if isinstance(x, float) and not math.isfinite(x):
            return str(x)
        if isinstance(x, bytes):
            return x.hex()
        if isinstance(x, dict):
            return {k: fix(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [fix(v) for v in x]
        return x
    p.write_text(json.dumps(fix(obj), indent=1, sort_keys=False) + '\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('on', 'off', 'null'):
        p = sub.add_parser(name)
        p.add_argument('--original', required=True)
        p.add_argument('--engine', required=True)
        p.add_argument('--out', required=True)
    p = sub.add_parser('suite')
    p.add_argument('--spec', required=True)
    p.add_argument('--out', required=True)
    sub.add_parser('constants')
    a = ap.parse_args(argv)
    if a.cmd == 'constants':
        print(json.dumps(CONSTANTS, indent=1))
        return 0
    try:
        if a.cmd == 'suite':
            res = run_suite(json.loads(pathlib.Path(a.spec).read_text()))
            write_new(a.out, res)
            print(summarize(res))
            return EXIT[res['aggregate']['verdict']]
        res = {'on': arm_on, 'off': arm_off, 'null': arm_null}[a.cmd](a.original, a.engine)
        write_new(a.out, res)
        print(a.cmd, res['verdict'], res.get('reason', ''))
        return EXIT[res['verdict']]
    except SystemExit:
        raise
    except Exception:       # a crash of this tool is a TOOL_ERROR (exit 4), never a FAIL and never a PASS
        traceback.print_exc()
        return EXIT['TOOL_ERROR']


if __name__ == '__main__':
    sys.exit(main())
