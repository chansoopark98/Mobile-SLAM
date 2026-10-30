#!/usr/bin/env python3
"""us025-lc-e2e-gate.v2 (D-V2-2): whole-run comparison of the engine ON replay against an ORIGINAL lc=1 run in which the original never succeeded in loop close.

  python3 -B -I compare_full_us025.py full --original <lc=1 run dir> --engine <replay dir, loop close ON> --out <new json>
  python3 -B -I compare_full_us025.py constants

Why this file exists. compare_us025.py `on` answers BLOCKED for such a run ("nothing to compare", registration v1 section 10, "the engine ON side of a run without an
original success is not compared"). v1 therefore could not see an engine that makes an attempt, a decision or a success the original did not make, or whose trace
differs, in the 22 of 36 runs that never triggered. This module compares those runs over the WHOLE run. It does not modify compare_us025.py or compare_us026.py (both are
pinned shared tools); it imports compare_us025.py and calls its functions as they are.

What is compared (the same quantities, the same equality rules, the same declared U2 allowance as the F-limited ON arm of compare_us025.py; the only difference is that
there is no first success F, so the window is the whole run):
  (a) attempt frame set and per attempt slot count / live / free / newest handle        (compare_us025.judge_attempts, literally)
  (b) ring state and the candidate id after every call   (candidate id: judge_attempts; ring state: ring_state_check, which also counts a U2 ring word CARRIED forward)
  (c) the stage at which every attempt ended (plain original run: the four non-applied outcomes are one class)   (idem)
  (f) the agate global descriptor of every slot at every attempt                        (idem)
  (g) NEW: the engine makes NO successful loop close anywhere (the original made none): engineSuccessCalls, tolerance 0.
  (e) the trace and world point output of EVERY frame bit for bit: compare_us026.compare, pose components 0 ulp, memprobe integers (compare_us025.trace_check
      with no frame limits).
Not compared: C, P_old and the PnP pose (no success call exists), the match and correspondence counts (the original is a plain, uninstrumented run).
Tolerances are those of compare_us025.py (integers and ids exact, trace pose 0 ulp); nothing here chooses a tolerance.

The mechanism: compare_us025.judge_attempts needs a "first success"; it is given a VIRTUAL one at a frame beyond the run (frame 2**30). Its two success-specific checks
(successCall, triggerFrameSet) are removed from the output and replaced by (g). tests: test_compare_full_us025.py (synthetic) and the mutation / null evidence of the registration.

Verdicts: PASS, PASS_DECLARED (pass except declared U2 picks), FAIL, INVALID (the ENGINE side or the loaded inputs cannot support a whole-run comparison), ORIGINAL_UNUSABLE (exit 14: the original ended with an error status or re-staged frames; the gate treats it as INFRA of that run), BLOCKED (the original DID succeed: this
module does not apply, the F-limited arm decides), TOOL_ERROR (this script crashed; never FAIL). Exit codes as compare_us025.py: 0 / 0 / 10 / 11 / 13, TOOL_ERROR 4, ORIGINAL_UNUSABLE 14.
"""
import argparse
import importlib.util
import json
import pathlib
import sys
import traceback

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
_spec = importlib.util.spec_from_file_location('compare_us025', REPO / 'slam_refact' / 'tools' / 'harness' / 'us025' / 'compare_us025.py')
C = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(C)

SCHEMA = 'slam-refact-us025-lc-e2e-v2-compare-full-v1'
MIN_TRACE_FRAMES = 30        # same floor as the NULL arm (judge.mjs MIN_TRACE_FRAMES): a whole-run comparison of fewer frames is not evidence
VIRTUAL_F = 2 ** 30          # a frame beyond every real run: judge_attempts then covers the whole run
EXIT = {**C.EXIT, 'ORIGINAL_UNUSABLE': 14}   # v2: the ORIGINAL run is unusable for a whole-run comparison (error status, re-staged frames): INFRA of that run (pre-review C5)
CONSTANTS = {'schema': SCHEMA, 'MIN_TRACE_FRAMES': MIN_TRACE_FRAMES, 'VIRTUAL_F': VIRTUAL_F, 'EXIT': EXIT, 'imports': 'compare_us025.py (sha256 pinned in the manifest)',
             'compare_us025': {k: C.CONSTANTS[k] for k in ('PNP_ULP', 'MIN_SLOTS', 'MATCHES_REQUIRED', 'CORR_REQUIRED', 'WASM_SHA256')}}
ATTEMPT_CHECKS_DROPPED = ('successCall', 'triggerFrameSet')
RING_WORDS = (2, 3, 4)       # ringPost = [last newest index, last newest id, ring0, ring1, ring2, cursor]


def ring_state_check(orig, eng):
    """Replacement of compare_us025's `ringState` check for a whole-run window. compare_us025 declares a ring mismatch only at the attempt whose candidate pick is a
    declared-U2 pick (the original's candidate list tail is uninitialized heap, so the id it writes into the ring is not reproducible). That ring word stays in the ring
    until it is overwritten, so over a whole run the SAME unreproducible pair (original word, engine word) is seen again at every later attempt. This check counts such a
    pair as CARRIED U2 (reported under declaredU2, never as match). Exemption (pre-review C2): at an attempt whose pick is a declared-U2 pick, the only ring differences
    declared are (1) the single ring word the pick is written to, ring[(cursor + 2) % 3], and (2) the cursor (the original advances it on the unreproducible candidate, the
    engine, which finds none, does not: seen in the v1 data); a difference in ANY other ring word, in the newest index or in the newest id is a MISMATCH (compare_us025
    itself would declare the whole ring there). The (word, original, engine) pair of the pick word is remembered; at a later attempt whose pick is not declared the same
    pair, still sitting unchanged in that word, is carried and declared. A different pair, or a differing cursor / newest index / newest id there, is a MISMATCH. The
    carry rule was added after the v1 data showed the propagation (registration v2 section 2); the C2 narrowing after the pre-review."""
    ea = eng['attempts']
    common = sorted((a for a in orig['attempts'] if a['frame'] in ea), key=lambda a: a['frame'])
    tainted, mm, decl = set(), [], []
    for a in common:
        e = ea[a['frame']]
        if list(e['ringPost']) == list(a['ringPost']):
            continue
        f = a['frame']
        e_pick = e['ringPost'][2:5][(e['ringPost'][5] + 2) % 3]
        declared_here = a['nSlots'] >= C.MIN_SLOTS and e_pick != a['pick'] and a['pick'] != 0 and a['pick'] not in a['liveIds']
        diff = [k for k, (x, y) in enumerate(zip(a['ringPost'], e['ringPost'])) if x != y]
        rec = {'frame': f, 'original': a['ringPost'], 'engine': e['ringPost']}
        pick_word = RING_WORDS[0] + (a['ringPost'][5] + 2) % 3     # the one ring word the unreproducible pick is written to: ring[(cursor + 2) % 3]
        carried = lambda k: k in RING_WORDS and (k, a['ringPost'][k], e['ringPost'][k]) in tainted
        if declared_here and all(k in (pick_word, 5) or carried(k) for k in diff):
            decl.append(rec)
            tainted.update((k, a['ringPost'][k], e['ringPost'][k]) for k in diff if k == pick_word)
        elif not declared_here and diff and all(carried(k) for k in diff):
            decl.append({**rec, 'carriedU2': True})
        else:
            mm.append(rec)
    return C.check('ringState', compared=len(common), match=len(common) - len(mm) - len(decl), mismatches=mm, declared=decl,
                   note='[last newest index, id, ring0..2, cursor] after the call; declared = declared-U2 pick at that attempt or the same U2 ring-word pair carried forward')


def virtual_first():
    """The placeholder 'first success' that moves the window of judge_attempts to the whole run. Only its frame is ever used for windowing; the success-specific fields
    are read by judge_attempts only when the engine has an attempt AT the virtual frame, which no replay has (frames are input indices)."""
    return {'frame': VIRTUAL_F, 'seq': -1, 'outLcOk': 1, 'pick': 0, 'newest': [0, 0], 'outLcMat': b'', 'newestExtrinsic': b'', 'hooks': {}}


def judge_full_attempts(orig, eng):
    """Whole-run checks (a)(b)(c)(f)(g). `orig` from compare_us025.load_original_lc with no success, `eng` from load_engine_lc. Pure over the loaded data."""
    if orig['firstSuccess'] is not None:
        raise C.Invalid('the original has a loop close success: use the F-limited ON arm')
    o2 = dict(orig)
    o2['firstSuccess'] = virtual_first()
    checks, detail = C.judge_attempts(o2, eng)
    checks = [ring_state_check(orig, eng) if c['name'] == 'ringState' else c for c in checks if c['name'] not in ATTEMPT_CHECKS_DROPPED]
    applied = sorted(f for f, r in eng['attempts'].items() if r['applied'])
    mm = [{'frame': f, 'original': 'no loop close success (no trigger)', 'engine': 'applied'} for f in applied]
    checks.append(C.check('engineSuccessCalls', compared=len(eng['attempts']), match=len(eng['attempts']) - len(applied), mismatches=mm,
                          note='the original run never succeeded; the engine must not either (tolerance 0, no declared allowance)'))
    detail = {'attemptsCompared': detail['attemptsCompared'], 'attemptsInWindow': detail['attemptsInWindow'], 'engineAttempts': len(eng['attempts']),
              'engineSuccessFrames': applied, 'window': 'whole run (no first success F)'}
    return checks, detail


def verdict_full(checks):
    v = C.verdict_of(checks)
    return v


def arm_full(original, engine):
    out = {'arm': 'full', 'original': str(original), 'engine': str(engine),
           'compareRange': 'the WHOLE run: the original lc=1 run never succeeded in loop close, so there is no F (and no N2 truncation); the engine ON replay must agree everywhere'}
    try:
        orig = C.load_original_lc(original, 1)
        eng = C.load_engine_lc(engine)
        o = C.U26.load_original(original)
        e = C.U26.load_engine(engine)
    except C.Invalid as ex:
        out.update({'verdict': 'INVALID', 'reason': str(ex)})
        return out
    out['originalRun'] = {'success': orig['firstSuccess'] is not None, 'attempts': len(orig['attempts']), 'hookMode': orig['hooked'], 'restaged': orig.get('restaged'),
                          'loadAvgAtStart': orig['meta'].get('loadAvgAtStart'), 'status': orig['meta'].get('status')}
    out['engineAttemptsTotal'] = len(eng['attempts'])
    if orig['firstSuccess'] is not None:
        out.update({'verdict': 'BLOCKED', 'reason': 'the original run succeeded in loop close: this whole-run comparison does not apply (the F-limited ON arm decides)'})
        return out
    if not eng['attempts']:
        out.update({'verdict': 'INVALID', 'reason': 'engine made no LoopClose call (loop close off in this replay?)'})
        return out
    status = orig['meta'].get('status')
    if status != 'COMPLETED':
        out.update({'verdict': 'ORIGINAL_UNUSABLE', 'reason': 'original lc=1 run status %s without a loop close success: the original state after an error is not defined for a whole-run comparison' % status})
        return out
    if orig.get('restaged'):
        out.update({'verdict': 'ORIGINAL_UNUSABLE', 'reason': 'frames staged twice in the original (exception recovery): %s' % orig['restaged']})
        return out
    try:
        checks, detail = judge_full_attempts(orig, eng)
    except C.Invalid as ex:
        out.update({'verdict': 'INVALID', 'reason': str(ex)})
        return out
    tr, _ = C.trace_check(o, e, None, None, 'traceAndWorldPointsWholeRun')
    checks.append(tr)
    out.update({'checks': checks, 'detail': detail, 'verdict': verdict_full(checks)})
    if tr['compared'] < MIN_TRACE_FRAMES:
        out['verdict'] = 'INVALID'
        out['reason'] = 'whole-run trace compared %d frames < %d' % (tr['compared'], MIN_TRACE_FRAMES)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('full')
    p.add_argument('--original', required=True)
    p.add_argument('--engine', required=True)
    p.add_argument('--out', required=True)
    sub.add_parser('constants')
    a = ap.parse_args(argv)
    if a.cmd == 'constants':
        print(json.dumps(CONSTANTS, indent=1))
        return 0
    try:
        res = arm_full(a.original, a.engine)
        C.write_new(a.out, res)
        print('full', res['verdict'], res.get('reason', ''))
        return EXIT[res['verdict']]
    except SystemExit:
        raise
    except Exception:       # a crash of this tool is a TOOL_ERROR (exit 4), never a FAIL and never a PASS
        traceback.print_exc()
        return EXIT['TOOL_ERROR']


if __name__ == '__main__':
    sys.exit(main())
