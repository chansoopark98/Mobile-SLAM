#!/usr/bin/env python3
"""Synthetic unit tests of compare_full_us025.py (D-V2-2): python3 -B -I -m unittest test_compare_full_us025 -v   (run from this directory; gate_lc.test.mjs runs it too).
No original SDK, no replay, no files: judge_full_attempts / ring_state_check over hand-made loaded data. The data tests (null on the v1 runs, mutants) are the registration's
dev evidence, not unit tests."""
import copy
import importlib.util
import pathlib
import sys
import unittest

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('compare_full_us025', HERE / 'compare_full_us025.py')
F = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(F)
C = F.C


def make(frames_slots, pick_of=None):
    """frames_slots: [(frame, nSlots)]. Returns (orig, eng) loaded-data dicts that agree everywhere. pick_of(frame) -> original candidate id (0 = none)."""
    pick_of = pick_of or (lambda f: 0)
    oa, ea = [], {}
    for i, (f, n) in enumerate(frames_slots):
        ids = [1000 + k for k in range(n)]
        pick = pick_of(f) if n >= C.MIN_SLOTS else 0
        ring = [n - 1, ids[-1], pick, 0, 0, 1 if pick else 0]
        oa.append({'seq': i, 'frame': f, 'nSlots': n, 'nLive': n, 'free': 0, 'newest': [n - 1, ids[-1]], 'ringPre': [0] * 6, 'ringPost': list(ring), 'pick': pick,
                   'liveIds': sorted(ids), 'outLcOk': 0, 'desc': bytes(32 * n), 'outLcMat': b'', 'newestExtrinsic': b'', 'hooks': {}})
        outcome = 0 if n < C.MIN_SLOTS else (1 if pick == 0 else 2)
        ea[f] = {'frame': f, 'nSlots': n, 'nLive': n, 'newest': [n - 1, ids[-1]], 'slots': list(ids), 'desc': ['00' * 32] * n, 'ringPre': [0] * 6, 'ringPost': list(ring),
                 'outcome': outcome, 'applied': False, 'outLcOk': 0, 'C': '', 'poseAfter': '', 'poseBefore': '', 'nMatches': 0, 'nCorr': 0}
    orig = {'attempts': oa, 'hooked': False, 'firstSuccess': None, 'restaged': {}, 'meta': {}}
    eng = {'attempts': ea, 'meta': {}}
    return orig, eng


FRAMES = [(10, 2), (20, 4), (30, 6), (40, 7), (50, 8)]


def by_name(checks):
    return {c['name']: c for c in checks}


def verdict(orig, eng):
    checks, _ = F.judge_full_attempts(orig, eng)
    return F.verdict_full(checks), by_name(checks)


class WholeRunAttempts(unittest.TestCase):
    def test_identical_run_passes_over_the_whole_window(self):
        orig, eng = make(FRAMES)
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'PASS')
        self.assertNotIn('successCall', c)
        self.assertNotIn('triggerFrameSet', c)
        self.assertEqual(c['attemptFrameSet']['compared'], 5)
        self.assertEqual(c['engineSuccessCalls']['compared'], 5)
        self.assertEqual(c['engineSuccessCalls']['mismatch'], 0)

    def test_an_original_with_a_success_is_refused(self):
        orig, eng = make(FRAMES)
        orig['firstSuccess'] = orig['attempts'][-1]
        with self.assertRaises(C.Invalid):
            F.judge_full_attempts(orig, eng)

    def test_the_last_attempt_is_inside_the_window(self):
        orig, eng = make(FRAMES)
        eng['attempts'][50]['nLive'] -= 1          # last attempt of the run, far beyond any early window
        self.assertEqual(verdict(orig, eng)[0], 'FAIL')

    def test_engine_success_where_the_original_made_none_fails(self):
        orig, eng = make(FRAMES)
        e = eng['attempts'][40]
        e['applied'], e['outcome'], e['outLcOk'] = True, 6, 1
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'FAIL')
        self.assertEqual(c['engineSuccessCalls']['mismatch'], 1)
        self.assertEqual(c['engineSuccessCalls']['first']['frame'], 40)
        self.assertGreaterEqual(c['thresholdStage']['mismatch'], 1)

    def test_engine_success_alone_fails_even_when_the_other_quantities_agree(self):
        orig, eng = make(FRAMES)
        eng['attempts'][50]['applied'] = True      # only the flag: no other quantity changed
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'FAIL')
        self.assertEqual(c['engineSuccessCalls']['mismatch'], 1)

    def test_attempt_frame_set_slot_and_newest_differences_fail(self):
        for mutate in (lambda o, e: e['attempts'].pop(30),
                       lambda o, e: e['attempts'].__setitem__(35, {**copy.deepcopy(e['attempts'][30]), 'frame': 35}),
                       lambda o, e: e['attempts'][20].__setitem__('nSlots', 5),
                       lambda o, e: e['attempts'][20].__setitem__('newest', [3, 7]),
                       lambda o, e: e['attempts'][30]['desc'].__setitem__(0, 'ff' + '00' * 31)):
            orig, eng = make(FRAMES)
            mutate(orig, eng)
            self.assertEqual(verdict(orig, eng)[0], 'FAIL')

    def test_ring_cursor_and_ring_word_differences_fail_without_a_declared_pick(self):
        orig, eng = make(FRAMES)
        eng['attempts'][30]['ringPost'][5] = 2
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'FAIL')
        self.assertEqual(c['ringState']['mismatch'], 1)
        orig, eng = make(FRAMES)
        eng['attempts'][40]['ringPost'][3] = 12345   # a ring word differs where no U2 pick was declared
        self.assertEqual(verdict(orig, eng)[0], 'FAIL')


class CarriedU2(unittest.TestCase):
    """The original's candidate list tail is uninitialized heap: a pick that is not a live keyframe id (declared U2) writes an unreproducible id into the ring, and the
    same pair stays in the ring until overwritten."""

    GARBAGE_O, GARBAGE_E = 7777777, 8888888

    def run_with_u2(self):
        # the original writes a non-live id into ring word 0 at frame 30 (a declared-U2 pick); at frames 40 and 50 the cursor has moved on (the pick is word 1 = none), the
        # old word is still in the ring on both sides: the same unreproducible pair, carried
        orig, eng = make(FRAMES, pick_of=lambda f: self.GARBAGE_O if f == 30 else 0)
        eng['attempts'][30]['ringPost'][2] = self.GARBAGE_E
        for a in orig['attempts']:
            if a['frame'] in (40, 50):
                a['ringPost'][2], a['ringPost'][5] = self.GARBAGE_O, 2
                eng['attempts'][a['frame']]['ringPost'][2], eng['attempts'][a['frame']]['ringPost'][5] = self.GARBAGE_E, 2
        return orig, eng

    def test_the_declared_pick_and_the_carried_ring_word_are_declared_not_failed(self):
        orig, eng = self.run_with_u2()
        v, c = verdict(orig, eng)
        self.assertEqual(c['ringState']['mismatch'], 0)
        self.assertEqual(c['ringState']['declaredU2'], 3)        # frame 30 (pick) and frames 40, 50 (carried)
        self.assertEqual(v, 'PASS_DECLARED')
        self.assertEqual(c['ringState']['match'], 2)

    def test_a_different_pair_later_is_a_mismatch(self):
        orig, eng = self.run_with_u2()
        eng['attempts'][50]['ringPost'][2] = 9999999           # not the pair first seen at the declared pick
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'FAIL')
        self.assertEqual(c['ringState']['mismatch'], 1)
        self.assertEqual(c['ringState']['first']['frame'], 50)

    def test_a_carried_word_with_a_differing_cursor_or_newest_is_a_mismatch(self):
        orig, eng = self.run_with_u2()
        eng['attempts'][40]['ringPost'][5] = (eng['attempts'][40]['ringPost'][5] + 1) % 3
        self.assertEqual(verdict(orig, eng)[0], 'FAIL')
        orig, eng = self.run_with_u2()
        eng['attempts'][40]['ringPost'][1] += 1
        self.assertEqual(verdict(orig, eng)[0], 'FAIL')

    def test_a_ring_difference_before_any_declared_pick_is_never_carried(self):
        orig, eng = self.run_with_u2()
        eng['attempts'][20]['ringPost'][2] = self.GARBAGE_E     # same pair as the later one, but seen BEFORE the declared pick
        self.assertEqual(verdict(orig, eng)[0], 'FAIL')

    def test_at_a_declared_pick_only_the_pick_word_and_the_cursor_are_exempt(self):
        # the cursor is exempt (the original advances it on the unreproducible candidate, the engine does not: v1 data)
        orig, eng = self.run_with_u2()
        eng['attempts'][30]['ringPost'][5] = 2
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'PASS_DECLARED')
        self.assertEqual(c['ringState']['mismatch'], 0)
        # pre-review C2: a difference in a ring word that is NOT the pick word, or in the newest index / id, is a mismatch even at the declared attempt
        for k, delta in ((3, 4242), (4, 4242), (1, 1), (0, 1)):
            orig, eng = self.run_with_u2()
            eng['attempts'][30]['ringPost'][k] += delta
            v, c = verdict(orig, eng)
            self.assertEqual(v, 'FAIL', k)
            self.assertEqual(c['ringState']['first']['frame'], 30)

    def test_only_the_pick_word_pair_is_remembered_for_carrying(self):
        orig, eng = self.run_with_u2()
        # a second, non-pick word differing at frame 30 is a mismatch there; it must not become carried at frame 40 either
        eng['attempts'][30]['ringPost'][3] = 4242
        for f in (40, 50):
            orig['attempts'][[a['frame'] for a in orig['attempts']].index(f)]['ringPost'][3] = 0
            eng['attempts'][f]['ringPost'][3] = 4242
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'FAIL')
        self.assertGreaterEqual(c['ringState']['mismatch'], 3)

    def test_a_live_pick_difference_is_not_declared(self):
        orig, eng = make(FRAMES, pick_of=lambda f: 1001 if f == 30 else 0)   # 1001 is a live id of that attempt: a real pick
        eng['attempts'][30]['ringPost'][2] = 1002
        v, c = verdict(orig, eng)
        self.assertEqual(v, 'FAIL')
        self.assertEqual(c['candidateId']['mismatch'], 1)


class Constants(unittest.TestCase):
    def test_constants_pin_the_imported_comparator_values(self):
        self.assertEqual(F.CONSTANTS['MIN_TRACE_FRAMES'], 30)
        self.assertEqual(F.CONSTANTS['compare_us025']['PNP_ULP'], 3)
        self.assertEqual(F.CONSTANTS['compare_us025']['MIN_SLOTS'], 5)
        self.assertEqual(F.EXIT['FAIL'], 10)
        self.assertEqual(F.EXIT['BLOCKED'], 13)


if __name__ == '__main__':
    unittest.main()
