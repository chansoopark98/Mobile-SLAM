"""Unit tests of lc_positional_probe.py on synthetic attempts (no gate output is read).

  python3 -m unittest slam_refact/tools/harness/us025/diag/test_lc_positional_probe.py -v
"""
import importlib.util
import pathlib
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('lc_positional_probe', HERE / 'lc_positional_probe.py')
P = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(P)

ZERO = bytes(32)


def desc(bit_count=0):
    """A 32-byte descriptor with `bit_count` low bits set (Hamming distance to ZERO)."""
    return (int('1' * bit_count, 2) if bit_count else 0).to_bytes(32, 'little')


def mk(frame, ids, newest_idx, pick_id=0, ring_pre=None, descs=None):
    """A side record. ids: handle id per slot (0 = free); the newest keyframe is ids[newest_idx]."""
    n = len(ids)
    nid = ids[newest_idx]
    post = [newest_idx, nid, pick_id, 0, 0, 1 if pick_id else 0]
    return {'frame': frame, 'n_slots': n, 'ids': list(ids), 'desc': list(descs) if descs else [ZERO] * n, 'newest': [newest_idx, nid],
            'ring_pre': list(ring_pre) if ring_pre else [0, 0, 0, 0, 0, 0], 'ring_post': post}


IDS = [11, 12, 13, 14, 15, 16]          # six live slots, newest at slot 4 (id 15) -> positions 0..5, position 4 skipped


class Hamming(unittest.TestCase):
    def test_hamming(self):
        self.assertEqual(P.hamming(ZERO, ZERO), 0)
        self.assertEqual(P.hamming(ZERO, desc(5)), 5)
        self.assertEqual(P.hamming(desc(3), desc(5)), 2)


class AnalyseSide(unittest.TestCase):
    def test_required_s2_is_the_skipped_prefix(self):
        a = P.analyse_side(mk(1, IDS, 4, pick_id=14))                    # id 14 sits at position 3
        self.assertEqual(a['pick_pos'], 3)
        self.assertEqual(a['nos2_pos'], 0)
        self.assertEqual(a['required'], [0, 1, 2])

    def test_newest_index_is_applied_to_the_position_not_the_slot(self):
        ids = [11, 0, 13, 14, 15]                                          # slot 1 free: positions 0..3 = slots 0,2,3,4; newest.index 4 is beyond the live list
        a = P.analyse_side(mk(1, ids, 4))
        self.assertEqual(a['live_slots'], [0, 2, 3, 4])
        self.assertEqual(a['live_ids'][3], 15)
        self.assertEqual(a['eligible'], [0, 1, 2, 3])                      # the newest keyframe (position 3) is NOT skipped: the U2 index mismatch

    def test_pick_equals_newest_through_the_index_mismatch(self):
        ids = [11, 0, 13, 15]                                              # newest id 15 at slot 3 = position 2; index 3 skips position 3, which does not exist
        a = P.analyse_side(mk(1, ids, 3, pick_id=15))
        self.assertEqual(a['pick_pos'], 2)
        self.assertEqual(a['required'], [0, 1])

    def test_no_candidate_requires_every_eligible_position(self):
        a = P.analyse_side(mk(1, IDS, 4))
        self.assertEqual(a['pick_id'], 0)
        self.assertEqual(a['required'], [0, 1, 2, 3, 5])

    def test_threshold_is_strict(self):
        descs = [desc(80), desc(79), ZERO, ZERO, ZERO, ZERO]
        a = P.analyse_side(mk(1, IDS, 4, descs=descs))                     # distances to the newest (ZERO): 80, 79, 0, 0, 0, 0
        self.assertEqual(a['dist'], [80, 79, 0, 0, 0, 0])
        self.assertNotIn(0, a['eligible'])                                 # distance == threshold is not < threshold
        self.assertIn(1, a['eligible'])
        self.assertEqual(a['nos2_pos'], 2)                                 # first strictly smaller wins: 79 at position 1 is replaced by 0 at position 2
        self.assertEqual(P.analyse_side(mk(1, IDS, 4, descs=descs), threshold=81.0)['eligible'][0], 0)

    def test_ring_is_cleared_when_newest_changed_and_applies_otherwise(self):
        changed = P.analyse_side(mk(1, IDS, 4, ring_pre=[3, 14, 11, 0, 0, 1]))
        self.assertTrue(changed['ring_reset'])
        self.assertEqual(changed['ring_eff'], [0, 0, 0])
        self.assertEqual(changed['nos2_pos'], 0)
        same = P.analyse_side(mk(1, IDS, 4, ring_pre=[4, 15, 11, 0, 0, 1]))
        self.assertFalse(same['ring_reset'])
        self.assertEqual(same['ring_eff'], [11, 0, 0])
        self.assertEqual(same['nos2_pos'], 1)                              # id 11 at position 0 is excluded by the ring
        self.assertEqual(same['eligible'][0], 1)

    def test_nonlive_ring_word_is_flagged(self):
        a = P.analyse_side(mk(1, IDS, 4, ring_pre=[3, 14, 1065353216, 0, 0, 1]))
        self.assertEqual(a['ring_nonlive'], [1065353216])

    def test_distance_ties_keep_the_first_position(self):
        descs = [desc(2), desc(2), desc(1), desc(1), ZERO, desc(1)]
        a = P.analyse_side(mk(1, IDS, 4, pick_id=14, descs=descs))        # distances 2,2,1,1,-,1: min 1 first at position 2; pick id 14 is position 3 (a tie)
        self.assertEqual(a['nos2_pos'], 2)
        self.assertEqual(a['required'], [2])                                # positions 0,1 are worse (2 > 1) and need no exclusion; position 2 ties and is earlier

    def test_ineligible_pick_is_an_anomaly(self):
        a = P.analyse_side(mk(1, IDS, 4, pick_id=15))                      # the newest keyframe itself is skipped by index
        self.assertIsNone(a['required'])
        self.assertIn('not eligible', a['anomaly'])


class CompareAttempt(unittest.TestCase):
    def test_first_position_match(self):
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=11), mk(1, IDS, 4, pick_id=11))
        self.assertEqual(r['class'], 'MATCH_FIRST')

    def test_skipped_match(self):
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=14), mk(1, IDS, 4, pick_id=14))
        self.assertEqual(r['class'], 'MATCH_SKIPPED')
        self.assertEqual(r['skipped_o'], 3)

    def test_no_pick_on_both_sides(self):
        self.assertEqual(P.compare_attempt(mk(1, IDS, 4), mk(1, IDS, 4))['class'], 'MATCH_NO_PICK')

    def test_original_skipped_prefix_engine_did_not(self):
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=14), mk(1, IDS, 4, pick_id=11))
        self.assertEqual(r['class'], 'MISMATCH_SKIPPED')
        self.assertEqual((r['pick_pos_o'], r['pick_pos_e']), (3, 0))
        self.assertEqual((r['required_s2_size_o'], r['required_s2_size_e']), (3, 0))
        self.assertEqual((r['witness_pos'], r['witness_s2_member_of']), (0, 'original'))

    def test_engine_skipped_prefix_original_did_not(self):
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=11), mk(1, IDS, 4, pick_id=13))
        self.assertEqual(r['class'], 'MISMATCH_FIRST')
        self.assertEqual((r['witness_pos'], r['witness_s2_member_of']), (0, 'engine'))

    def test_original_no_candidate_engine_picks(self):
        r = P.compare_attempt(mk(1, IDS, 4), mk(1, IDS, 4, pick_id=12))
        self.assertTrue(r['class'].startswith('MISMATCH'))
        self.assertEqual((r['witness_pos'], r['witness_s2_member_of']), (1, 'original'))

    def test_declared_u2_when_original_pick_is_not_live(self):
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=999), mk(1, IDS, 4, pick_id=11))
        self.assertEqual(r['class'], 'DECLARED_U2')
        self.assertIsNone(r['pick_pos_o'])
        self.assertIsNone(r['witness_pos'])

    def test_same_position_different_handle_id_is_not_an_s2_difference(self):
        eng_ids = [21, 22, 23, 24, 25, 26]
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=13), mk(1, eng_ids, 4, pick_id=23))
        self.assertEqual(r['class'], 'POS_EQUAL_ID_DIFF')
        self.assertFalse(r['ids_equal'])
        self.assertEqual(r['ids_diff_slots'], [0, 1, 2, 3, 4, 5])
        self.assertIsNone(r['witness_pos'])

    def test_precondition_difference_blocks_the_claim(self):
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=14), mk(1, [11, 12, 13, 0, 15, 16], 4, pick_id=11))
        self.assertEqual(r['class'], 'PRECONDITION_DIFF')
        self.assertIn('live_slot_pattern', r['pre_diff'])
        self.assertIsNone(r['witness_pos'])
        d = [ZERO] * 6
        d[2] = desc(1)
        r = P.compare_attempt(mk(1, IDS, 4, pick_id=11), mk(1, IDS, 4, pick_id=11, descs=d))
        self.assertEqual(r['pre_diff'], ['live_descriptors'])

    def test_too_few_slots_and_after_f(self):
        small = [11, 12, 13, 14]
        self.assertEqual(P.compare_attempt(mk(1, small, 3), mk(1, small, 3))['class'], 'TOO_FEW_SLOTS')
        self.assertEqual(P.compare_attempt(mk(1, IDS, 4), mk(1, IDS, 4), after_f=True)['class'], 'AFTER_F')


class BuildAndSummarize(unittest.TestCase):
    def test_rows_summary_and_first_mismatch(self):
        o = {10: mk(10, IDS, 4, pick_id=11), 20: mk(20, IDS, 4, pick_id=14), 30: mk(30, IDS, 4, pick_id=14), 40: mk(40, IDS, 4)}
        e = {10: mk(10, IDS, 4, pick_id=11), 20: mk(20, IDS, 4, pick_id=14), 30: mk(30, IDS, 4, pick_id=11), 50: mk(50, IDS, 4)}
        rows = P.build_rows(o, e)
        self.assertEqual([r['class'] for r in rows], ['MATCH_FIRST', 'MATCH_SKIPPED', 'MISMATCH_SKIPPED', 'ATTEMPT_SET_DIFF', 'ATTEMPT_SET_DIFF'])
        s = P.summarize(rows)
        self.assertEqual(s['firstPickIdMismatchFrame'], 30)
        self.assertEqual(s['firstPickPositionMismatchFrame'], 30)
        self.assertEqual(s['originalSkippedLivePick'], {'attempts': 2, 'engineMatches': 1, 'engineMismatches': 1})
        self.assertEqual(s['originalFirstPosition'], 1)
        self.assertEqual(s['firstMismatch']['required_s2_o'], [0, 1, 2])
        self.assertEqual(s['witnesses'], [{'frame': 30, 'position': 0, 's2ContainsIt': 'original'}])
        self.assertFalse(s['firstMismatch']['pickBeyondLive'])

    def test_frames_after_first_success_are_not_compared(self):
        o = {10: mk(10, IDS, 4, pick_id=14), 20: mk(20, IDS, 4, pick_id=14)}
        e = {10: mk(10, IDS, 4, pick_id=14), 20: mk(20, IDS, 4, pick_id=11), 30: mk(30, IDS, 4)}
        rows = P.build_rows(o, e, first_success_frame=10)
        self.assertEqual([r['class'] for r in rows], ['MATCH_SKIPPED', 'AFTER_F', 'AFTER_F'])
        self.assertIsNone(P.summarize(rows)['firstPickIdMismatchFrame'])


class Records(unittest.TestCase):
    def test_side_from_original_and_engine(self):
        a = {'seq': 7, 'frame': 5, 'nSlots': 2, 'desc': ZERO + desc(1), 'newest': [1, 22], 'ringPre': [0] * 6, 'ringPost': [1, 22, 0, 0, 0, 0]}
        ev = {'slots': [{'s': 1, 'id': 22}, {'s': 0, 'id': 21}]}
        s = P.side_from_original(a, ev)
        self.assertEqual(s['ids'], [21, 22])
        self.assertEqual(s['desc'][1], desc(1))
        with self.assertRaises(P.ProbeError):
            P.side_from_original(dict(a, nSlots=3, desc=ZERO * 3), ev)
        r = {'frame': 5, 'nSlots': 2, 'slots': [21, 22], 'desc': [ZERO.hex(), desc(1).hex()], 'newest': [1, 22], 'ringPre': [0] * 6, 'ringPost': [1, 22, 0, 0, 0, 0],
             'cand': [0, 0], 'candDist': -1, 'outcome': 0}
        self.assertEqual(P.side_from_engine(r)['desc'][1], desc(1))
        with self.assertRaises(P.ProbeError):
            P.side_from_engine(dict(r, slots=[21]))

    def test_tsv_has_one_header_and_one_line_per_row(self):
        rows = P.build_rows({1: mk(1, IDS, 4, pick_id=14)}, {1: mk(1, IDS, 4, pick_id=11)})
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / 'x.tsv'
            P.write_tsv(p, rows)
            lines = p.read_text().splitlines()
        self.assertEqual(len(lines), 2)
        header = lines[0].split('\t')
        self.assertEqual(header, P.TSV_COLS)
        self.assertEqual(len(lines[1].split('\t')), len(header))
        self.assertEqual(lines[1].split('\t')[header.index('class')], 'MISMATCH_SKIPPED')

    def test_usage_error_exits_2(self):
        with self.assertRaises(SystemExit) as cm:
            P.main(['--out-dir', '/nonexistent-never-written'])
        self.assertEqual(cm.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
