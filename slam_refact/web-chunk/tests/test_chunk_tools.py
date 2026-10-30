"""Python tests of the chunk tools (RM-4c gate rm4c-chunk-gate.v1.md): RM4C-A5(b) and the GL comparison helpers.

  python3 -I -m unittest discover -s slam_refact/web-chunk/tests -p 'test_chunk_tools.py' -v
  SLAM_REFACT_REPORT_PY=<copy of chunk_ab_report.py>   (null-run against the pre-RM-4c report script)
"""
import importlib.util
import os
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[3]
REPORT = pathlib.Path(os.environ.get('SLAM_REFACT_REPORT_PY', ROOT / 'slam_refact/tools/re/runtime/chunk_ab_report.py'))


def load_report():
    sys.path.insert(0, str(ROOT / 'slam_refact/tools/harness/eval'))
    spec = importlib.util.spec_from_file_location('chunk_ab_report_under_test', REPORT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ReportEngineCheck(unittest.TestCase):
    def test_RM4C_A5b_c72_engine_check_is_fail_closed(self):
        report = load_report()
        check = report.engine_bytes_unreplaced
        self.assertTrue(check({'matchesManifest': True, 'glueReplaced': False}))
        # RM-4a2 review m-F: a missing glueReplaced used to pass (`not get(...)`)
        self.assertFalse(check({'matchesManifest': True}))
        self.assertFalse(check({'matchesManifest': True, 'glueReplaced': None}))
        self.assertFalse(check({'matchesManifest': True, 'glueReplaced': True}))
        self.assertFalse(check({'matchesManifest': False, 'glueReplaced': False}))
        self.assertFalse(check({'matchesManifest': 1, 'glueReplaced': False}))
        self.assertFalse(check({}))
        self.assertFalse(check(None))

    def test_RM4C_A5b_report_uses_the_check(self):
        text = REPORT.read_text()
        self.assertIn('engine_bytes_unreplaced(chunk_diag.get(', text)
        self.assertNotIn("not (chunk_diag.get('engine') or {}).get('glueReplaced')", text)


class RoiGlCompare(unittest.TestCase):
    """The GL comparison apparatus itself (gate G-GL): identical planes give zero error, the registered mutations give a large one."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(ROOT / 'slam_refact/web-chunk/tools'))
        import numpy as np
        import roi_gl_compare as rg
        cls.np, cls.rg = np, rg
        cls.cam = rg.camera_gl(rg.SEED_SMOKE)

    def entry(self, plane):
        import base64
        return {'w': plane.shape[1], 'h': plane.shape[0], 'r_b64': base64.b64encode(plane.tobytes()).decode()}

    def test_identical_plane_has_no_error(self):
        np, rg = self.np, self.rg
        m = rg.rr.warps_a()[1]
        exp, _ = rg.rr.slot_expected(self.cam, m, 252, 336)
        r = rg.metrics_for(self.cam, m, self.entry(exp))
        self.assertEqual((r['A1'], r['A3n'], r['A3_mean_abs_REPORT']), (0.0, 0.0, 0.0))
        self.assertLessEqual(r['A2n'], 0.0)

    def test_registered_mutations_violate_the_tolerance(self):
        rg = self.rg
        for name in ('MF', 'MZ', 'MH'):
            violated = 0
            for m in rg.rr.warps_a():
                drawn, _ = rg.rr.slot_expected(self.cam, rg.mutated(name, m), 252, 336)
                r = rg.metrics_for(self.cam, m, self.entry(drawn))
                violated += r['A1'] > rg.TOL['A1'] or r['A2n'] is None or r['A2n'] > rg.TOL['A2n'] or r['A3n'] is None or r['A3n'] > rg.TOL['A3n']
            self.assertEqual(violated, 4, f'{name} must violate the tolerance for all four oracle warps')

    def test_a_constant_offset_is_judged_against_the_registered_A2n_tolerance(self):
        np, rg = self.np, self.rg
        m = rg.rr.warps_a()[0]
        exp, mask = rg.rr.slot_expected(self.cam, m, 252, 336)
        noisy = exp.copy().astype(int)
        # +-3 level noise everywhere: below A2n tolerance 3 + 0.25 * range
        noisy[mask > 0] += 3
        r = rg.metrics_for(self.cam, m, self.entry(np.clip(noisy, 0, 255).astype(np.uint8)))
        self.assertLessEqual(r['A2n'], 3.0)
        noisy[mask > 0] += 8
        r = rg.metrics_for(self.cam, m, self.entry(np.clip(noisy, 0, 255).astype(np.uint8)))
        self.assertGreater(r['A2n'], 3.0)

    def test_RM4D_A3n_contrast_normalised_mean_excess(self):
        """v3 GL-A3n: the quarter-texel allowance is taken off per pixel, so edge noise that A3 (raw mean) charges is not charged."""
        np, rg = self.np, self.rg
        m = rg.rr.warps_a()[1]
        exp, mask = rg.rr.slot_expected(self.cam, m, 252, 336)
        r0 = rg.metrics_for(self.cam, m, self.entry(exp))
        self.assertEqual(r0['A3n'], 0.0)
        # a +-1 level flicker of every covered pixel: below the allowance of every pixel with local range >= 4, so A3n stays small while a
        # whole-slot offset of 8 levels is charged in full
        noisy = exp.astype(int)
        noisy[mask > 0] += 1
        small = rg.metrics_for(self.cam, m, self.entry(np.clip(noisy, 0, 255).astype(np.uint8)))
        self.assertLessEqual(small['A3n'], rg.TOL['A3n'] * 20)
        noisy[mask > 0] += 7
        big = rg.metrics_for(self.cam, m, self.entry(np.clip(noisy, 0, 255).astype(np.uint8)))
        self.assertGreater(big['A3n'], rg.TOL['A3n'])
        self.assertGreater(big['A3_mean_abs_REPORT'], 7.0)

    def test_RM4D_A4_one_pixel_is_a_violation(self):
        """GL-A4 is an integer count with tolerance 0 (exact tier): a single surviving pixel is the sub-resolution mutation."""
        np, rg = self.np, self.rg
        zero = np.zeros((336, 252), dtype=np.uint8)
        self.assertEqual(rg.a4_pixels([self.entry(zero)]), 0)
        one = zero.copy()
        one[100, 100] = 1
        self.assertEqual(rg.a4_pixels([self.entry(one), self.entry(zero)]), 1)
        self.assertGreater(rg.a4_pixels([self.entry(one)]), rg.TOL['A4'])

    def test_RM4D_E5_position_delta_is_roi_sensitive(self):
        """v3 GL-E5: identical detections pass, a 2 cm shift fails, a missing detection on one side is counted, not hidden.
        v4: it needs at least E5_MIN_MATCHED_SOURCES (80) sources seen by both paths (the null-run has 88)."""
        rg = self.rg
        img = lambda x: [{'name': 'synth-flat', 'position': {'x': x, 'y': 0.17, 'z': -0.02}}]
        sources = range(10, 100)  # 90 sources
        cpu = [{'source': s, 'images': img(0.1 + 0.001 * s)} for s in sources]
        chunk = [{'source': s, 'images': img(0.1 + 0.001 * s)} for s in sources] + [{'source': 19, 'images': img(9.0)}]  # a repeated answer is ignored
        ok = rg.e5_position(chunk, cpu)
        self.assertTrue(ok['pass'])
        self.assertEqual((ok['matched_sources'], ok['max_delta_m']), (90, 0.0))
        shifted = [{'source': s, 'images': img(0.1 + 0.001 * s + 0.02)} for s in sources]
        self.assertFalse(rg.e5_position(shifted, cpu)['pass'])
        self.assertAlmostEqual(rg.e5_position(shifted, cpu)['max_delta_m'], 0.02, places=6)
        missing = rg.e5_position(chunk[:88], cpu)
        self.assertEqual((missing['matched_sources'], missing['only_cpu_REPORT']), (88, 2))
        self.assertTrue(missing['pass'])
        self.assertFalse(rg.e5_position([], cpu)['pass'])

    def test_RM4E_E5_needs_the_minimum_number_of_matched_sources(self):
        """v4 (review G-2): one matched frame within the tolerance used to be a PASS; the lower bound is 80 and the boundary is exact."""
        rg = self.rg
        self.assertEqual(rg.E5_MIN_MATCHED_SOURCES, 80)
        img = [{'name': 'synth-flat', 'position': {'x': 0.1, 'y': 0.17, 'z': -0.02}}]
        side = lambda n: [{'source': s, 'images': img} for s in range(n)]
        self.assertFalse(rg.e5_position(side(1), side(1))['pass'])
        self.assertFalse(rg.e5_position(side(79), side(79))['pass'])
        self.assertTrue(rg.e5_position(side(80), side(80))['pass'])
        # a large delta still fails with plenty of matches, and matches on one side only do not count
        self.assertFalse(rg.e5_position(side(80), [{'source': s, 'images': [{'name': 'x', 'position': {'x': 0.2, 'y': 0.17, 'z': -0.02}}]} for s in range(80)])['pass'])
        self.assertFalse(rg.e5_position(side(100), side(79))['pass'])
        self.assertEqual(rg.e5_position(side(80), side(80))['scope'], 'IMAGE_TARGET slot')

    # ---- GL-E6 (v4): the ROI wiring in integers ----
    def stage_list(self, n, per_stage=4, source='IMAGE_TARGET'):
        return [{'slot': i, 'source': source, 'name': 'x', 'warp': [float(i + 1)] * 16} for i in range(per_stage)]

    def stages(self, n, passed=lambda a: a, draws=lambda a, nd: 10 + 4 + nd, per_stage=4, source='IMAGE_TARGET'):
        out = []
        for _ in range(n):
            asked = self.stage_list(1, per_stage, source)
            out.append({'asked': asked, 'passed': passed(asked), 'draws': draws(asked, per_stage if source != 'CURVY_IMAGE_TARGET' else 0)})
        return out

    def test_RM4E_E6_a_correct_chunk_has_no_mismatch(self):
        rg = self.rg
        e = rg.e6_wiring(self.stages(120), 4, {'roiStages': 120, 'roiDraws': 480, 'roiSkipped': 0})
        self.assertEqual((e['mismatches'], e['roi_stages'], e['instrumented']), (0, 120, True))
        self.assertTrue(rg.e6_pass(e))
        # stages without ROIs: the fixed quads and the slot clears only (10 + nSlots)
        mixed = self.stages(3, passed=lambda a: [], draws=lambda a, nd: 14, per_stage=0) + self.stages(120)
        for st in mixed[:3]:
            st['asked'] = []
        e = rg.e6_wiring(mixed, 4, {'roiStages': 120, 'roiDraws': 480, 'roiSkipped': 0})
        self.assertEqual(e['mismatches'], 0)

    def test_RM4E_E6_stage_that_draws_only_the_first_roi_is_caught(self):
        """The reviewer's MV6: stage() receives the first ROI of the engine's list. Every GL-E metric of v3 passed this; E6 counts it."""
        rg = self.rg
        mv6 = self.stages(120, passed=lambda a: a[:1], draws=lambda a, nd: 10 + 4 + 1)
        e = rg.e6_wiring(mv6, 4, {'roiStages': 120, 'roiDraws': 120, 'roiSkipped': 0})
        self.assertEqual((e['list_mismatches'], e['draw_mismatches'], e['counter_mismatches']), (120, 120, 1))
        self.assertFalse(rg.e6_pass(e))

    def test_RM4E_E6_a_warp_draw_that_is_counted_but_not_made_is_caught_by_the_quad_count(self):
        """O1 keeps its own counter: only the compute context's draw count shows it."""
        rg = self.rg
        o1 = self.stages(120, draws=lambda a, nd: 10 + 4)
        e = rg.e6_wiring(o1, 4, {'roiStages': 120, 'roiDraws': 480, 'roiSkipped': 0})
        self.assertEqual((e['list_mismatches'], e['draw_mismatches'], e['counter_mismatches']), (0, 120, 0))
        self.assertFalse(rg.e6_pass(e))
        # one stage, one quad short, is enough (exact-class integer, no tolerance)
        one = self.stages(120)
        one[57]['draws'] -= 1
        e = rg.e6_wiring(one, 4, {'roiStages': 120, 'roiDraws': 480, 'roiSkipped': 0})
        self.assertEqual(e['mismatches'], 1)
        self.assertFalse(rg.e6_pass(e))
        # one list differing in one warp value
        import copy
        warp = self.stages(120)
        warp[3]['passed'] = copy.deepcopy(warp[3]['asked'])
        warp[3]['passed'][2]['warp'][5] += 1e-6
        self.assertEqual(rg.e6_wiring(warp, 4, {'roiStages': 120, 'roiDraws': 480, 'roiSkipped': 0})['list_mismatches'], 1)

    def test_RM4E_E6_curved_duplicate_and_invalid_slots_follow_the_pyramid_rules(self):
        """A curved ROI is cleared but not drawn (roiSkipped), a second ROI on a used slot and a slot outside the layout are skipped whole."""
        rg = self.rg
        curved = self.stages(100, draws=lambda a, nd: 10 + 4 + 0, source='CURVY_IMAGE_TARGET')
        self.assertEqual(rg.e6_wiring(curved, 4, {'roiStages': 100, 'roiDraws': 0, 'roiSkipped': 400})['mismatches'], 0)
        dup = [{'asked': self.stage_list(1, 2) + [{'slot': 1, 'source': 'IMAGE_TARGET', 'name': 'y', 'warp': [0.0] * 16}, {'slot': 9, 'source': 'IMAGE_TARGET', 'name': 'z', 'warp': []}],
                'passed': None, 'draws': 10 + 4 + 2}]
        dup[0]['passed'] = dup[0]['asked']
        e = rg.e6_wiring(dup, 4, {'roiStages': 1, 'roiDraws': 2, 'roiSkipped': 2})
        self.assertEqual(e['mismatches'], 0)

    def test_RM4E_E6_needs_the_probe_and_enough_stages_with_rois(self):
        rg = self.rg
        none = rg.e6_wiring([], 4, {})
        self.assertFalse(none['instrumented'])
        self.assertFalse(rg.e6_pass(none))
        blind = self.stages(120)
        blind[0]['asked'] = None  # a stage without an ask of its own (the chunk staged without asking the engine)
        e = rg.e6_wiring(blind, 4, {'roiStages': 120, 'roiDraws': 480, 'roiSkipped': 0})
        self.assertFalse(rg.e6_pass(e))
        few = rg.e6_wiring(self.stages(99), 4, {'roiStages': 99, 'roiDraws': 396, 'roiSkipped': 0})
        self.assertEqual(few['mismatches'], 0)
        self.assertFalse(rg.e6_pass(few), '99 stages with ROIs < 100')
        self.assertTrue(rg.e6_pass(rg.e6_wiring(self.stages(100), 4, {'roiStages': 100, 'roiDraws': 400, 'roiSkipped': 0})))

    def test_RM4E_A3n_tolerance_is_the_registered_one_and_the_device_equals_the_manifest(self):
        """Review G-1: the device said 0.05 while the md and the manifest registered 0.03. The literal here is the registered value; the manifest
        comparison covers every tolerance the device applies (the manifest is written after the null-runs: until then this test fails).
        v5: the manifest is an input, not a constant: SLAM_REFACT_RM4C_MANIFEST names the registered manifest (a path, absolute or relative to the repository root); without it the
        v4 manifest is read, which registers the same tolerances (v5 changes no definition and no tolerance)."""
        import json
        import os
        rg = self.rg
        self.assertEqual(rg.TOL['A3n'], 0.03)
        named = os.environ.get('SLAM_REFACT_RM4C_MANIFEST')
        manifest_path = ROOT / (named or 'build/slam-refact-full-20261019/preregistration/rm4c-chunk-gate.v4.manifest.json')
        manifest = json.loads(manifest_path.read_text())
        registered = {k: v for k, v in manifest['tolerance'].items() if k != 'node_tests'}
        self.assertEqual(rg.registered_values(), registered)
        tiers = {t['id']: t['tolerance']['value'] for t in manifest['tiers']}
        self.assertEqual(tiers['GL-A3n'], rg.TOL['A3n'])
        self.assertEqual((tiers['GL-A1'], tiers['GL-A2n'], tiers['GL-E5']), (rg.TOL['A1'], rg.TOL['A2n'], rg.E5_MAX_POSITION_DELTA_M))


if __name__ == '__main__':
    unittest.main()
