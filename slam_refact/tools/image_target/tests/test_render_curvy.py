"""Unit tests of the curved-target renderer and the curvy ROI oracle (US-014; render_synthetic.py --shape,
curvy_reference.py). They check the oracle's own geometry against known answers and invariants, so that a
gate result is not a result of a wrong oracle (the oracle is independent of the engine).

Run (repo root): python3 -I -m unittest discover -s slam_refact/tools/image_target/tests -p 'test_render_curvy.py' -v
"""
import copy
import math
import pathlib
import sys
import unittest

import numpy as np

TOOL = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))

import curvy_reference as cr  # noqa: E402
import render_synthetic as rs  # noqa: E402

# The registered variant has no default seed any more (render_synthetic.use_seed_file). These tests need a registered
# variant with KNOWN answers, so they create it from the v3 coordinator seed; the value is a test constant here (it is the
# first token of preregistration/curvy-it-gate.v3.coordinator-seed.txt, checked below when that file is present).
V3_SEED = 3018981447
rs.use_coordinator_seed(V3_SEED)


class PhysicalModel(unittest.TestCase):
    def test_known_answer_dumela_label(self):
        # c8/geometry/parameterized-geometry.h documents "Dumela African Brew: 15.3cm x 10cm spanning the front ~270
        # degrees ... 2*pi*r = 15.3 / 0.725 => r = 3.35872" (the object's radius in cm, input arithmetic) and then
        # "r is thus 3.35872 / 10 / (0.9 - 0.1) = 0.41984". The engine normalises to the height of the CROP (= 1), so
        # for the same label (the whole image is the crop) its radius is 3.35872 / 10 = 0.335872; the comment's
        # 0.41984 = 0.335872 / 0.8 divides once more by the activation height (0.9 - 0.1), a normalisation the
        # engine does not use (UNVERIFIED which convention the comment's second step stands for; it is not reproduced).
        v = rs.VARIANTS['dev']['shapes']
        saved = dict(v['cylinder'])
        try:
            v['cylinder'] = {'name': 'dumela', 'type': 'CYLINDER', 'circumference': 15.3 / 0.725, 'arc': 0.725}
            model = rs.curvy_model('cylinder', w0=1530, h0=1000, variant='dev')
        finally:
            v['cylinder'] = saved
        self.assertAlmostEqual(2 * math.pi * model['rt'] * 10.0, 15.3 / 0.725, places=5)   # 2 pi r = 21.1 cm (r in cm)
        self.assertAlmostEqual(model['rt'] * 10.0, 3.35872, places=4)                     # the documented 3.35872
        self.assertAlmostEqual(model['rt'], 0.335872, places=5)                            # crop-height units
        self.assertAlmostEqual(model['rt'] / 0.8, 0.41984, places=4)                       # the comment's number: /0.8
        self.assertEqual(model['rt'], model['rb'])

    def test_cone_slant_relation(self):
        m = rs.curvy_model('cone')
        spec = rs.variant_data('dev')['shapes']['cone']
        # side length of the unrolled label S = arc * wider circumference * h0 / w0 (pixel aspect of the wider
        # edge); in model units S_m = sqrt(1 + (rt - rb)^2) and the model radii satisfy rt / rb = base.
        self.assertAlmostEqual(m['rt'] / m['rb'], spec['base'], places=9)
        wider = spec['arc'] * max(spec['circumference_top'], spec['circumference_top'] / spec['base'])
        side_phys = wider * m['h0'] / m['w0']
        ratio = side_phys / m['props']['cylinderSideLength']
        self.assertAlmostEqual(ratio, 1.0, places=9)
        # the wider edge (top) has arc length arc * 2 pi rt in model units = pixel width w0 * (S_m / h0)
        self.assertAlmostEqual(spec['arc'] * 2 * math.pi * m['rt'],
                               m['w0'] / m['h0'] * math.sqrt(1.0 + (m['rt'] - m['rb']) ** 2), places=9)

    def test_cylinder_label_has_the_image_aspect(self):
        m = rs.curvy_model('cylinder')
        # label arc length (model units) / label height (1) = w0 / h0
        self.assertAlmostEqual(2 * math.pi * m['rt'] * (m['right'] - m['left']), m['w0'] / m['h0'], places=9)

    def test_surface_point_and_label_coordinates_round_trip(self):
        for shape in ('cylinder', 'cone'):
            m = rs.curvy_model(shape)
            col = np.array([0.0, 100.5, 479.0, 240.0])
            row = np.array([0.0, 639.0, 320.25, 17.0])
            p = rs.surface_point(m, col, row)
            c2, r2, inside = rs.label_coordinates(m, p)
            np.testing.assert_allclose(c2, col, atol=1e-6)
            np.testing.assert_allclose(r2, row, atol=1e-6)
            self.assertTrue(inside.all())
            # on the surface: x^2 + z^2 = rho(y)^2
            rho = m['rb'] + (m['rt'] - m['rb']) * (p[:, 1] + 0.5)
            np.testing.assert_allclose(np.hypot(p[:, 0], p[:, 2]), rho, atol=1e-12)
            # the label centre column faces -z (angle fraction 0.5)
            mid = rs.surface_point(m, np.array([(m['w0'] - 1) / 2.0]), np.array([319.5]))
            self.assertAlmostEqual(float(mid[0, 0]), 0.0, places=9)
            self.assertLess(float(mid[0, 2]), 0.0)

    def test_ray_through_a_surface_point_hits_that_point_first(self):
        for shape in ('cylinder', 'cone'):
            m = rs.curvy_model(shape)
            r, t = cr.pose_from(20.0, 0.1, 2.1, 5.0)
            col = np.array([60.0, 240.0, 400.0])
            row = np.array([30.0, 320.0, 600.0])
            p = rs.surface_point(m, col, row)
            d = p - t
            hit, outer, q = rs.cast_curvy(m, t, d)
            visible = ((p[:, 0] * (t[0] - p[:, 0])) + (p[:, 2] * (t[2] - p[:, 2])) > 0)
            for i in range(3):
                if visible[i]:
                    self.assertTrue(hit[i] and outer[i])
                    np.testing.assert_allclose(q[i], p[i], atol=1e-9)

    def test_ray_from_behind_hits_the_inside(self):
        m = rs.curvy_model('cylinder')
        origin = np.array([0.0, 0.0, 0.0])           # inside the cylinder
        hit, outer, q = rs.cast_curvy(m, origin, np.array([[0.3, 0.0, 1.0]]))
        self.assertTrue(hit[0])
        self.assertFalse(outer[0])                   # the inner wall

    def test_rendered_label_is_the_label_texture(self):
        # frontal camera on the axis: the label front (centre column, centre row) is seen at the image centre
        m = rs.curvy_model('cylinder')
        tex = cr.smooth_texture_b()
        r, t = cr.pose_from(0.0, 0.0, 2.1, 0.0)
        img = rs.render_curvy(tex, m, cr.K, r, t, True, np.random.default_rng(0), noise=0.0)
        self.assertAlmostEqual(float(img[320, 240]), float(tex[320, 240]), delta=3.0)
        # the label is not mirrored for the frontal viewer: increasing label columns go to +x (camera right),
        # x = -rho sin(2 pi f) with f > 0.5 right of the centre
        p = rs.surface_point(m, np.array([300.0]), np.array([320.0]))
        self.assertGreater(float(p[0, 0]), 0.0)

    def test_not_visible_frames_are_the_background(self):
        m = rs.curvy_model('cone')
        tex = cr.smooth_texture_b()
        r, t = rs.camera_pose_curvy(0, 0, False, 'dev')
        img = rs.render_curvy(tex, m, cr.K, r, t, False, np.random.default_rng(0), noise=0.0)
        gy, gx = np.mgrid[0:640, 0:480].astype(np.float64)
        bg = np.clip(np.rint(90.0 + 30.0 * gx / 480 + 20.0 * gy / 640), 0, 255)
        self.assertLessEqual(float(np.abs(img.astype(float) - bg).max()), 1.0)

    def test_registered_trajectories_pass_their_range_check(self):
        for traj in ('registered', 'dev'):  # variants (the registered one is derived from the coordinator seed)
            poses = [rs.camera_pose_curvy(k, s, True, traj)
                     for _, s, e, vis in rs.SEGMENTS if vis for k in range(s, e)]
            lines, ok = rs.check_curvy_trajectory_ranges(poses)
            self.assertTrue(ok, lines)

    def test_json_mutations_change_the_geometry_and_the_shape_set(self):
        for shape, ids in (('cylinder', {'circ13', 'arc80', 'cyl2cone'}),
                           ('cone', {'circ13', 'arc80', 'swapTB', 'cone2cyl'})):
            m = rs.curvy_model(shape)
            muts = rs.curvy_json_mutations(m)
            self.assertEqual(set(muts), ids)
            for props in muts.values():
                self.assertNotEqual(props, m['props'])

    def test_crop_case_expected_values(self):
        c = rs.curvy_crop_case()
        arc = rs.variant_data()['shapes']['cylinder']['arc']
        self.assertAlmostEqual(c['right'] - c['left'], arc * 480 / 800, places=12)
        self.assertAlmostEqual(c['expected']['height'], 900 / 640, places=12)
        # label arc length / label height = original width / original height (full label, model units)
        full_len = c['expected']['targetCircumferenceTop']
        self.assertAlmostEqual(full_len / c['expected']['height'], 800 / 900, places=9)


class Variants(unittest.TestCase):
    def test_coordinator_seed_matches_the_seed_file_when_present(self):
        f = pathlib.Path(__file__).resolve().parents[4] / 'build/slam-refact-full-20261019/preregistration/curvy-it-gate.v3.coordinator-seed.txt'
        if f.exists():
            self.assertEqual(int(f.read_text().split()[0]), V3_SEED)

    def test_registered_variant_does_not_exist_without_a_seed_and_follows_the_seed_given(self):
        import importlib, os, tempfile
        fresh = importlib.import_module('render_synthetic')
        saved = (fresh.COORDINATOR_SEED, fresh.VARIANTS.pop('registered', None))
        try:
            fresh.COORDINATOR_SEED = None
            with self.assertRaises(ValueError):
                fresh.set_variant('registered')
            with self.assertRaises(RuntimeError):
                fresh.derive_registered()
            os.environ.pop(fresh.SEED_FILE_ENV, None)
            with self.assertRaises(ValueError):
                fresh.use_seed_file()           # no argument, no environment variable: no fallback
            with tempfile.TemporaryDirectory() as d:
                good = pathlib.Path(d, 'seed.txt')
                good.write_text('12345 comment words\n')
                self.assertEqual(fresh.read_seed_file(str(good)), 12345)
                for bad in ('', '   \n', 'abc\n', '012\n', '-5\n', '1.5\n', '18446744073709551616\n', '0x10\n'):
                    f = pathlib.Path(d, 'bad.txt')
                    f.write_text(bad)
                    with self.assertRaises(ValueError, msg=repr(bad)):
                        fresh.read_seed_file(str(f))
                self.assertEqual(fresh.read_seed_file(str(pathlib.Path(d, 'seed.txt'))), 12345)
                fresh.use_seed_file(str(good))
                other = fresh.VARIANTS['registered']
                fresh.use_coordinator_seed(V3_SEED)
                self.assertNotEqual(other, fresh.VARIANTS['registered'])
                self.assertEqual(fresh.VARIANTS['registered']['seeds'], {'cylinder': 358097, 'cone': 771091})
                os.environ[fresh.SEED_FILE_ENV] = str(good)
                try:
                    fresh.use_seed_file()       # the environment variable is used when no file is given
                    self.assertEqual(fresh.COORDINATOR_SEED, 12345)
                finally:
                    os.environ.pop(fresh.SEED_FILE_ENV, None)
        finally:
            fresh.COORDINATOR_SEED, reg = saved
            fresh.VARIANTS['registered'] = reg if reg is not None else fresh.derive_registered()

    def test_registered_variant_is_a_pure_function_of_the_seed_and_differs_from_dev(self):
        a, b = rs.derive_registered(), rs.derive_registered()
        self.assertEqual(a, b)
        self.assertNotEqual(a['shapes'], rs.VARIANTS['dev']['shapes'])
        self.assertNotEqual(a['seeds'], rs.VARIANTS['dev']['seeds'])
        # known values of the derivation (sha256 of '<seed>:<label>'); a change of the procedure changes them
        self.assertEqual(a['shapes']['cylinder']['circumference'], 152.3)
        self.assertEqual(a['shapes']['cylinder']['arc'], 0.526)
        self.assertEqual(a['shapes']['cone']['base'], 1.451)
        self.assertEqual(a['shapes']['cone']['arc'], 0.68)
        self.assertEqual(a['seeds'], {'cylinder': 358097, 'cone': 771091})

    def test_derived_values_stay_in_their_documented_ranges(self):
        r = rs.derive_registered()
        self.assertTrue(150.0 <= r['shapes']['cylinder']['circumference'] <= 250.0)
        self.assertTrue(0.45 <= r['shapes']['cylinder']['arc'] <= 0.60)
        self.assertTrue(1.25 <= r['shapes']['cone']['base'] <= 1.60)
        self.assertTrue(0.55 <= r['shapes']['cone']['arc'] <= 0.70)
        for i, (az, y, d, roll, yaw, pitch) in enumerate(r['b_poses'][:4]):
            self.assertTrue(10.0 <= abs(az) <= 30.0 and -0.2 <= y <= 0.2 and 1.9 <= d <= 2.3 and -6.0 <= roll <= 6.0)
            self.assertEqual((yaw, pitch), (0.0, 0.0))
        # the two extreme poses are the trajectory limits, not derived values
        self.assertEqual(r['b_poses'][4], (35.0, 0.3, 1.8, 8.5, 4.5, 4.5))
        self.assertEqual(r['b_poses'][5], (-35.0, -0.3, 2.4, -8.5, -4.5, -4.5))

    def test_registered_shapes_have_valid_geometry(self):
        for shape in ('cylinder', 'cone'):
            m = rs.curvy_model(shape, variant='registered')
            self.assertGreater(m['rt'], 0)
            self.assertGreater(m['rb'], 0)
            self.assertTrue(0 < m['left'] < m['right'] < 1)

    def test_limit_poses_are_inside_the_trajectory_range_check_bounds(self):
        for q in rs.VARIANTS['registered']['b_poses']:
            r, t = cr.pose_from(*q)
            az, y, dist, yaw, pitch, roll = rs.curvy_trajectory_quantities(r, t)
            self.assertLessEqual(abs(az), 37.0)
            self.assertLessEqual(abs(y), 0.31)
            self.assertTrue(1.79 <= dist <= 2.41)
            self.assertLessEqual(abs(yaw), 5.5)
            self.assertLessEqual(abs(pitch), 5.5)
            self.assertLessEqual(abs(roll), 9.0)

    def test_variant_is_recorded_by_the_renderer_and_the_oracle(self):
        import subprocess, sys, tempfile
        with tempfile.TemporaryDirectory() as d:
            subprocess.run([sys.executable, '-I', str(TOOL / 'curvy_reference.py'), 'prepare', '--out', d + '/o',
                            '--slot-w', '252', '--slot-h', '336', '--variant', 'dev', '--camera-kind', 'constant'], check=True)
            self.assertEqual(pathlib.Path(d, 'o/variant.txt').read_text().strip(), 'dev')

    def test_cli_registered_variant_needs_a_seed_file(self):
        import os, subprocess, sys, tempfile
        env = {k: v for k, v in os.environ.items() if k != rs.SEED_FILE_ENV}
        with tempfile.TemporaryDirectory() as d:
            for tool, args in (('curvy_reference.py', ['prepare', '--out', d + '/o', '--slot-w', '252', '--slot-h', '336']),
                               ('render_synthetic.py', ['--shape', 'cylinder', '--check-only', '--out', d + '/r'])):
                r = subprocess.run([sys.executable, '-I', str(TOOL / tool), *args, '--variant', 'registered'],
                                   env=env, capture_output=True, text=True)
                self.assertEqual(r.returncode, 2, (tool, r.stdout, r.stderr))
                self.assertIn('coordinator seed', r.stderr)
            self.assertFalse(pathlib.Path(d, 'o').exists())
            seed = pathlib.Path(d, 'seed.txt')
            seed.write_text(f'{V3_SEED}\n')
            r = subprocess.run([sys.executable, '-I', str(TOOL / 'render_synthetic.py'), '--shape', 'cylinder', '--check-only',
                                '--out', d + '/r', '--variant', 'registered', '--seed-file', str(seed)],
                               env=env, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, (r.stdout, r.stderr))
            self.assertIn('coordinator seed file sha256', r.stdout)
            self.assertNotIn(str(V3_SEED), r.stdout)   # the seed value is never printed


class ShaderModelOracle(unittest.TestCase):
    def test_constant_camera_gives_the_constant_where_drawn(self):
        cam = np.full((640, 480), 100, dtype=np.uint8)
        for case in cr.cases_a():
            exp, mask = cr.slot_shader_model(cam, case, 252, 336)
            self.assertTrue((exp[mask > 0] == 100).all())
            self.assertTrue((exp[mask == 0] == 0).all())
            self.assertGreater(int((mask > 0).sum()), 20000)

    def test_float32_and_float64_agree(self):
        cam = cr.rr.camera_a('textured')
        for case in cr.cases_a():
            e32, m32 = cr.slot_shader_model(cam, case, 252, 336, np.float32)
            e64, m64 = cr.slot_shader_model(cam, case, 252, 336, np.float64)
            both = (m32 > 0) & (m64 > 0)
            self.assertLessEqual(int(np.abs(e32.astype(int) - e64.astype(int))[both].max()), 1)
            self.assertLess(float(((m32 > 0) != (m64 > 0)).mean()), 1e-4)

    def test_geometric_rule_agrees_with_the_shader_model_on_a_rendered_camera(self):
        tex = cr.smooth_texture_b()
        for shape in ('cylinder', 'cone'):
            m = rs.curvy_model(shape)
            r, t = cr.pose_from(25.0, 0.0, 2.1, 0.0)
            cam = rs.render_curvy(tex, m, cr.K, r, t, True, np.random.default_rng(0), noise=0.0)
            exp, mask, strong = cr.geometric_expected(tex, m, r, t, 252, 336)
            case = dict(radius=m['rt'], radius_bottom=m['rb'], src_rows=640, src_cols=480, left=m['left'],
                        right=m['right'], K=cr.K, R=r.reshape(-1).tolist(), t=t.tolist(), rotation_deg=0)
            sh, shm = cr.slot_shader_model(cam, case, 252, 336)
            self.assertLess(float(((shm > 0) != mask).mean()), 1e-3)
            e = cr.normalised_rms(sh, exp, np.where(cr.erode(strong, cr.ERODE_PX), 255, 0).astype(np.uint8))
            self.assertLess(e, 0.1)

    def test_shader_model_detects_the_input_mutations(self):
        tex = cr.smooth_texture_b()
        m = rs.curvy_model('cone')
        r, t = cr.pose_from(25.0, 0.0, 2.1, 0.0)
        cam = rs.render_curvy(tex, m, cr.K, r, t, True, np.random.default_rng(0), noise=0.0)
        exp, mask, strong = cr.geometric_expected(tex, m, r, t, 252, 336)
        er = np.where(cr.erode(strong, cr.ERODE_PX), 255, 0).astype(np.uint8)
        good = dict(radius=m['rt'], radius_bottom=m['rb'], src_rows=640, src_cols=480, left=m['left'],
                    right=m['right'], K=cr.K, R=r.reshape(-1).tolist(), t=t.tolist(), rotation_deg=0)
        mutations = {
            'radius x1.3': dict(good, radius=m['rt'] * 1.3, radius_bottom=m['rb'] * 1.3),
            'top/bottom swapped': dict(good, radius=m['rb'], radius_bottom=m['rt']),
            'arc x0.8': dict(good, right=m['left'] + (m['right'] - m['left']) * 0.8),
            'mirrored arc': dict(good, left=m['right'], right=m['left']),
            'radius sign': dict(good, radius=-m['rt'], radius_bottom=-m['rb']),
        }
        for name, case in mutations.items():
            sh, shm = cr.slot_shader_model(cam, case, 252, 336)
            e = cr.normalised_rms(sh, exp, er)
            cov = float(((shm > 0) != mask).mean())
            self.assertTrue(e > 0.15 or cov > 0.002, f'{name}: e {e:.3f} coverage {cov:.4f}')


if __name__ == '__main__':
    unittest.main()
