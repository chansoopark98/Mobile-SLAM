import importlib.util
import json
import pathlib
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path(__file__).resolve().parents[1]
def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
G = load('synthetic_generator', ROOT / 'scripts/dev/generate-synthetic-vio.py')

class SyntheticOracleTests(unittest.TestCase):
    def test_stationary_force_and_camera_lever_arm(self):
        spec = G.make_spec(101, 'stationary')
        state = G.body_state(4.0, spec)
        np.testing.assert_allclose(state['accel_measurement'], [0, 0, 9.81], atol=1e-14)
        np.testing.assert_array_equal(state['gyro'], [0, 0, 0])
        r, p = G.camera_pose(state, spec)
        np.testing.assert_allclose(p, state['position'] + state['rotation'] @ np.array(spec['camera']['t_ic']))
        np.testing.assert_allclose(r, state['rotation'] @ np.array(spec['camera']['r_ic']).reshape(3, 3))

    def test_closed_form_derivatives_and_noncommuting_body_rate(self):
        spec = G.make_spec(101, 'clean')
        for time in [1.25, 1.7, 2.4, 4.3, 10.6, 18.4]:
            s = G.body_state(time, spec)
            errors = []
            for h in [1e-3, 1e-4, 1e-5]:
                before, after = G.body_state(time-h, spec), G.body_state(time+h, spec)
                v = (after['position']-before['position'])/(2*h)
                a = (after['velocity']-before['velocity'])/(2*h)
                derivative = s['rotation'].T @ ((after['rotation']-before['rotation'])/(2*h))
                w = np.array([derivative[2,1]-derivative[1,2], derivative[0,2]-derivative[2,0], derivative[1,0]-derivative[0,1]])/2
                errors.append([np.max(abs(v-s['velocity'])),np.max(abs(a-s['acceleration'])),np.max(abs(w-s['gyro']))])
            self.assertTrue((np.min(errors, axis=0) <= [1e-7,1e-5,1e-7]).all(), errors)
            np.testing.assert_allclose(s['rotation'].T @ s['rotation'], np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(s['rotation']),1,places=12)
            np.testing.assert_allclose(s['rotation'] @ s['accel_measurement'] + [0,0,-9.81], s['acceleration'], atol=1e-12)

    def test_constant_velocity_and_rotation_degeneracy(self):
        for scenario in ['pure-rotation','constant-velocity']:
            spec = G.make_spec(101, scenario)
            self.assertEqual(spec['camera']['t_ic'], [0,0,0])
            a = G.body_state(3, spec)
            np.testing.assert_allclose(a['acceleration'],0,atol=1e-14)
            np.testing.assert_allclose(a['rotation'] @ a['accel_measurement'],[0,0,9.81],atol=1e-12)
        np.testing.assert_allclose(G.body_state(3,G.make_spec(101,'pure-rotation'))['position'],[0,0,1.2])

    def test_nearest_surface_occlusion_and_behind_camera(self):
        surfaces = [G.surface([0,0,4],[1,0,0],[0,1,0],[2,2],0),
                    G.surface([0,0,2],[1,0,0],[0,1,0],[.5,.5],1),
                    G.surface([0,0,-1],[1,0,0],[0,1,0],[4,4],2)]
        rays = np.array([[0,0,1],[.4,0,1],[5,0,1]],dtype=float)
        gray, depth, ids = G.cast_rays(np.zeros(3), rays, surfaces,[np.full((4,4),60),np.full((4,4),180),np.full((4,4),250)])
        np.testing.assert_array_equal(gray,[180,60,16])
        np.testing.assert_allclose(depth[:2],[2,4])
        self.assertTrue(np.isinf(depth[2]))
        np.testing.assert_array_equal(ids,[1,0,-1])

    def test_landmark_projection_velocity_matches_independent_pixel_difference(self):
        spec=G.make_spec(101,'clean'); point=np.array([[.1,4,1.4]])
        s=G.body_state(4.2,spec)
        observation=G.project_points(point,s,spec)
        h=1e-5
        before=G.project_points(point,G.body_state(4.2-h,spec),spec)
        after=G.project_points(point,G.body_state(4.2+h,spec),spec)
        np.testing.assert_allclose((after['normalized']-before['normalized'])/(2*h),observation['velocity'],atol=1e-8)
        r,p=G.camera_pose(s,spec); camera=(r.T @ (point[0]-p))
        expected=[260*camera[0]/camera[2]+159.5,255*camera[1]/camera[2]+119.5]
        np.testing.assert_allclose(observation['pixels'][0],expected,atol=1e-12)

    def test_generator_repeat_hashes_and_no_overwrite(self):
        spec=G.make_spec(101,'clean');spec['duration_s']=.15;spec['frame_count']=3;spec['fixture_kind']='oracle_unit'
        with tempfile.TemporaryDirectory() as tmp:
            first=G.generate(pathlib.Path(tmp)/'a',spec)
            second=G.generate(pathlib.Path(tmp)/'b',spec)
            self.assertEqual(first['files'],second['files'])
            inp=json.loads((pathlib.Path(tmp)/'a/input.json').read_text())
            self.assertEqual(len(inp['frames']),3)
            self.assertLessEqual(inp['imu'][0][0],inp['frames'][0]['timestamp'])
            self.assertGreaterEqual(inp['imu'][-1][0],inp['frames'][-1]['timestamp'])
            self.assertEqual(inp['schema'],'mobile-slam-synthetic-vio-v1')
            self.assertEqual(np.linalg.norm(np.array(inp['config']['t_ic'])),.03)
            with self.assertRaises(FileExistsError):G.generate(pathlib.Path(tmp)/'a',spec)

    def test_json_timestamp_bits_equal_native_csv_nanoseconds_multiplication(self):
        spec=G.make_spec(101,'clean');spec['duration_s']=.2;spec['frame_count']=4;spec['fixture_kind']='oracle_unit'
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)/'sample';G.generate(root,spec)
            data=json.loads((root/'input.json').read_text())
            camera_ns=[int(line.split(',')[0]) for line in (root/'mav0/cam0/data.csv').read_text().splitlines()[1:]]
            imu_ns=[int(line.split(',')[0]) for line in (root/'mav0/imu0/data.csv').read_text().splitlines()[1:]]
            self.assertEqual([frame['timestamp'] for frame in data['frames']],[float(ns)*1e-9 for ns in camera_ns])
            self.assertEqual([row[0] for row in data['imu']],[float(ns)*1e-9 for ns in imu_ns])

    def test_mobile_noise_quantization_is_deterministic_separate_from_truth(self):
        spec=G.make_spec(101,'mobile40');a,truth=G.sample_imu(spec);b,_=G.sample_imu(spec)
        np.testing.assert_array_equal(a,b)
        np.testing.assert_allclose(a[:,1:4]/.1,np.round(a[:,1:4]/.1),atol=1e-12)
        step=np.deg2rad(.1)
        np.testing.assert_allclose(a[:,4:]/step,np.round(a[:,4:]/step),atol=1e-12)
        self.assertGreater(np.max(abs(a[:,1:]-truth[:,1:])),0)
        self.assertEqual(spec['injected_noise']['acc_density'],.005)
        self.assertEqual(spec['estimator_profile']['config']['acc_n'],.5)


E = load('synthetic_evaluator', ROOT / 'scripts/dev/evaluate-synthetic-vio.py')

class SyntheticEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.spec=G.make_spec(101,'clean');self.truth={'frames':[]};self.rows=[]
        for i in range(400):
            t=i/20;state=G.body_state(t,self.spec)
            # Independent known global gauge: transform both position and orientation.
            gauge=Rotation.from_euler('xyz',[.2,-.3,.4]).as_matrix();offset=np.array([1,2,-3])
            gt={'index':i,'timestamp':t,'body':{'position':state['position'].tolist(),'quaternion_xyzw':Rotation.from_matrix(state['rotation']).as_quat().tolist()}}
            self.truth['frames'].append(gt)
            pose=np.eye(4);pose[:3,:3]=gauge@state['rotation'];pose[:3,3]=gauge@state['position']+offset
            self.rows.append({'frame':i,'inputTimestamp':float(round(t*1e9))*1e-9,'poseTimestamp':t,'poseValid':True,'poseFresh':True,'engineEpoch':1,'pose':pose.ravel().tolist()})
    def test_metric_gauge_and_physical_frame_time_oracle(self):
        result=E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.assertTrue(result['passed'],result['failures'])
        self.assertLess(result['metrics']['ate_translation_m']['rmse'],1e-12)
        self.assertLess(result['metrics']['ape_rotation_deg']['rmse'],1e-10)
        for row in self.rows:
            row['sourceFrameIndex']=row['frame'];row['inputTimestamp']+=.03;row['poseTimestamp']+=.03
        result=E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.assertTrue(result['passed'],'declared physical IDs retain GT association under reported-clock fault')
        self.assertAlmostEqual(result['reported_pose_timestamps'][0],.03)
    def test_camera_to_body_uses_declared_nonzero_lever_arm_and_rotation(self):
        rbc=np.array(self.spec['camera']['r_ic']).reshape(3,3)
        tbc=np.array(self.spec['camera']['t_ic'])
        for row in self.rows:
            body=np.array(row['pose']).reshape(4,4)
            camera=body.copy();camera[:3,3]=body[:3,3]+body[:3,:3]@tbc;camera[:3,:3]=body[:3,:3]@rbc
            row['pose']=camera.ravel().tolist()
        result=E.evaluate_rows(self.spec,self.truth,self.rows,'camera')
        self.assertTrue(result['passed'],result['failures'])
        self.assertLess(result['metrics']['ate_translation_m']['rmse'],1e-12)
        self.assertLess(result['metrics']['ape_rotation_deg']['rmse'],1e-10)

    def test_degenerate_zero_output_requires_complete_scheduled_processing(self):
        self.spec['acceptance']['expected_degenerate']=True
        for row in self.rows:row['poseValid']=False;row['pose']=None
        self.assertTrue(E.evaluate_rows(self.spec,self.truth,self.rows,'body')['passed'])
        self.assertFalse(E.evaluate_rows(self.spec,self.truth,self.rows[:-1],'body')['passed'])

    def test_all_processed_rows_require_unique_ordered_source_identity_even_without_pose(self):
        self.spec['acceptance']['expected_degenerate']=True
        empty=dict(self.rows[0],poseValid=False,pose=None)
        with self.assertRaisesRegex(ValueError,'Duplicate'):
            E.evaluate_rows(self.spec,self.truth,[empty]*400,'body')
        with self.assertRaisesRegex(ValueError,'identity|inputTimestamp'):
            E.evaluate_rows(self.spec,self.truth,[{} for _ in range(400)],'body')
        no_pose=[dict(row,poseValid=False,pose=None) for row in self.rows]
        no_pose[100],no_pose[101]=no_pose[101],no_pose[100]
        with self.assertRaisesRegex(ValueError,'ordered'):
            E.evaluate_rows(self.spec,self.truth,no_pose,'body')

    def test_truth_cannot_reduce_fixed_coverage_denominator_or_shift_time_grid(self):
        with self.assertRaisesRegex(ValueError,'truth.*count|Truth.*count'):
            E.evaluate_rows(self.spec,dict(self.truth,frames=self.truth['frames'][:200]),self.rows[:200],'body')
        self.truth['frames'][100]['timestamp']+=.001
        with self.assertRaisesRegex(ValueError,'truth.*grid|Truth.*grid'):
            E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.truth['frames'][100]['timestamp']=5.
        self.spec['acceptance']['scheduled_warmup_frames']=299
        with self.assertRaisesRegex(ValueError,'warmup'):
            E.evaluate_rows(self.spec,self.truth,self.rows,'body')

    def test_truth_indices_and_invalid_row_timestamps_are_verified(self):
        self.truth['frames'][120]['index']=119
        with self.assertRaisesRegex(ValueError,'truth.*index|Truth.*index'):
            E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.truth['frames'][120]['index']=120
        self.rows[120]=dict(self.rows[120],poseValid=False,pose=None,inputTimestamp=6.05)
        with self.assertRaisesRegex(ValueError,'timestamp'):
            E.evaluate_rows(self.spec,self.truth,self.rows,'body')

    def test_actual_groundtruth_hash_is_checked_against_manifest(self):
        import hashlib
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)
            truth=dict(self.truth,schema='mobile-slam-synthetic-groundtruth-v1')
            blobs={'spec.json':G.json_bytes(self.spec),'groundtruth.json':G.json_bytes(truth)}
            manifest={'schema':'mobile-slam-synthetic-manifest-v1',
                'spec_sha256':hashlib.sha256(blobs['spec.json']).hexdigest(),
                'files':{name:{'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()} for name,data in blobs.items()}}
            for name,data in blobs.items():(root/name).write_bytes(data)
            (root/'manifest.json').write_bytes(G.json_bytes(manifest))
            _,_,provenance=E.load_dataset(root)
            self.assertEqual(provenance['groundtruth_sha256'],hashlib.sha256(blobs['groundtruth.json']).hexdigest())
            (root/'groundtruth.json').write_bytes(blobs['groundtruth.json']+b' ')
            with self.assertRaisesRegex(ValueError,'groundtruth.json.*hash|groundtruth.json.*manifest'):
                E.load_dataset(root)

    def test_source_index_cannot_relabel_a_stale_pose_as_fresh(self):
        self.rows[100]['poseTimestamp']-=.05
        result=E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.assertFalse(result['passed'])
        self.assertEqual(result['invalid_pose_rows'],1)
        self.rows[100]['sourceFrameIndex']=100
        result=E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.assertFalse(result['passed'])
        self.assertEqual(result['invalid_pose_rows'],1)

    def test_no_similarity_scale_rescue_or_adaptive_coverage(self):
        for row in self.rows:
            for offset in [3,7,11]:row['pose'][offset]*=2
        result=E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.assertFalse(result['passed'])
        self.assertGreater(result['metrics']['ate_translation_m']['rmse'],.1)
        sparse=E.evaluate_rows(self.spec,self.truth,self.rows[200:],'body')
        self.assertEqual(sparse['postwarmup_source_frames'],300)
        self.assertAlmostEqual(sparse['postwarmup_coverage'],200/300)
        self.assertIn('initialization deadline missed',sparse['failures'])
    def test_duplicate_missing_rotation_zero_output_and_epoch_resets_fail(self):
        with self.assertRaisesRegex(ValueError,'Duplicate'):E.evaluate_rows(self.spec,self.truth,[self.rows[0],self.rows[0]],'body')
        self.assertFalse(E.evaluate_rows(self.spec,self.truth,[],'body')['passed'])
        self.rows[0]['pose']=[0]*16
        self.assertFalse(E.evaluate_rows(self.spec,self.truth,self.rows,'body')['passed'])
        self.rows[0]=dict(self.rows[1],frame=0,inputTimestamp=0,poseTimestamp=0)
        for row in self.rows[200:]:row['engineEpoch']=2
        result=E.evaluate_rows(self.spec,self.truth,self.rows,'body')
        self.assertFalse(result['passed']);self.assertEqual(result['observed_epoch_transitions'],1)

if __name__=='__main__':unittest.main()
