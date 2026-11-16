"""scripts/dev/rs0-gate.py: verdicts, cache reuse / invalidation, fast-tier cut, env scrub, WASM and exact plumbing.

A fake slam_refact_replay (python script) and a fake WASM engine stand in for the real ones.
"""
import contextlib
import importlib.util
import io
import json
import os
import pathlib
import shutil
import stat
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rs0_gate', ROOT / 'scripts/dev/rs0-gate.py')
rs0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rs0)

FAKE_REPLAY = r'''#!/usr/bin/env python3
import json, os, random, sys
MODE, CV = %r, %d
a = dict(zip(sys.argv[1::2], sys.argv[2::2]))
b, out = a['--bundle'], a['--out']
os.mkdir(out)
frames = [l.split('\t') for l in open(b + '/frames.tsv').read().splitlines() if l]
n = 0
with open(out + '/trace.jsonl', 'w') as f:
    for r in frames:
        if r[0] != 'frame':
            continue
        i = int(r[1]); n += 1
        p = [i * 0.1, 0.0, 0.0]
        if MODE == 'pose' and i == 2:
            p[1] = 1e-9
        row = {'i': i, 'fresh': r[5] == '1', 'p': p, 'q_wxyz': [1, 0, 0, 0], 'status': 'NORMAL', 'worldPoints': i,
               'srStageMs': {'frontend': random.random()}}
        if MODE == 'stall' and i == 2:
            row['srStageMs']['mapTracker'] = 1500.0
        if MODE == 'nondet':
            row['worldPoints'] = random.randint(0, 10 ** 9)
        f.write(json.dumps(row) + '\n')
open(out + '/worldpoints-sampled.jsonl', 'w').write('{"n": 1}\n')
open(out + '/accuracy-config.json', 'w').write(json.dumps({'enabled': 'SLAM_REFACT_ACCURACY' in os.environ}) + '\n')
meta = {'schema': 'fake', 'bundle': b, 'contractVersion': CV, 'wallSeconds': random.random(), 'rows': n}
if MODE == 'metaextra':
    meta['processedFrames'] = -1
json.dump(meta, open(out + '/meta.json', 'w'))
sys.exit(3 if MODE == 'crash' else 0)
'''

FAKE_ENGINE = '''module.exports = async () => ({ SlamRefactGlue: class {
  constructor() { this.n = 0; } pushDeviceMotion() {} pushDeviceOrientation() {}
  processFrame(t) { this.n += 1; return { t, n: this.n, %s stageMs: { x: Math.random() } }; } delete() {} } });
'''


def write_bundle(d, n_frames, image):
    d.mkdir(parents=True)
    cfg = {'estimate_scale': '0', 'disable_vio': '0', 'capture_width': '4', 'capture_height': '2', 'rotation_deg': '0',
           'manufacturer': '', 'model': 'K', 'pyramid_model': 'x', 'origin': '0 2 0', 'world_point_sample_every': '50',
           'pyramid_injection': 'none'}
    (d / 'engine-config.tsv').write_text(''.join('%s\t%s\n' % kv for kv in cfg.items()))
    rows = ['bootstrap\t-1\t1000\t1000\t0\t1\t%s\t-' % image]
    rows += ['frame\t%d\t%d\t%d\t0\t%d\t%s\t-' % (i, 2000 + i, 2000 + i, 0 if i == 3 else 1, image) for i in range(n_frames)]
    (d / 'frames.tsv').write_text('\n'.join(rows) + '\n')
    sens = ['bootstrap\t900' + '\t0' * 13] + ['run\t%d' % (2000 + i) + '\t0' * 13 for i in range(n_frames + 2)]
    (d / 'sensors.tsv').write_text('\n'.join(sens) + '\n')
    (d / 'bundle.json').write_text('{}\n')


class Rs0GateTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.tmp.name)
        img = self.root / 'data/img.pgm'
        img.parent.mkdir(parents=True)
        img.write_bytes(b'P5\n4 2\n255\n' + bytes(range(8)))
        self.img = img
        for s, n in (('scA', 8), ('scB', 5)):
            write_bundle(self.root / 'bundles' / s / 'par', n, img)
        write_bundle(self.root / 'offb/x1/k', 6, img)
        write_bundle(self.root / 'offb/x2/k', 4, img)
        self.manifest = self.root / 'manifest.json'
        self.manifest.write_text(json.dumps({
            'parity': {'bundle_root': 'bundles', 'items': [['scA', 'par'], ['scB', 'par']]},
            'off': {'reps': ['a', 'b'], 'items': [['x1', 'offb/x1/k'], ['x2', 'offb/x2/k']]},
            'wasm': {'bundle_root': 'bundles', 'items': [['scA', 'par', 6], ['scB', 'par', 3]]},
            'exact': {'lists': []},
            'fast': {'native_frames': 3, 'wasm_frames': 2},
            'defaults': {'frozen_native': 'bin/frozen', 'frozen_wasm': 'wasm/frozen', 'quarantine_links': []}}))
        self.binary('frozen', 'ok', 1)
        self.n_out = 0

    def tearDown(self):
        os.environ.pop('SLAM_REFACT_ACCURACY', None)
        self.tmp.cleanup()

    def binary(self, name, mode, cv):
        p = self.root / 'bin' / name
        p.parent.mkdir(exist_ok=True)
        p.write_text(FAKE_REPLAY % (mode, cv) + '# %s\n' % name)  # each fake binary has its own sha256
        p.chmod(p.stat().st_mode | stat.S_IEXEC)
        return p

    def engine(self, name, extra=''):
        d = self.root / 'wasm' / name
        d.mkdir(parents=True, exist_ok=True)
        (d / 'slam_refact_engine.js').write_text(FAKE_ENGINE % extra + '// %s\n' % name)
        return d

    def gate(self, new, tier='full', sections='parity,off', extra=()):
        self.n_out += 1
        out = self.root / ('out%d' % self.n_out)
        argv = ['--tier', tier, '--new-native', str(new), '--out', str(out), '--root', str(self.root),
                '--manifest', str(self.manifest), '--jobs', '4', '--sections', sections] + list(extra)
        with contextlib.redirect_stdout(io.StringIO()):
            rc = rs0.main(argv)
        return rc, json.loads((out / 'SUMMARY.json').read_text()), out

    def test_contract_version_only_difference_passes(self):
        rc, s, out = self.gate(self.binary('new', 'ok', 2))
        self.assertEqual((rc, s['overall']), (0, 'PASS'))
        self.assertEqual(s['RS-0b']['meta_keys_differing'], ['contractVersion', 'wallSeconds'])
        self.assertEqual(s['RS-0d']['zero_diff'], 2)
        rows = json.loads((out / 'parity-compare.json').read_text())
        self.assertEqual([(r['scenario'], r['identical']) for r in rows], [('scA', True), ('scB', True)])
        self.assertIn('srStageMs', rows[0]['fields_differing'])  # counted, excluded from the hash

    def test_pose_difference_fails_parity(self):
        rc, s, out = self.gate(self.binary('new', 'pose', 1))
        self.assertEqual((rc, s['overall'], s['RS-0b']['verdict']), (1, 'FAIL', 'FAIL'))
        self.assertIn('p', json.loads((out / 'parity-compare.json').read_text())[0]['fields_differing'])

    def test_other_meta_key_fails_parity(self):
        _, s, _ = self.gate(self.binary('new', 'metaextra', 1))
        self.assertEqual(s['RS-0b']['verdict'], 'FAIL')

    def test_nondeterminism_fails_off(self):
        _, s, _ = self.gate(self.binary('new', 'nondet', 1), sections='off')
        self.assertEqual(s['RS-0d']['verdict'], 'FAIL')
        self.assertGreater(s['RS-0d']['trace_diff_rows'], 0)

    def test_crash_fails_and_is_not_cached(self):
        new = self.binary('new', 'crash', 1)
        rc, s, _ = self.gate(new, sections='off')
        self.assertEqual((rc, s['RS-0d']['verdict']), (1, 'FAIL'))
        _, s2, _ = self.gate(new, sections='off')
        self.assertEqual(s2['timing']['replays_cached'], 0)

    def test_unchanged_inputs_are_reused_and_fresh_reruns_new_only(self):
        new = self.binary('new', 'ok', 1)
        _, s1, o1 = self.gate(new)
        self.assertEqual(s1['timing']['replays_run'], 8)  # 2 frozen + 2 new + 2 x 2 off reps
        _, s2, o2 = self.gate(new)
        self.assertEqual((s2['timing']['replays_run'], s2['overall']), (4, 'PASS'))  # only the RS-0d reps
        trace = lambda o, rel: (o / rel / 'trace.jsonl').read_bytes()
        self.assertEqual(trace(o1, 'parity/new/scA/par'), trace(o2, 'parity/new/scA/par'))
        self.assertNotEqual(trace(o1, 'off/a/x1'), trace(o2, 'off/a/x1'))  # new srStageMs: a real second run
        _, s3, o3 = self.gate(new, extra=['--fresh'])
        self.assertEqual(s3['timing']['replays_run'], 6)  # frozen stays cached
        self.assertNotEqual(trace(o2, 'parity/new/scA/par'), trace(o3, 'parity/new/scA/par'))  # re-run replaces
        self.assertEqual(trace(o2, 'parity/frozen/scA/par'), trace(o3, 'parity/frozen/scA/par'))
        self.assertTrue(any((self.root / 'build/rs0-cache/superseded').iterdir()))

    def test_new_binary_or_changed_image_invalidates(self):
        self.gate(self.binary('new', 'ok', 1))
        _, s2, _ = self.gate(self.binary('new', 'ok', 3))  # different bytes -> different sha
        self.assertEqual(s2['timing']['replays_run'], 6)  # frozen stays cached
        self.img.write_bytes(b'P5\n4 2\n255\n' + bytes(range(1, 9)))
        _, s3, _ = self.gate(self.binary('new', 'ok', 3))
        self.assertEqual(s3['timing']['replays_run'], 8)

    def test_env_is_scrubbed(self):
        os.environ['SLAM_REFACT_ACCURACY'] = '1'
        _, s, out = self.gate(self.binary('new', 'ok', 1), sections='off')
        self.assertEqual(json.loads((out / 'off/a/x1/accuracy-config.json').read_text()), {'enabled': False})
        self.assertNotIn('SLAM_REFACT_ACCURACY', rs0.scrubbed_env())

    def test_fast_tier_cuts_frames_and_keeps_names(self):
        rc, s, out = self.gate(self.binary('new', 'ok', 1), tier='fast')
        self.assertEqual(s['overall'], 'PASS')
        rows = json.loads((out / 'parity-compare.json').read_text())
        self.assertEqual([r['frames'] for r in rows], [3, 3])
        bundle = pathlib.Path(json.loads((out / 'parity/new/scA/par/meta.json').read_text())['bundle'])
        self.assertEqual((bundle.parent.name, bundle.name), ('scA', 'par'))
        lines = (bundle / 'frames.tsv').read_text().splitlines()
        self.assertEqual([l.split('\t')[0] for l in lines], ['bootstrap', 'frame', 'frame', 'frame'])
        src = self.root / 'bundles/scA/par'
        self.assertEqual((bundle / 'sensors.tsv').read_bytes(), (src / 'sensors.tsv').read_bytes())

    def test_stalled_runs_are_reported_and_not_cached(self):
        new = self.binary('new', 'stall', 1)
        rc, s, out = self.gate(new, sections='parity')
        self.assertEqual((rc, s['RS-0b']['verdict']), (0, 'VERIFIED'))  # verdict untouched
        self.assertTrue(all(x['stall_frames'] == 1 for x in s['stalled_replays']))
        self.assertEqual({x['id'] for x in s['stalled_replays']}, {'parity/new/scA/par', 'parity/new/scB/par'})
        self.assertIn('stall_note', s)
        _, s2, _ = self.gate(new, sections='parity')
        self.assertEqual(s2['timing']['replays_run'], 2)  # stalled new runs were not cached; frozen was
        self.assertTrue(any((self.root / 'build/rs0-cache/unstable').iterdir()))

    def test_full_tier_passes_bundle_path_as_spelled(self):
        _, s, out = self.gate(self.binary('new', 'ok', 1), sections='off')
        self.assertEqual(json.loads((out / 'off/a/x1/meta.json').read_text())['bundle'], 'offb/x1/k')

    def test_merge_gate_needs_full_tier_all_sections_and_pins(self):
        self.engine('frozen')
        new, eng = self.binary('new', 'ok', 1), self.engine('new')
        _, s, _ = self.gate(new, sections='parity,off,wasm,exact', extra=['--new-wasm', str(eng)])
        self.assertEqual((s['overall'], s['merge_gate']), ('FAIL', True))  # gate run; empty exact list is not VERIFIED
        _, s, _ = self.gate(new)
        self.assertEqual((s['overall'], s['merge_gate']), ('PASS', False))  # partial sections
        m = json.loads(self.manifest.read_text())
        m['defaults']['frozen_pins'] = {'native': '0' * 64}
        self.manifest.write_text(json.dumps(m))
        with self.assertRaises(SystemExit):
            self.gate(new)
        _, s, _ = self.gate(new, extra=['--allow-unpinned-frozen'])
        self.assertEqual((s['overall'], s['merge_gate']), ('PASS', False))
        with self.assertRaises(SystemExit):
            self.gate(self.root / 'bin/frozen', extra=['--allow-unpinned-frozen'])

    def test_truncate_keeps_rows_before_cut(self):
        dest = rs0.truncate_bundle(self.root / 'offb/x1/k', 2, self.root / 'cut', 'h' * 40)
        self.assertEqual((dest / 'frames.tsv').read_text().count('\n'), 3)
        self.assertEqual(rs0.processed_frames(dest), 2)
        self.assertEqual(rs0.processed_frames(self.root / 'bundles/scA/par'), 7)  # frame 3 is drop-masked

    def test_longest_first(self):
        cache = rs0.Cache(self.root / 'c')
        new = self.binary('new', 'ok', 1)
        tasks = [dict(id='t%d' % e, kind='native', role='new', tool=str(new), tool_sha='x',
                      bundle=str(self.root / 'offb/x1/k'), bundle_hash='b%d' % e, extra={}, est=e, key='k%d' % e)
                 for e in (1, 5, 3)]
        seen = []
        rs0.run_tasks(tasks, cache, 1, self.root, log=seen.append)
        self.assertEqual([l.split()[1] for l in seen[1:]], ['t5', 't3', 't1'])

    @unittest.skipUnless(shutil.which('node'), 'node not installed')
    def test_wasm_parity_items_and_cache(self):
        self.engine('frozen')
        rc, s, out = self.gate(self.binary('new', 'ok', 1), sections='wasm', extra=['--new-wasm', str(self.engine('new'))])
        self.assertEqual((rc, s['RS-0c']), (0, {'items': 2, 'identical': 2, 'verdict': 'VERIFIED'}))
        rows = json.loads((out / 'wasm-parity.json').read_text())
        self.assertEqual([(r['maxFrames'], r['new']['rows'], r['new']['fresh']) for r in rows], [(6, 6, 5), (3, 3, 3)])
        _, s, _ = self.gate(self.binary('new', 'ok', 1), sections='wasm',
                            extra=['--new-wasm', str(self.engine('new2', 'bad: 1,'))])
        self.assertEqual((s['RS-0c']['verdict'], s['timing']['replays_run']), ('FAIL', 2))  # frozen reused

    def test_exact_section_uses_cached_replays(self):
        ref = self.root / 'ref/runA'
        ref.parent.mkdir()
        subprocess.check_call([str(self.root / 'bin/frozen'), '--bundle', str(self.root / 'bundles/scA/par'), '--out', str(ref)])
        b = self.root / 'exb/scA/runA'
        shutil.copytree(self.root / 'bundles/scA/par', b)
        (b / 'bundle.json').write_text(json.dumps({'diagnosticVariant': {'argv': ['--capture', str(ref)]}}))
        (self.root / 'list1.txt').write_text('exb/scA/runA\n')
        m = json.loads(self.manifest.read_text())
        m['exact']['lists'] = ['list1.txt']
        self.manifest.write_text(json.dumps(m))
        rc, s, out = self.gate(self.binary('new', 'ok', 1), sections='exact')
        self.assertEqual((rc, s['RS-0a']['verdict'], s['RS-0a']['all_fraction_within_1']), (0, 'VERIFIED', 1))
        rep = json.loads((out / 'exact/report1/report.json').read_text())
        self.assertEqual(rep['cases'][0]['replay_status'], 'reused')
        self.assertIsNotNone(rep['binary_sha256'])
        _, s, _ = self.gate(self.binary('new2', 'pose', 1), sections='exact')
        self.assertEqual(s['RS-0a']['verdict'], 'FAIL')
        _, s, out = self.gate(self.binary('new', 'ok', 1), tier='fast', sections='exact')
        self.assertEqual(s['RS-0a']['verdict'], 'VERIFIED')
        case = json.loads((out / 'exact/report1/report.json').read_text())['cases'][0]
        self.assertEqual(case['result']['frames_compared'], 3)


if __name__ == '__main__':
    unittest.main()
