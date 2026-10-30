"""Unit tests of slam_refact/tools/deploy_xr.py (no browser, nothing written outside a temp dir)."""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('deploy_xr', ROOT / 'slam_refact/tools/deploy_xr.py')
dx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dx)


class RewriteTests(unittest.TestCase):
    def test_root_absolute_paths_become_relative_and_gate_goes_first(self):
        html = b'<head>\n<script src="/engine/xr.js" async></script>\n<script src="/samples/common/probe.js"></script></head>'
        text, n = dx.rewrite_sample('threejs/index.html', html)
        self.assertEqual(n, 3)
        self.assertLess(text.index('../../gate.js'), text.index('../../engine/xr.js'))
        self.assertIn('src="../../samples/common/probe.js"', text)
        self.assertFalse(dx.ROOT_ABSOLUTE.search(text))

    def test_js_import_is_rewritten_without_gate(self):
        text, n = dx.rewrite_sample('threejs/app.js', b"import * as THREE from '/three/three.module.min.js';")
        self.assertEqual((text, n), ("import * as THREE from '../../three/three.module.min.js';", 1))

    def test_entry_page_needs_its_anchors(self):
        phone = b'<meta charset="utf-8"><p id="start-note" class="note"></p>'
        page = dx.make_entry_page(phone, '<section id="xr-links"></section>\n')
        self.assertIn('<base href="slam-refact-xr/samples/phone/">', page)
        self.assertLess(page.index('start-note'), page.index('xr-links'))
        with self.assertRaises(SystemExit):
            dx.make_entry_page(b'<html></html>', 'x')
        with self.assertRaises(SystemExit):
            dx.make_entry_page(phone.replace(b'<meta', b'<base href="x"><meta'), 'x')


class AuthorizationTests(unittest.TestCase):
    def check(self, doc, name='a.json'):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / name
            p.write_text(json.dumps(doc))
            return dx.check_authorization(p, 'w' * 64, 'j' * 64, 'b' * 64)

    def good(self, **kw):
        return {'authorized': True, 'candidate_sha256': 'w' * 64, 'bundle_sha256': 'b' * 64, 'approved_by': 'user, chat, "..."', **kw}

    def test_accepts_matching_authorization(self):
        self.assertEqual(self.check(self.good())['approved_by'], 'user, chat, "..."')

    def test_refusals(self):
        for doc in ({**self.good(), 'authorized': False}, self.good(candidate_sha256='x'), self.good(bundle_sha256='x'), self.good(candidate_js_sha256='x'),
                    self.good(approved_by='FILL: user'), {k: v for k, v in self.good().items() if k != 'approved_by'}):
            with self.assertRaises(SystemExit, msg=doc):
                self.check(doc)

    def test_legacy_authorization_path_refused(self):
        with self.assertRaises(SystemExit):
            dx.check_authorization(dx.LEGACY_AUTHORIZATION, 'w', 'j', 'b')


class BundleTests(unittest.TestCase):
    def test_engine_input_manifest_private_fields_are_not_copied_into_public_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            engine = Path(directory)
            artifacts = {}
            for key, data in [('js', b'fixture js'), ('wasm', b'fixture wasm')]:
                name = 'slam_refact_engine.' + key
                (engine / name).write_bytes(data)
                artifacts[key] = {'file': name, 'sha256': dx.sha256_bytes(data), 'bytes': len(data),
                                  'source': '/home/example/private-input'}
            (engine / 'manifest.json').write_text(json.dumps({'schema': dx.ENGINE_MANIFEST_SCHEMA,
                'artifacts': artifacts, 'build': {'source': '/home/example/build'}}))
            _, blobs, original = dx.read_engine(engine)
            self.assertIn('build', original)
            public = json.loads(blobs['manifest.json'])
            self.assertEqual(public['record_kind'], 'source-artifact-index')
            self.assertNotIn('build', public)
            self.assertEqual(set(public['artifacts']['js']), {'file', 'bytes', 'sha256'})
            self.assertNotIn('/home/', json.dumps(public))

    def test_public_record_excludes_operator_and_workstation_details(self):
        private = {'engine': {'js_sha256': 'j' * 64, 'wasm_sha256': 'w' * 64, 'wasm_bytes': 10,
                              'dir': '/home/example/build', 'build_provenance': {'file': 'private'}},
                   'xr_js': {'sha256': 'x' * 64, 'bytes': 20, 'source': '/home/example/sdk'},
                   'bundle_sha256': 'b' * 64, 'bundle_digest_rule': 'rule',
                   'files': [{'path': 'slam-refact-xr/gate.js', 'bytes': 1, 'sha256': 'f' * 64,
                              'source': '/home/example/source', 'transform': 'private note'}],
                   'authorization': {'approved_by': 'private operator'}, 'git': {'branch': 'private'}}
        record = dx.public_artifact_record(private)
        self.assertEqual(record['record_kind'], 'source-artifact-index')
        self.assertEqual(set(record['engine']), {'js_sha256', 'wasm_sha256', 'wasm_bytes'})
        self.assertEqual(set(record['files'][0]), {'path', 'bytes', 'sha256'})
        for term in ('/home/', 'authorization', 'approved_by', 'private', 'transform'):
            self.assertNotIn(term, json.dumps(record))

    def test_digest_ignores_deploy_json_and_is_order_independent(self):
        a, b = dx.Bundle(), dx.Bundle()
        x, y, z = 'slam-refact-xr/gate.js', 'slam-refact-xr/engine/xr.js', 'slam-refact-xr/engine/sensors.js'
        a.add(x, b'1', 's'); a.add(y, b'2', 's'); a.add('slam-refact-xr/deploy.json', b'one', 's')
        b.add('slam-refact-xr/deploy.json', b'two', 's'); b.add(y, b'2', 's'); b.add(x, b'1', 's')
        self.assertEqual(a.digest(), b.digest())
        b.add(z, b'3', 's')
        self.assertNotEqual(a.digest(), b.digest())

    def test_real_inputs_build_a_closed_tree_without_root_absolute_urls(self):
        args = type('A', (), dict(chunk_dir=ROOT / 'slam_refact/web-chunk', samples_dir=ROOT / 'slam_refact/web-chunk/samples', xr_src_dir=ROOT / 'slam_refact/web-xr',
                                  engine_dir=dx.DEFAULT_ENGINE_DIR, sdk_dir=dx.DEFAULT_SDK_DIR, three_dir=dx.DEFAULT_THREE_DIR, aframe_dir=dx.DEFAULT_AFRAME_DIR))()
        if not (args.engine_dir / 'manifest.json').is_file():
            self.skipTest('engine pin not present')
        b, _, blobs, _ = dx.build_bundle(args)
        engine_manifest = json.loads(blobs['manifest.json'])
        self.assertEqual(engine_manifest['record_kind'], 'source-artifact-index')
        self.assertNotIn('build', engine_manifest)
        names = set(b.files)
        self.assertIn('slam-refact-xr.html', names)
        self.assertIn('slam-refact-xr/engine/xr.js', names)
        self.assertIn('slam-refact-xr/engine/slam-refact-engine/slam_refact_engine.wasm', names)
        self.assertTrue(all(n == 'slam-refact-xr.html' or n.startswith('slam-refact-xr/') for n in names))
        self.assertFalse([n for n in names if '/.' in n or n.endswith(('.map', '.bak', '.sha256', '.py', '.mjs'))])
        self.assertNotIn('slam-refact-xr/samples/phone/index.html', names)


class ServedPathTests(unittest.TestCase):
    BAD = [
        'slam-refact-xr/samples/phone/targets/../../../../ESCAPED.txt', 'slam-refact-xr/../vio_engine.js', '../vio_engine.js', '/etc/passwd',
        '/slam-refact-xr/gate.js', 'slam-refact-xr\\gate.js', 'slam-refact-xr/engine//xr.js', 'slam-refact-xr/./gate.js', 'slam-refact-xr/engine/../gate.js',
        'slam-refact-xr/.hidden.js', 'slam-refact-xr/gate.js/', '', 'vio_engine.js', 'vio_engine.manifest.json', 'vio_engine.wasm',
        'slam-refact-xr/samples/common/vio_engine.js', 'slam-refact-xr/samples/phone/targets/vio_engine.manifest.json', 'slam-refact.html',
        'slam-refact-xr/unlisted.js', 'slam-refact-xr/samples/other/app.js', 'slam-refact-xr/gate.js%2e', 'slam-refact-xr/gate.js\n', 'slam-refact-xr/gate.js\x00',
    ]

    def test_bad_paths_refused_by_check_and_by_add(self):
        for served in self.BAD:
            with self.assertRaises(SystemExit, msg=repr(served)):
                dx.check_served(served)
            with self.assertRaises(SystemExit, msg=repr(served)):
                dx.Bundle().add(served, b'x', 's')

    def test_good_paths_accepted(self):
        for served in ('slam-refact-xr.html', 'slam-refact-xr/deploy.json', 'slam-refact-xr/engine/slam-refact-engine/slam_refact_engine.wasm',
                       'slam-refact-xr/samples/phone/app.js', 'slam-refact-xr/samples/phone/targets/print/sheet-1.pdf', 'slam-refact-xr/samples/phone/targets/manifest.json'):
            self.assertEqual(dx.check_served(served), served)

    def test_fixed_set_matches_server_allowlist(self):
        server = (ROOT / 'web/server.js').read_text()
        block = server[server.index('const SLAM_REFACT_XR_FILES'):server.index('const PRIVATE_NAMES')]
        for name in dx.SERVED_FIXED:
            tail = name.rsplit('/', 1)[-1]
            self.assertIn(tail, block, name)

    def test_write_tree_cannot_escape_root(self):
        b = dx.Bundle()
        b.files['../ESCAPED.txt'] = (b'x', 's', None)  # bypass add() to prove write_tree checks again
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(SystemExit):
                dx.write_tree(b, Path(d) / 'new')
            self.assertFalse((Path(d) / 'ESCAPED.txt').exists())

    def test_vio_engine_never_written_by_execute(self):
        with tempfile.TemporaryDirectory() as d:
            web = Path(d) / 'web'
            web.mkdir()
            (web / 'vio_engine.js').write_text('original')
            b = dx.Bundle()
            b.files['slam-refact-xr/samples/phone/targets/../../../../vio_engine.js'] = (b'evil', 's', None)  # bypass add()
            args = type('A', (), dict(web_root=web, authorization=Path(d) / 'a.json', evidence_dir=Path(d) / 'ev'))()
            with self.assertRaises(SystemExit):
                dx.execute(args, b, lambda m, a: {}, 'w', 'j')
            self.assertEqual((web / 'vio_engine.js').read_text(), 'original')
            self.assertEqual(sorted(p.name for p in web.iterdir()), ['vio_engine.js'])


class ExecuteGuardTests(unittest.TestCase):
    def setUp(self):
        self._orig = dx.input_paths_dirty

    def tearDown(self):
        dx.input_paths_dirty = self._orig

    def run_execute(self, dirty, symlink=False):
        dx.input_paths_dirty = lambda: dirty
        with tempfile.TemporaryDirectory() as d:
            web = Path(d) / 'web'
            web.mkdir()
            if symlink:
                (web / 'slam-refact-xr').symlink_to(Path(d))
            b = dx.Bundle()
            b.add('slam-refact-xr.html', b'<html></html>', 's')
            b.add('slam-refact-xr/gate.js', b'//', 's')
            auth = Path(d) / 'a.json'
            auth.write_text('{}')
            args = type('A', (), dict(web_root=web, authorization=auth, evidence_dir=Path(d) / 'ev'))()
            try:
                dx.execute(args, b, lambda m, a: {}, 'w', 'j')
            finally:
                self.assertEqual(sorted(p.name for p in web.iterdir()), ['slam-refact-xr'] if symlink else [])
                self.assertFalse((Path(d) / 'ev').exists())

    def test_execute_refuses_dirty_inputs(self):
        with self.assertRaises(SystemExit) as cm:
            self.run_execute([' M slam_refact/engine/CMakeLists.txt'])
        self.assertIn('git-dirty', str(cm.exception))

    def test_execute_refuses_symlink_destination(self):
        with self.assertRaises(SystemExit) as cm:
            self.run_execute([], symlink=True)
        self.assertIn('symlink', str(cm.exception))

    def test_clean_inputs_pass_the_dirty_gate(self):
        # clean inputs get past the dirty gate and stop later at the (invalid) authorization, before anything is written
        with self.assertRaises(SystemExit) as cm:
            self.run_execute([])
        self.assertIn('authoriz', str(cm.exception))


if __name__ == '__main__':
    unittest.main()
