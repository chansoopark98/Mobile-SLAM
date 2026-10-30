"""slam_refact/tools/deploy_web.py: authorization gate, destination guard, manifest contents."""
import contextlib
import hashlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('deploy_web', ROOT / 'slam_refact/tools/deploy_web.py')
deploy_web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deploy_web)


class DeployWebTest(unittest.TestCase):
    def setUp(self):
        source = mock.patch.object(deploy_web, 'source_provenance', return_value=(
            {'head': '1' * 40, 'engine_dirty': False}, {'engine_sources_sha256': '2' * 64}))
        source.start()
        self.addCleanup(source.stop)
        self.tmp = tempfile.TemporaryDirectory()
        t = Path(self.tmp.name)
        self.build = t / 'build/wasm'
        self.build.mkdir(parents=True)
        (self.build / 'slam_refact_engine.js').write_bytes(b'var SlamRefactEngine;')
        (self.build / 'slam_refact_engine.wasm').write_bytes(b'\0asm-fixture')
        self.web = t / 'web'
        self.web.mkdir()
        (self.web / 'vio_engine.js').write_bytes(b'VINS')
        self.wasm_sha = hashlib.sha256(b'\0asm-fixture').hexdigest()
        self.js_sha = hashlib.sha256(b'var SlamRefactEngine;').hexdigest()

    def tearDown(self):
        self.tmp.cleanup()

    def auth(self, **data):
        p = Path(self.tmp.name) / f'auth-{len(list(Path(self.tmp.name).glob("auth-*")))}.json'
        p.write_text(json.dumps(data))
        return p

    def run_deploy(self, auth, *extra):
        with contextlib.redirect_stdout(io.StringIO()):
            return deploy_web.main(['--authorization', str(auth), '--build-dir', str(self.build),
                                    '--web-root', str(self.web), *extra])

    def test_refuses_without_matching_authorization(self):
        for data in ({'authorized': False, 'candidate_sha256': self.wasm_sha},
                     {'authorized': True, 'candidate_sha256': '0' * 64},
                     {'authorized': True, 'candidate_sha256': self.wasm_sha, 'candidate_js_sha256': '0' * 64}):
            with self.assertRaises(SystemExit):
                self.run_deploy(self.auth(**data))
        self.assertFalse((self.web / 'slam-refact').exists())

    def test_refuses_legacy_vins_authorization_path(self):
        with self.assertRaises(SystemExit) as cm:
            deploy_web.check_authorization(deploy_web.LEGACY_AUTHORIZATION, {'wasm': self.wasm_sha, 'js': self.js_sha})
        self.assertIn('historical', str(cm.exception))

    def test_destination_must_be_web_slam_refact(self):
        with self.assertRaises(SystemExit):
            deploy_web.check_destination(self.web, self.web)
        with self.assertRaises(SystemExit):
            deploy_web.check_destination(self.web / 'other', self.web)

    def test_deploys_artifacts_and_manifest_without_touching_vio_engine(self):
        evidence = Path(self.tmp.name) / 'evidence'
        auth = self.auth(authorized=True, candidate_sha256=self.wasm_sha, candidate_js_sha256=self.js_sha,
                         approved_by='user (test)', reason='test')
        self.assertEqual(self.run_deploy(auth, '--evidence-dir', str(evidence)), 0)
        dest = self.web / 'slam-refact'
        self.assertEqual((dest / 'slam_refact_engine.wasm').read_bytes(), b'\0asm-fixture')
        m = json.loads((dest / 'manifest.json').read_text())
        self.assertEqual(m['artifacts']['wasm']['sha256'], self.wasm_sha)
        self.assertEqual(m['artifacts']['js']['sha256'], self.js_sha)
        self.assertEqual(m['record_kind'], 'source-artifact-index')
        self.assertNotIn('git', m)
        self.assertNotIn('build', m)
        self.assertNotIn('authorization', m)
        private = json.loads((evidence / 'deployment-record.private.json').read_text())
        self.assertEqual(private['git']['head'], '1' * 40)
        self.assertEqual(private['authorization']['approved_by'], 'user (test)')
        self.assertEqual((evidence / 'manifest.json').read_text(), (dest / 'manifest.json').read_text())
        self.assertEqual((self.web / 'vio_engine.js').read_bytes(), b'VINS')
        self.assertEqual(sorted(p.name for p in self.web.iterdir()), ['slam-refact', 'vio_engine.js'])
        # A second deploy keeps a backup of the replaced files and refuses a reused evidence dir.
        with self.assertRaises(SystemExit):
            self.run_deploy(auth, '--evidence-dir', str(evidence))
        self.assertEqual(self.run_deploy(auth, '--evidence-dir', str(evidence) + '-2'), 0)
        self.assertTrue((Path(str(evidence) + '-2') / 'replaced/manifest.json').is_file())


if __name__ == '__main__':
    unittest.main()
