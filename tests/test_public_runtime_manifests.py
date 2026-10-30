"""Public runtime metadata must verify artifacts without exposing workstation records."""
import hashlib
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
PRIVATE_KEYS = {'authorization', 'approved_by', 'source_root', 'build_directory', 'core_compile_commands',
                'git', 'build', 'deployed_at', 'generated_at', 'historical_receipt', 'source_snapshot_path'}


def load(name):
    return json.loads((ROOT / name).read_text())


class PublicRuntimeManifestsTest(unittest.TestCase):
    def assert_public(self, value):
        if isinstance(value, dict):
            self.assertFalse(PRIVATE_KEYS.intersection(value))
            for item in value.values():
                self.assert_public(item)
        elif isinstance(value, list):
            for item in value:
                self.assert_public(item)
        elif isinstance(value, str):
            self.assertNotIn('/home/', value)
            self.assertNotIn('/Users/', value)
            self.assertFalse(value.startswith('build/'))

    def assert_artifact(self, base, entry):
        data = (ROOT / base / entry['file']).read_bytes()
        self.assertEqual(entry['sha256'], hashlib.sha256(data).hexdigest())
        self.assertEqual(entry['bytes'], len(data))

    def test_vio_manifest_matches_the_actual_embedded_artifact(self):
        manifest = load('web/vio_engine.manifest.json')
        self.assertEqual(manifest['record_kind'], 'source-artifact-index')
        self.assertTrue(manifest['single_file_embedded_wasm'])
        self.assert_artifact('web', manifest['artifact'])
        self.assert_public(manifest)

    def test_refact_manifest_pairs_keep_runtime_schemas_and_hashes(self):
        for base, schema in [('web/slam-refact', 'slam-refact-web-deploy-v1'),
                             ('web/slam-refact-xr/engine/slam-refact-engine', 'slam-refact-chunk-engine-v1')]:
            with self.subTest(base=base):
                manifest = load(base + '/manifest.json')
                self.assertEqual(manifest['schema'], schema)
                self.assertEqual(manifest['record_kind'], 'source-artifact-index')
                for entry in manifest['artifacts'].values():
                    self.assert_artifact(base, entry)
                self.assert_public(manifest)

    def test_xr_inventory_is_public_and_exact(self):
        public = load('web/slam-refact-xr/deploy.json')
        inventory = load('config/runtime-integrity/xr-tree.json')
        self.assertEqual(public['record_kind'], 'source-artifact-index')
        self.assertEqual(inventory['schema'], 'slam-refact-runtime-integrity-v1')
        for record in (public, inventory):
            self.assert_public(record)
        actual = {'slam-refact-xr.html'} | {
            p.relative_to(ROOT / 'web').as_posix() for p in (ROOT / 'web/slam-refact-xr').rglob('*')
            if p.is_file() and p.name != 'deploy.json'}
        self.assertEqual({r['path'] for r in public['files']}, actual)
        self.assertEqual({r['path'] for r in inventory['files']}, actual)
        digest = hashlib.sha256()
        for record in sorted(public['files'], key=lambda row: row['path']):
            content = (ROOT / 'web' / record['path']).read_bytes()
            value = hashlib.sha256(content).hexdigest()
            self.assertEqual(record['bytes'], len(content))
            self.assertEqual(record['sha256'], value)
            digest.update(record['path'].encode() + b'\0' + value.encode() + b'\n')
        self.assertEqual(public['bundle_sha256'], digest.hexdigest())
        self.assertEqual(inventory['bundle_sha256'], digest.hexdigest())

    def test_vio_promotion_separates_public_index_from_local_operator_record(self):
        spec = importlib.util.spec_from_file_location('candidate_privacy_fixture', ROOT / 'scripts/dev/deploy-candidate.py')
        tool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tool)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'web').mkdir()
            candidate = root / 'candidate.js'
            candidate.write_bytes(b'fixture embedded module')
            digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
            original = {'artifact': {'sha256': digest}, 'single_file_embedded_wasm': True,
                        'source_root': '/home/example/private', 'core_compile_commands': ['private compiler path']}
            manifest, authorization = root / 'build.json', root / 'authorization.json'
            manifest.write_text(json.dumps(original))
            authorization.write_text(json.dumps({'authorized': True, 'candidate_sha256': digest}))
            args = ['deploy-candidate.py', '--candidate', str(candidate), '--manifest', str(manifest),
                    '--authorization', str(authorization)]
            with mock.patch.object(tool, 'ROOT', root), mock.patch.object(sys, 'argv', args), contextlib.redirect_stdout(io.StringIO()):
                tool.main()
            public = json.loads((root / 'web/vio_engine.manifest.json').read_text())
            self.assertEqual(public['record_kind'], 'source-artifact-index')
            self.assertEqual(public['artifact']['sha256'], digest)
            self.assert_public(public)
            private = list((root / 'build/refactor-deploy').glob('*/build-manifest.private.json'))
            self.assertEqual(len(private), 1)
            self.assertEqual(json.loads(private[0].read_text())['source_root'], original['source_root'])


if __name__ == '__main__':
    unittest.main()
