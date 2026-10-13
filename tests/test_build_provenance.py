"""Build evidence must describe the compiled source tree, including frozen snapshots."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location(
    'build_provenance', Path(__file__).resolve().parents[1] / 'scripts/dev/build-provenance.py')
provenance = importlib.util.module_from_spec(spec)
spec.loader.exec_module(provenance)


class BuildProvenanceTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.live, self.frozen, self.build = [self.base / name for name in ['live', 'frozen', 'build']]
        for root in [self.live, self.frozen]:
            (root / 'src').mkdir(parents=True)
            (root / 'include').mkdir()
            (root / 'CMakeLists.txt').write_text('frozen project' if root == self.frozen else 'live sentinel')
            (root / 'src/core.cpp').write_text('frozen source' if root == self.frozen else 'live source sentinel')
            (root / 'include/core.h').write_text('frozen header' if root == self.frozen else 'live header sentinel')
        self.build.mkdir()
        self.artifact = self.build / 'native_replay'
        self.artifact.write_bytes(b'independent fixture executable')
        self.records = [dict(directory=str(self.build), file=str(self.frozen / 'src/core.cpp'),
                             command=f'c++ -DVARIANT={variant} -c {self.frozen / "src/core.cpp"}')
                        for variant in [1, 2]]
        unrelated = dict(directory=str(self.build), file=str(self.live / 'src/core.cpp'), command='unrelated live compile')
        (self.build / 'compile_commands.json').write_text(json.dumps(self.records + [unrelated]))
        self.cache(self.frozen)

    def cache(self, source):
        (self.build / 'CMakeCache.txt').write_text(f'CMAKE_HOME_DIRECTORY:INTERNAL={source}\n')

    def record(self, **kwargs):
        return provenance.record(self.build, self.artifact, **kwargs)

    def test_frozen_content_and_duplicate_compile_variants_ignore_live_changes(self):
        with patch.object(provenance.subprocess, 'check_output', side_effect=AssertionError('snapshot must not inherit surrounding Git')):
            before = self.record()
        (self.live / 'include/core.h').write_text('changed while snapshot build remains fixed')
        after = self.record()
        self.assertEqual(before['source_root'], str(self.frozen))
        self.assertEqual(before['source_files']['include/core.h'], hashlib.sha256(b'frozen header').hexdigest())
        self.assertEqual(before['source_manifest_sha256'], after['source_manifest_sha256'])
        self.assertEqual(before['core_compile_commands'], self.records)
        self.assertIsNone(before['head'])
        self.assertIsNone(before['dirty_diff_sha256'])

    def test_wasm_cmake_directory_resolves_parent_project(self):
        (self.frozen / 'wasm').mkdir()
        (self.frozen / 'wasm/CMakeLists.txt').write_text('wasm fixture')
        self.cache(self.frozen / 'wasm')
        self.assertEqual(self.record()['source_root'], str(self.frozen))

    def test_explicit_source_root_overrides_cache(self):
        self.cache(self.live)
        actual = self.record(source_root=self.frozen)
        self.assertEqual(actual['source_root'], str(self.frozen))
        self.assertEqual(actual['core_compile_commands'], self.records)
        self.assertEqual(actual['source_root_resolution'], 'explicit')

    def test_relative_compile_files_resolve_against_compile_directory(self):
        entries = [dict(entry, file='../frozen/src/core.cpp') for entry in self.records]
        (self.build / 'compile_commands.json').write_text(json.dumps(entries))
        self.assertEqual(self.record()['core_compile_commands'], entries)

    def test_missing_cmake_source_is_not_replaced_with_live_tree(self):
        self.cache(self.base / 'unavailable-snapshot')
        with self.assertRaises(FileNotFoundError):
            self.record()

    def test_external_native_ceres_archive_uses_actual_cache_path(self):
        archive = self.base / 'actual-dependency/lib/libceres.a'
        archive.parent.mkdir(parents=True)
        archive.write_bytes(b'actual linked dependency fixture')
        with (self.build / 'CMakeCache.txt').open('a') as stream:
            stream.write(f'Ceres_BINARY_DIR:STATIC={archive.parent.parent}\n')
        actual = self.record()['nativeCeresArchives']
        self.assertEqual(len(actual), 1)
        self.assertEqual(actual[0]['path'], str(archive))
        self.assertEqual(actual[0]['sha256'], hashlib.sha256(archive.read_bytes()).hexdigest())

    def test_explicit_imported_archive_is_recorded_without_claiming_local_build(self):
        archive = self.base / 'imported/libceres.a'
        archive.parent.mkdir()
        archive.write_bytes(b'explicit imported library')
        actual = self.record(native_ceres_archives=[archive])
        self.assertEqual(actual['nativeCeresArchives'][0]['path'], str(archive))
        self.assertEqual(actual['native_ceres_archive_resolution'], 'explicit')


if __name__ == '__main__':
    unittest.main()
