"""Bounded synthetic staging contracts; never read or extract the NAS datasets."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tarfile
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


class StagingFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.inventory = self.root / 'docs/audit/2026-10-06/datasets-inventory.json'
        self.inventory.parent.mkdir(parents=True)
        self.staging = self.root / 'build/refactor-data/tum'
        self.staging.mkdir(parents=True)
        self.entries = {}
        spec = importlib.util.spec_from_file_location('stage_tum_fixture', ROOT / 'scripts/dev/stage-tum-room4.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.module.ROOT = self.root
        self.fixture()

    def fixture(self, sequence='room4', rows=None, png=None, extras=(), missing_image=False):
        dataset = f'dataset-{sequence}_512_16'
        archive = self.root / f'{dataset}.tar'
        camera_rows = rows or [('1', '1.png'), ('2', '2.png')]
        png = png or b'\x89PNG\r\n\x1a\n' + b'\0' * 8 + struct.pack('>II', 512, 512)
        files, csv_entries = {}, {}
        for sensor in ('cam0', 'cam1', 'imu0', 'mocap0'):
            records = camera_rows if sensor.startswith('cam') else [('1', '0'), ('2', '0')]
            data = ('#timestamp,data\n' + ''.join(','.join(row) + '\n' for row in records)).encode()
            member = f'{dataset}/mav0/{sensor}/data.csv'
            files[member] = data
            csv_entries[sensor] = {'member': member, 'sha256': hashlib.sha256(data).hexdigest(), 'rows': len(records)}
            if sensor.startswith('cam'):
                for _, name in records:
                    if '/' not in name and not (missing_image and sensor == 'cam0' and name == '1.png'):
                        files[f'{dataset}/mav0/{sensor}/data/{name}'] = png
        yaml_member = f'{dataset}/dso/camchain.yaml'
        files[yaml_member] = b'camera: fixture\n'
        with tarfile.open(archive, 'w:') as output:
            for name, data in files.items():
                member = tarfile.TarInfo(name)
                member.size = len(data)
                output.addfile(member, io.BytesIO(data))
            for member, data in extras:
                output.addfile(member, io.BytesIO(data) if data is not None else None)
        entry = {'archive': str(archive), 'archive_root': dataset, 'csv': csv_entries,
                 'yaml': [{'member': yaml_member, 'sha256': hashlib.sha256(files[yaml_member]).hexdigest()}]}
        self.entries[sequence] = entry
        self.write_inventory()
        return archive, entry

    def write_inventory(self):
        self.inventory.write_text(json.dumps({'tum_vi': list(self.entries.values())}))

    def run_stage(self, *arguments):
        output = io.StringIO()
        with mock.patch.object(sys, 'argv', ['stage-tum-room4.py', *map(str, arguments)]), contextlib.redirect_stdout(output):
            self.module.main()
        return json.loads(output.getvalue())

    def assert_rejected(self, *arguments, error=ValueError):
        with self.assertRaises(error):
            self.run_stage(*arguments)
        self.assertFalse((self.staging / 'dataset-room4_512_16').exists())
        self.assertFalse(list(self.staging.glob('.*-stage-*')))


class StagingRegressionTest(StagingFixture):
    def test_default_room4_path_split_and_read_only_source(self):
        source = Path(self.entries['room4']['archive'])
        before = source.read_bytes(), source.stat().st_mtime_ns
        report = self.run_stage()
        destination = self.staging / 'dataset-room4_512_16'
        self.assertEqual(report['destination'], str(destination))
        self.assertEqual(report['split'], 'selection_validation_not_locked_final')
        self.assertTrue(report['source_read_only'])
        self.assertEqual(report['verified_csv']['cam0']['records'], 2)
        self.assertEqual(json.loads((destination / 'staging-manifest.json').read_text()), report)
        self.assertEqual(before, (source.read_bytes(), source.stat().st_mtime_ns))

    def test_existing_explicit_destination_cli(self):
        destination = self.root / 'build/custom-room4'
        self.assertEqual(self.run_stage('--destination', destination)['destination'], str(destination))

    def test_existing_destination_is_preserved(self):
        destination = self.staging / 'dataset-room4_512_16'
        destination.mkdir()
        sentinel = destination / 'sentinel'
        sentinel.write_bytes(b'keep')
        with self.assertRaises(FileExistsError):
            self.run_stage()
        self.assertEqual(sentinel.read_bytes(), b'keep')

    def test_outside_and_symlink_destinations_are_rejected(self):
        with self.assertRaises(ValueError):
            self.run_stage('--destination', self.root / 'outside')
        link = self.root / 'build/outside-link'
        link.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.run_stage('--destination', link / 'new-stage')
        self.assertFalse((self.root / 'new-stage').exists())

    def test_archive_traversal_absolute_and_wrong_root_are_rejected(self):
        for name in ('../escape', '/absolute', 'dataset-room4_512_16/../escape', 'other-root/file'):
            with self.subTest(name=name):
                member = tarfile.TarInfo(name)
                member.size = 1
                self.fixture(extras=[(member, b'x')])
                self.assert_rejected()
        self.assertFalse((self.root / 'escape').exists())

    def test_duplicate_and_hardlink_members_are_rejected(self):
        member = tarfile.TarInfo('dataset-room4_512_16/mav0/cam0/data.csv')
        member.size = 1
        self.fixture(extras=[(member, b'x')])
        self.assert_rejected()
        member = tarfile.TarInfo('dataset-room4_512_16/hardlink')
        member.type = tarfile.LNKTYPE
        member.linkname = 'dataset-room4_512_16/mav0/cam0/data.csv'
        self.fixture(extras=[(member, None)])
        self.assert_rejected()

    def test_symlink_alias_is_recorded_without_extraction(self):
        member = tarfile.TarInfo('dataset-room4_512_16/dso/images')
        member.type = tarfile.SYMTYPE
        member.linkname = '/outside/private'
        self.fixture(extras=[(member, None)])
        report = self.run_stage()
        self.assertEqual(report['omitted_symlinks'][0]['target'], '/outside/private')
        self.assertFalse((Path(report['destination']) / 'dso/images').exists())

    def test_csv_hash_count_and_order_are_required(self):
        for field in ('hash', 'count', 'order'):
            with self.subTest(field=field):
                _, entry = self.fixture(rows=[('2', '2.png'), ('1', '1.png')] if field == 'order' else None)
                if field == 'hash':
                    entry['csv']['cam0']['sha256'] = '0' * 64
                if field == 'count':
                    entry['csv']['cam0']['rows'] = 3
                self.write_inventory()
                self.assert_rejected()

    def test_csv_image_name_cannot_escape(self):
        self.fixture(rows=[('1', '../escape.png'), ('2', '2.png')])
        self.assert_rejected()

    def test_png_signature_and_resolution_are_required(self):
        for png in (b'not-a-png' + b'\0' * 16,
                    b'\x89PNG\r\n\x1a\n' + b'\0' * 8 + struct.pack('>II', 640, 512)):
            with self.subTest(png=png):
                self.fixture(png=png)
                self.assert_rejected()

    def test_missing_referenced_image_is_rejected(self):
        self.fixture(missing_image=True)
        self.assert_rejected(error=FileNotFoundError)

    def test_calibration_hash_is_required(self):
        self.entries['room4']['yaml'][0]['sha256'] = '0' * 64
        self.write_inventory()
        self.assert_rejected()

    def test_archive_byte_and_member_budgets_are_required(self):
        # Mock only tar metadata: no 5 GiB file or 30,001 filesystem writes.
        member = tarfile.TarInfo('dataset-room4_512_16/directory')
        member.type = tarfile.DIRTYPE
        member.size = 5 * 1024 ** 3 + 1
        source = mock.MagicMock()
        source.__enter__.return_value = [member]
        with mock.patch.object(self.module.tarfile, 'open', return_value=source):
            self.assert_rejected()
        member.size = 0
        source.__enter__.return_value = [member] * 30001
        with mock.patch.object(self.module.tarfile, 'open', return_value=source), mock.patch.object(Path, 'mkdir'):
            self.assert_rejected()


class SequenceSelectionTest(StagingFixture):
    def test_sequence_choices_have_dynamic_default_destination(self):
        for sequence in ('room1', 'room2', 'room3', 'room4'):
            with self.subTest(sequence=sequence):
                archive, _ = self.fixture(sequence)
                report = self.run_stage('--sequence', sequence)
                self.assertEqual(report['source'], str(archive))
                self.assertEqual(report['destination'], str(self.staging / f'dataset-{sequence}_512_16'))
                self.assertEqual(report['split'], 'locked_final_confirmation' if sequence == 'room2' else 'selection_validation_not_locked_final')

    def test_room2_explicit_destination_and_inventory(self):
        archive, _ = self.fixture('room2')
        destination = self.root / 'build/locked-room2'
        report = self.run_stage('--sequence', 'room2', '--inventory', self.inventory, '--destination', destination)
        self.assertEqual(report['source'], str(archive))
        self.assertEqual(report['destination'], str(destination))
        self.assertEqual(report['split'], 'locked_final_confirmation')

    def test_argparse_rejects_unknown_sequence_without_staging(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            self.run_stage('--sequence', 'room5')
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(list(self.staging.iterdir()))

    def test_inventory_root_must_match_selected_sequence(self):
        _, entry = self.fixture('room2')
        entry['archive_root'] = 'dataset-room4_512_16'
        self.write_inventory()
        with self.assertRaisesRegex(ValueError, 'selected sequence'):
            self.run_stage('--sequence', 'room2')
        self.assertFalse(list(self.staging.iterdir()))

    def test_help_lists_all_sequence_choices(self):
        output = io.StringIO()
        with mock.patch.object(sys, 'argv', ['stage-tum-room4.py', '--help']), contextlib.redirect_stdout(output), self.assertRaises(SystemExit) as error:
            self.module.main()
        self.assertEqual(error.exception.code, 0)
        self.assertIn('--sequence {room1,room2,room3,room4}', output.getvalue())


if __name__ == '__main__':
    unittest.main()
