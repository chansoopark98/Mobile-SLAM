#!/usr/bin/env python3
"""Synthetic tests of make_variant_v2.py (fail-closed monotonic check, optional --tail-shift) against the pinned make_variant.py.

  python3 -m unittest slam_refact/tools/harness/divergence/test_make_variant_v2.py -v

The fixture is a small prepare_inputs-style base bundle (bootstrap + 12 frames every 40 ms, run sensor rows every 10 ms, row j carries the
unique orientation (j, j / 2, j / 4)) and a capture_window-style meta.json whose stageCalls cover frames 0..LAST (the capture run ended
there). `period_ms` is the capture run's frame period: 40 = as fast as the base, 100 = a slow original (the re-timed last staged frame ends
after the base time of the first tail frame).
"""
import gzip
import importlib.util
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
V1 = HERE / 'make_variant.py'
V2 = HERE / 'make_variant_v2.py'
N_FRAMES, LAST = 12, 4
FRAME_NS0 = 1_002_955_000

REPO = HERE.parents[3]
sys.path.insert(0, str(REPO / 'scripts/dev'))
from reference_metadata import load_reference_metadata


class CaptureSchemaCompatibilityTest(unittest.TestCase):
    def setUp(self):
        self.tools = []
        for path in (V1, V2):
            spec = importlib.util.spec_from_file_location(path.stem, path)
            tool = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(tool)
            self.tools.append(tool)

    def load(self, tool, header):
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp)
            with gzip.open(path / 'recorded-inputs.jsonl.gz', 'wt') as f:
                f.write(json.dumps(header) + '\n')
            return tool.load_capture_meta(path)

    def header(self, schema):
        return {'schema': schema, 'wrapped': True, 'droppedCalls': 0,
                'epochWallMs': 123, 'publications': [{'i': 4}]}

    def test_current_and_original_headers_keep_metadata_unchanged(self):
        current = 'slam-refact-xr_reference-recorded-inputs-v1'
        legacy = f"slam-refact-{load_reference_metadata(REPO)['main_repository_name']}-recorded-inputs-v1"
        for tool in self.tools:
            for schema in (current, legacy):
                with self.subTest(tool=tool.__name__, schema=schema):
                    header = self.header(schema)
                    meta, actual_header = self.load(tool, header)
                    self.assertEqual(actual_header, header)
                    self.assertEqual(meta, {'probe': {'epochWallMs': 123, 'frames': [{'i': 4}]},
                                            'stageCalls': [], 'captures': []})

    def test_other_versions_families_and_schema_types_are_rejected(self):
        current = 'slam-refact-xr_reference-recorded-inputs-v1'
        legacy = f"slam-refact-{load_reference_metadata(REPO)['main_repository_name']}-recorded-inputs-v1"
        invalid = [current[:-1] + '2', legacy[:-1] + '2', 'other-recorded-inputs-v1', None, [], {}, 1]
        for tool in self.tools:
            for schema in invalid:
                with self.subTest(tool=tool.__name__, schema=schema), self.assertRaises(SystemExit):
                    self.load(tool, self.header(schema))

    def test_wrapped_and_dropped_call_checks_stay_strict(self):
        current = 'slam-refact-xr_reference-recorded-inputs-v1'
        for tool in self.tools:
            for bad in ({'wrapped': False}, {'droppedCalls': 1}):
                with self.subTest(tool=tool.__name__, bad=bad), self.assertRaises(SystemExit):
                    self.load(tool, {**self.header(current), **bad})


def orient(j):
    return (float(j), j / 2.0, j / 4.0)


def make_base(d):
    d.mkdir(parents=True)
    rows = ['bootstrap\t986955000\t0\t0\t0\t-\t-\t-\t0\t0\t0\t24.03\t%r\t%r\t%r' % orient(0)]
    for j in range(N_FRAMES * 4 + 10):
        rows.append('run\t%d\t0\t0\t0\t-\t-\t-\t0\t0\t0\t24.03\t%r\t%r\t%r' % ((1_000_000_000 + 10_000_000 * j,) + orient(j)))
    (d / 'sensors.tsv').write_text('\n'.join(rows) + '\n')
    fr = ['bootstrap\t-1\t986955000\t986955000\t-0.013\t1\timg0\t-']
    for k in range(N_FRAMES):
        t = FRAME_NS0 + 40_000_000 * k
        fr.append('frame\t%d\t%d\t%d\t%r\t1\timg%d\t-' % (k, t, t, t / 1e9 - 1.0, k))
    (d / 'frames.tsv').write_text('\n'.join(fr) + '\n')
    (d / 'engine-config.tsv').write_text('scenario\ts\ndrop_mode\treference:x\npyramid_injection\tnone\n')
    (d / 'bundle.json').write_text(json.dumps({'schema': 'slam-refact-replay-bundle-v1', 'argv': ['prepare_inputs.py']}, indent=1) + '\n')


def make_capture(d, period_ms):
    """stageCalls for frames 0..LAST; frame k binds row 4k + 2; args[13] = timeNanos so that T0 + (ns/1e6 - epoch) ms = 1 s + period * k + 3 ms."""
    d.mkdir(parents=True)
    stage = []
    for k in range(LAST + 1):
        args = [0] * 14
        args[4:7] = list(orient(4 * k + 2))
        args[12] = (period_ms * k + 3) / 1000.0
        args[13] = (period_ms * k + 3) * 1e6
        stage.append({'call': k, 'args': args, 'inputFrame': k, 'queue': None})
    (d / 'meta.json').write_text(json.dumps({'probe': {'epochWallMs': 0.0, 'frames': []}, 'stageCalls': stage, 'captures': []}))


def run(script, base, out, cap, *extra):
    cmd = [sys.executable, '-I', '-B', str(script), '--base', str(base), '--out', str(out), '--capture', str(cap),
           '--capture-binding', '--capture-wallclock'] + list(extra)
    return subprocess.run(cmd, capture_output=True, text=True)


def read_bundle(d):
    return {n: (d / n).read_text() for n in ('frames.tsv', 'sensors.tsv', 'engine-config.tsv')}


def frame_times(d):
    return [int(l.split('\t')[2]) for l in (d / 'frames.tsv').read_text().splitlines()]


def run_times(d):
    return [int(l.split('\t')[1]) for l in (d / 'sensors.tsv').read_text().splitlines() if l.startswith('run\t')]


def diag(d):
    b = json.loads((d / 'bundle.json').read_text())['diagnosticVariant']
    b.pop('argv')
    return b


class MakeVariantV2(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = pathlib.Path(self._tmp.name)
        self.base = self.tmp / 'base'
        make_base(self.base)
        self.fast, self.slow = self.tmp / 'cap-fast', self.tmp / 'cap-slow'
        make_capture(self.fast, 40)
        make_capture(self.slow, 100)

    def tearDown(self):
        self._tmp.cleanup()

    def test_normal_input_is_byte_identical_to_v1(self):
        o1, o2, o3 = self.tmp / 'o1', self.tmp / 'o2', self.tmp / 'o3'
        r1 = run(V1, self.base, o1, self.fast)
        self.assertEqual(r1.returncode, 0, r1.stderr)
        r2 = run(V2, self.base, o2, self.fast)
        self.assertEqual(r2.returncode, 0, r2.stderr)
        self.assertEqual(read_bundle(o1), read_bundle(o2))
        self.assertEqual(diag(o1), diag(o2))                 # bundle.json differs only by the recorded argv (script name, --out)
        self.assertNotIn('tailShift', diag(o2))
        # --tail-shift with delta == 0 changes nothing either (and adds no report key)
        r3 = run(V2, self.base, o3, self.fast, '--tail-shift')
        self.assertEqual(r3.returncode, 0, r3.stderr)
        self.assertEqual(read_bundle(o1), read_bundle(o3))
        self.assertNotIn('tailShift', diag(o3))
        t = frame_times(o2)
        self.assertTrue(all(b > a for a, b in zip(t, t[1:])))

    def test_v1_writes_a_non_monotonic_bundle_for_a_slow_original(self):
        """The defect: the parent tool exits 0 and writes decreasing frame times."""
        o1 = self.tmp / 'o1'
        r = run(V1, self.base, o1, self.slow)
        self.assertEqual(r.returncode, 0, r.stderr)
        t = frame_times(o1)
        self.assertTrue(any(b <= a for a, b in zip(t, t[1:])))

    def test_slow_original_fails_closed(self):
        o2 = self.tmp / 'o2'
        r = run(V2, self.base, o2, self.slow)
        self.assertEqual(r.returncode, 21, r.stderr)
        self.assertFalse(o2.exists(), 'no bundle may be written')
        err = json.loads(r.stderr.strip().splitlines()[-1])
        self.assertGreater(err['framesNotIncreasing']['count'], 0)
        self.assertNotIn(r.returncode, (0, 1, 2))

    def test_tail_shift_gives_strictly_increasing_frames_and_keeps_spacing(self):
        o2 = self.tmp / 'o2'
        r = run(V2, self.base, o2, self.slow, '--tail-shift')
        self.assertEqual(r.returncode, 0, r.stderr)
        t = frame_times(o2)
        self.assertTrue(all(b > a for a, b in zip(t, t[1:])), t)
        rt = run_times(o2)
        self.assertTrue(all(b >= a for a, b in zip(rt, rt[1:])))
        base_t = frame_times(self.base)            # tail frames keep their base spacing
        tail = list(range(1 + LAST + 1, 1 + N_FRAMES))
        self.assertEqual([t[i + 1] - t[i] for i in tail[:-1]], [base_t[i + 1] - base_t[i] for i in tail[:-1]])
        ts = diag(o2)['tailShift']
        self.assertGreater(ts['deltaNs'], 0)
        self.assertEqual(ts['tailFramesShifted'], N_FRAMES - LAST - 1)
        # the first tail frame is exactly one ns after the last staged frame (the formula's minimum delta)
        self.assertEqual(t[1 + LAST + 1], t[1 + LAST] + 1)
        self.assertEqual(t[1 + LAST + 1] - base_t[1 + LAST + 1], ts['deltaNs'])

    def test_tail_shift_needs_wallclock(self):
        r = subprocess.run([sys.executable, '-I', '-B', str(V2), '--base', str(self.base), '--out', str(self.tmp / 'o'), '--tail-shift'],
                           capture_output=True, text=True)
        self.assertNotEqual(r.returncode, 0)
        self.assertFalse((self.tmp / 'o').exists())

    def test_unmodified_base_with_decreasing_frames_is_refused(self):
        bad = self.tmp / 'bad'
        make_base(bad)
        lines = (bad / 'frames.tsv').read_text().splitlines()
        f = lines[5].split('\t')
        f[2] = str(int(f[2]) - 100_000_000)
        lines[5] = '\t'.join(f)
        (bad / 'frames.tsv').write_text('\n'.join(lines) + '\n')
        o = self.tmp / 'o'
        r = subprocess.run([sys.executable, '-I', '-B', str(V2), '--base', str(bad), '--out', str(o)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 21, r.stderr)
        self.assertFalse(o.exists())

    def test_decreasing_sensor_rows_are_refused(self):
        bad = self.tmp / 'bad'
        make_base(bad)
        lines = (bad / 'sensors.tsv').read_text().splitlines()
        f = lines[7].split('\t')
        f[1] = '1'
        lines[7] = '\t'.join(f)
        (bad / 'sensors.tsv').write_text('\n'.join(lines) + '\n')
        o = self.tmp / 'o'
        r = subprocess.run([sys.executable, '-I', '-B', str(V2), '--base', str(bad), '--out', str(o)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 21, r.stderr)
        self.assertFalse(o.exists())


if __name__ == '__main__':
    unittest.main()
