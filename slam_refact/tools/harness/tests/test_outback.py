"""Tests for the out-and-back (palindrome) replay tools in ../outback."""
import hashlib
import json
import pathlib
import sys
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parents[1]
REPO = HARNESS.parents[2]
sys.path.insert(0, str(HARNESS))

from outback import mirror_eval, palindrome_bundle, speed_bundle  # noqa: E402

N_FRAMES = 8
T0_NS = 1_000_000_000
FRAME_DT_NS = 50_000_000
SENSOR_DT_NS = 20_000_000


def _write_source_bundle(path):
    """Tiny synthetic bundle: 1 bootstrap + 8 frames, 1 bootstrap + 24 run rows."""
    path.mkdir(parents=True)
    (path / 'engine-config.tsv').write_text('scenario\tsynthetic\nlabel\tsrc\n')
    frames = ['\t'.join(['bootstrap', '-1', str(T0_NS - 16_000_000), str(T0_NS - 16_000_000),
                         '-0.016', '1', '/img/00000.pgm', '-'])]
    for i in range(N_FRAMES):
        ns = T0_NS + i * FRAME_DT_NS
        frames.append('\t'.join(['frame', str(i), str(ns), str(ns), repr((ns - T0_NS) / 1e9),
                                 '1', f'/img/{i:05d}.pgm', '-']))
    (path / 'frames.tsv').write_text('\n'.join(frames) + '\n')
    sensors = ['\t'.join(['bootstrap', str(T0_NS - 16_000_000), '0.5', '5.6', '8.0', '-', '-', '-',
                          '1.5', '-2.5', '0.25', '16.667', '1.0', '2.0', '3.0'])]
    for s in range(24):  # the last frame is at 1.35 s: rows 18.. come after it
        ns = T0_NS + s * SENSOR_DT_NS
        gyro = ['-', '-', '-'] if s == 5 else [repr(0.5 + s), repr(-1.0 - s), repr(0.125 * s)]
        sensors.append('\t'.join(['run', str(ns), '0.5', '5.6', '8.0', '-', '-', '-', *gyro,
                                  '16.667', '1.0', '2.0', '3.0']))
    (path / 'sensors.tsv').write_text('\n'.join(sensors) + '\n')
    (path / 'bundle.json').write_text(json.dumps(
        {'schema': 'slam-refact-replay-bundle-v1', 'label': 'src', 'frames': N_FRAMES,
         'processedFrames': N_FRAMES, 'sensorRows': 25}, indent=1) + '\n')


def _lines(path):
    return path.read_text().splitlines()


class OutbackBase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = pathlib.Path(tmp.name)
        self.src = self.tmp / 'src'
        _write_source_bundle(self.src)

    def make_palindrome(self, name='pal'):
        dst = self.tmp / name
        palindrome_bundle.palindrome_bundle(self.src, dst)
        return dst


class PalindromeBundleTest(OutbackBase):
    def test_valid_bundle_and_counts(self):
        dst = self.make_palindrome()
        info = palindrome_bundle.validate_palindrome(dst)
        self.assertEqual(info['N'], N_FRAMES)
        self.assertEqual(info['frames'], 2 * N_FRAMES - 1)
        self.assertEqual(info['turnNs'], T0_NS + (N_FRAMES - 1) * FRAME_DT_NS)
        frames = [l for l in _lines(dst / 'frames.tsv') if l.startswith('frame')]
        self.assertEqual(len(frames), 2 * N_FRAMES - 1)

    def test_extra_config_appended(self):
        dst = self.tmp / 'pal-extra'
        palindrome_bundle.palindrome_bundle(self.src, dst, ['key\tvalue'])
        self.assertTrue(_lines(dst / 'engine-config.tsv')[-1] == 'key\tvalue')

    def test_refuses_existing_dst(self):
        dst = self.make_palindrome()
        with self.assertRaises(FileExistsError):
            palindrome_bundle.palindrome_bundle(self.src, dst)

    def test_gyro_negated_in_mirror_and_dash_kept(self):
        dst = self.make_palindrome()
        turn = T0_NS + (N_FRAMES - 1) * FRAME_DT_NS
        src_rows = {int(r[1]): r for r in (l.split('\t') for l in _lines(self.src / 'sensors.tsv'))
                    if r[0] == 'run' and int(r[1]) <= turn}
        out_rows = {int(r[1]): r for r in (l.split('\t') for l in _lines(dst / 'sensors.tsv'))
                    if r[0] == 'run' and int(r[1]) > turn}
        self.assertTrue(out_rows)
        checked_dash = checked_neg = 0
        for ns, mirrored in out_rows.items():
            orig = src_rows[2 * turn - ns]
            for col in (8, 9, 10):
                if orig[col] == '-':
                    self.assertEqual(mirrored[col], '-')
                    checked_dash += 1
                else:
                    self.assertEqual(float(mirrored[col]), -float(orig[col]))
                    checked_neg += 1
            self.assertEqual(mirrored[2:8], orig[2:8])
            self.assertEqual(mirrored[11:], orig[11:])
        self.assertGreater(checked_dash, 0)
        self.assertGreater(checked_neg, 0)

    def test_rows_after_last_frame_are_dropped(self):
        dst = self.make_palindrome()
        turn = T0_NS + (N_FRAMES - 1) * FRAME_DT_NS
        n_src_after = sum(1 for l in _lines(self.src / 'sensors.tsv')
                          if l.startswith('run') and int(l.split('\t')[1]) > turn)
        self.assertGreater(n_src_after, 0)
        times = [int(l.split('\t')[1]) for l in _lines(dst / 'sensors.tsv') if l.startswith('run')]
        self.assertEqual(times, sorted(times))

    def _corrupt(self, name, edit):
        dst = self.make_palindrome(name)
        edit(dst)
        return dst

    def test_validate_rejects_swapped_sensor_rows(self):
        def edit(dst):
            lines = _lines(dst / 'sensors.tsv')
            lines[3], lines[4] = lines[4], lines[3]
            (dst / 'sensors.tsv').write_text('\n'.join(lines) + '\n')
        dst = self._corrupt('bad-order', edit)
        with self.assertRaisesRegex(ValueError, 'run rows not in time order'):
            palindrome_bundle.validate_palindrome(dst)

    def test_validate_rejects_bootstrap_not_first(self):
        def edit(dst):
            lines = _lines(dst / 'sensors.tsv')
            lines[0], lines[1] = lines[1], lines[0]
            (dst / 'sensors.tsv').write_text('\n'.join(lines) + '\n')
        dst = self._corrupt('bad-boot', edit)
        with self.assertRaisesRegex(ValueError, 'bootstrap row must come first'):
            palindrome_bundle.validate_palindrome(dst)

    def test_validate_rejects_missing_frame_row(self):
        def edit(dst):
            lines = _lines(dst / 'frames.tsv')
            del lines[-1]
            (dst / 'frames.tsv').write_text('\n'.join(lines) + '\n')
        dst = self._corrupt('bad-count', edit)
        with self.assertRaisesRegex(ValueError, r'expected 15 frame rows \(2N-1'):
            palindrome_bundle.validate_palindrome(dst)

    def _mirror_row(self, dst, k):
        lines = _lines(dst / 'frames.tsv')
        idx = next(i for i, l in enumerate(lines) if l.startswith('frame\t%d\t' % k))
        return lines, idx

    def test_validate_rejects_wrong_image(self):
        def edit(dst):
            lines, idx = self._mirror_row(dst, N_FRAMES + 1)
            c = lines[idx].split('\t')
            c[6] = '/img/99999.pgm'
            lines[idx] = '\t'.join(c)
            (dst / 'frames.tsv').write_text('\n'.join(lines) + '\n')
        dst = self._corrupt('bad-image', edit)
        with self.assertRaisesRegex(ValueError, r'mirrored frame 9 differs from frame 5 in column 6'):
            palindrome_bundle.validate_palindrome(dst)

    def test_validate_rejects_wrong_time(self):
        def edit(dst):
            lines, idx = self._mirror_row(dst, N_FRAMES + 1)
            c = lines[idx].split('\t')
            # Still strictly increasing, but not 2*T - t_j.
            c[2] = c[3] = str(int(c[2]) + 1_000_000)
            lines[idx] = '\t'.join(c)
            (dst / 'frames.tsv').write_text('\n'.join(lines) + '\n')
        dst = self._corrupt('bad-time', edit)
        with self.assertRaisesRegex(ValueError, r'mirrored frame 9 has time .* expected .* \(2\*T - t_5\)'):
            palindrome_bundle.validate_palindrome(dst)

    def test_validate_rejects_wrong_frame_index(self):
        def edit(dst):
            lines, idx = self._mirror_row(dst, N_FRAMES + 1)
            c = lines[idx].split('\t')
            c[1] = '77'
            lines[idx] = '\t'.join(c)
            (dst / 'frames.tsv').write_text('\n'.join(lines) + '\n')
        dst = self._corrupt('bad-index', edit)
        with self.assertRaisesRegex(ValueError, 'has index 77, expected 9'):
            palindrome_bundle.validate_palindrome(dst)


class SpeedBundleTest(OutbackBase):
    def test_k3(self):
        dst = self.tmp / 'x3'
        n, rows = speed_bundle.speed_bundle(self.src, dst, 3)
        self.assertEqual(n, 3)  # frames 0, 3, 6
        frames = [l.split('\t') for l in _lines(dst / 'frames.tsv')]
        kept = [f for f in frames if f[0] == 'frame']
        self.assertEqual([f[1] for f in kept], ['0', '1', '2'])
        self.assertEqual([f[6] for f in kept], ['/img/00000.pgm', '/img/00003.pgm', '/img/00006.pgm'])
        for idx, src_i in enumerate((0, 3, 6)):
            t_rel = src_i * FRAME_DT_NS / 1e9 / 3
            self.assertEqual(int(kept[idx][2]), round((t_rel + 1.0) * 1e9))
        boot = frames[0]
        self.assertEqual(boot[0], 'bootstrap')
        self.assertEqual(int(boot[2]), int(kept[0][2]) - 16_000_000)
        src_run = [l.split('\t') for l in _lines(self.src / 'sensors.tsv') if l.startswith('run')]
        out = [l.split('\t') for l in _lines(dst / 'sensors.tsv')]
        out_run = [r for r in out if r[0] == 'run']
        self.assertEqual(len(out_run), 8)  # 24 run rows, every 3rd
        self.assertEqual(rows, len(out))
        self.assertEqual(out[0][1], boot[2])
        for n_out, r in enumerate(out_run):
            s = src_run[3 * n_out]
            self.assertEqual(int(r[1]), round(1e9 + (int(s[1]) - 1e9) / 3))
            for col in (8, 9, 10):
                if s[col] == '-':
                    self.assertEqual(r[col], '-')
                else:
                    self.assertEqual(float(r[col]), float(s[col]) * 3)
        b = json.loads((dst / 'bundle.json').read_text())
        self.assertEqual(b['speed']['k'], 3)
        self.assertEqual(b['files']['frames.tsv'],
                         hashlib.sha256((dst / 'frames.tsv').read_bytes()).hexdigest())

    def test_refuses_existing_dst(self):
        dst = self.tmp / 'x3'
        speed_bundle.speed_bundle(self.src, dst, 3)
        with self.assertRaises(FileExistsError):
            speed_bundle.speed_bundle(self.src, dst, 3)


def _write_run(path, bundle_dir, rows, meta_bundle=None):
    path.mkdir(parents=True)
    (path / 'trace.jsonl').write_text('\n'.join(json.dumps(r) for r in rows) + '\n')
    if meta_bundle is not None:
        (path / 'meta.json').write_text(json.dumps({'bundle': meta_bundle}))


class MirrorEvalTest(OutbackBase):
    N = 50  # forward frames, enough for E_start frames 20..40

    def setUp(self):
        super().setUp()
        # Build a palindrome bundle with N forward frames.
        self.src = self.tmp / 'src50'
        self.src.mkdir()
        (self.src / 'engine-config.tsv').write_text('label\tsrc\n')
        frames = ['\t'.join(['bootstrap', '-1', '984000000', '984000000', '-0.016', '1', '/i/0', '-'])]
        for i in range(self.N):
            ns = T0_NS + i * FRAME_DT_NS
            frames.append('\t'.join(['frame', str(i), str(ns), str(ns), repr((ns - T0_NS) / 1e9),
                                     '1', f'/i/{i}', '-']))
        (self.src / 'frames.tsv').write_text('\n'.join(frames) + '\n')
        sens = ['\t'.join(['bootstrap', '984000000'] + ['0'] * 13)]
        for s in range(self.N * 2):
            sens.append('\t'.join(['run', str(T0_NS + s * 25_000_000)] + ['0'] * 13))
        (self.src / 'sensors.tsv').write_text('\n'.join(sens) + '\n')
        (self.src / 'bundle.json').write_text(json.dumps({'label': 'src'}))
        self.bundle = self.tmp / 'bundles' / 'pal'
        (self.tmp / 'bundles').mkdir()
        palindrome_bundle.palindrome_bundle(self.src, self.bundle)

    def trace(self, pos_fn, extra=None):
        """Rows i = 0..2N-2; pos_fn(i) gives the position."""
        extra = extra or {}
        rows = []
        for i in range(2 * self.N - 1):
            r = {'i': i, 'p': list(pos_fn(i)), 'q_wxyz': [1, 0, 0, 0], 'status': 'NORMAL',
                 'srMapTrackerState': 1, 'srMapReset': False, 'srKeyframes': 2,
                 'srStageMs': {'mapTracker': 5, 'mapBuilder': 1}}
            r.update(extra.get(i, {}))
            rows.append(r)
        return rows

    @staticmethod
    def out_and_back(n, vel=(1.0, 0.5, 0.0)):
        """Position at row i for a path with constant velocity that returns."""
        def pos(i):
            j = i if i < n else 2 * (n - 1) - i
            return (vel[0] * j, vel[1] * j, vel[2] * j)
        return pos

    def run_of(self, rows, name='run'):
        run = self.tmp / 'runs' / name
        _write_run(run, self.bundle, rows)
        return run

    def test_perfect_mirror_is_zero(self):
        run = self.run_of(self.trace(self.out_and_back(self.N)))
        m = mirror_eval.evaluate_run(run, self.bundle)
        self.assertEqual(m['forwardFrames'], self.N)
        self.assertEqual(m['eStart'], 0)
        self.assertEqual(m['eAll'], 0)
        self.assertEqual(m['eStartPairs'], 21)
        self.assertTrue(m['eStartComplete'])
        self.assertEqual(m['resets'], 0)
        self.assertEqual(m['mapLost'], 0)
        self.assertEqual(m['normalRatio'], 1.0)
        self.assertEqual(m['stall'], 0)
        self.assertEqual(m['maxKeyframes'], 2)
        self.assertAlmostEqual(m['path'], (1.25 ** 0.5) * (self.N - 1))

    def test_known_offset_on_return_leg(self):
        base = self.out_and_back(self.N)
        off = (0.3, 0.4, 0.0)  # length 0.5

        def pos(i):
            p = base(i)
            return tuple(a + (b if i >= self.N else 0) for a, b in zip(p, off))
        m = mirror_eval.evaluate_run(self.run_of(self.trace(pos)), self.bundle)
        self.assertAlmostEqual(m['eStart'], 0.5, places=12)
        self.assertAlmostEqual(m['eAll'], 0.5, places=12)

    def test_wrong_index_mapping_gives_nonzero_error(self):
        # Return leg shifted by one frame (as if the mirror were 2N-1-j).
        def pos(i):
            j = i if i < self.N else 2 * self.N - 1 - i
            return (j, 0.5 * j, 0.0)
        m = mirror_eval.evaluate_run(self.run_of(self.trace(pos)), self.bundle)
        self.assertAlmostEqual(m['eStart'], (1.25 ** 0.5), places=12)
        self.assertGreater(m['eAll'], 0)

    def test_event_counters_and_flags(self):
        n = self.N
        extra = {
            10: {'srMapReset': True},
            n + 3: {'srMapReset': True},                     # back leg, mirrored frame n-5
            5: {'srMapTrackerState': 0},                     # map lost
            n + 2: {'srStageMs': {'mapTracker': 1000}},      # stall
            n + 4: {'srStageMs': {'mapBuilder': 200}, 'srKeyframes': 9, 'status': 'LIMITED'},
        }
        m = mirror_eval.evaluate_run(self.run_of(self.trace(self.out_and_back(n), extra)), self.bundle)
        self.assertEqual(m['resets'], 2)
        self.assertEqual(m['mapLost'], 1)
        self.assertEqual(m['stall'], 2)
        self.assertEqual(m['maxKeyframes'], 9)
        self.assertEqual(m['resetFramesOut'], [10])
        self.assertEqual(m['resetFramesBackMirrored'], [2 * (n - 1) - (n + 3)])
        self.assertAlmostEqual(m['normalRatio'], (2 * n - 2) / (2 * n - 1))

    def test_incomplete_e_start_is_flagged(self):
        rows = [r for r in self.trace(self.out_and_back(self.N)) if r['i'] not in (25, 30)]
        m = mirror_eval.evaluate_run(self.run_of(rows), self.bundle)
        self.assertEqual(m['eStartPairs'], 19)
        self.assertFalse(m['eStartComplete'])

    def test_evaluate_rejects_invalid_bundle(self):
        lines = _lines(self.bundle / 'frames.tsv')
        (self.bundle / 'frames.tsv').write_text('\n'.join(lines[:-1]) + '\n')
        run = self.run_of(self.trace(self.out_and_back(self.N)))
        with self.assertRaisesRegex(ValueError, 'frame rows'):
            mirror_eval.evaluate_run(run, self.bundle)

    def test_find_bundle_via_meta(self):
        run = self.tmp / 'runs' / 'meta-run'
        _write_run(run, self.bundle, self.trace(self.out_and_back(self.N)), 'bundles/pal')
        self.assertEqual(mirror_eval.find_bundle(run).resolve(), self.bundle.resolve())

    def test_find_bundle_missing_is_clear_error(self):
        run = self.tmp / 'runs' / 'meta-bad'
        _write_run(run, self.bundle, self.trace(self.out_and_back(self.N)), 'bundles/nope')
        with self.assertRaisesRegex(FileNotFoundError, 'bundle.json for meta bundle'):
            mirror_eval.find_bundle(run)

    def test_cli_json_and_detail(self):
        run = self.run_of(self.trace(self.out_and_back(self.N)), 'cli')
        out = self.tmp / 'out.json'
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            mirror_eval.main(['--json', str(out), '--detail', f'{run}={self.bundle}'])
        text = buf.getvalue()
        self.assertIn('E_start/path', text)
        self.assertIn('mirror error p50', text)
        data = json.loads(out.read_text())
        self.assertEqual(data[0]['run'], str(run))
        self.assertEqual(data[0]['bundle'], str(self.bundle))
        self.assertEqual(data[0]['eStart'], 0)


EVIDENCE_SRC = REPO / 'build/slam-refact-20261015/harness/bundles/flip6-arcore-l1/acc-base-karcore'
EVIDENCE_PARITY_BUNDLE = REPO / 'build/xr-fix-screen-20261116/bundles-v2/l1-x3/parity'
EVIDENCE_PARITY_RUN = REPO / 'build/xr-fix-screen-20261116/runs-v2/l1-x3/parity'


class GoldenTest(unittest.TestCase):
    """Runs only when the recorded evidence directories exist."""

    def setUp(self):
        if not (EVIDENCE_SRC / 'frames.tsv').is_file() or not (EVIDENCE_PARITY_BUNDLE / 'bundle.json').is_file():
            self.skipTest('evidence bundles not present')

    def test_pipeline_reproduces_recorded_hashes(self):
        with tempfile.TemporaryDirectory() as tmp:
            x3 = pathlib.Path(tmp) / 'x3'
            pal = pathlib.Path(tmp) / 'parity'
            speed_bundle.speed_bundle(EVIDENCE_SRC, x3, 3)
            palindrome_bundle.palindrome_bundle(x3, pal)
            expected = json.loads((EVIDENCE_PARITY_BUNDLE / 'bundle.json').read_text())['files']
            for name in ('sensors.tsv', 'frames.tsv', 'engine-config.tsv'):
                got = hashlib.sha256((pal / name).read_bytes()).hexdigest()
                self.assertEqual(got, expected[name], name)

    def test_recorded_run_metrics(self):
        if not (EVIDENCE_PARITY_RUN / 'trace.jsonl').is_file():
            self.skipTest('evidence run not present')
        m = mirror_eval.evaluate_run(EVIDENCE_PARITY_RUN, EVIDENCE_PARITY_BUNDLE)
        self.assertEqual(m['resets'], 10)
        self.assertEqual(m['mapLost'], 76)
        self.assertEqual(round(m['eStart'], 3), 0.250)


if __name__ == '__main__':
    unittest.main()
