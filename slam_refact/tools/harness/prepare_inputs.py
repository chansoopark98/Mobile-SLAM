#!/usr/bin/env python3
"""Original reference inputs -> replay bundle for slam_refact_replay (architecture.md §6.1-6.2, WP5).

Format conversion only: the W3C values the original page dispatched are written unchanged; the
sensors.ts / func 647 conversions are the engine's SensorIntake (D-14).

Inputs (the same bytes the original headless runs consumed):
  * sensors: build/slam-refact-20261015/reference/runs/<scenario>/engine-input-sensors.jsonl.gz
    (run-reference.mjs: t, accelIncludingGravity, rotationRateDegS [alpha, beta, gamma], orientation
    [alpha, beta, gamma], qEarthFromDevice). For scale 'absolute' scenarios also the
    DERIVED-LINACC-OS-EQUIVALENT file of derive_linear_accel.py (W3C `acceleration`).
  * frames: the scenario dataset exactly as run-reference.mjs loadDataset() reads it (TUM fixture
    input.json, Flip6 record.json; Flip6 t = timestamp - t0, t0 = last IMU sample <= first frame).
  * optional captured pyramids (spec_frontend/capture_pyramid.mjs layout: meta.json captures[c]
    .inputFrame, cap-CCC.rgba.gz; capture with inputFrame null = bootstrap) for injection (H7, V-H7).
  * optional drop mask: frames with fresh == false in a reference trace are not processed; their
    sensor events stay queued for the next processed frame (§6.2).

Bundle (text, read by tools/harness/replay/bundle_reader.cc):
  engine-config.tsv  key<TAB>value (EngineConfig / GlueConfig / provenance)
  sensors.tsv        phase receive_time_ns aIG(x y z) lin(x y z | - - -) rate(alpha beta gamma) interval_ms
                     orientation(alpha beta gamma); phase = bootstrap | run; run rows in time order
  frames.tsv         kind index time_ns video_time_ns t process image pyramid; kind = bootstrap | frame;
                     image/pyramid are paths ('-' = none)
  bundle.json        provenance and checks (not read by C++)

Time: receive_time_ns = time_ns = video_time_ns = round((t + T0) * 1e9), T0 = 1.0 s (0 is a sentinel).
Bootstrap (D-4, INFERRED simplification): one stage before frame 0 with frame-0 pixels (or capture 0),
time = t(frame 0) - 0.016 s, sensors = the first packet once at that time; its output is discarded.
Binding (INFERRED): before frame k every run sensor row with receive_time_ns <= time_ns(k) is pushed.

Usage (repo root):
  python3 -I slam_refact/tools/harness/prepare_inputs.py --scenario flip6-s2 --label v-h7 \
      --drop-mask-from r1 --pyramid-capture build/slam-refact-20261015/spec/frontend/capture/flip6-s2/c2-73 --max-frames 72
"""
import argparse
import gzip
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
RUNTIME = ROOT / 'slam_refact/tools/re/runtime'
EVIDENCE = ROOT / 'build/slam-refact-20261015'
RUNS = EVIDENCE / 'reference/runs'
BUNDLES = EVIDENCE / 'harness/bundles'
T0_S = 1.0
BOOTSTRAP_LEAD_S = 0.016
# deviceEstimate of the original runs (reference/runs/*/r1/meta.json publicApi.deviceEstimate).
DEVICE = {'pixel7': ('Google', 'Pixel 7'), 'reduced': ('', 'K')}


def sha(b):
    return hashlib.sha256(b).hexdigest()


def ns(t):
    return int(round((t + T0_S) * 1e9))


def pgm_payload(path, width, height):
    """Pixel bytes of a binary PGM (P5, maxval 255), as replay-android-input.mjs parsePGM serves them."""
    data = pathlib.Path(path).read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(data[start:pos])
    pos += 1
    if fields[0] != b'P5' or int(fields[1]) != width or int(fields[2]) != height or int(fields[3]) != 255:
        raise SystemExit('unexpected PGM header %s in %s' % (fields, path))
    return data[pos:pos + width * height]


def load_dataset(scenario):
    """Python transcription of run-reference.mjs loadDataset() (frames and time base)."""
    d = ROOT / scenario['dataset']
    if scenario['kind'] == 'tum':
        inp = json.loads((d / 'input.json').read_text())
        frames = [{'index': f['index'], 't': f['physicalTimestamp'], 'file': d / 'mav0/cam0/data' / f['file'], 'sha256': f['sha256']}
                  for f in inp['frames']]
        return {'dir': d, 'width': inp['config']['width'], 'height': inp['config']['height'], 'frames': frames}
    if scenario['kind'] == 'flip6':
        rec = json.loads((d / 'record.json').read_text())['recording']
        first = rec['frames'][0]['timestamp']
        t0 = max(r[0] for r in rec['imu'] if r[0] <= first)
        frames = [{'index': i, 't': round(f['timestamp'] - t0, 9), 'file': d / 'mav0/cam0/data' / f['file'], 'sha256': f['sha256']}
                  for i, f in enumerate(rec['frames'])]
        return {'dir': d, 'width': rec['width'], 'height': rec['height'], 'frames': frames, 't0Source': t0}
    raise SystemExit('unknown scenario kind')


def interval_ms(sensor_ts):
    """run-reference.mjs: +(median(dt) * 1000).toFixed(3), median = sorted[floor(n/2)]."""
    dts = sorted(b - a for a, b in zip(sensor_ts, sensor_ts[1:]))
    return float('%.3f' % (dts[len(dts) // 2] * 1000))


def read_derived(path):
    rows = []
    with gzip.open(path, 'rt') as f:
        for line in f:
            if not line.startswith('#'):
                rows.append(tuple(float(v) for v in line.split('\t')))
    return rows


def fmt(v):
    return repr(float(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--label', required=True, help='new bundle label [a-z0-9-]+ (never overwritten)')
    ap.add_argument('--out-root', type=pathlib.Path, default=BUNDLES)
    ap.add_argument('--sensors', type=pathlib.Path, help='default: reference/runs/<scenario>/engine-input-sensors.jsonl.gz')
    ap.add_argument('--linear-accel', type=pathlib.Path, help='derive_linear_accel.py output (required for scale absolute)')
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--drop-mask-from', help="reference run label (e.g. r1) or a trace.jsonl path; omit = ideal mode")
    ap.add_argument('--pyramid-capture', type=pathlib.Path, help='capture dir with meta.json + cap-CCC.rgba.gz')
    ap.add_argument('--pyramid-model', choices=['swiftshader-approx', 'ieee'], default='swiftshader-approx')
    ap.add_argument('--intrinsics', help='"fx fy cx cy" K override of the capture image (EngineConfig.intrinsics_override)')
    ap.add_argument('--params', help='"Key=b:1;Key=f:0.5" process-global parameter overrides (EngineConfig.param_overrides)')
    a = ap.parse_args()
    if not a.label.replace('-', '').isalnum() or a.label != a.label.lower():
        raise SystemExit('--label [a-z0-9-]+')
    scenarios = json.loads((RUNTIME / 'scenarios.json').read_text())
    scenario = scenarios[a.scenario]
    out = a.out_root / a.scenario / a.label
    if out.exists():
        raise SystemExit('refusing to overwrite ' + str(out))
    data = load_dataset(scenario)
    frames = data['frames'][:a.max_frames] if a.max_frames > 0 else data['frames']
    sensors_path = a.sensors or RUNS / a.scenario / 'engine-input-sensors.jsonl.gz'
    with gzip.open(sensors_path, 'rt') as f:
        sensors = [json.loads(line) for line in f if line.strip()]
    # The sensor file holds the full-run stream; DeviceMotion `interval` is its median dt (the value the
    # reference runs dispatched), also when --max-frames shortens the replay.
    ims = interval_ms([s['t'] for s in sensors])
    sensors = [s for s in sensors if s['t'] <= frames[-1]['t'] + 0.05]  # run-reference.mjs imu filter
    absolute = scenario.get('scale') == 'absolute'
    lin = None
    if absolute:
        if not a.linear_accel:
            raise SystemExit('scale absolute: --linear-accel <derived-linear-acceleration.tsv.gz> required')
        lin_rows = read_derived(a.linear_accel)
        if [r[0] for r in lin_rows[:len(sensors)]] != [s['t'] for s in sensors]:
            raise SystemExit('derived linear acceleration timestamps differ from the sensor stream')
        lin = [r[1:] for r in lin_rows[:len(sensors)]]
    elif a.linear_accel:
        raise SystemExit('--linear-accel is only for scale absolute scenarios (responsive originals had acceleration null)')
    checks = {}
    ref_meta = RUNS / a.scenario / 'r1' / 'meta.json'
    if ref_meta.exists():
        checks['interval_ms_equals_reference_meta'] = json.loads(ref_meta.read_text())['input']['imuIntervalMsMedian'] == ims
    if ref_meta.exists() and a.max_frames == 0:
        m = json.loads(ref_meta.read_text())['input']
        checks['sensor_rows_equal_reference_meta'] = m['imuRows'] == len(sensors)
        checks['frames_equal_reference_meta'] = m['frames'] == len(frames)
        with open(RUNS / a.scenario / 'r1' / 'trace.jsonl') as f:
            ref_t = [json.loads(line)['t'] for line in f]
        checks['frame_t_equal_reference_trace'] = ref_t == [fr['t'] for fr in frames]
    if not all(checks.values()):
        raise SystemExit('bundle disagrees with reference meta: %s' % checks)

    # Drop mask.
    process = [True] * len(frames)
    mask_src = None
    if a.drop_mask_from:
        p = pathlib.Path(a.drop_mask_from)
        mask_src = p if p.suffix == '.jsonl' else RUNS / a.scenario / a.drop_mask_from / 'trace.jsonl'
        with open(mask_src) as f:
            fresh = {r['i']: r['fresh'] for r in map(json.loads, f)}
        process = [bool(fresh[fr['index']]) for fr in frames]

    # Pyramid capture: capture c with inputFrame k (null = bootstrap).
    pyr = {}
    cap_meta = None
    if a.pyramid_capture:
        a.pyramid_capture = a.pyramid_capture.resolve()
        cap_meta = json.loads((a.pyramid_capture / 'meta.json').read_text())
        if cap_meta['scenario'] != a.scenario.replace('-absolute', ''):
            raise SystemExit('capture scenario %s != %s' % (cap_meta['scenario'], a.scenario))
        for c in cap_meta['captures']:
            path = a.pyramid_capture / ('cap-%03d.rgba.gz' % c['capture'])
            raw = gzip.decompress(path.read_bytes())
            if sha(raw) != c['rawSha256'] or len(raw) != 1024 * cap_meta['rows'] * 4:
                raise SystemExit('capture %s: sha256/size mismatch' % path)
            key = 'bootstrap' if c['inputFrame'] is None else c['inputFrame']
            pyr[key] = path

    # Pixel hashes as run-reference.mjs re-verifies them per served frame.
    w, h = data['width'], data['height']
    for fr in frames:
        if sha(pgm_payload(fr['file'], w, h)) != fr['sha256']:
            raise SystemExit('pixel sha256 mismatch ' + str(fr['file']))

    out.mkdir(parents=True)
    t_boot = frames[0]['t'] - BOOTSTRAP_LEAD_S
    if t_boot > sensors[0]['t'] + 0.016:
        raise SystemExit('bootstrap model assumes the first sensor packet precedes frame 0 by < 16 ms')
    lines = []

    def sensor_line(phase, t_ns, s, k):
        l3 = ('-', '-', '-') if lin is None else tuple(fmt(v) for v in lin[k])
        return '\t'.join([phase, str(t_ns), *map(fmt, s['accelIncludingGravity']), *l3, *map(fmt, s['rotationRateDegS']),
                          fmt(ims), *map(fmt, s['orientation'])]) + '\n'

    lines.append(sensor_line('bootstrap', ns(t_boot), sensors[0], 0))
    for k, s in enumerate(sensors):
        lines.append(sensor_line('run', ns(s['t']), s, k))
    (out / 'sensors.tsv').write_text(''.join(lines))

    flines = ['\t'.join(['bootstrap', '-1', str(ns(t_boot)), str(ns(t_boot)), fmt(t_boot), '1', str(frames[0]['file']),
                         str(pyr.get('bootstrap', '-'))]) + '\n']
    for fr, pr in zip(frames, process):
        flines.append('\t'.join(['frame', str(fr['index']), str(ns(fr['t'])), str(ns(fr['t'])), fmt(fr['t']), '1' if pr else '0',
                                 str(fr['file']), str(pyr.get(fr['index'], '-'))]) + '\n')
    (out / 'frames.tsv').write_text(''.join(flines))

    manufacturer, model = DEVICE[scenario.get('ua', 'pixel7')]
    cfg = [('scenario', a.scenario), ('label', a.label), ('capture_width', w), ('capture_height', h), ('rotation_deg', 0),
           ('manufacturer', manufacturer), ('model', model), ('estimate_scale', int(absolute)), ('disable_vio', 0),
           ('pyramid_model', a.pyramid_model), ('origin', '0 2 0'), ('facing_wxyz', '1 0 0 0'),
           ('left_handed_axes', 0), ('mirrored_display', 0), ('t0_s', fmt(T0_S)), ('world_point_sample_every', 50),
           ('drop_mode', 'reference:' + str(mask_src) if mask_src else 'ideal'),
           ('pyramid_injection', str(a.pyramid_capture) if a.pyramid_capture else 'none')]
    if a.intrinsics:
        if len(a.intrinsics.split()) != 4:
            raise SystemExit('--intrinsics "fx fy cx cy"')
        cfg.append(('intrinsics', a.intrinsics))
    if a.params:
        cfg.append(('params', a.params))
    (out / 'engine-config.tsv').write_text(''.join('%s\t%s\n' % kv for kv in cfg))
    injected = sorted(k for k in pyr if k != 'bootstrap')
    bundle = {
        'schema': 'slam-refact-replay-bundle-v1', 'scenario': a.scenario, 'scenarioSpec': scenario, 'label': a.label,
        'argv': sys.argv, 'dataset': str(data['dir']), 't0Source': data.get('t0Source'),
        'frames': len(frames), 'processedFrames': sum(process), 'droppedFrames': [fr['index'] for fr, p in zip(frames, process) if not p],
        'sensorRows': len(sensors), 'intervalMs': ims, 'checks': checks,
        'sensors': {'file': str(sensors_path), 'sha256': sha(pathlib.Path(sensors_path).read_bytes())},
        'linearAcceleration': None if lin is None else {'file': str(a.linear_accel), 'label': 'DERIVED-LINACC-OS-EQUIVALENT',
                                                        'sha256': sha(pathlib.Path(a.linear_accel).read_bytes())},
        'dropMask': None if mask_src is None else {'file': str(mask_src), 'sha256': sha(pathlib.Path(mask_src).read_bytes())},
        'pyramidCapture': None if cap_meta is None else {'dir': str(a.pyramid_capture), 'label': cap_meta['label'],
                                                         'rows': cap_meta['rows'], 'bootstrap': 'bootstrap' in pyr,
                                                         'frames': [injected[0], injected[-1]] if injected else None},
        'time': {'T0_s': T0_S, 'rule': 'ns = round((t + T0) * 1e9)'},
        'bootstrap': {'t': t_boot, 'rule': 'D-4 (INFERRED): frame-0 pixels (capture 0 when injecting), t(frame0) - 0.016 s, first sensor packet once'},
        'binding': 'before frame k push every run sensor row with receive_time_ns <= time_ns(k) (INFERRED, §6.2)',
        'files': {n: sha((out / n).read_bytes()) for n in ('sensors.tsv', 'frames.tsv', 'engine-config.tsv')},
    }
    (out / 'bundle.json').write_text(json.dumps(bundle, indent=1) + '\n')
    print(json.dumps({k: bundle[k] for k in ('scenario', 'label', 'frames', 'processedFrames', 'sensorRows', 'intervalMs', 'checks')}))
    print(out)


if __name__ == '__main__':
    main()
