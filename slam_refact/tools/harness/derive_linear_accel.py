#!/usr/bin/env python3
"""DERIVED-LINACC-OS-EQUIVALENT: linear acceleration for the absolute-scale reference runs
(architecture.md §6.5, S5 §8.4).

    lin_D[k] = aIG_D[k] - R(q_E_D[k])^T * (0, 0, 9.80665)        m/s^2, W3C device frame
      aIG_D  = engine-input-sensors accelIncludingGravity (measured sample)
      q_E_D  = engine-input-sensors qEarthFromDevice (w, x, y, z), the attitude the original already
               receives as DeviceOrientation (sensor_adapter.mjs gyro integration; no ground truth)
      9.80665 = Android SensorManager.GRAVITY_EARTH

One output sample per measured sample, same timestamp; nothing is resampled, smoothed or tuned. The
arithmetic below is written in the same operation order as the opt-in branch of
slam_refact/tools/re/runtime/page-runtime.js (linearAccelerationOf), so the original (JS doubles) and
the C++ harness (this file) receive bit-identical values; run-reference events.json records a
summary of what the page dispatched and --verify-events compares it with this file.

R(q)^T * (0, 0, g) = g * (third row of R(q)) with R(q) the matrix of sensor_adapter.mjs qMatrix; q is
used as stored (already unit within 1 ulp; no re-normalisation in either implementation).

Self-check (recorded only, never a gate, W4): |lin| of the first sample, p50/p95 of |lin|, median
|lin| per 10 s window and its least-squares slope, median |lin| over stationary samples
(|omega| < 5 deg/s).

Usage (repo root):
  python3 -I slam_refact/tools/harness/derive_linear_accel.py \
      --sensors build/slam-refact-20261015/reference/runs/<scenario>/engine-input-sensors.jsonl.gz \
      --out-dir build/slam-refact-20261015/reference/runs/<scenario>-absolute
  python3 -I slam_refact/tools/harness/derive_linear_accel.py --out-dir <dir> --verify-events <run>/events.json
"""
import argparse
import gzip
import hashlib
import json
import math
import pathlib
import statistics

GRAVITY_EARTH = 9.80665
LABEL = 'DERIVED-LINACC-OS-EQUIVALENT'
OUT_NAME = 'derived-linear-acceleration.tsv.gz'


def linear_acceleration(a, q):
    w, x, y, z = q
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    return (a[0] - GRAVITY_EARTH * r20, a[1] - GRAVITY_EARTH * r21, a[2] - GRAVITY_EARTH * r22)


def read_sensors(path):
    with gzip.open(path, 'rt') as f:
        return [json.loads(line) for line in f if line.strip()]


def read_derived(path):
    rows = []
    with gzip.open(path, 'rt') as f:
        for line in f:
            if line.startswith('#'):
                continue
            t, x, y, z = (float(v) for v in line.split('\t'))
            rows.append((t, x, y, z))
    return rows


def selfcheck(samples, lin):
    norms = [math.sqrt(x * x + y * y + z * z) for x, y, z in lin]
    ts = [s['t'] for s in samples]
    windows = {}
    for t, n in zip(ts, norms):
        windows.setdefault(int(t // 10.0), []).append(n)
    wmed = [(10.0 * k + 5.0, statistics.median(v)) for k, v in sorted(windows.items())]
    slope = None
    if len(wmed) >= 2:
        mx = sum(c for c, _ in wmed) / len(wmed)
        my = sum(m for _, m in wmed) / len(wmed)
        sxx = sum((c - mx) ** 2 for c, _ in wmed)
        slope = sum((c - mx) * (m - my) for c, m in wmed) / sxx
    still = [n for s, n in zip(samples, norms) if math.sqrt(sum(v * v for v in s['rotationRateDegS'])) < 5.0]
    srt = sorted(norms)
    q = lambda p: srt[min(len(srt) - 1, int(math.floor((len(srt) - 1) * p)))]  # noqa: E731
    return {'label': LABEL, 'gate': False, 'samples': len(norms), 'first_norm': norms[0], 'p50_norm': q(0.5),
            'p95_norm': q(0.95), 'window_10s_median_norm': [[c, m] for c, m in wmed],
            'window_median_slope_per_s': slope, 'stationary_samples': len(still),
            'stationary_median_norm': statistics.median(still) if still else None,
            'percentile': 'floor_index', 'stationary_rule': '|rotationRateDegS| < 5 deg/s'}


def derive(sensors_path, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / OUT_NAME
    if out.exists():
        raise SystemExit('refusing to overwrite ' + str(out))
    samples = read_sensors(sensors_path)
    lin = [linear_acceleration(s['accelIncludingGravity'], s['qEarthFromDevice']) for s in samples]
    lines = ['# %s t_fixture_s lin_x lin_y lin_z (m/s^2, W3C device frame); source %s\n' % (LABEL, sensors_path)]
    lines += ['%r\t%r\t%r\t%r\n' % (s['t'], x, y, z) for s, (x, y, z) in zip(samples, lin)]
    data = ''.join(lines).encode()
    with open(out, 'wb') as f:
        f.write(gzip.compress(data, compresslevel=9, mtime=0))
    check = selfcheck(samples, lin)
    check['sha256_uncompressed'] = hashlib.sha256(data).hexdigest()
    check['source'] = {'file': str(sensors_path), 'sha256': hashlib.sha256(pathlib.Path(sensors_path).read_bytes()).hexdigest()}
    check['formula'] = 'lin = accelIncludingGravity - 9.80665 * (2(xz - yw), 2(yz + xw), 1 - 2(x^2 + y^2)), q = qEarthFromDevice (w,x,y,z)'
    check['limits'] = ('not the OS fusion of a real device; gyro integration drift of qEarthFromDevice remains; parity with the '
                       'headless original only, not device accuracy (S5 U9 BLOCKED)')
    (out_dir / 'selfcheck.json').write_text(json.dumps(check, indent=1) + '\n')
    return out, check


def verify_events(out_dir, events_path):
    """Compares the page-side dispatch summary (events.json) with the derived file."""
    rows = read_derived(out_dir / OUT_NAME)
    events = json.loads(pathlib.Path(events_path).read_text())['events']
    summ = next((e for e in events if e.get('name') == 'derivedLinearAcceleration'), None)
    if summ is None:
        return {'ok': False, 'reason': 'no derivedLinearAcceleration event in ' + str(events_path)}
    n = summ['dispatched']
    sums = [0.0, 0.0, 0.0]
    for r in rows[:n]:  # plain left-to-right accumulation like the page (builtin sum() is compensated)
        for k in range(3):
            sums[k] += r[k + 1]
    first = list(rows[0][1:])
    last = list(rows[n - 1][1:])
    res = {'dispatched': n, 'derived_rows': len(rows), 'first_equal': summ['first'] == first,
           'last_equal': summ['last'] == last, 'bootstrap_equal': summ['bootstrap'] == first,
           'sum_page': summ['sum'], 'sum_file': sums,
           'sum_max_abs_diff': max(abs(a - b) for a, b in zip(summ['sum'], sums))}
    res['ok'] = res['first_equal'] and res['last_equal'] and res['bootstrap_equal'] and res['sum_max_abs_diff'] == 0.0
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sensors', type=pathlib.Path)
    ap.add_argument('--out-dir', type=pathlib.Path, required=True)
    ap.add_argument('--verify-events', type=pathlib.Path)
    a = ap.parse_args()
    if a.verify_events:
        res = verify_events(a.out_dir, a.verify_events)
        print(json.dumps(res, indent=1))
        raise SystemExit(0 if res['ok'] else 1)
    out, check = derive(a.sensors, a.out_dir)
    print(json.dumps({'out': str(out), **{k: check[k] for k in ('samples', 'first_norm', 'p50_norm', 'p95_norm',
                                                               'window_median_slope_per_s', 'stationary_median_norm')}}, indent=1))


if __name__ == '__main__':
    main()
