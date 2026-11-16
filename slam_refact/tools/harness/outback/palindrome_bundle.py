#!/usr/bin/env python3
"""Out-and-back (palindrome) replay bundle.

The source bundle is replayed forward, then the same frames in reverse order
(the last frame is not repeated). In the reverse part frame j is shown at
2*T - t_j; each sensor row at time s becomes a row at 2*T - s with the gyro
negated and the same acceleration / orientation values.

usage: palindrome_bundle.py <src bundle> <dst bundle> [extra engine-config 'key<TAB>value' ...]
"""
import hashlib
import json
import pathlib
import sys

GYRO_COLUMNS = (8, 9, 10)
HASHED_FILES = ('sensors.tsv', 'frames.tsv', 'engine-config.tsv')
# frames.tsv columns that differ between a frame and its mirror: index, two
# ns timestamps and the time in seconds. Everything else must be identical.
FRAME_VARYING_COLUMNS = (1, 2, 3, 4)


def _read_rows(path):
    return [line.split('\t') for line in pathlib.Path(path).read_text().splitlines()]


def validate_palindrome(bundle_dir):
    """Check the structure of a palindrome bundle; raise ValueError if wrong.

    Returns {'N', 'frames', 'sensorRows', 'turnNs'} on success.
    """
    bundle_dir = pathlib.Path(bundle_dir)
    meta = json.loads((bundle_dir / 'bundle.json').read_text())
    n = meta['palindrome']['forwardFrames']

    # sensors.tsv: bootstrap row first, run rows in non-decreasing time order.
    sensors = _read_rows(bundle_dir / 'sensors.tsv')
    if not sensors or sensors[0][0] != 'bootstrap':
        raise ValueError('sensors.tsv: the bootstrap row must come first')
    prev = None
    for idx, row in enumerate(sensors):
        if row[0] != 'run':
            continue
        t = int(row[1])
        if prev is not None and t < prev:
            raise ValueError(
                f'sensors.tsv: run rows not in time order at line {idx + 1} '
                f'({t} ns after {prev} ns)')
        prev = t

    # frames.tsv: 2N-1 frame rows, indices 0..2N-2, strictly increasing time.
    frames = [r for r in _read_rows(bundle_dir / 'frames.tsv') if r[0] == 'frame']
    if len(frames) != 2 * n - 1:
        raise ValueError(
            f'frames.tsv: expected {2 * n - 1} frame rows (2N-1, N={n}), '
            f'found {len(frames)}')
    for k, row in enumerate(frames):
        if int(row[1]) != k:
            raise ValueError(f'frames.tsv: frame row {k} has index {row[1]}, expected {k}')
        if k and int(row[2]) <= int(frames[k - 1][2]):
            raise ValueError(f'frames.tsv: frame times not strictly increasing at frame {k}')

    # Mirror mapping: row k (N <= k <= 2N-2) shows frame j = 2(N-1)-k at 2T - t_j.
    turn = int(frames[n - 1][2])
    for k in range(n, 2 * n - 1):
        j = 2 * (n - 1) - k
        a, b = frames[k], frames[j]
        for col in range(len(a)):
            if col in FRAME_VARYING_COLUMNS:
                continue
            if col >= len(b) or a[col] != b[col]:
                raise ValueError(
                    f'frames.tsv: mirrored frame {k} differs from frame {j} '
                    f'in column {col} (wrong image)')
        if int(a[2]) != 2 * turn - int(b[2]):
            raise ValueError(
                f'frames.tsv: mirrored frame {k} has time {a[2]} ns, '
                f'expected {2 * turn - int(b[2])} ns (2*T - t_{j})')
    return {'N': n, 'frames': len(frames), 'sensorRows': len(sensors), 'turnNs': turn}


def palindrome_bundle(src, dst, extra_config=()):
    """Write the out-and-back bundle of `src` to the new directory `dst`.

    `extra_config` lines ('key<TAB>value') are appended to engine-config.tsv.
    Validates the result and returns validate_palindrome()'s dict plus the
    forward frame count.
    """
    src, dst = pathlib.Path(src), pathlib.Path(dst)
    extra_config = list(extra_config)
    dst.mkdir(parents=True)
    cfg = (src / 'engine-config.tsv').read_text()
    for extra in extra_config:
        cfg += extra + '\n'
    (dst / 'engine-config.tsv').write_text(cfg)

    frames = _read_rows(src / 'frames.tsv')
    boot = [f for f in frames if f[0] == 'bootstrap']
    fwd = [f for f in frames if f[0] == 'frame']
    turn = int(fwd[-1][2])
    out = ['\t'.join(f) for f in boot + fwd]
    n = len(fwd)
    for j in range(len(fwd) - 2, -1, -1):
        c = list(fwd[j])
        ns = 2 * turn - int(c[2])
        c[1] = str(n)
        c[2] = c[3] = str(ns)
        c[4] = repr((ns - 1_000_000_000) / 1e9)
        n += 1
        out.append('\t'.join(c))
    (dst / 'frames.tsv').write_text('\n'.join(out) + '\n')

    rows = _read_rows(src / 'sensors.tsv')
    # Forward part: rows up to the turn only (rows after the last frame would
    # be out of order with the mirrored ones).
    sens = ['\t'.join(r) for r in rows if r[0] != 'run' or int(r[1]) <= turn]
    rev = []
    for r in rows:
        if r[0] != 'run' or int(r[1]) > turn:
            continue
        c = list(r)
        c[1] = str(2 * turn - int(r[1]))
        for k in GYRO_COLUMNS:
            if c[k] != '-':
                c[k] = repr(-float(c[k]))
        rev.append(c)
    rev.sort(key=lambda c: int(c[1]))
    sens += ['\t'.join(c) for c in rev if int(c[1]) > turn]
    (dst / 'sensors.tsv').write_text('\n'.join(sens) + '\n')

    b = json.loads((src / 'bundle.json').read_text())
    b['label'] = dst.name
    b['palindrome'] = {'from': str(src), 'forwardFrames': len(fwd), 'frames': n,
                       'turnNs': turn, 'extraConfig': extra_config}
    b['frames'] = b['processedFrames'] = n
    b['sensorRows'] = len(sens)
    b['files'] = {f: hashlib.sha256((dst / f).read_bytes()).hexdigest()
                  for f in HASHED_FILES}
    (dst / 'bundle.json').write_text(json.dumps(b, indent=1) + '\n')

    result = validate_palindrome(dst)
    result['forwardFrames'] = len(fwd)
    return result


def main(argv):
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    dst = pathlib.Path(argv[2])
    r = palindrome_bundle(argv[1], dst, argv[3:])
    print(dst, 'frames', r['frames'], '(forward', r['forwardFrames'], ') sensor rows', r['sensorRows'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
