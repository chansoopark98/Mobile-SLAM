#!/usr/bin/env python3
"""Copy a replay bundle with the time axis compressed by k.

Every k-th frame and every k-th sensor run row is kept, times are divided by k
(relative to T0 = 1 s) and the gyro columns are multiplied by k.

usage: speed_bundle.py <src bundle> <dst bundle> <k>
"""
import hashlib
import json
import pathlib
import shutil
import sys

T0 = 1.0
GYRO_COLUMNS = (8, 9, 10)
HASHED_FILES = ('sensors.tsv', 'frames.tsv', 'engine-config.tsv')


def _ns(t_rel):
    return round((t_rel + T0) * 1e9)


def speed_bundle(src, dst, k):
    """Write a k-times faster copy of bundle `src` to the new directory `dst`.

    Returns (kept frame count, sensor row count). Fails if `dst` exists.
    """
    src, dst = pathlib.Path(src), pathlib.Path(dst)
    dst.mkdir(parents=True)
    shutil.copy(src / 'engine-config.tsv', dst / 'engine-config.tsv')

    out, n, boot = [], 0, None
    for line in (src / 'frames.tsv').read_text().splitlines():
        c = line.split('\t')
        if c[0] == 'bootstrap':
            boot = c
            continue
        if int(c[1]) % k:
            continue
        t = float(c[4]) / k
        c[1] = str(n)
        c[2] = c[3] = str(_ns(t))
        c[4] = repr(t)
        n += 1
        if n == 1:
            # The bootstrap frame sits 16 ms before the first kept frame.
            tb = t - 0.016
            boot[2] = boot[3] = str(_ns(tb))
            boot[4] = repr(tb)
            out.append('\t'.join(boot))
        out.append('\t'.join(c))
    (dst / 'frames.tsv').write_text('\n'.join(out) + '\n')

    out, i = [], 0
    for line in (src / 'sensors.tsv').read_text().splitlines():
        c = line.split('\t')
        if c[0] == 'run':
            i += 1
            if (i - 1) % k:
                continue
        if c[0] == 'bootstrap':
            c[1] = boot[2]  # bootstrap packet = bootstrap frame time
        else:
            c[1] = str(round(1e9 + (int(c[1]) - 1e9) / k))
        for j in GYRO_COLUMNS:
            if c[j] != '-':
                c[j] = repr(float(c[j]) * k)
        out.append('\t'.join(c))
    (dst / 'sensors.tsv').write_text('\n'.join(out) + '\n')

    b = json.loads((src / 'bundle.json').read_text())
    b['label'] = dst.name
    b['speed'] = {'from': str(src), 'k': k}
    b['frames'] = b['processedFrames'] = n
    b['sensorRows'] = len(out)
    b['files'] = {f: hashlib.sha256((dst / f).read_bytes()).hexdigest()
                  for f in HASHED_FILES}
    (dst / 'bundle.json').write_text(json.dumps(b, indent=1) + '\n')
    return n, len(out)


def main(argv):
    if len(argv) != 4:
        print(__doc__, file=sys.stderr)
        return 2
    dst = pathlib.Path(argv[2])
    n, rows = speed_bundle(argv[1], dst, int(argv[3]))
    print(dst, 'frames', n, 'sensor rows', rows)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
