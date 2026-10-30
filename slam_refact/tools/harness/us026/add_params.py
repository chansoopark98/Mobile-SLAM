#!/usr/bin/env python3
"""US-026 (design §2.5, §3.2): copy a replay bundle and add one `params` line to engine-config.tsv.

The base bundle is never modified; the copy goes to a new directory. Only engine-config.tsv changes (one appended
line `params<TAB><Key=t:v;...>`, the format tools/harness/replay/bundle_reader.cc parses); frames.tsv and sensors.tsv
are copied byte for byte. bundle.json gets diagnosticVariant = {argv, params, baseVariant: <base diagnosticVariant>}
so divergence/exact_input_report.py still finds the reference run (--capture) through baseVariant.

  python3 -I add_params.py --base <bundle dir> --out <new bundle dir> --params 'MapBuilder.bundleAdjustmentIterativeEnabled=b:1'
"""
import argparse
import json
import pathlib
import re
import shutil
import sys

ITEM = re.compile(r'^[A-Za-z][A-Za-z0-9]*\.[A-Za-z][A-Za-z0-9]*=[bifd]:[^;=\s]+$')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--params', required=True, help='"Key=b:1;Key=i:9" (bundle_reader.cc params format)')
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    items = a.params.split(';')
    if not items or not all(ITEM.match(x) for x in items):
        raise SystemExit('bad --params (expected Key.Name=t:value;... with t in b,i,f,d): ' + a.params)
    cfg = (a.base / 'engine-config.tsv').read_text()
    if any(line.split('\t', 1)[0] == 'params' for line in cfg.splitlines()):
        raise SystemExit('base already has a params line: ' + str(a.base))
    a.out.mkdir(parents=True)
    for name in ('frames.tsv', 'sensors.tsv'):
        shutil.copyfile(a.base / name, a.out / name)
    (a.out / 'engine-config.tsv').write_text(cfg.rstrip('\n') + '\nparams\t%s\n' % a.params)
    meta = json.loads((a.base / 'bundle.json').read_text())
    variant = {'argv': sys.argv, 'params': a.params, 'base': str(a.base)}
    if meta.get('diagnosticVariant'):
        variant['baseVariant'] = meta['diagnosticVariant']
    meta['diagnosticVariant'] = variant
    (a.out / 'bundle.json').write_text(json.dumps(meta, indent=1) + '\n')
    print(json.dumps({'out': str(a.out), 'params': a.params}))


if __name__ == '__main__':
    main()
