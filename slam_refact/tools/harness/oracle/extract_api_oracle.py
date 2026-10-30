"""Turn original-SDK XrController oracle records into plain TSV for the C++ evidence tests (spec 06).

  python3 -I slam_refact/tools/harness/oracle/extract_api_oracle.py \
      --run <run dir with api-oracle.json.gz> --coords "ox oy oz fw fx fy fz scale left mirrored" \
      --dataset <dataset dir with record.json> --out <new dir>
  python3 -I slam_refact/tools/harness/oracle/extract_api_oracle.py --intrinsics-runs <runs root> --out <new dir>

Hit file (hits.tsv): C coords | S id frame px py pz qw qx qy qz m0..m15 npoints | P x y z | Q id x y mask nhits
{type px py pz rw rx ry rz distance}*.  Lighting file (lighting.tsv): L fresh frame exposure temperature prev_frame
pgm_path (prev_frame = input frame of the previous fresh callback, the 1-frame-lag hypothesis; the first callback's
pipeline frame is a bootstrap copy of frame 0).  Intrinsics file (intrinsics.tsv): I run manufacturer model capW capH
texW texH near far m0..m15.  Values keep the sign of zero ('-0' in the oracle JSON).  New output dirs only.
"""
import argparse, gzip, hashlib, json, pathlib, sys

TYPES = {'UNSPECIFIED': 0, 'FEATURE_POINT': 1, 'ESTIMATED_SURFACE': 2, 'DETECTED_SURFACE': 3}
DEVICE = {'pixel7': ('Google', 'Pixel 7'), 'reduced': ('', 'K')}


def num(v):
    return -0.0 if v == '-0' else float(v)


def f(v):
    return repr(num(v))


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run')
    ap.add_argument('--coords')
    ap.add_argument('--dataset')
    ap.add_argument('--intrinsics-runs')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    prov = {'argv': sys.argv, 'sources': {}}
    if a.intrinsics_runs:
        root = pathlib.Path(a.intrinsics_runs)
        lines = []
        for ev in sorted(root.glob('*/*/events.json')):
            meta = json.loads((ev.parent / 'meta.json').read_text())
            spec = meta.get('scenarioSpec', {})
            manu, model = DEVICE[spec.get('ua', 'pixel7')]
            inp = meta.get('input', {})
            near, far = 0.01, 1000.0  # page-runtime.js renderCameraConfiguration
            # api-oracle runs may re-send cam (pixelRect) after page-runtime: the display size is then that cam.
            cam = ((meta.get('apiOracle') or {}).get('options') or {}).get('cam') or {}
            for c in json.loads(ev.read_text()).get('intrinsicsChanges', []):
                tw, th = cam.get('pixelRectWidth', c['textureWidth']), cam.get('pixelRectHeight', c['textureHeight'])
                lines.append('\t'.join(['I', str(ev.parent.relative_to(root)), manu or '-', model, str(inp['width']), str(inp['height']),
                                        str(tw), str(th), repr(near), repr(far)] + [repr(float(x)) for x in c['intrinsics']]))
            prov['sources'][str(ev)] = sha(ev)
        (out / 'intrinsics.tsv').write_text('\n'.join(lines) + '\n')
        print('intrinsics rows', len(lines))
    if a.run:
        run = pathlib.Path(a.run)
        d = json.load(gzip.open(run / 'api-oracle.json.gz'))
        prov['sources'][str(run / 'api-oracle.json.gz')] = sha(run / 'api-oracle.json.gz')
        coords = a.coords.split()
        assert len(coords) == 10
        lines = ['C\t' + '\t'.join(coords)]
        n_q = 0
        for sid, s in enumerate(d['hits']):
            p, q = s['position'], s['rotation']
            lines.append('\t'.join(['S', str(sid), str(s['frameIndex']), f(p['x']), f(p['y']), f(p['z']), f(q['w']), f(q['x']), f(q['y']), f(q['z'])]
                                   + [repr(float(x)) for x in s['intrinsics']] + [str(len(s['worldPoints']))]))
            for wp in s['worldPoints']:
                lines.append('\t'.join(['P', f(wp[2]), f(wp[3]), f(wp[4])]))
            for qu in s['queries']:
                mask = 0
                for t in qu['types']:
                    mask |= 1 << TYPES.get(t, 0)
                fields = ['Q', str(sid), repr(float(qu['x'])), repr(float(qu['y'])), str(mask), str(len(qu['hits']))]
                for h in qu['hits']:
                    hp, hr = h['position'], h['rotation']
                    fields += [str(TYPES[h['type']]), f(hp['x']), f(hp['y']), f(hp['z']), f(hr['w']), f(hr['x']), f(hr['y']), f(hr['z']), f(h['distance'])]
                lines.append('\t'.join(fields))
                n_q += 1
        (out / 'hits.tsv').write_text('\n'.join(lines) + '\n')
        record = json.loads((pathlib.Path(a.dataset) / 'record.json').read_text())['recording']
        files = [str(pathlib.Path(a.dataset) / 'mav0/cam0/data' / fr['file']) for fr in record['frames']]
        prov['sources'][str(pathlib.Path(a.dataset) / 'record.json')] = sha(pathlib.Path(a.dataset) / 'record.json')
        ll = []
        prev = 0  # bootstrap frames repeat frame 0
        for row in d['lighting']:
            if row['frameIndex'] is None:
                continue
            l = row['lighting']
            ll.append('\t'.join(['L', str(row['fresh']), str(row['frameIndex']), f(l['exposure']), f(l['temperature']), str(prev), files[prev]]))
            prev = row['frameIndex']
        (out / 'lighting.tsv').write_text('\n'.join(ll) + '\n')
        print('samples', len(d['hits']), 'queries', n_q, 'lighting rows', len(ll))
    (out / 'provenance.json').write_text(json.dumps(prov, indent=1) + '\n')


if __name__ == '__main__':
    main()
