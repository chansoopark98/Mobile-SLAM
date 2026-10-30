"""Turn the A/B hit recordings of the original SDK (api-oracle.js option "ab", RM-3b / review M2) into plain TSV
for the C++ evidence test (engine/tests/engine/xr_api_hit_ab_test.cc).

  python3 -I slam_refact/tools/harness/oracle/extract_hit_ab.py \
      --run build/slam-refact-full-20261019/oracle-ab/flip6-s2/ab1 --out build/slam-refact-full-20261019/oracle-ab/tsv-ab1

One sample = one onUpdate callback of the original in which the whole step sequence ran (all steps see the same
internal tracker state):

  S sample frame status reason px py pz qw qx qy qz m0..m15 nworldpoints     (reality.* of the callback, base config)
  W id confidence x y z                                                      (nworldpoints rows)
  T step_id ncalls                                                          (one step: its calls, then its queries)
  U ox oy oz fw fx fy fz                                                    (updateCameraProjectionMatrix call, '-' = absent)
  G mirroredDisplay leftHandedAxes                                          (configure call, '-' = absent, else 0/1)
  Q x y nhits {type px py pz rw rx ry rz distance}*                         (XrController.hitTest(x, y, []))

Values keep the sign of zero ('-0' in the oracle JSON) and the full double precision (repr). type is the
XrHitTestResult.ResultType number. New output dir only; provenance.json holds the sha256 of the input and this script.
"""
import argparse, gzip, hashlib, json, pathlib, sys

TYPES = {'UNSPECIFIED': 0, 'FEATURE_POINT': 1, 'ESTIMATED_SURFACE': 2, 'DETECTED_SURFACE': 3}


def num(v):
    return -0.0 if v == '-0' else float(v)


def f(v):
    return repr(num(v))


def flag(v):
    return '-' if v is None else ('1' if v else '0')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    src = run / 'api-oracle.json.gz'
    d = json.load(gzip.open(src))
    if d.get('error') or not d.get('ab'):
        raise SystemExit('no usable ab recording in ' + str(src))
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    lines = []
    counts = {'samples': 0, 'steps': 0, 'queries': 0, 'hits': 0}
    for sid, s in enumerate(d['ab']):
        p, q = s['position'], s['rotation']
        lines.append('\t'.join(['S', str(sid), str(s['frameIndex']), s['status'], s['reason'], f(p['x']), f(p['y']), f(p['z']),
                                f(q['w']), f(q['x']), f(q['y']), f(q['z'])] + [repr(float(x)) for x in s['intrinsics']]
                               + [str(len(s['worldPoints']))]))
        for wp in s['worldPoints']:
            lines.append('\t'.join(['W', str(wp[0]), repr(float(wp[1])), f(wp[2]), f(wp[3]), f(wp[4])]))
        counts['samples'] += 1
        for st in s['steps']:
            lines.append('\t'.join(['T', st['id'], str(len(st['calls']))]))
            counts['steps'] += 1
            for c in st['calls']:
                args = c['args']
                if c['fn'] == 'update':
                    o, fa = args.get('origin'), args.get('facing')
                    assert set(args) <= {'origin', 'facing'}, args
                    lines.append('\t'.join(['U'] + (['-'] * 3 if o is None else [repr(float(o[k])) for k in 'xyz'])
                                           + (['-'] * 4 if fa is None else [repr(float(fa[k])) for k in 'wxyz'])))
                else:
                    assert set(args) <= {'mirroredDisplay', 'leftHandedAxes'}, args
                    lines.append('\t'.join(['G', flag(args.get('mirroredDisplay')), flag(args.get('leftHandedAxes'))]))
            for qu in st['queries']:
                fields = ['Q', repr(float(qu['x'])), repr(float(qu['y'])), str(len(qu['hits']))]
                for h in qu['hits']:
                    fields += [str(TYPES[h['type']]), f(h['position']['x']), f(h['position']['y']), f(h['position']['z']),
                               f(h['rotation']['w']), f(h['rotation']['x']), f(h['rotation']['y']), f(h['rotation']['z']), f(h['distance'])]
                    counts['hits'] += 1
                lines.append('\t'.join(fields))
                counts['queries'] += 1
    (out / 'hit_ab.tsv').write_text('\n'.join(lines) + '\n')
    sha = lambda p: hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
    (out / 'provenance.json').write_text(json.dumps({
        'argv': sys.argv, 'counts': counts,
        'sources': {str(src): sha(src), str(run / 'meta.json'): sha(run / 'meta.json'),
                    'slam_refact/tools/harness/oracle/extract_hit_ab.py': sha(__file__)},
        'tsv_sha256': sha(out / 'hit_ab.tsv')}, indent=1) + '\n')
    print(json.dumps(counts))


if __name__ == '__main__':
    main()
