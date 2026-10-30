#!/usr/bin/env python3
"""Per-frame comparison of the image-target pose chain between the ORIGINAL (instrumented wasm, itprobe.jsonl.gz) and the ENGINE (sr_e5_diag itprobe.jsonl):
  hom   solveImageTargetHomography camMotion before/after (tag 1  <->  stage "hom" pre/post)
  scale scaleTranslation(s, cam) -> result inside ImageTrackingState::track (tag 7 <-> stage "scaleT")
  pose  TrackedImage pose after track() (the next glImageTargetWarp input; tag 5 pose of the IMAGE_TARGET call)
Words are float bit patterns (matrix 16 + inverse 16).  usage: chain_compare.py --orig itprobe.jsonl.gz --engine eng.itprobe.jsonl [--frames 0-40]"""
import argparse, base64, gzip, json, re, struct


def words(b):
    return ['%08x' % w for w in struct.unpack('<%dI' % (len(b) // 4), b)]


def f(w):
    return struct.unpack('<f', struct.pack('<I', int(w, 16)))[0]


def ulp(a, b):
    def key(x):
        return x if x < 0x80000000 else 0x80000000 - (x & 0x7fffffff)
    return abs(key(int(a, 16)) - key(int(b, 16)))


def diff(a, b):
    d = [k for k in range(min(len(a), len(b))) if a[k] != b[k]]
    return d, max([ulp(a[k], b[k]) for k in d], default=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--orig', required=True); ap.add_argument('--engine', required=True); ap.add_argument('--frames', default='0-40'); ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()
    lo, hi = map(int, a.frames.split('-'))
    rows = [json.loads(l) for l in gzip.open(a.orig, 'rt')][1:]
    O = {}
    for r in rows:
        if r.get('i') is not None and r['i'] not in O:
            O[r['i']] = r
    E = {}
    for l in open(a.engine):
        r = json.loads(re.sub(r'-?nan', 'null', re.sub(r'-?inf', 'null', l)))
        E[r['i']] = r
    first = None
    for i in range(lo, hi + 1):
        if i not in O or i not in E:
            continue
        oh = [(s['a'], words(base64.b64decode(s['raw']))) for s in O[i]['slots'] if s['tag'] == 1]
        eh = [s for s in E[i]['stages'] if s['fn'] == 'hom']
        os_ = [words(base64.b64decode(s['raw'])) for s in O[i]['slots'] if s['tag'] == 7]
        es_ = [s for s in E[i]['stages'] if s['fn'] == 'scaleT']
        msg = []
        for k, ((ret, raw), e) in enumerate(zip(oh, eh)):
            post, pre = raw[0:32], raw[33:65]
            dp, up = diff(post, e['post'].split()); dq, uq = diff(pre, e['pre'].split())
            if dp or dq:
                msg.append(f'hom#{k} pre differ {len(dq)} (ulp {uq}) post differ {len(dp)} (ulp {up}) [ret orig {ret} eng {e["ok"]}, n eng {e["n"]}]')
        if len(os_) != len(es_):
            msg.append(f'scaleT calls orig {len(os_)} eng {len(es_)}')
        for k, (o, e) in enumerate(zip(os_, es_)):
            s_o, cam_o, out_o = o[0:1], o[1:33], o[34:66]
            s_e, cam_e, out_e = e['s'].split(), e['cam'].split(), e['out'].split()
            ds, us = diff(s_o, s_e); dc, uc = diff(cam_o, cam_e); do, uo = diff(out_o, out_e)
            if ds or dc or do:
                msg.append(f'scaleT#{k}: s differ {len(ds)} cam-in differ {len(dc)} (ulp {uc}) out differ {len(do)} (ulp {uo})')
        if a.verbose or msg:
            print(f'f{i}: ' + ('; '.join(msg) if msg else 'chain equal'))
        if msg and first is None:
            first = i
    print('FIRST frame with a differing chain value:', first)


if __name__ == '__main__':
    main()
