#!/usr/bin/env python3
"""Compare glImageTargetWarp calls (inputs and result) of the ORIGINAL (tag-5 slots of itprobe.jsonl.gz, instrument_it_wat.py) with the ENGINE (ITPROBE_WARP_OUT of
sr_e5_diag) per frame.  Calls of a frame are matched by their targetK/camK/pose input words (HIRES_SCAN calls have constant poses; the IMAGE_TARGET call is the one
whose pose is not a pure z-translation).  Words are float bit patterns; matrix = 16 floats + inverse 16 floats.
  warp_compare.py --orig itprobe.jsonl.gz --engine warp.jsonl [--frames 0-40] [--ulps]"""
import argparse, base64, gzip, json, struct, sys


def words(b):
    return ['%08x' % w for w in struct.unpack('<%dI' % (len(b) // 4), b)]


def load_orig(path):
    rows = [json.loads(l) for l in gzip.open(path, 'rt')]
    out = {}
    for r in rows[1:]:
        if r.get('i') is None:
            continue
        calls = []
        for s in r['slots']:
            if s['tag'] == 5:
                raw = base64.b64decode(s['raw'])
                calls.append({'tk': words(raw[0:24]), 'ck': words(raw[24:48]), 'pose': words(raw[48:48 + 128]), 'out': words(raw[180:180 + 128])})
        out.setdefault(r['i'], calls)
    return out


def load_engine(path):
    out = {}
    for l in open(path):
        r = json.loads(l)
        out.setdefault(r['i'], []).append({'tk': r['tk'].split(), 'ck': r['ck'].split(), 'pose': r['pose'].split(), 'out': r['out'].split()})
    return out


def f(w):
    return struct.unpack('<f', struct.pack('<I', int(w, 16)))[0]


def ulp(a, b):
    ia, ib = int(a, 16), int(b, 16)
    def key(x):
        return x if x < 0x80000000 else 0x80000000 - (x & 0x7fffffff)
    return abs(key(ia) - key(ib))


def is_hires(c):  # pure z translation pose: matrix entries other than the diagonal and [14] are zero
    return all(f(c['pose'][k]) == 0.0 for k in (1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--orig', required=True); ap.add_argument('--engine', required=True); ap.add_argument('--frames', default='0-70')
    a = ap.parse_args()
    lo, hi = map(int, a.frames.split('-'))
    O, E = load_orig(a.orig), load_engine(a.engine)
    first = {'pose_input': None, 'out_given_equal_input': None, 'target_present': None}
    for i in range(lo, hi + 1):
        if i not in O or i not in E:
            continue
        ot = [c for c in O[i] if not is_hires(c)]
        et = [c for c in E[i] if not is_hires(c)]
        oh = [c for c in O[i] if is_hires(c)]
        eh = [c for c in E[i] if is_hires(c)]
        line = f'f{i}: calls orig {len(O[i])} eng {len(E[i])} | HIRES in/out equal: '
        line += ''.join('=' if (a_['pose'] == b_['pose'] and a_['out'] == b_['out']) else 'x' for a_, b_ in zip(oh, eh)) or '-'
        if ot and et:
            a_, b_ = ot[0], et[0]
            pin = [k for k in range(32) if a_['pose'][k] != b_['pose'][k]]
            kin = a_['tk'] != b_['tk'] or a_['ck'] != b_['ck']
            outd = [k for k in range(32) if a_['out'][k] != b_['out'][k]]
            line += f' | TARGET pose words differing {len(pin)} (max ulp {max([ulp(a_["pose"][k], b_["pose"][k]) for k in pin], default=0)}), K differ {kin}, out words differing {len(outd)} (max ulp {max([ulp(a_["out"][k], b_["out"][k]) for k in outd], default=0)})'
            if pin and first['pose_input'] is None:
                first['pose_input'] = i
            if not pin and not kin and outd and first['out_given_equal_input'] is None:
                first['out_given_equal_input'] = i
        elif ot or et:
            line += ' | TARGET call only in ' + ('orig' if ot else 'engine')
            if first['target_present'] is None:
                first['target_present'] = i
        print(line)
    print('FIRST:', first)


if __name__ == '__main__':
    main()
