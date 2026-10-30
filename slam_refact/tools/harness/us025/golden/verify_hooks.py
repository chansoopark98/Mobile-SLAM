#!/usr/bin/env python3
"""US-025 LC-G: consistency checks on the hook-mode records of run_lcgold.mjs --probe-wasm runs (G-D/G-E/G-F/G-G; spec 12 s5, s8).

For every loop-close attempt that reached the pose stage (records 1146-in/1146-out/660-in/660-out/655-in/655-out of the same seq):
  F1  660 input A (P_old) == 1146 input pose == kf extrinsic before 655 (bit equal; HMatrix 0x84 B)
  F2  660 input B (P_new) == 1146 output pose == 655 pose argument == kf extrinsic after 655 (bit equal)
  F3  660 output C == the 0x84 B the event read from MapBuilderOutput+0x20 after the call (bit equal)
  F4  C.forward == A.inverse * B.forward in float32 with the func 789 summation order ((a12*b3 + a8*b2) + a0*b0) + a4*b1 (bit equal) and, as a separate
      REPORT, with the vendored unrolledMul order ((a0*b0 + a4*b1) + a8*b2) + a12*b3 (number of differing words, max ulp)  [U10]
  F5  C.inverse == B.inverse * A.forward with the same two orders (660 computes it when B is invertible)
  D1  1128 output: n matches, each PointMatch 20 B = (wordsIdx u32, dictionaryIdx u32, rotation f32, descriptorDist f32, dictionaryScale u8 + 3 pad); dictionaryIdx strictly increasing
  usage: verify_hooks.py --run <name>=<run dir> [...] --out <new json>"""
import argparse
import json
import math
import pathlib
import struct
import sys

import numpy as np


def f32(x):
    return np.float32(x)


def mul(a, b, order):
    """column-major 4x4 float32 product out = a*b; order 789 or 'unrolled'."""
    a = np.asarray(a, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    out = np.zeros(16, dtype=np.float32)
    # explicit loops (numpy float32 scalars keep IEEE single rounding at each operation)
    for c in range(4):
        for r in range(4):
            p0, p1, p2, p3 = (f32(a[r] * b[4 * c + 0]), f32(a[4 + r] * b[4 * c + 1]), f32(a[8 + r] * b[4 * c + 2]), f32(a[12 + r] * b[4 * c + 3]))
            if order == '789':
                out[4 * c + r] = f32(f32(f32(p3 + p2) + p0) + p1)
            else:
                out[4 * c + r] = f32(f32(f32(p0 + p1) + p2) + p3)
    return out


def ulps(x, y):
    xi = struct.unpack('<i', struct.pack('<f', float(x)))[0]
    yi = struct.unpack('<i', struct.pack('<f', float(y)))[0]
    xi = xi if xi >= 0 else -2147483648 - xi
    yi = yi if yi >= 0 else -2147483648 - yi
    return abs(xi - yi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    results = []
    for spec in a.run:
        name, d = spec.split('=', 1)
        lg = pathlib.Path(d) / 'lcgold'
        blobs = (lg / 'blobs.bin').read_bytes()
        rd = lambda r: blobs[r['off']:r['off'] + r['len']]
        hooks = [json.loads(l) for l in (lg / 'hooks.jsonl').read_text().splitlines() if l]
        events = {e['seq']: e for e in (json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l) if 'skipped' not in e}
        by = {}
        for h in hooks:
            by.setdefault(h['seq'], {}).setdefault(h['id'], []).append(h)
        for seq, recs in sorted(by.items()):
            row = {'run': name, 'seq': seq}
            if '1128-out' in recs:
                m = rd(recs['1128-out'][0]['matches'])
                n = recs['1128-out'][0]['nMatches']
                ent = [struct.unpack_from('<IIffB', m, 20 * k) for k in range(n)]   # PointMatch: wordsIdx, dictionaryIdx, rotation f32, descriptorDist f32, dictionaryScale u8 (+3 pad)
                row['D1'] = {'n': n, 'dictIdxStrictlyIncreasing': all(ent[k][1] < ent[k + 1][1] for k in range(n - 1)), 'maxDescriptorDist': max((e[3] for e in ent), default=None), 'scales': sorted({e[4] for e in ent}),
                             'selfMatch': all(e[0] == e[1] for e in ent)}
            if '660-in' in recs and '660-out' in recs and '1146-out' in recs and '655-in' in recs and '655-out' in recs:
                H = lambda r: rd(r)[:130]   # HMatrix = 2 x 16 f32 + 2 flag bytes = 0x82 B; the 2 bytes after it are uninitialised padding
                A, B, C = H(recs['660-in'][0]['A']), H(recs['660-in'][0]['B']), H(recs['660-out'][0]['C'])
                p_in, p_out = H(recs['1146-in'][0]['pose']), H(recs['1146-out'][0]['pose'])
                e_before, e_after = H(recs['655-in'][0]['extrinsicBefore']), H(recs['655-out'][0]['extrinsicAfter'])
                p_arg = H(recs['655-in'][0]['pose'])
                ev = events.get(seq)
                row['F1'] = A == p_in == e_before
                row['F2'] = B == p_out == p_arg == e_after
                row['F3'] = ev is not None and rd(ev['outLcMat'])[:130] == C
                Af, Ai = np.frombuffer(A[:64], dtype='<f4'), np.frombuffer(A[64:128], dtype='<f4')
                Bf, Bi = np.frombuffer(B[:64], dtype='<f4'), np.frombuffer(B[64:128], dtype='<f4')
                Cf, Ci = np.frombuffer(C[:64], dtype='<f4'), np.frombuffer(C[64:128], dtype='<f4')
                for key, (x, y, want) in {'F4': (Ai, Bf, Cf), 'F5': (Bi, Af, Ci)}.items():
                    res = {}
                    for order in ('789', 'unrolled'):
                        got = mul(x, y, order)
                        diff = [ulps(g, w) for g, w in zip(got, want)]
                        res[order] = {'bitEqual': bool(all(v == 0 for v in diff)), 'differingWords': sum(1 for v in diff if v), 'maxUlp': max(diff)}
                    row[key] = res
                row['flags'] = {'A': A[128:130].hex(), 'B': B[128:130].hex(), 'C': C[128:130].hex()}
                row['correctionTranslation'] = [float(v) for v in Cf[12:15]]
                row['poseRetValue'] = recs['1146-out'][0]['ret']
            results.append(row)
    ok = [r for r in results if 'F1' in r]
    summary = {'rows': len(results), 'pose_stage_rows': len(ok),
               'F1_all': all(r['F1'] for r in ok), 'F2_all': all(r['F2'] for r in ok), 'F3_all': all(r['F3'] for r in ok),
               'F4_789_bitEqual_all': all(r['F4']['789']['bitEqual'] for r in ok), 'F4_unrolled_bitEqual_all': all(r['F4']['unrolled']['bitEqual'] for r in ok),
               'F5_789_bitEqual_all': all(r['F5']['789']['bitEqual'] for r in ok), 'F5_unrolled_bitEqual_all': all(r['F5']['unrolled']['bitEqual'] for r in ok)}
    a.out.write_text(json.dumps({'summary': summary, 'rows': results}, indent=1) + '\n')
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
