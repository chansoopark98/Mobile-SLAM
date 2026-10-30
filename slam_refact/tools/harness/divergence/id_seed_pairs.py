#!/usr/bin/env python3
"""(draw index, original uid) pairs for a reference run: the engine replay (fixed seed 5489, ONE draw at
construction = old binary) published the same points at the same positions, so engine uid -> draw index j_e
under mt19937(5489); the original's draw index is j_e + 1 (the VioTracker ctor's second draw, DCMP:28431).
usage: id_seed_pairs.py <ref run dir> <engine replay dir> [extra]  (extra = +1 for the old binary)"""
import json, sys


def mt19937(seed, n):
    """First n raw outputs of std::mt19937(seed) (libc++ per-draw regeneration; same values)."""
    x = [seed]
    for k in range(1, 624):
        x.append((1812433253 * (x[-1] ^ (x[-1] >> 30)) + k) & 0xffffffff)
    out, i = [], 0
    while len(out) < n:
        j = (i + 1) % 624
        y = (x[i] & 0x80000000) | (x[j] & 0x7fffffff)
        v = x[(i + 397) % 624] ^ (y >> 1) ^ (0x9908b0df if y & 1 else 0)
        x[i] = v
        y = v ^ (v >> 11)
        y ^= (y << 7) & 0x9d2c5680
        y ^= (y << 15) & 0xefc60000
        y ^= y >> 18
        out.append(y)
        i = j
    return out

extra = int(sys.argv[3]) if len(sys.argv) > 3 else 1  # 0 for an engine with the +7240 draw (261da3b and later)
pos = {v: k for k, v in enumerate(mt19937(5489, 20000))}
ref = [json.loads(l) for l in open(sys.argv[1] + '/worldpoints-sampled.jsonl')]
eng = [json.loads(l) for l in open(sys.argv[2] + '/worldpoints-sampled.jsonl')]
pairs = {}
for a, b in zip(ref, eng):
    if a['frameIndex'] != b['frameIndex']:
        continue
    for p, q in zip(a['points'], b['points']):
        if p[2:] == q[2:] and q[0] in pos:
            pairs[pos[q[0]] + extra] = p[0]
ks = sorted(pairs)
print(' '.join('%d %d' % (k, pairs[k]) for k in ks[:4]), '| n', len(ks))

# Seed recovery (fix lane round 2): python3 -I id_seed_pairs.py <ref run> <engine replay of the same bundle> gives
# "j1 u1 j2 u2"; then gcc -O3 -march=x86-64-v3 id_seed_search.c -o id_seed_search and run
# id_seed_search j1 u1 j2 u2 <t> <T> for t = 0..T-1 (about 8 CPU-minutes for the whole 32-bit space);
# make_variant.py --id-seed <seed> writes it into a bundle.
