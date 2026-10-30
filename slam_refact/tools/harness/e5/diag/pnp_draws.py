#!/usr/bin/env python3
"""Draw-count model of the sampling part of robustPnP (pose-pnp.cc:627-641: sampleIndexTriples + sampleDataset) on the shared mt19937 stream, so that the
number of draws a frame consumed tells how many global matches (numPtsTotal) robustPnP saw.  Pure python, uses mt_tools (libc++ step).
  draws_for(state, N) -> draws consumed by robustPnP's sampling for N points starting from the libc++ state (x, i)."""
import mt_tools as mt

NEEDED = 105  # floor(log(0.01) / log(1 - 0.35^3)), RobustPnPParams defaults


class Stream:
    def __init__(self, x, i):
        self.x, self.i, self.n = list(x), i, 0

    def raw(self):
        o, self.i = mt.step(self.x, self.i)
        self.n += 1
        return o

    def below(self, e):  # libc++ uniform_int_distribution<uint32/int32>(0, e-1) as f_lha (dcmp func895): rejection on the w low bits
        if e == 1:
            return 0
        w = e.bit_length() - 1 + (0 if (e & (e - 1)) == 0 else 1)
        mask = (1 << w) - 1
        while True:
            u = self.raw() & mask
            if u < e:
                return u


def draws_for(x, i, N, needed=NEEDED):
    s = Stream(x, i)
    max_samples = (N - 2) * (N - 1) * N // 6
    if max_samples <= 5 * needed:
        raise NotImplementedError('enumerate-all branch (N too small)')
    seen = set()
    while len(seen) < needed:
        r1, r2, r3 = s.raw() % N, s.raw() % N, s.raw() % N
        if r1 == r2 or r1 == r3 or r2 == r3:
            continue
        seen.add(tuple(sorted((r1, r2, r3))))
    if N > min(N, 100):  # sampleDataset: shuffle of N indices (libc++ shuffle: for i = N-1 .. 1, index in [0, i])
        for k in range(N - 1, 0, -1):
            s.below(k + 1)
    return s.n
