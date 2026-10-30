#!/usr/bin/env python3
"""MT19937 state tools for the it-scale L0 diagnosis (pure python, no numpy).

The ORIGINAL (wasm, libc++) keeps x[624] updated in place plus an index i (state = 2500 bytes: 624 u32 LE + u32 index); the ENGINE (native,
libstdc++) keeps the twisted block and a position (text form "x0 .. x623 p").  Both produce the standard MT19937 output stream, so the common,
representation-independent signature of a state is its next 8 raw outputs (`sig8`).  `to_libstdcxx_text` converts a libc++ state to the text
libstdc++'s operator>> reads, so the engine can be put into the original's state (itprobe.cc ITPROBE_RNG_SET).
libc++ step (func 895 of the original, dcmp f_lha, and llvm mersenne_twister_engine::operator()):
  j = (i+1) % 624; y = (x[i] & 0x80000000) | (x[j] & 0x7fffffff); x[i] = x[(i+397) % 624] ^ (y >> 1) ^ (0x9908b0df if x[j] & 1 else 0); i = j; temper(x[i_old])"""
import base64, struct

N, M = 624, 397


def parse_state(b64_or_bytes):
    raw = base64.b64decode(b64_or_bytes) if isinstance(b64_or_bytes, str) else bytes(b64_or_bytes)
    assert len(raw) == 2500, len(raw)
    w = list(struct.unpack('<625I', raw))
    return w[:N], w[N]


def temper(z):
    z ^= (z >> 11)
    z ^= (z << 7) & 0x9d2c5680
    z ^= (z << 15) & 0xefc60000
    return (z ^ (z >> 18)) & 0xffffffff


def step(x, i):
    """one in-place libc++ draw; returns (output, new index)"""
    j = (i + 1) % N
    y = (x[i] & 0x80000000) | (x[j] & 0x7fffffff)
    x[i] = x[(i + M) % N] ^ (y >> 1) ^ (0x9908b0df if (x[j] & 1) else 0)
    return temper(x[i]), j


def seed_state(seed=5489):
    x = [seed & 0xffffffff]
    for k in range(1, N):
        x.append((1812433253 * (x[-1] ^ (x[-1] >> 30)) + k) & 0xffffffff)
    return x, 0


def sig8(x, i, n=8):
    x = list(x)
    out = []
    for _ in range(n):
        o, i = step(x, i)
        out.append(o)
    return out


def sig8_hex(x, i, n=8):
    return ['%08x' % v for v in sig8(x, i, n)]


def advance(x, i, n):
    x = list(x)
    for _ in range(n):
        _, i = step(x, i)
    return x, i


def to_libstdcxx_text(x, i):
    """libstdc++ text state: block B of the current generation + the number of outputs already consumed from it"""
    x = list(x)
    if i == 0:
        # all 624 words of the current block were produced and consumed: libstdc++ twists on the next call (p = 624)
        return ' '.join(str(v) for v in x) + ' 624'
    k = i
    for _ in range(N - i):          # finish the generation (words >= i are still of the previous one)
        _, k = step(x, k)
    assert k == 0
    return ' '.join(str(v) for v in x) + ' ' + str(i)


def state_key(x, i):
    return tuple(sig8(x, i, 4))


if __name__ == '__main__':
    import sys
    # self test: standard check value of std::mt19937 (default seed 5489): the 10000th output is 4123659995
    x, i = seed_state()
    last = None
    for _ in range(10000):
        last, i = step(x, i)
    assert last == 4123659995, last
    x0, i0 = seed_state()
    assert sig8(x0, i0, 3) == [3499211612, 581869302, 3890346734]
    print('mt_tools self test ok')
