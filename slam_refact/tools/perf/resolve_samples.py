#!/usr/bin/env python3
"""Resolves sigprof_sampler.c output against the binary's symbol table (nm -C) into self / inclusive per-function sample counts.
usage: resolve_samples.py <binary> <samples file> <out json> [--top N] [--split-frac F] [--exclude-fn SUBSTR]
--split-frac F: samples are chronological; the first fraction F (pre-init share of CPU time) and the rest are reported separately (approximation)."""
import bisect, json, re, subprocess, sys
binary, samples, out = sys.argv[1:4]
top = int(sys.argv[sys.argv.index('--top') + 1]) if '--top' in sys.argv else 40
syms = []
for line in subprocess.run(['nm', '-C', '-S', '--defined-only', binary], capture_output=True, text=True, check=True).stdout.splitlines():
    p = line.split(' ', 3)
    if len(p) == 4 and p[2] in 'tTwW':
        syms.append((int(p[0], 16), int(p[1], 16), p[3]))
syms.sort()
starts = [s[0] for s in syms]
def name(addr):
    i = bisect.bisect_right(starts, addr) - 1
    if i < 0: return '?'
    a, size, n = syms[i]
    return n if addr < a + max(size, 1) or addr - a < 4096 else '?'
def short(n):
    n = n.replace('(anonymous namespace)', '{anon}')
    return (n.split('(')[0] if '(' in n else n)[:150]
rows = []
maps = []
def libname(addr):
    for lo, hi, path in maps:
        if lo <= addr < hi: return '[' + path.rsplit('/', 1)[-1] + ']'
    return '?'
with open(samples) as f:
    base = int(f.readline().split()[1], 16)
    exe_end = max(s[0] + s[1] for s in syms)
    pos = f.tell()
    for line in f:
        if line.startswith('map '):
            _, lo, hi, path = line.split(None, 3); maps.append((int(lo, 16), int(hi, 16), path.strip()))
        else:
            fr = [int(x, 16) for x in line.split()]
            if fr:
                names = [name(a - base) if 0 <= a - base < exe_end else libname(a) for a in fr]
                if '[sigprof_sampler.so]' not in names:  # samples taken while the sampler itself dumps its buffer (mono timer keeps firing) are artifacts
                    rows.append(names)
def agg(rs):
    selfc, inc = {}, {}
    for names in rs:
        selfc[names[0]] = selfc.get(names[0], 0) + 1
        for n in set(names): inc[n] = inc.get(n, 0) + 1
    t = len(rs)
    rank = lambda d: [{'fn': short(k), 'samples': v, 'pct': round(100.0 * v / t, 2)} for k, v in sorted(d.items(), key=lambda kv: -kv[1])[:top]] if t else []
    return {'samples': t, 'self': rank(selfc), 'inclusive': rank(inc)}
res = {'all': agg(rows)}
if '--exclude-fn' in sys.argv:
    ex = sys.argv[sys.argv.index('--exclude-fn') + 1]
    kept = [n for n in rows if not any(ex in x for x in n)]
    res['without_' + re.sub(r'\W+', '_', ex)] = agg(kept)
if '--split-frac' in sys.argv:
    k = int(len(rows) * float(sys.argv[sys.argv.index('--split-frac') + 1]))
    res['pre_init_approx'], res['post_init_approx'] = agg(rows[:k]), agg(rows[k:])
json.dump(res, open(out, 'w'), indent=1)
print(res['all']['samples'], 'samples')
for r in res['all']['self'][:25]: print(f"self {r['pct']:6.2f}%  {r['fn']}")
