#!/usr/bin/env python3
"""Per-Wasm-function CPU table from a CDP .cpuprofile of a headless reference runtime run (lane C, US-004).

Identifies the xr-slam module through wasm-scripts.json (scriptId -> sha256 of the module bytes, written by
run-reference.mjs --profile), maps every profile node of that module to a function index (from the V8 name
"$funcN"/"wasm-function[N]" when present, otherwise from the code offset in callFrame.columnNumber using the
code section of the canonical xr-slam.wasm), and aggregates self/total time.  Function indices are in the full
Wasm function index space (imports first), the same space wasm-decompile/wasm-objdump use.
If lane A's function-index.json exists it is joined by index; otherwise the table is index-only.

Usage: profile_table.py --run <run dir> [--wasm build/slam-refact-20261015/inputs/xr-slam.wasm]
                        [--function-index build/slam-refact-20261015/re/function-index.json]
"""
import argparse
import bisect
import collections
import json
import pathlib
import re


def leb(buf, pos):
    result, shift = 0, 0
    while True:
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            return result, pos


def wasm_layout(path):
    """Return (imported function count, [(size_field_offset, body_end, func_index)]) for defined functions."""
    buf = path.read_bytes()
    assert buf[:4] == b'\0asm'
    pos, n_imported_funcs, bodies = 8, 0, []
    while pos < len(buf):
        sid = buf[pos]
        size, p = leb(buf, pos + 1)
        end = p + size
        if sid == 2:  # import section
            count, q = leb(buf, p)
            for _ in range(count):
                ln, q = leb(buf, q); q += ln
                ln, q = leb(buf, q); q += ln
                kind = buf[q]; q += 1
                if kind == 0:
                    _, q = leb(buf, q); n_imported_funcs += 1
                elif kind == 1:
                    q += 1; flags, q = leb(buf, q); _, q = leb(buf, q)
                    if flags & 1:
                        _, q = leb(buf, q)
                elif kind == 2:
                    flags, q = leb(buf, q); _, q = leb(buf, q)
                    if flags & 1:
                        _, q = leb(buf, q)
                elif kind == 3:
                    q += 2
                elif kind == 4:
                    q += 1; _, q = leb(buf, q)
        elif sid == 10:  # code section
            count, q = leb(buf, p)
            for i in range(count):
                start = q
                body, q = leb(buf, q)
                bodies.append((start, q + body, n_imported_funcs + i))
                q += body
        pos = end
    return n_imported_funcs, bodies


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=pathlib.Path)
    root = pathlib.Path(__file__).resolve().parents[4]
    ap.add_argument('--wasm', type=pathlib.Path, default=root / 'build/slam-refact-20261015/inputs/xr-slam.wasm')
    ap.add_argument('--function-index', type=pathlib.Path, default=root / 'build/slam-refact-20261015/re/function-index.json')
    ap.add_argument('--top', type=int, default=60)
    a = ap.parse_args()
    prof = json.loads((a.run / 'profile.cpuprofile').read_text())
    scripts = json.loads((a.run / 'wasm-scripts.json').read_text())
    xr_ids = {s['scriptId'] for s in scripts if s.get('isXrSlam')}
    wasm_ids = {s['scriptId']: s for s in scripts}
    n_imp, bodies = wasm_layout(a.wasm)
    starts = [b[0] for b in bodies]

    def offset_to_index(col):
        k = bisect.bisect_right(starts, col) - 1
        if 0 <= k < len(bodies) and bodies[k][0] <= col < bodies[k][1]:
            return bodies[k][2]
        return None

    nodes = {n['id']: n for n in prof['nodes']}
    parent = {}
    for n in prof['nodes']:
        for c in n.get('children', []):
            parent[c] = n['id']
    name_hits = offset_hits = mismatches = 0
    key_of = {}
    for nid, n in nodes.items():
        cf = n['callFrame']
        sid = cf.get('scriptId')
        if sid in xr_ids:
            m = re.match(r'^(?:\$func|wasm-function\[)(\d+)', cf['functionName'])
            by_off = offset_to_index(cf.get('columnNumber', -1))
            idx = int(m.group(1)) if m else by_off
            if m:
                name_hits += 1
                if by_off is not None and by_off != idx:
                    mismatches += 1
            elif by_off is not None:
                offset_hits += 1
            key_of[nid] = ('xr-slam', idx, cf['functionName'])
        elif sid in wasm_ids:
            key_of[nid] = ('wasm-other:' + (wasm_ids[sid].get('sha256') or '?')[:12], None, cf['functionName'])
        else:
            key_of[nid] = ('js' if cf.get('url') else 'native', None, cf['functionName'] or '(anonymous)', cf.get('url', '').split('/')[-1])
    samples, deltas = prof['samples'], prof['timeDeltas']
    dur = [deltas[i + 1] if i + 1 < len(deltas) else sorted(deltas)[len(deltas) // 2] for i in range(len(samples))]
    self_us, total_us = collections.Counter(), collections.Counter()
    xr_self_by_idx, xr_total_by_idx = collections.Counter(), collections.Counter()
    in_xr_us = 0
    stack_cache = {}

    def stack(nid):
        if nid not in stack_cache:
            chain, cur = [], nid
            while cur is not None:
                chain.append(cur)
                cur = parent.get(cur)
            stack_cache[nid] = chain
        return stack_cache[nid]

    for nid, d in zip(samples, dur):
        k = key_of[nid]
        self_us[k] += d
        if k[0] == 'xr-slam':
            xr_self_by_idx[k[1]] += d
        seen, seen_idx, xr_seen = set(), set(), False
        for s in stack(nid):
            ks = key_of[s]
            if ks not in seen:
                seen.add(ks); total_us[ks] += d
            if ks[0] == 'xr-slam':
                xr_seen = True
                if ks[1] not in seen_idx:
                    seen_idx.add(ks[1]); xr_total_by_idx[ks[1]] += d
        in_xr_us += d if xr_seen else 0
    total = sum(dur)
    findex = None
    if a.function_index.exists():
        raw = json.loads(a.function_index.read_text())
        items = raw.get('functions', raw) if isinstance(raw, dict) else raw
        if isinstance(items, dict):
            findex = {int(k): v for k, v in items.items()}
        else:
            findex = {int(it.get('index', it.get('func_index', -1))): it for it in items}
    names = collections.defaultdict(set)
    for k in key_of.values():
        if k[0] == 'xr-slam':
            names[k[1]].add(k[2])
    rows = []
    for idx, us in sorted(xr_total_by_idx.items(), key=lambda kv: -xr_self_by_idx[kv[0]]):
        row = {'funcIndex': idx, 'profileNames': sorted(names[idx]), 'selfMs': xr_self_by_idx[idx] / 1000, 'totalMs': us / 1000,
               'selfPctOfProfile': 100 * xr_self_by_idx[idx] / total, 'totalPctOfProfile': 100 * us / total,
               'selfPctOfXrSlam': 100 * xr_self_by_idx[idx] / in_xr_us if in_xr_us else None,
               'totalPctOfXrSlam': 100 * us / in_xr_us if in_xr_us else None}
        if findex is not None:
            row['laneA'] = findex.get(idx)
        rows.append(row)
    categories = collections.Counter()
    for k, us in self_us.items():
        categories[k[0] if not k[0].startswith('wasm-other') else 'wasm-other'] += us
        if k[0] == 'native':
            categories['native:' + k[2]] += us
    top_js = sorted(((k, v) for k, v in self_us.items() if k[0] == 'js'), key=lambda kv: -kv[1])[:25]
    out = {'run': str(a.run), 'profileDurationMs': (prof['endTime'] - prof['startTime']) / 1000, 'sampledMs': total / 1000,
           'samples': len(samples), 'xrSlamScriptIds': sorted(xr_ids), 'importedFunctionCount': n_imp, 'definedFunctionCount': len(bodies),
           'nodeIndexResolution': {'fromV8Name': name_hits, 'fromCodeOffset': offset_hits, 'nameOffsetMismatches': mismatches},
           'inclusiveXrSlamMs': in_xr_us / 1000, 'inclusiveXrSlamPct': 100 * in_xr_us / total,
           'selfByCategoryMs': {k: v / 1000 for k, v in categories.most_common()},
           'functionIndexJoin': str(a.function_index) if findex is not None else None,
           'functionIndexJoinNote': None if findex is not None else 'lane A function-index.json not present at analysis time; table is index-only',
           'topJsSelfMs': [{'function': k[2], 'file': k[3], 'selfMs': v / 1000} for k, v in top_js],
           'xrSlamFunctions': rows}
    (a.run / 'profile-table.json').write_text(json.dumps(out, indent=1) + '\n')
    with open(a.run / 'profile-table.tsv', 'w') as f:
        f.write('func_index\tself_ms\ttotal_ms\tself_pct_xrslam\ttotal_pct_xrslam\tprofile_name\tlaneA\n')
        for r in rows:
            f.write(f"{r['funcIndex']}\t{r['selfMs']:.1f}\t{r['totalMs']:.1f}\t{(r['selfPctOfXrSlam'] or 0):.2f}\t{(r['totalPctOfXrSlam'] or 0):.2f}\t"
                    f"{'|'.join(r['profileNames'])}\t{json.dumps(r.get('laneA')) if findex is not None else ''}\n")
    brief = {k: out[k] for k in ('profileDurationMs', 'sampledMs', 'inclusiveXrSlamMs', 'inclusiveXrSlamPct', 'nodeIndexResolution', 'functionIndexJoinNote')}
    brief['selfByCategoryMs'] = dict(list(out['selfByCategoryMs'].items())[:10])
    brief['top'] = [(r['funcIndex'], round(r['selfMs']), round(r['totalMs']), r['profileNames'][:1]) for r in rows[:a.top // 3]]
    print(json.dumps(brief, indent=1))


if __name__ == '__main__':
    main()
