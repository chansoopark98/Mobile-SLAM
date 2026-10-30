#!/usr/bin/env python3
"""Annotate a line range of wasm-decompile output with resolved strings and callee identities.

Usage (python3 -I, repo root):
  annotate_dcmp.py --dcmp DCMP --wasm WASM --index FUNCTION_INDEX_JSON --start L0 --end L1 [--out FILE]

For each line it appends ` //@ ...` notes:
  * integer literals that point at the start of a printable NUL-terminated string in the data
    segments (len >= 3) -> the string;
  * `load offset` / `[N]` forms are not resolved (struct offsets);
  * calls `f_xxx(` -> `funcN class/module` from the function index.
Also prints a call summary (line, callee, class/module) to stderr-free section at the end when --calls.
Stdlib only.
"""
import argparse
import json
import re
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wasm_data import load_memory  # noqa: E402

FUNC_RE = re.compile(r'^(?:export )?function ([A-Za-z0-9_$]+)\(.*// func(\d+)')
CALL_RE = re.compile(r'\b([A-Za-z_$][A-Za-z0-9_$]*)\(')
INT_RE = re.compile(r'(?<![\w.\[])(\d{3,7})(?![\w.\]])')
SEG_RE = re.compile(r'd_eGg8CBvTUUUUUUgB9BkPFNUnO53q\[(\d+)\]')
SEG_BASE = 1024  # offset of the first data segment (dcmp line 15)


def printable_string(mem, addr):
    if addr <= 0 or addr >= len(mem):
        return None
    if addr > 0 and mem[addr - 1] != 0:
        return None
    end = mem.find(b'\0', addr)
    if end < 0 or end - addr < 3 or end - addr > 300:
        return None
    s = mem[addr:end]
    if all(32 <= c < 127 or c in (9, 10) for c in s):
        return s.decode('ascii')
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dcmp', required=True)
    ap.add_argument('--wasm', required=True)
    ap.add_argument('--index', required=True)
    ap.add_argument('--start', type=int, required=True)
    ap.add_argument('--end', type=int, required=True)
    ap.add_argument('--out')
    ap.add_argument('--calls', action='store_true')
    a = ap.parse_args()

    mem = load_memory(a.wasm)
    idx = json.load(open(a.index))
    recs = idx if isinstance(idx, list) else idx.get('functions', idx)
    if isinstance(recs, dict):
        recs = list(recs.values())
    by_index = {r['index']: r for r in recs}

    name_to_idx = {}
    lines = open(a.dcmp, encoding='utf-8', errors='replace').read().split('\n')
    for ln in lines:
        m = FUNC_RE.match(ln)
        if m:
            name_to_idx[m.group(1)] = int(m.group(2))

    out = []
    calls = []
    for no in range(a.start, a.end + 1):
        ln = lines[no - 1]
        notes = []
        for m in CALL_RE.finditer(ln):
            n = m.group(1)
            if n in name_to_idx:
                i = name_to_idx[n]
                r = by_index.get(i, {})
                tag = f"func{i} {r.get('class', '?')}/{r.get('module') or '-'}"
                notes.append(tag)
                calls.append((no, i, r.get('class'), r.get('module'), r.get('size')))
        for m in INT_RE.finditer(ln):
            s = printable_string(mem, int(m.group(1)))
            if s:
                notes.append(f'"{s}"')
        # segment-relative refs: d_<seg>[N] with the first segment based at 1024 (inline SSO copies)
        for m in SEG_RE.finditer(ln):
            addr = SEG_BASE + int(m.group(1))
            end = mem.find(b'\0', addr)
            raw = mem[addr:min(end, addr + 48)]
            if raw and all(32 <= c < 127 for c in raw):
                notes.append(f'seg+{m.group(1)}="{raw.decode()}"')
        out.append(f'{no}: {ln}' + ('  //@ ' + ' | '.join(notes) if notes else ''))
    if a.calls:
        out.append('==== calls ====')
        for c in calls:
            out.append('%d func%d %s/%s size=%s' % c)
    text = '\n'.join(out) + '\n'
    if a.out:
        open(a.out, 'w').write(text)
    else:
        sys.stdout.write(text)


if __name__ == '__main__':
    main()
