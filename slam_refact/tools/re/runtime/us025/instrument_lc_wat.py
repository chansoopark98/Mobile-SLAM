#!/usr/bin/env python3
"""Probe build of the ORIGINAL xr-slam.wasm for US-025 LC-G (G-C/G-D/G-E/G-F/G-G, U2, U3; diagnostic only, never served as the reference artifact).

Method (extends instrument_1146_wat.py / orient/instrument_647_wat.py): instead of copying memory inside the module, every probe point calls the EXISTING
import env.emscripten_memcpy_big (func 245, type (i32 i32 i32) -> ()) with a magic destination 0xFFF10000 | id (never a valid address) and two i32
arguments.  The page-side script (lcgold.js hook mode + lchooks.js) answers that call and reads linear memory at that moment, so no new import, export,
global or function index is added and the code of every original function is unchanged; only stack-neutral `i32.const; <a>; <b>; call 245` sequences are
inserted (and one `local.get`/`local.set` pair is not needed: the 1146 result is already stored in local 13 by the original code).

Hook ids (id: place; arguments (a, b)):
  inside func 936 (MapBuilder::update), loop close block G936:4159-5754:
    0x01 LC start, right after `wm = *piVar56` of the loop-close block:           (frame base j = local 9, wm = local 22)
    0x02 same place:                                                                (MapBuilder = local 0, 0)
    0x10 U2 pre-select: dist vector built, selection loop not started:             (dist.begin = local 1, n_desc = local 10)
    0x11 same place:                                                                (newest.index = local 2, loop count = local 14)
    0x12 U2 post-select: after the second WorldMaps getter call (func 1113):        (best distance bits = reinterpret(local 58), cand.index = local 3)
    0x13 same place:                                                                (cand.id = local 46, found flag = local 39)
    0x31 after call 1128 (local match)                    0x32 after call 1146 + `local.set 13` (a = result in local 13)
    0x33 after call 660 (correction matrix)               0x34 after call 655 (LC setKeyframeExtrinsic)
    0x35 after call 1124 (mergePoints)                    0x36 / 0x37 after the two associate calls (658) of the match loop
    0x38 after the unlink-loop associate call (658 with a null handle)       0x39 before it (a = tree node = local 13, b = observation index = local 10)
    0x3A after it (a = node, b = obs index)                0x3B before the inner call 1127 (adjustPoints): state after the unlink loop
    0x3C unlink loop, every iteration, right after the observation index k = node->value was loaded and before getPoint(pts[k]) (a = node, b = k)
  entry of the callees (args are the function's parameters; i64 parameters are split into low/high words):
    0x40..0x42 func 1128 (p0,p1) (p2,p3) (p4,0)            0x43..0x45 func 1146 (p0,p1) (p2,p3) (p4,p5)
    0x46..0x47 func 660 (p0,p1) (p2,0)                      0x48..0x49 func 655 (p0,p1) (p2,0)
    0x4A..0x4C func 658 (p0,p1) (p2,lo p3) (hi p3,0)        0x4D..0x4F func 1124 (p0,lo p1) (hi p1,lo p2) (hi p2,0)
    0x50 func 1127 (p0,0)
usage: instrument_lc_wat.py <xr.wat> <out.wat>
  xr.wat = wasm2wat --enable-all build/slam-refact-20261015/inputs/xr-slam.wasm (wabt 1.0.4x; byte-identical to build/slam-refact-full-20261019/orient/probe647/xr.wat);
  then wat2wasm WITH ITS DEFAULT features (--enable-all emits an import encoding V8 rejects) out.wat -o xr-probe.wasm; run_lcgold.mjs --probe-wasm xr-probe.wasm.
Every anchor is asserted (the script aborts when one differs)."""
import sys

HOOK_CALL = 245  # import env.emscripten_memcpy_big, type (i32 i32 i32) -> ()


def hook(indent, hid, a, b):
    return [f'{indent}i32.const {0xFFF10000 | hid}', f'{indent}{a}', f'{indent}{b}', f'{indent}call {HOOK_CALL}']


def main():
    src, dst = sys.argv[1], sys.argv[2]
    text = open(src).read()
    assert f'(import "env" "emscripten_memcpy_big" (func (;{HOOK_CALL};) (type 4)))' in text
    assert '(type (;4;) (func (param i32 i32 i32)))' in text
    funcs = {}

    def split(n):
        head = f'  (func (;{n};)'
        s = text.index(head)
        e = text.index(f'  (func (;{n + 1};)')
        return s, e

    out = []
    pos = 0
    for n in (655, 658, 660, 936, 1124, 1127, 1128, 1146):
        s, e = split(n)
        out.append(text[pos:s])
        lines = text[s:e].split('\n')
        pos = e
        assert lines[0].startswith(f'  (func (;{n};)')
        if n != 936:
            # entry hooks: after the (local ...) line (first body line is `global.get 0`)
            assert lines[1].strip().startswith('(local') or n == 1127, (n, lines[1][:60])
            li = 1 if lines[1].strip().startswith('(local') else 0
            ins = []
            ind = '    '
            if n == 1128:
                ins = hook(ind, 0x40, 'local.get 0', 'local.get 1') + hook(ind, 0x41, 'local.get 2', 'local.get 3') + hook(ind, 0x42, 'local.get 4', 'i32.const 0')
            elif n == 1146:
                ins = hook(ind, 0x43, 'local.get 0', 'local.get 1') + hook(ind, 0x44, 'local.get 2', 'local.get 3') + hook(ind, 0x45, 'local.get 4', 'local.get 5')
            elif n == 660:
                ins = hook(ind, 0x46, 'local.get 0', 'local.get 1') + hook(ind, 0x47, 'local.get 2', 'i32.const 0')
            elif n == 655:
                ins = hook(ind, 0x48, 'local.get 0', 'local.get 1') + hook(ind, 0x49, 'local.get 2', 'i32.const 0')
            elif n == 658:
                ins = (hook(ind, 0x4A, 'local.get 0', 'local.get 1') + hook(ind, 0x4B, 'local.get 2', 'local.get 3\n    i32.wrap_i64')
                       + hook(ind, 0x4C, 'local.get 3\n    i64.const 32\n    i64.shr_u\n    i32.wrap_i64', 'i32.const 0'))
            elif n == 1124:
                ins = (hook(ind, 0x4D, 'local.get 0', 'local.get 1\n    i32.wrap_i64')
                       + hook(ind, 0x4E, 'local.get 1\n    i64.const 32\n    i64.shr_u\n    i32.wrap_i64', 'local.get 2\n    i32.wrap_i64')
                       + hook(ind, 0x4F, 'local.get 2\n    i64.const 32\n    i64.shr_u\n    i32.wrap_i64', 'i32.const 0'))
            elif n == 1127:
                ins = hook(ind, 0x50, 'local.get 0', 'i32.const 0')
            assert ins
            lines[li + 1:li + 1] = ins
            out.append('\n'.join(lines))
            continue
        # ---- func 936: anchors on stripped instruction sequences
        L = [l.strip() for l in lines]

        def find(seq):
            hits = [i for i in range(len(L) - len(seq) + 1) if all(L[i + j] == seq[j] for j in range(len(seq)))]
            assert len(hits) == 1, (seq, hits)
            return hits[0]

        def indent_of(i):
            l = lines[i]
            return l[:len(l) - len(l.lstrip())]

        after = {}   # line index -> hook lines to insert AFTER that line
        before = {}  # line index -> hook lines to insert BEFORE that line
        i = find(['call 651', 'i32.const 0', 'i32.load8_u offset=606264', 'i32.eqz', 'br_if 17 (;@2;)', 'local.get 15', 'i32.load', 'local.set 22']) + 7
        after[i] = hook(indent_of(i), 0x01, 'local.get 9', 'local.get 22') + hook(indent_of(i), 0x02, 'local.get 0', 'i32.const 0')
        i = find(['local.tee 10', 'i32.const 1', 'local.get 10', 'i32.const 1', 'i32.gt_u', 'select', 'local.set 14', 'local.get 0', 'i32.const 2616']) + 6
        after[i] = hook(indent_of(i), 0x10, 'local.get 1', 'local.get 10') + hook(indent_of(i), 0x11, 'local.get 2', 'local.get 14')
        i = find(['call 1113', 'f32.const -0x1p+0 (;=-1;)', 'local.set 59']) + 2
        after[i] = hook(indent_of(i), 0x12, 'local.get 58\n    i32.reinterpret_f32', 'local.get 3') + hook(indent_of(i), 0x13, 'local.get 46', 'local.get 39')
        i = find(['call 1128']); after[i] = hook(indent_of(i), 0x31, 'i32.const 0', 'i32.const 0')
        i = find(['call 1146', 'local.set 13']) + 1; after[i] = hook(indent_of(i), 0x32, 'local.get 13', 'i32.const 0')
        i = find(['call 660']); after[i] = hook(indent_of(i), 0x33, 'i32.const 0', 'i32.const 0')
        # 655: the last one in the function is the loop-close one (the other three are earlier, in the BA / prune parts)
        idx655 = [k for k, x in enumerate(L) if x == 'call 655']
        assert len(idx655) == 4
        i = idx655[-1]; after[i] = hook(indent_of(i), 0x34, 'i32.const 0', 'i32.const 0')
        i = find(['call 1124']); after[i] = hook(indent_of(i), 0x35, 'i32.const 0', 'i32.const 0')
        idx658 = [k for k, x in enumerate(L) if x == 'call 658']
        assert len(idx658) == 7
        for k, hid in zip(idx658[-3:], (0x36, 0x37)):
            after[k] = hook(indent_of(k), hid, 'i32.const 0', 'i32.const 0')
        k = idx658[-1]
        after[k] = hook(indent_of(k), 0x38, 'i32.const 0', 'i32.const 0') + hook(indent_of(k), 0x3A, 'local.get 13', 'local.get 10')
        before[k] = hook(indent_of(k), 0x39, 'local.get 13', 'local.get 10')
        k = find(['local.get 13', 'i32.load offset=16', 'local.tee 10', 'i32.const 80', 'i32.mul']) + 2
        after[k] = hook(indent_of(k), 0x3C, 'local.get 13', 'local.get 10')
        idx1127 = [k for k, x in enumerate(L) if x == 'call 1127']
        assert len(idx1127) == 6
        k = idx1127[-2]
        before[k] = hook(indent_of(k), 0x3B, 'i32.const 0', 'i32.const 0')
        # the loop-close ordinals above rely on the order in the function text; assert the neighbours
        assert L[idx658[-1] - 1].startswith('local.get') or L[idx658[-1] - 1].startswith('i64.')
        newlines = []
        for k, l in enumerate(lines):
            if k in before:
                newlines.extend(before[k])
            newlines.append(l)
            if k in after:
                newlines.extend(after[k])
        out.append('\n'.join(newlines))
    out.append(text[pos:])
    open(dst, 'w').write(''.join(out))
    print('patched')


if __name__ == '__main__':
    main()
