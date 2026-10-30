#!/usr/bin/env python3
"""Probe build of the ORIGINAL xr-slam.wasm for the image-target tracker (diagnostic only; never served as the reference artifact).

usage: instrument_it_wat.py <xr.wat> <out.wat> [--test-exports]
  xr.wat = wasm2wat build/slam-refact-20261015/inputs/xr-slam.wasm (default features; wat2wasm with default features, as for the 1146 probe).

Adds (original functions keep their bodies; only instructions that copy/count memory are added, results are passed through unchanged):
  global 9 (mut i32)  exported as "sr_probe_it": pointer to a log buffer allocated with func 370 (operator new) at the first log call.
                      +0 u32 slot count since the last read (JS resets it after every processStagedFrame), +4 u32 last RandomNumbers* seen at func 784,
                      slots at +64 + k*2560 (k < 12; further slots go to a dummy slot at +30784 but still count).
  slot layout         +0 tag, +4 a, +8 b, +12 c, +16 d, +32.. copy of the 2500 bytes of the mt19937 (x[624] then the libc++ index at +2496)
    tag 3  func 784 entry (VioTracker::trackWithTargets-level call, once per processed frame): state before any RNG use of the frame
    tag 2  func 992 (ImageTrackingState::nextTargetViewPose, the only caller is func 938): entry state; a = *found after the call, b = RNG index after
    tag 1  func 1022 (solveImageTargetHomography, both call sites inside 992): a = return value, b = number of target rays (HPoint2 = 12 B: x, y, w), c = non-zero inlier bytes
    tag 5  func 670 (glImageTargetWarp inlined: called from func 509 and func 784): payload targetK 24 B @+32, camK 24 B @+56, camMotion HMatrix 132 B @+80, result HMatrix 132 B @+212, geometry 8 B @+344
    tag 6  func 992 after the call: a = found, nextPose HMatrix (132 B) @+32;  tag 1 also carries the camMotion HMatrix after (+32) and before (+164) the solve
    tag 7  func 661 (scaleTranslation) call sites inside func 938: scale float @+32, cam HMatrix 132 B @+36, result HMatrix 132 B @+168
    tag 4  func 1025 (robustP2P, call site inside 992 only): a = return value, b = number of world points, c = non-zero inlier bytes
  robustPnP is inlined into func 992 (no scope string, sampling loop with 3 draws per trio at dcmp 965-1010): its outcome is `found` of tag 2 when
  no tag-1 slot of the same call succeeded.
Unit checks: tools/harness/e5/diag/unit_check_itprobe.mjs (Node, stub imports, function level; no browser).
"""
import sys

PARAMS7 = '(param i32 i32 i32 i32 i32 i32 i32)'
HELPERS = r'''
  (func $sr_slot (result i32)
    (local $n i32)
    global.get 9
    i32.eqz
    if
      i32.const 36000
      call 370
      global.set 9
      global.get 9
      i32.const 0
      i32.store
    end
    global.get 9
    i32.load
    local.set $n
    global.get 9
    local.get $n
    i32.const 1
    i32.add
    i32.store
    local.get $n
    i32.const 12
    i32.lt_u
    if (result i32)
      global.get 9
      i32.const 64
      i32.add
      local.get $n
      i32.const 2560
      i32.mul
      i32.add
    else
      global.get 9
      i32.const 30784
      i32.add
    end)
  (func $sr_log_state (param $rng i32) (param $tag i32) (result i32)
    (local $s i32)
    call $sr_slot
    local.set $s
    local.get $s
    local.get $tag
    i32.store
    local.get $rng
    i32.eqz
    if
    else
      global.get 9
      local.get $rng
      i32.store offset=4
      local.get $s
      i32.const 32
      i32.add
      local.get $rng
      i32.const 2500
      call 279
      drop
    end
    local.get $s)
  (func $sr_nonzero (param $vec i32) (result i32)
    (local $p i32) (local $e i32) (local $c i32)
    local.get $vec
    i32.load
    local.set $p
    local.get $vec
    i32.load offset=4
    local.set $e
    block
      loop
        local.get $p
        local.get $e
        i32.ge_u
        br_if 1
        local.get $c
        local.get $p
        i32.load8_u
        i32.const 0
        i32.ne
        i32.add
        local.set $c
        local.get $p
        i32.const 1
        i32.add
        local.set $p
        br 0
      end
    end
    local.get $c)
  (func $sr_w1022 ''' + PARAMS7 + r''' (result i32)
    (local $ret i32) (local $s i32)
    call $sr_slot
    local.set $s
    local.get $s
    i32.const 164
    i32.add
    local.get 4
    i32.const 132
    call 279
    drop
    local.get 0
    local.get 1
    local.get 2
    local.get 3
    local.get 4
    local.get 5
    local.get 6
    call 1022
    local.set $ret
    local.get $s
    i32.const 1
    i32.store
    local.get $s
    local.get $ret
    i32.store offset=4
    local.get $s
    local.get 0
    i32.load offset=4
    local.get 0
    i32.load
    i32.sub
    i32.const 12
    i32.div_u
    i32.store offset=8
    local.get $s
    local.get 5
    call $sr_nonzero
    i32.store offset=12
    local.get $s
    i32.const 32
    i32.add
    local.get 4
    i32.const 132
    call 279
    drop
    local.get $ret)
  (func $sr_w1025 (param i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32) (result i32)
    (local $ret i32) (local $s i32)
    local.get 0
    local.get 1
    local.get 2
    local.get 3
    local.get 4
    local.get 5
    local.get 6
    local.get 7
    local.get 8
    local.get 9
    local.get 10
    call 1025
    local.set $ret
    call $sr_slot
    local.set $s
    local.get $s
    i32.const 4
    i32.store
    local.get $s
    local.get $ret
    i32.store offset=4
    local.get $s
    local.get 0
    i32.load offset=4
    local.get 0
    i32.load
    i32.sub
    i32.const 4
    i32.shr_u
    i32.store offset=8
    local.get $s
    local.get 8
    call $sr_nonzero
    i32.store offset=12
    local.get $ret)
  (func $sr_w992 ''' + PARAMS7 + r'''
    (local $s i32)
    local.get 6
    i32.const 2
    call $sr_log_state
    local.set $s
    local.get 0
    local.get 1
    local.get 2
    local.get 3
    local.get 4
    local.get 5
    local.get 6
    call 992
    local.get $s
    local.get 5
    i32.load8_u
    i32.store offset=4
    local.get $s
    local.get 6
    i32.load offset=2496
    i32.store offset=8
    call $sr_slot
    local.set $s
    local.get $s
    i32.const 6
    i32.store
    local.get $s
    local.get 5
    i32.load8_u
    i32.store offset=4
    local.get $s
    i32.const 32
    i32.add
    local.get 4
    i32.const 132
    call 279
    drop)
  (func $sr_w661 (param i32 f32 i32)
    (local $s i32)
    call $sr_slot
    local.set $s
    local.get $s
    i32.const 7
    i32.store
    local.get $s
    local.get 1
    f32.store offset=32
    local.get $s
    i32.const 36
    i32.add
    local.get 2
    i32.const 132
    call 279
    drop
    local.get 0
    local.get 1
    local.get 2
    call 661
    local.get $s
    i32.const 168
    i32.add
    local.get 0
    i32.const 132
    call 279
    drop)
  (func $sr_w670 (param i32 i32 i32 i32 i32)
    (local $s i32)
    call $sr_slot
    local.set $s
    local.get $s
    i32.const 5
    i32.store
    local.get $s
    i32.const 32
    i32.add
    local.get 1
    i32.const 24
    call 279
    drop
    local.get $s
    i32.const 56
    i32.add
    local.get 2
    i32.const 24
    call 279
    drop
    local.get $s
    i32.const 80
    i32.add
    local.get 3
    i32.const 132
    call 279
    drop
    local.get $s
    i32.const 344
    i32.add
    local.get 4
    i32.const 8
    call 279
    drop
    local.get 0
    local.get 1
    local.get 2
    local.get 3
    local.get 4
    call 670
    local.get $s
    i32.const 212
    i32.add
    local.get 0
    i32.const 132
    call 279
    drop)
'''
ENTRY_784 = '''    local.get 4
    i32.const 3
    call $sr_log_state
    drop'''


def main():
    src, dst = sys.argv[1], sys.argv[2]
    test_exports = '--test-exports' in sys.argv[3:]
    lines = open(src).read().split('\n')
    out = []
    cur = None
    n_repl = {'938:call 992': 0, '992:call 1022': 0, '992:call 1025': 0}
    done = {'784': False, 'helpers': False, 'global': False, 'export': False}
    i = 0
    while i < len(lines):
        l = lines[i]
        s = l.strip()
        s_ = s
        if l.startswith('  (func (;'):
            cur = l.split('(;')[1].split(';)')[0]
            if cur == '5713' and False:
                pass
        # helpers go right before the first non-func module field that follows the last function (table)
        if l.startswith('  (table (;0;)') and not done['helpers']:
            out.append(HELPERS.rstrip('\n'))
            done['helpers'] = True
        if cur == '938' and s == 'call 661':
            out.append(l.replace('call 661', 'call $sr_w661'))
            n_repl['661:938'] = n_repl.get('661:938', 0) + 1
            i += 1
            continue
        if cur in ('509', '784') and s == 'call 670':
            out.append(l.replace('call 670', 'call $sr_w670'))
            n_repl['670:' + cur] = n_repl.get('670:' + cur, 0) + 1
            i += 1
            continue
        if cur == '784' and l.startswith('  (func (;784;)') and not done['784']:
            out.append(l)
            loc = lines[i + 1]
            assert loc.strip().startswith('(local '), loc[:60]
            out.append(loc)
            out.append(ENTRY_784)
            i += 2
            done['784'] = True
            continue
        if cur == '938' and s == 'call 992':
            out.append(l.replace('call 992', 'call $sr_w992'))
            n_repl['938:call 992'] += 1
            i += 1
            continue
        if cur == '992' and s == 'call 1022':
            out.append(l.replace('call 1022', 'call $sr_w1022'))
            n_repl['992:call 1022'] += 1
            i += 1
            continue
        if cur == '992' and s == 'call 1025':
            out.append(l.replace('call 1025', 'call $sr_w1025'))
            n_repl['992:call 1025'] += 1
            i += 1
            continue
        if l.startswith('  (global (;8;)'):
            out.append(l)
            out.append('  (global (;9;) (mut i32) (i32.const 0))')
            done['global'] = True
            i += 1
            continue
        if l.startswith('  (export "memory"') and not done['export']:
            out.append(l)
            out.append('  (export "sr_probe_it" (global 9))')
            if test_exports:
                out.append('  (export "sr_w1022" (func $sr_w1022))')
                out.append('  (export "sr_log_state" (func $sr_log_state))')
                out.append('  (export "sr_w992" (func $sr_w992))')
                out.append('  (export "sr_orig_1022" (func 1022))')
                out.append('  (export "sr_orig_992" (func 992))')
                out.append('  (export "sr_w670" (func $sr_w670))')
                out.append('  (export "sr_orig_670" (func 670))')
                out.append('  (export "sr_w661" (func $sr_w661))')
                out.append('  (export "sr_orig_661" (func 661))')
            done['export'] = True
            i += 1
            continue
        out.append(l)
        i += 1
    assert n_repl == {'938:call 992': 1, '992:call 1022': 2, '992:call 1025': 1, '670:509': 1, '670:784': 1, '661:938': 4}, n_repl
    assert all(done.values()), done
    open(dst, 'w').write('\n'.join(out))
    print('patched', n_repl)


main()
