#!/usr/bin/env python3
"""Probe build of the ORIGINAL xr-slam.wasm (diagnostic only; never served as the reference artifact):
func 1146 (poseEstimationAnalytical) copies its inputs at entry into a buffer reachable from the new exported
global "sr_probe_1146" (global 9): +0 call count (memprobe resets it after every processStagedFrame), slot k at
+64 + k*8192 for the first 4 calls: [n, input pose 0x84 bytes, obs n*24 bytes, world points n*16 bytes].
The computation of every original function is unchanged; only instructions that copy memory are added.
usage: instrument_1146_wat.py <xr.wat> <out.wat>
  xr.wat = wasm2wat --enable-all build/slam-refact-20261015/inputs/xr-slam.wasm; then wat2wasm (DEFAULT features:
  --enable-all emits an import encoding V8 rejects) out.wat -o xr-probe.wasm; capture_window.mjs --probe-wasm xr-probe.wasm.
  Validation (fix lane round 2): the instrumented 1146 returns the original's output bits on 252/252 engine inputs."""
import sys
src, dst = sys.argv[1], sys.argv[2]
lines = open(src).read().split('\n')
out = []
i = 0
in1146 = False
done_func = done_global = done_export = False
INSTR = '''    global.get 9
    i32.eqz
    if
      i32.const 32832
      call 370
      global.set 9
      global.get 9
      i32.const 0
      i32.store
    end
    global.get 9
    i32.load
    local.tee 82
    i32.const 4
    i32.lt_u
    if
      global.get 9
      i32.const 64
      i32.add
      local.get 82
      i32.const 8192
      i32.mul
      i32.add
      local.set 83
      local.get 0
      i32.load offset=4
      local.get 0
      i32.load
      i32.sub
      i32.const 24
      i32.div_u
      local.set 84
      local.get 84
      i32.const 150
      i32.le_u
      if
        local.get 83
        local.get 84
        i32.store
        local.get 83
        i32.const 4
        i32.add
        local.get 3
        i32.const 132
        call 279
        drop
        local.get 83
        i32.const 136
        i32.add
        local.get 0
        i32.load
        local.get 84
        i32.const 24
        i32.mul
        call 279
        drop
        local.get 83
        i32.const 136
        i32.add
        local.get 84
        i32.const 24
        i32.mul
        i32.add
        local.get 1
        i32.load
        local.get 84
        i32.const 16
        i32.mul
        call 279
        drop
      end
    end
    global.get 9
    local.get 82
    i32.const 1
    i32.add
    i32.store'''
while i < len(lines):
    l = lines[i]
    if l.startswith('  (func (;1146;)'):
        out.append(l)
        loc = lines[i + 1]
        assert loc.strip().startswith('(local ') and len(loc.split('(local')[1].split()) == 76, loc[:80]
        out.append(loc)
        out.append('    (local i32 i32 i32)')
        out.append(INSTR)
        i += 2
        done_func = True
        continue
    if l.startswith('  (global (;8;)'):
        out.append(l)
        out.append('  (global (;9;) (mut i32) (i32.const 0))')
        done_global = True
        i += 1
        continue
    if l.startswith('  (export "memory"') and not done_export:
        out.append(l)
        out.append('  (export "sr_probe_1146" (global 9))')
        done_export = True
        i += 1
        continue
    out.append(l)
    i += 1
assert done_func and done_global and done_export
open(dst, 'w').write('\n'.join(out))
print('patched')
