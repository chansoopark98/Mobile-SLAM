#!/usr/bin/env python3
"""Probe build of the ORIGINAL xr-slam.wasm for US-024 / US-023 (diagnostic only; never served as the reference
artifact): at the exit of func 647 (prepareTrackingSensorFrame, tracking-sensor-event.cc:74-200) the sorted
frame->sensorEvents vector is copied into a probe buffer reachable from the new exported global "sr_probe_647"
(global 9, a pointer; 0 until the first call).
Layout (all little endian): +0 u32 number of 647 calls since the JS side last cleared it (not capped);
slot k (k < 16) at +16 + k * 10256: +0 u32 event count (copied only when <= 256, else the count is still stored and
the events are not copied), +16 .. events of 40 bytes each (TrackingSensorEvent: i32 kind@0, i64 eventTimeNanos@8,
i64 intervalNanos@16, f32 x@24, f32 y@28, f32 z@32). The computation of every original function is unchanged; only
instructions that read the frame and copy memory are added at the end of func 647.
usage: instrument_647_wat.py <xr.wat> <out.wat>
  xr.wat = wasm2wat --enable-all build/slam-refact-20261015/inputs/xr-slam.wasm (wabt 1.0.41); then wat2wasm WITH ITS
  DEFAULT features (--enable-all emits an import encoding V8 rejects, same as the func 1146 probe) out.wat -o xr-probe.wasm;
  run-orient.mjs --probe-wasm xr-probe.wasm.
Anchors checked here (the script aborts when one differs): func 647 is `(func (;647;) (type 10) (param i32 i32 i32 i32
i32)` with 44 local entries, malloc = func 300 (export "malloc"), memcpy-like helper = func 279 ((i32 i32 i32) -> i32,
the helper the func 1146 probe used), 9 globals (0..8), the function ends with `global.set 0)` before func 648."""
import sys

src, dst = sys.argv[1], sys.argv[2]
text = open(src).read()

HEADER = '  (func (;647;) (type 10) (param i32 i32 i32 i32 i32)\n    (local '
i = text.index(HEADER)
assert text.count('(func (;647;)') == 1
line_end = text.index('\n', i + len(HEADER))
locals_line = text[i + len(HEADER) - len('(local '):line_end]
entries = locals_line.replace('(local ', '').rstrip(')').split()
assert len(entries) == 44, len(entries)
first_new = 5 + len(entries)  # params 0..4, locals 5..48
n_local, slot_local, cnt_local = first_new, first_new + 1, first_new + 2

assert '(func (;300;) (type 1) (param i32) (result i32)' in text
assert '(export "malloc" (func 300))' in text
assert '(func (;279;) (type 6) (param i32 i32 i32) (result i32)' in text
GLOBAL_ANCHOR = '  (global (;8;) i32 (i32.const 529860))\n'
assert text.count(GLOBAL_ANCHOR) == 1 and '(global (;9;)' not in text

TAIL = '    local.get 5\n    i32.const 832\n    i32.add\n    global.set 0)\n  (func (;648;) (type 2) (param i32 i32)'
assert text.count(TAIL) == 1
PROBE = f'''    local.get 5
    i32.const 832
    i32.add
    global.set 0
    global.get 9
    i32.eqz
    if
      i32.const 164000
      call 300
      global.set 9
      global.get 9
      i32.const 0
      i32.store
    end
    global.get 9
    i32.load
    local.tee {n_local}
    i32.const 16
    i32.lt_u
    if
      global.get 9
      i32.const 16
      i32.add
      local.get {n_local}
      i32.const 10256
      i32.mul
      i32.add
      local.set {slot_local}
      local.get 4
      i32.load offset=148
      local.get 4
      i32.load offset=144
      i32.sub
      i32.const 40
      i32.div_u
      local.set {cnt_local}
      local.get {slot_local}
      local.get {cnt_local}
      i32.store
      local.get {cnt_local}
      i32.const 256
      i32.le_u
      if
        local.get {slot_local}
        i32.const 16
        i32.add
        local.get 4
        i32.load offset=144
        local.get {cnt_local}
        i32.const 40
        i32.mul
        call 279
        drop
      end
    end
    global.get 9
    global.get 9
    i32.load
    i32.const 1
    i32.add
    i32.store)
  (func (;648;) (type 2) (param i32 i32)'''
text = text.replace(TAIL, PROBE)
# locals of func 647: three new i32
text = text.replace(locals_line, locals_line[:-1] + ' i32 i32 i32)', 1)
text = text.replace(GLOBAL_ANCHOR, GLOBAL_ANCHOR + '  (global (;9;) (mut i32) (i32.const 0))\n', 1)
EXPORT_ANCHOR = '  (export "memory" (memory 0))\n'
assert text.count(EXPORT_ANCHOR) == 1
text = text.replace(EXPORT_ANCHOR, EXPORT_ANCHOR + '  (export "sr_probe_647" (global 9))\n', 1)
open(dst, 'w').write(text)
print('wrote', dst, 'locals', n_local, slot_local, cnt_local)
