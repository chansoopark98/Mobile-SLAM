#!/usr/bin/env python3
"""Extract a run-orient.mjs recording into the TSV files the orientation gate test reads (tests/engine/
orient_gate_test.cc). Format conversion only: no value is corrected or filtered.

  python3 -I extract_orient.py <recording dir> [--out <dir, default <recording>/tsv>]

Files (tab separated, '#' lines are comments; floats are written as float32 bit patterns in hex):
  device.tsv      key value: scenario, ua, orientation, manufacturer, model, os, osVersion (XR8.XrDevice.deviceEstimate()),
                  deviceinfo_strings (ASCII strings of the capnp DeviceInfo message handed to c8EmAsm_initDeviceInfo),
                  engine_init (last engineInit/engineOrientationChange/engineVideoSizeChange: captureW captureH rotation),
                  served_width served_height, probe_wasm_sha256 (empty without a probe build)
  raw-events.tsv  msg idx kind timestampNanos intervalNanos xbits ybits zbits   one row per event of every capnp event queue
                  (c8EmAsm_setEventQueue message, in call order; the input of func 647)
  ev647.tsv       call idx kind timeNanos intervalNanos xbits ybits zbits     the sorted frame->sensorEvents after every func 647
                  call (needs a probe build)
  packets.tsv     i intervalMs accx accy accz alpha beta gamma                 the DeviceMotion packets the page dispatched
                  (accelerationIncludingGravity m/s^2, rotationRate deg/s, `interval` number as given to the event)
  intrinsics.tsv  textureWidth textureHeight orientation m0..m15 (bits)        every change of reality.intrinsics
  captures.tsv    capture inputFrame rotation camWidth camHeight rawSha256 servedPgm   GL pyramid readbacks that were kept
"""
import argparse
import base64
import gzip
import json
import pathlib
import re
import struct
import sys


def f32bits(v):
    return '%08x' % struct.unpack('<I', struct.pack('<f', v))[0]


def hex8(h):
    """The page ships float32 bit patterns as 8 hex digits (orient-probe.js __orientHex / __orientReadProbe647): checked, not converted."""
    if not isinstance(h, str) or len(h) != 8 or any(c not in '0123456789abcdef' for c in h):
        raise SystemExit('expected an 8 digit lower case hex float32 bit pattern, got %r (a JSON number would have lost the sign of a zero)' % (h,))
    return h


def decode_event_queue(b64):
    """Single-segment capnp RequestPose message -> events (same layout as capnp-event-queue.mjs / make_variant.py)."""
    b = base64.b64decode(b64)
    nseg = struct.unpack_from('<I', b, 0)[0] + 1
    if nseg != 1:
        raise ValueError('multi-segment event queue')
    seg = 8

    def ptr(p):
        lo, hi = struct.unpack_from('<II', b, p)
        if lo == 0 and hi == 0:
            return None
        off = (lo if lo < 2**31 else lo - 2**32) >> 2
        return lo & 3, p + 8 + off * 8, hi
    kind, at, hi = ptr(seg)
    dw, np_ = hi & 0xffff, hi >> 16
    lst = None
    for k in range(np_):
        lst = ptr(at + (dw + k) * 8)
        if lst:
            break
    if not lst:
        return []
    _, lat, lhi = lst
    tlo, thi = struct.unpack_from('<II', b, lat)
    n, edw, ep = tlo >> 2, thi & 0xffff, thi >> 16
    out = []
    for e in range(n):
        base = lat + 8 + e * (edw + ep) * 8
        v = ptr(base + edw * 8)
        x, y, z = struct.unpack_from('<fff', b, v[1]) if v else (0.0, 0.0, 0.0)
        out.append({'kind': struct.unpack_from('<H', b, base)[0], 'timestampNanos': struct.unpack_from('<q', b, base + 8)[0],
                    'eventTimestampNanos': struct.unpack_from('<q', b, base + 16)[0],
                    'intervalNanos': struct.unpack_from('<q', b, base + 24)[0], 'x': x, 'y': y, 'z': z})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('recording', type=pathlib.Path)
    ap.add_argument('--out', type=pathlib.Path)
    a = ap.parse_args()
    rec = a.recording
    out = a.out or rec / 'tsv'
    out.mkdir(parents=True, exist_ok=False)
    meta = json.loads((rec / 'meta.json').read_text())
    calls = json.loads((rec / 'calls.json').read_text())
    est = meta['deviceEstimate']
    d_strings = ''
    for c in calls:
        if c[1] == 'D' and c[4]:
            raw = base64.b64decode(c[4])
            d_strings = ','.join(s.decode() for s in re.findall(rb'[ -~]{2,}', raw))
            break
    inits = [c for c in calls if c[1] in ('I', 'O', 'V')]
    init = inits[-1][3] if inits else [0, 0, 0]
    dev = [('scenario', meta['scenario']), ('label', meta['label']), ('ua', meta['ua']), ('orientation', meta['orientation']),
           ('manufacturer', est['manufacturer']), ('model', est['model']), ('os', est['os']), ('osVersion', est['osVersion']),
           ('deviceinfo_strings', d_strings), ('engine_init', ' '.join(str(int(v)) for v in init)),
           ('served_width', meta['input']['servedWidth']), ('served_height', meta['input']['servedHeight']),
           ('probe_wasm_sha256', (meta.get('probeWasm') or {}).get('sha256', ''))]
    (out / 'device.tsv').write_text(''.join('%s\t%s\n' % kv for kv in dev))

    lines = []
    msg = 0
    for c in calls:
        if c[1] != 'Q':
            continue
        for i, e in enumerate(decode_event_queue(c[4])):
            lines.append('\t'.join(map(str, [msg, i, e['kind'], e['timestampNanos'], e['intervalNanos'], f32bits(e['x']), f32bits(e['y']), f32bits(e['z'])])))
        msg += 1
    lines_raw = lines
    (out / 'raw-events.tsv').write_text('# msg idx kind timestampNanos intervalNanos xbits ybits zbits (capnp kind numbers: accel 1, gyro 2, mag 3, lin 4)\n' + '\n'.join(lines) + '\n')

    lines = []
    probe_file = rec / 'probe647.json'
    call_no = 0
    if probe_file.exists():
        for p_count, total, plist in json.loads(probe_file.read_text()):
            for call in plist:
                if call['count'] > 256:
                    raise SystemExit('probe slot overflow (> 256 events)')
                for i, e in enumerate(call['events']):
                    lines.append('\t'.join(map(str, [call_no, i, e[0], e[1], e[2], *map(hex8, e[3:6])])))
                if not call['events']:
                    lines.append('\t'.join(map(str, [call_no, -1, 0, 0, 0, '00000000', '00000000', '00000000'])) + '\t# empty call')
                call_no += 1
            if total > 16:
                raise SystemExit('probe call counter overflow (> 16 calls in one processStagedFrame)')
    (out / 'ev647.tsv').write_text('# call idx kind timeNanos intervalNanos xbits ybits zbits (post-647 kind numbers: accel 1, mag 2, gyro 3, lin 4; idx -1 = a call with no events)\n' + '\n'.join(lines) + '\n')

    pk = json.loads((rec / 'packets.json').read_text()) if (rec / 'packets.json').exists() else []
    (out / 'packets.tsv').write_text('# i intervalMs accx accy accz alpha beta gamma (doubles as repr)\n' + ''.join(
        '\t'.join([str(i), repr(float(p['intervalMs'])), *[repr(float(v)) for v in p['deviceAcceleration']],
                   repr(float(p['rotationRate']['alpha'])), repr(float(p['rotationRate']['beta'])), repr(float(p['rotationRate']['gamma']))]) + '\n'
        for i, p in enumerate(pk)))

    # stages.tsv: one row per c8EmAsm_stageFrame with the Q message set before it (exact-input replay).
    lines, msg_seen, last_msg = [], 0, -1
    pub = json.loads((rec / 'published-frames.json').read_text()) if (rec / 'published-frames.json').exists() else []
    by_us = {int(f[1]): int(f[0]) for f in pub}
    for c in calls:
        if c[1] == 'Q':
            last_msg = msg_seen
            msg_seen += 1
        elif c[1] == 'S':
            args = c[3]
            us = round(float(args[11]) * 1e6)
            frame = by_us.get(us, by_us.get(us - 1, by_us.get(us + 1, -1)))
            lines.append('\t'.join(map(str, [c[0], last_msg, frame, repr(float(args[11])), repr(float(args[12])), repr(float(args[13])),
                                              repr(float(args[4])), repr(float(args[5])), repr(float(args[6])), int(args[3]), int(args[1]), int(args[2])])))
    (out / 'stages.tsv').write_text('# stageCall msg inputFrame(-1 = bootstrap) videoTimeS frameTimeS timeNanos alpha beta gamma rotation width height (repr doubles; msg = index of the event queue set before this stage)\n' + '\n'.join(lines) + '\n')

    ich = json.loads((rec / 'intrinsics.json').read_text())
    (out / 'intrinsics.tsv').write_text('# textureWidth textureHeight orientation m0..m15 (float32 bits, copied from the page)\n' + ''.join(
        '\t'.join([str(c['textureWidth']), str(c['textureHeight']), str(c['orientation']), *map(hex8, c['bits'])]) + '\n' for c in ich))

    lines = []
    for c in meta.get('captures', []):
        args = c['stageArgs']
        served = rec / ('served-%05d.pgm' % c['inputFrame']) if c['inputFrame'] is not None else None
        lines.append('\t'.join(map(str, [c['capture'], c['inputFrame'], int(args[3]), int(args[1]), int(args[2]), c['rawSha256'],
                                         served.name if served and served.exists() else '-'])))
    (out / 'captures.tsv').write_text('# capture inputFrame rotation camWidth camHeight rawSha256 servedPgm\n' + '\n'.join(lines) + ('\n' if lines else ''))
    nan_inputs = sum(1 for l in lines_raw for h in l.split('\t')[5:8] if (int(h, 16) & 0x7f800000) == 0x7f800000 and (int(h, 16) & 0x7fffff))
    print(json.dumps({'out': str(out), 'nanInputs': nan_inputs, 'rawMessages': msg, 'ev647Calls': call_no, 'packets': len(pk), 'intrinsics': len(ich), 'captures': len(lines)}))


if __name__ == '__main__':
    main()
