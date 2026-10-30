#!/usr/bin/env python3
"""Copy the small golden vectors of architecture.md §7.2 into slam_refact/engine/tests/data.

The sources live in the (gitignored) evidence tree build/slam-refact-20261015/ and the RE lane
outputs. This script is the single place that lists them; it writes tests/data/MANIFEST.tsv with
the source path, source sha256, copied sha256, size, kind and consumer of every file. Re-running
it after the evidence changed rewrites the copies and the manifest (review the diff).

  sync_test_data.py [--repo REPO]          copy + write MANIFEST.tsv
Verification of the copies (always) and of the sources (when present) is done by
tools/check_vendor.py --data.
"""
import argparse
import gzip
import hashlib
import json
import os
import shutil
import sys

SPEC = 'build/slam-refact-20261015/spec'
CAP = SPEC + '/frontend/capture/tum-room1-full/c2'

# (destination under tests/data, source under repo, consumer). kind = copy unless noted.
COPIES = [
    ('frontend/binary-tables.json', SPEC + '/frontend/binary-tables.json', 'F-H9'),
    ('frontend/layouts.jsonl', SPEC + '/frontend/layouts.jsonl', 'F-LAY'),
    ('frontend/selftest.json', SPEC + '/frontend/selftest.json', 'F-FAST'),
    ('frontend/binary-gr8/c2-cap001.json', SPEC + '/frontend/binary-gr8/c2-cap001.json', 'F-H2 unit'),
    ('frontend/capture/tum-room1-full/c2/cap-001.rgba.gz', CAP + '/cap-001.rgba.gz', 'F-H2/H3/H4/H7, F-H8'),
    ('frontend/capture/tum-room1-full/c2/cap-002.rgba.gz', CAP + '/cap-002.rgba.gz', 'F-H2/H3/H4/H7, F-H8'),
    ('frontend/capture/tum-room1-full/c2/cap-003.rgba.gz', CAP + '/cap-003.rgba.gz', 'F-H2/H3/H4/H7, F-H8'),
    ('frontend/capture/tum-room1-full/c2/cam-001.rgba.gz', CAP + '/cam-001.rgba.gz', 'F-H5/H6'),
    ('frontend/capture/tum-room1-full/c2/cam-002.rgba.gz', CAP + '/cam-002.rgba.gz', 'F-H5/H6'),
    ('frontend/capture/tum-room1-full/c2/cam-003.rgba.gz', CAP + '/cam-003.rgba.gz', 'F-H5/H6'),
    ('frontend/gates/cpu-half-tum-room1-full-c2.json', SPEC + '/frontend/gates/cpu-half-tum-room1-full-c2.json',
     'F-H2 evidence'),
    ('frontend/gates/cpu-half-flip6-s2-c1.json', SPEC + '/frontend/gates/cpu-half-flip6-s2-c1.json', 'F-H2 evidence'),
    ('map-tracker/probe-yaw-replace-r1/cases.json', SPEC + '/map-tracker/probe-yaw-replace-r1/cases.json', 'MT-P1'),
    ('map-tracker/probe-yaw-replace-r1/summary.json', SPEC + '/map-tracker/probe-yaw-replace-r1/summary.json',
     'MT-P1'),
    ('map-tracker/status-hysteresis.json', SPEC + '/map-tracker/status-hysteresis.json', 'MT-P2'),
    ('frame-frame-tracker/probe-fft-1279-r3/golden-1279.json', 'build/slam-refact-20261015/wp2-pre/probe-fft-1279-r3/golden-1279.json', 'V-FFT-ISO'),
    ('frame-frame-tracker/probe-fft-1279-r3/cases.json', 'build/slam-refact-20261015/wp2-pre/probe-fft-1279-r3/cases.json', 'V-FFT-ISO (draw counts)'),
    ('frame-frame-tracker/probe-fft-1267-r1/cases.json', 'build/slam-refact-20261015/wp2-pre/probe-fft-1267-r1/cases.json', 'ClassifyHorizon (1267)'),
    ('frame-frame-tracker/rng-vectors.json', 'build/slam-refact-20261015/wp2-pre/rng-vectors.json', 'V-FFT (d)'),
    ('map-builder/probes/triangulate-r1/result.json', SPEC + '/map-builder/probes/triangulate-r1/result.json',
     'MB-TRI'),
    ('map-builder/cadence.json', SPEC + '/map-builder/cadence.json', 'G-M proxy'),
    ('vio-tracker/golden-motion-prediction.json', SPEC + '/vio-tracker/golden-motion-prediction.json', 'V-MP'),
    ('vio-tracker/probe-tail.json', SPEC + '/vio-tracker/probe-tail.json', 'V-*'),
    ('vio-tracker/runtime-checks.json', SPEC + '/vio-tracker/runtime-checks.json', 'V-*'),
    ('vio-tracker/reference-variability.json', SPEC + '/vio-tracker/reference-variability.json', 'V-*'),
    ('vio-tracker/probe-disable-vio.log', SPEC + '/vio-tracker/probe-disable-vio.log', 'V-3DOF (disable-vio)'),
    ('vio-tracker/wp4-tail-golden.json', 'build/slam-refact-20261015/wp4/tail-golden-r1/wp4-tail-golden.json',
     'V-H1/H2/H3, V-XR, V-3DOF, V-LATCH'),
    ('metric-scale/probe-e/result.json', SPEC + '/metric-scale/probe-e/result.json', 'MS-*'),
    ('metric-scale/fft-golden.txt', SPEC + '/metric-scale/fft-golden.txt', 'MS-*'),
    ('metric-scale/eigen-fft-check.txt', SPEC + '/metric-scale/eigen-fft-check.txt', 'MS-*'),
    # worker-mse: original 944 bits on recorded inputs (slam_refact/tools/harness/mse_precision_golden.mjs).
    ('metric-scale/mse-precision/flip6-s2-absolute-core2.json',
     'build/slam-refact-20261016/mse/golden/flip6-s2-absolute-core2.json', 'MsePrecision'),
    ('config/config-defaults.json', 'build/rea-analysis-20261015/xr_reference/config-defaults.json', 'config (211 keys)'),
    ('vendor/probe-wasm.out', 'build/slam-refact-20261015/public-build/probe-wasm.out', 'vendor probe'),
]

META_EXCERPT = ('frontend/capture/tum-room1-full/c2/meta-excerpt.json', CAP + '/meta.json', 'F-H2..H8 (meta)')
META_CAPTURES = (1, 2, 3)

# Files produced by a command rather than copied (source column = command).
AGATE_CMD = (
    'python3 -I slam_refact/tools/re/agate/gen_agate_test_data.py --golden build/slam-refact-pkgh-20261022/us025/golden --out slam_refact/engine/tests/data/agate')

LOOP_CLOSE_CMD = (
    'python3 -I slam_refact/tools/re/loop_close/gen_loop_close_test_data.py --golden build/slam-refact-pkgh-20261022/us025/golden --out slam_refact/engine/tests/data/loop_close')

GENERATED = [
    # US-025 LC-4: loop close golden (candidate-selection attempts, local-match / PnP / 660 records of the original); all 3 files by one command.
    ('loop_close/pairs.bin',
     LOOP_CLOSE_CMD,
     'loop_close_test.cc (US025 LC-4)'),
    ('loop_close/pairs.tsv',
     LOOP_CLOSE_CMD,
     'loop_close_test.cc (US025 LC-4)'),
    ('loop_close/select-attempts.tsv',
     LOOP_CLOSE_CMD,
     'loop_close_test.cc (US025 LC-4)'),
    # US-025 LC-2b: agate golden (G-A real keyframes of 3 original runs, G-B synthetic func 1263 cases); all 8 files by one command.
    ('agate/ga-flip6-s2-ga-s2.bin',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('agate/ga-flip6-s2-ga-s2.tsv',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('agate/ga-flip6-s3-lowtex-ga-s3.bin',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('agate/ga-flip6-s3-lowtex-ga-s3.tsv',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('agate/ga-tum-room1-full-ga-room1.bin',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('agate/ga-tum-room1-full-ga-room1.tsv',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('agate/gb-synthetic.bin',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('agate/gb-synthetic.tsv',
     AGATE_CMD,
     'agate_test.cc (US025 LC-2b)'),
    ('compat/libcxx-golden.json',
     'slam_refact/engine/tools/gen_libcxx_golden.sh --out slam_refact/engine/tests/data/compat/libcxx-golden.json',
     'compat (libcxx golden)'),
    ('compat/libcxx14-sort-golden.json',
     'slam_refact/engine/tools/gen_libcxx14_sort_golden.sh --out slam_refact/engine/tests/data/compat/libcxx14-sort-golden.json',
     'compat (libc++ 14 sort golden)'),
    # US015-CC-1: the original's GPU readback of planar ROI slot windows (frames 10 and 31 of the it-scale-v4 ROI pixel diagnosis).
    ('frontend/roi-draw/seq-a-f10-f31.bin.gz',
     'python3 -I slam_refact/engine/tests/frontend/gen_roi_draw_golden.py --glcap build/slam-refact-full-20261019/it-scale-v4/diag/orig-gl2/runs/e5-seq-a/o1/glcap '
     '--camera-dir build/slam-refact-pkgh-20261022/it-scale-v3-run1/gate/seq-a/mav0/cam0/data --out slam_refact/engine/tests/data/frontend/roi-draw/seq-a-f10-f31.bin.gz',
     'roi_draw_golden_test.cc (US015-CC-1)'),
    # WP3 (MB-TRI inputs, MB-EPI original-wasm golden). Generators live in tests/map_builder/.
    ('map-builder/probes/triangulate-r1/cases.tsv',
     'node slam_refact/engine/tests/map_builder/gen_triangulate_cases.mjs --result '
     'slam_refact/engine/tests/data/map-builder/probes/triangulate-r1/result.json --output '
     'slam_refact/engine/tests/data/map-builder/probes/triangulate-r1/cases.tsv',
     'MB-TRI (inputs)'),
    ('map-builder/probes/epipolar-r1/golden.tsv',
     'node slam_refact/engine/tests/map_builder/gen_epipolar_golden.mjs --input '
     'build/slam-refact-20261015/inputs/xr-slam.wasm --output '
     'slam_refact/engine/tests/data/map-builder/probes/epipolar-r1',
     'MB-EPI'),
    ('map-builder/probes/epipolar-r1/summary.json',
     'node slam_refact/engine/tests/map_builder/gen_epipolar_golden.mjs (same run as golden.tsv)',
     'MB-EPI (summary)'),
    ('metric-scale/us026/flip6-arcore-s3-absolute-core2.dev.methods.json',
     'node slam_refact/tools/harness/us026/mse_method_golden.mjs (func 944 per call, original xr-slam.wasm, methods 1-5; recorded output, not re-run by sync)',
     'US-026 mse-unit v2 (dev golden)'),
    ('metric-scale/us026/synthetic-s26001.dev.kf6.json',
     'node slam_refact/tools/re/spec_metric_scale/mse-probe-kf6.mjs (func 1170/1169/1178 alone, original xr-slam.wasm; recorded output, not re-run by sync)',
     'US-026 mse-unit v2 (dev golden)'),
    ('orient/channel-null-run/calls.json',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/intrinsics.json',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/meta.json',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/packets.json',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/probe647.json',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/tsv/captures.tsv',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/tsv/device.tsv',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/tsv/ev647.tsv',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/tsv/intrinsics.tsv',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/tsv/packets.tsv',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/tsv/raw-events.tsv',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
    ('orient/channel-null-run/tsv/stages.tsv',
     'recorded output of the US-023/024 orient gate channel null-run (tools/re/runtime/orient, evidence channel-null-run.log); checked in 2026-10-19, source run directory not re-derivable by sync_test_data.py',
     'orient_gate_test.cc OrientGate channel null-run'),
]


def sha256(path):
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for chunk in iter(lambda: f.read(1 << 20), b''):
      h.update(chunk)
  return h.hexdigest()


def write_meta_excerpt(src, dst):
  with open(src) as f:
    meta = json.load(f)
  keep = {k: meta[k] for k in ('schema', 'scenario', 'label', 'browserVersion', 'sourceHashes', 'glInfo', 'input',
                               'probe', 'rows', 'status') if k in meta}
  if isinstance(keep.get('input'), dict) and isinstance(keep['input'].get('dataset'), str):
    keep['input'] = dict(keep['input'], dataset=os.path.basename(os.path.normpath(keep['input']['dataset'])))
  caps = [c for c in meta.get('captures', []) if c.get('capture') in META_CAPTURES]
  cams = [c for c in meta.get('cameraCaptures', []) if c.get('capture') in META_CAPTURES]
  calls = {c.get('pyramidDrawnInStageCall') for c in caps} | {c.get('usedByStageCall') for c in cams}
  keep['captures'] = caps
  keep['cameraCaptures'] = cams
  keep['stageCalls'] = [s for s in meta.get('stageCalls', []) if s.get('call') in calls]
  keep['excerpt'] = {
      'source': os.path.basename(src),
      'rule': 'top-level identity keys + captures/cameraCaptures with capture in %s + their stageCalls'
              % (list(META_CAPTURES),),
  }
  with open(dst, 'w') as f:
    json.dump(keep, f, indent=1, sort_keys=True)
    f.write('\n')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--repo', default=os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))
  a = ap.parse_args()
  data = os.path.join(a.repo, 'slam_refact/engine/tests/data')
  rows = []
  for dst, src, consumer in COPIES:
    s = os.path.join(a.repo, src)
    d = os.path.join(data, dst)
    os.makedirs(os.path.dirname(d), exist_ok=True)
    shutil.copyfile(s, d)
    rows.append((dst, 'copy', src, sha256(s), sha256(d), os.path.getsize(d), consumer))
  dst, src, consumer = META_EXCERPT
  s = os.path.join(a.repo, src)
  d = os.path.join(data, dst)
  write_meta_excerpt(s, d)
  rows.append((dst, 'excerpt', src, sha256(s), sha256(d), os.path.getsize(d), consumer))
  for dst, cmd, consumer in GENERATED:
    d = os.path.join(data, dst)
    if not os.path.isfile(d):
      sys.exit('missing generated file %s; run: %s' % (dst, cmd))
    rows.append((dst, 'generated', cmd, '-', sha256(d), os.path.getsize(d), consumer))
  # Gzip captures must decompress (catches truncated copies).
  for r in rows:
    if r[0].endswith('.gz'):
      with gzip.open(os.path.join(data, r[0])) as g:
        g.read()
  rows.sort()
  with open(os.path.join(data, 'MANIFEST.tsv'), 'w') as f:
    f.write('# slam_refact engine test data (architecture.md §7.2). Written by tools/sync_test_data.py.\n')
    f.write('# Verified by tools/check_vendor.py --data (copies always; sources when the evidence tree exists).\n')
    f.write('path\tkind\tsrc_path\tsrc_sha256\tsha256\tsize\tconsumer\n')
    for r in rows:
      f.write('\t'.join(str(x) for x in r) + '\n')
  total = sum(r[5] for r in rows)
  print('wrote %d files, %d bytes, MANIFEST.tsv' % (len(rows), total))
  return 0


if __name__ == '__main__':
  sys.exit(main())
