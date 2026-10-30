#!/usr/bin/env bash
# Records the registered gate recordings of orient-us024-gate.v1 / orient-us023-gate.v1 (and the REPORT recordings of
# orient-us023-gate.v1 section 6) with the ORIGINAL SDK in headless Chrome, one browser at a time, new labels only
# (run-orient.mjs refuses an existing label), then extracts the TSV files and writes SHA256SUMS per recording.
#   bash slam_refact/tools/re/runtime/orient/record_gate_recordings.sh   (repo root; ~ 25 min)
# The comparison with the engine is NOT done here (that is the held gate).
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
R=slam_refact/tools/re/runtime/orient
O=build/slam-refact-full-20261019/orient/original
P=build/slam-refact-full-20261019/orient/probe647/xr-probe.wasm
PAT="16,16.67,0.016667,20.32,8.33,33.4,0.008"
LOG=build/slam-refact-full-20261019/orient/record-gate-recordings.log
rec() {  # label, extra args...
  local label=$1; shift
  echo "=== $(date -Is) $label: run-orient.mjs --scenario flip6-s2 --label $label $*" | tee -a "$LOG"
  node $R/run-orient.mjs --scenario flip6-s2 --label "$label" --out-root "$O" "$@" 2>&1 | grep -E '"status"|"fresh"|error' | tee -a "$LOG"
  python3 -I $R/extract_orient.py "$O/flip6-s2/$label" | tee -a "$LOG"
  (cd "$O/flip6-s2/$label" && find . -type f ! -name SHA256SUMS | sort | sed 's|^\./||' | xargs sha256sum > SHA256SUMS)
}
# US-024 sensors (func 647 probe build)
rec rec-a-ios-12promax --ua iphone-12promax --max-frames 60 --interval-pattern "$PAT" --zero-every 4 --probe-wasm "$P"
rec rec-a-ios-14pro-nz --ua iphone-14pro --max-frames 60 --interval-pattern "$PAT" --zero-every 5 --zero-value=-1e-320 --probe-wasm "$P"
rec rec-a-android-pixel7 --ua pixel7 --max-frames 60 --interval-pattern "$PAT" --zero-every 4 --probe-wasm "$P"
rec rec-a-android-reduced-nz --ua reduced --max-frames 60 --interval-pattern "$PAT" --zero-every 5 --zero-value=-1e-320 --probe-wasm "$P"
# US-024 K (original wasm as shipped)
rec rec-k-ios-se1 --ua iphone-se1 --max-frames 6
rec rec-k-ios-8 --ua iphone-8 --max-frames 6
rec rec-k-ios-xr --ua iphone-xr --max-frames 6
rec rec-k-ios-12 --ua iphone-12 --max-frames 6
rec rec-k-ios-12promax --ua iphone-12promax --max-frames 6
rec rec-k-ios-14pro --ua iphone-14pro --max-frames 6
rec rec-k-ios-15plus --ua iphone-15plus --max-frames 6
# US-023 pyramid, K, pose
for o in 0 90 180 270; do
  rec rec-b-o$o --ua reduced --orientation $o --max-frames 150 --max-captures 60 --from-stage 8 --save-frames
done
rec rec-b-px90 --ua pixel7 --orientation 90 --max-frames 8
rec rec-b-px270 --ua pixel7 --orientation 270 --max-frames 8
# US-023 sensors in rotated orientations (probe build)
for o in 90 180 270; do
  rec rec-b-s$o --ua pixel7 --orientation $o --max-frames 60 --interval-pattern "$PAT" --zero-every 4 --probe-wasm "$P"
done
# REPORT recordings (orient-us023-gate.v1 section 6)
for r in o0-a o0-b o90-a o90-b; do
  o=${r%%-*}; o=${o#o}
  rec rec-r-$r --ua reduced --orientation $o --max-frames 300 --save-frames
done
echo "=== done $(date -Is)" | tee -a "$LOG"
