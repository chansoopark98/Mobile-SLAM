#!/usr/bin/env bash
# Serial 3-repeat WASM stage timing (node) on the three flip6-arcore bundles, pinned to ONE cpu. Desktop numbers, not a phone's.
#   bash slam_refact/tools/perf/run_wasm_series.sh <out dir under build/> <wasm dir> <bundle root> <cpu>
set -u
cd "$(dirname "$0")/../../.."
OUT=$1 WASM=$2 BR=$3 CPU=$4
mkdir -p "$OUT"
for r in 1 2 3; do for n in s2 s3 l1; do
  f="$OUT/wasm-$n-r$r.json"
  [ -e "$f" ] && continue
  echo "$(date -u +%FT%TZ) start $n r$r loadavg=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$OUT/wasm-series.log"
  taskset -c "$CPU" node slam_refact/tools/perf/wasm_stage_timing.mjs --bundle "$BR/flip6-arcore-$n/k425" --wasm-dir "$WASM" --out "$f" >> "$OUT/wasm-series.log" 2>&1
done; done
echo "$(date -u +%FT%TZ) done" >> "$OUT/wasm-series.log"
