#!/usr/bin/env bash
# Serial 3-repeat native replay (production binary, no probe, no sampler) on the three flip6-arcore bundles, pinned to ONE cpu. Desktop numbers, not a phone's.
#   bash slam_refact/tools/perf/run_native_series.sh <out dir under build/> <slam_refact_replay> <bundle root> <cpu>
set -u
cd "$(dirname "$0")/../../.."
OUT=$1 BIN=$2 BR=$3 CPU=$4
mkdir -p "$OUT"
for r in 1 2 3; do for n in s2 s3 l1; do
  d="$OUT/native-$n-r$r"
  [ -e "$d" ] && continue
  echo "$(date -u +%FT%TZ) start $n r$r loadavg=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$OUT/native-series.log"
  taskset -c "$CPU" "$BIN" --bundle "$BR/flip6-arcore-$n/k425" --out "$d" >> "$OUT/native-series.log" 2>&1
done; done
echo "$(date -u +%FT%TZ) done" >> "$OUT/native-series.log"
