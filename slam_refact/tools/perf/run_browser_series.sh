#!/usr/bin/env bash
# Serial 3-repeat browser timing (deployed chunk + deployed WASM, system Chrome headless + SwiftShader), pinned to P cores 0-15 like the US-010 series.
#   bash slam_refact/tools/perf/run_browser_series.sh <out root under build/> [max-frames]
set -u
cd "$(dirname "$0")/../../.."
OUT=$1 MF=${2:-300}
mkdir -p "$OUT"
for r in 1 2 3; do
  echo "$(date -u +%FT%TZ) start br-r$r loadavg=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$OUT/browser-series.log"
  taskset -c 0-15 node slam_refact/tools/perf/browser_stage_timing.mjs --label "br-r$r" --out-root "$OUT" --max-frames "$MF" --browser /usr/bin/google-chrome >> "$OUT/browser-series.log" 2>&1
done
echo "$(date -u +%FT%TZ) done" >> "$OUT/browser-series.log"
