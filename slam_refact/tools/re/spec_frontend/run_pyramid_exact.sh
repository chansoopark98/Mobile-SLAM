#!/bin/bash
# Run pyramid_exact (before/after binaries) over every capture set with original pyramids.
#   run_pyramid_exact.sh <bin_dir> <out_dir>
# bin_dir holds pyramid_exact_before / pyramid_exact_after (build_pyramid_exact.sh).
set -uo pipefail
BIN="$1"
OUT="$2"
R="$(cd "$(dirname "$0")/../../../.." && pwd)/build"
mkdir -p "$OUT"
SETS=(
  "tum-c2|$R/slam-refact-20261015/spec/frontend/capture/tum-room1-full/c2|240|320|Google|Pixel 7"
  "tum-w1|$R/slam-refact-20261016/divergence/capture/tum-room1-full/w1|240|320|Google|Pixel 7"
  "flip6-s2-c1|$R/slam-refact-20261015/spec/frontend/capture/flip6-s2/c1|480|640||K"
  "flip6-s2-c2-73|$R/slam-refact-20261015/spec/frontend/capture/flip6-s2/c2-73|480|640||K"
  "flip6-s2-w270-mp|$R/slam-refact-20261016/divergence/capture/flip6-s2/w270-mp|480|640||K"
  "flip6-s3-full1|$R/slam-refact-20261016/divergence/capture/flip6-s3-lowtex/full1|480|640||K"
)
for s in "${SETS[@]}"; do
  IFS='|' read -r name dir w h manuf model <<< "$s"
  # Both binaries run the swiftshader-approx model: before = slam-refact-3, after = this patch.
  for v in before after; do
    "$BIN/pyramid_exact_$v" "$dir" "$w" "$h" swiftshader "$manuf" "$model" > "$OUT/$name-$v.jsonl" &
  done
  wait
  for v in before after; do
    printf '%s %s ' "$name" "$v"; tail -1 "$OUT/$name-$v.jsonl"
  done
done
