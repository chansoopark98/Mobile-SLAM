#!/usr/bin/env bash
# Divergence lane: replay every gate scenario (the exact wp5b-r2 / wp5b-r2-inj bundles) with the
# diagnostic binary into build/slam-refact-20261016/divergence/runs/<scenario>/<label>. 3 at a time.
#   run_all_fix.sh <label>
set -euo pipefail
label=$1
H=slam_refact/tools/harness/divergence
BN=build/slam-refact-20261015/harness/bundles
OUT=build/slam-refact-20261016/divergence/runs
cd "$(dirname "$0")/../../../.."
jobs=(
  "tum-room1-full wp5b-r2"
  "tum-room1-full-absolute wp5b-r2"
  "flip6-s2 wp5b-r2"
  "flip6-n1 wp5b-r2"
  "flip6-s2-absolute wp5b-r2"
  "tum-room1-lowtex wp5b-r2"
  "flip6-s3-lowtex wp5b-r2"
  "flip6-s2 wp5b-r2-inj"
  "tum-room1-full wp5b-r2-inj"
)
for j in "${jobs[@]}"; do
  set -- $j
  suffix=""; [[ $2 == *-inj ]] && suffix="-inj"
  echo "$BN/$1/$2 $OUT/$1/$label$suffix"
done | xargs -P 3 -L 1 bash "$H/run_variant.sh"
