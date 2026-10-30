#!/usr/bin/env bash
# Rebuild the lane-A binary map (cwd = repo root). Python 3 stdlib only; c++filt for demangling.
set -euo pipefail
B=build/slam-refact-20261015/re
T=slam_refact/tools/re/binary_map
W=build/slam-refact-20261015/inputs/xr-slam.wasm
mkdir -p "$B/binary_map"
sha256sum "$W"
python3 -I "$T/libmatch.py" --wasm "$W" --archive wasm/libs/ceres/lib/libceres.a --out "$B/binary_map/libmatch.json"
python3 -I "$T/classify.py" --wasm "$W" --dcmp "$B/wabt/xr-slam.dcmp" \
  --public-src assets/references/xr_reference/official/xr_reference \
  --ghidra-export "$B/ghidra/export" --ghidra-extra "$B/ghidra-laneA/export" \
  --libmatch "$B/binary_map/libmatch.json" \
  --out "$B/function-index.json" --summary "$B/binary_map/summary.json" > /dev/null
python3 -I "$T/check_index.py" --index "$B/function-index.json" --wasm "$W"
python3 -I "$T/report.py" --index "$B/function-index.json" --wasm "$W" --out "$B/binary_map/report.json"
