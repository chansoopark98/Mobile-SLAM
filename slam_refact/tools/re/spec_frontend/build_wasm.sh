#!/usr/bin/env bash
# em++ (libc++) build of frontend_probe against lane B's WASM objects of the public sources, so the
# std::nth_element order in Gr8Gl::retainBest matches the libc++ family the original binary uses.
# Usage: build_wasm.sh [out_dir]  (default build/slam-refact-20261015/spec/frontend/bin)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
TOOL="$(cd "$(dirname "$0")" && pwd)"
PB="${ROOT}/build/slam-refact-20261015/public-build"
SRC="${ROOT}/assets/references/xr_reference/official/xr_reference"
OUT="${1:-${ROOT}/build/slam-refact-20261015/spec/frontend/bin}"
mkdir -p "${OUT}"
# shellcheck disable=SC1091
source "${HOME}/emsdk/emsdk_env.sh" >/dev/null 2>&1
mapfile -t OBJS < <(find "${PB}/obj/wasm/c8" "${PB}/obj/wasm/reality/engine/features" \
  "${PB}/obj/wasm/reality/engine/binning" "${PB}/obj/wasm/reality/engine/tracking" -name '*.o' | sort)
em++ -std=c++20 -O2 -msimd128 -w -DJAVASCRIPT -D_USE_MATH_DEFINES -I"${TOOL}" \
  -I"${ROOT}/slam_refact/tools/re/public_build/shim" -I"${PB}/gen" -I"${SRC}" -I/usr/include/eigen3 \
  "${TOOL}/frontend_probe.cc" "${TOOL}/gr8_gpu_ref.cc" \
  "${ROOT}/slam_refact/tools/re/public_build/shim_src/parameter-data-shim.cc" "${OBJS[@]}" \
  -sALLOW_MEMORY_GROWTH=1 -sSTACK_SIZE=1048576 -sENVIRONMENT=node -sNODERAWFS=1 -sEXIT_RUNTIME=1 \
  -o "${OUT}/frontend_probe_wasm.js"
echo "${OUT}/frontend_probe_wasm.js"
