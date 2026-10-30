#!/usr/bin/env bash
# Build the spec lane S1 frontend probe against lane B's public-build C++17 archives.
# Usage: build.sh [out_dir]   (default build/slam-refact-20261015/spec/frontend/bin)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
TOOL="$(cd "$(dirname "$0")" && pwd)"
PB="${ROOT}/build/slam-refact-20261015/public-build"
SRC="${ROOT}/assets/references/xr_reference/official/xr_reference"
OUT="${1:-${ROOT}/build/slam-refact-20261015/spec/frontend/bin}"
mkdir -p "${OUT}"
for f in "${PB}/lib/cxx17/libslamfeat.a" "${PB}/lib/cxx17/libc8core.a" "${PB}/overlay-cxx17"; do
  [[ -e "${f}" ]] || { echo "missing ${f} (run slam_refact/tools/re/public_build/build_probe.sh first)" >&2; exit 2; }
done
INC=(-I"${TOOL}" -I"${ROOT}/slam_refact/tools/re/public_build/shim" -I"${PB}/overlay-cxx17" -I"${PB}/gen"
     -I"${SRC}" -I/usr/include/eigen3)
g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -D_USE_MATH_DEFINES "${INC[@]}" \
  -c "${TOOL}/gr8_gpu_ref.cc" -o "${OUT}/gr8_gpu_ref.o"
# Public headers are compiled with -w (same as lane B); our own TU above is warning-clean.
g++ -std=c++17 -O2 -w -D_USE_MATH_DEFINES "${INC[@]}" "${TOOL}/frontend_probe.cc" "${OUT}/gr8_gpu_ref.o" \
  "${ROOT}/slam_refact/tools/re/public_build/shim_src/parameter-data-shim.cc" \
  "${PB}/lib/cxx17/libslamfeat.a" "${PB}/lib/cxx17/libc8core.a" -lpthread -o "${OUT}/frontend_probe"
echo "${OUT}/frontend_probe"
