#!/usr/bin/env bash
# Emscripten variant of the public xr_reference world-SLAM compile probe.
#
# Compiles the same closure as build_probe.sh (no cvlite) with em++ -std=c++20 -O2 -msimd128
# -DJAVASCRIPT (the define reference runtime's WASM build uses: single-threaded keypoint path, threadCount 0),
# using the UNMODIFIED public sources (clang accepts frame-point.h, so no overlay) plus the probe
# shims, links against the repo's WASM Ceres (wasm/libs/ceres, 2.2.0) and runs the result in node.
# Requires build_probe.sh to have generated <out_dir>/gen first.
#
# Env WASM_STACK_SIZE (bytes, default 65536 = emscripten default) sets -sSTACK_SIZE.
# Usage: build_probe_wasm.sh <out_dir> [jobs<=4]
set -euo pipefail
ROOT_REPO="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_ROOT="${ROOT_REPO}/assets/references/xr_reference/official/xr_reference"
TOOL_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT="$(cd "${1:?out dir}" && pwd)"
JOBS="${2:-4}"
if (( JOBS < 1 || JOBS > 4 )); then echo "jobs must be 1..4" >&2; exit 2; fi
# shellcheck disable=SC1091
source "${HOME}/emsdk/emsdk_env.sh" >/dev/null 2>&1
CERES_WASM="${ROOT_REPO}/wasm/libs/ceres"

mapfile -t SRCS < <(awk -F'\t' '$1=="OK" && $2=="cxx20" && $3 !~ /^link/ {print $3}' "${OUT}/build-status.tsv")
OBJ="${OUT}/obj/wasm"
mkdir -p "${OBJ}"
STATUS="${OUT}/build-status-wasm.tsv"
: > "${STATUS}"

compile_wasm() {
  local rel="$1"
  local src="${SRC_ROOT}/${rel}"
  [[ -f "${OUT}/gen/${rel}" ]] && src="${OUT}/gen/${rel}"
  local obj="${OBJ}/${rel%.*}.o"
  mkdir -p "$(dirname "${obj}")"
  if (cd "${SRC_ROOT}" && em++ -std=c++20 -O2 -msimd128 -w -DJAVASCRIPT -D_USE_MATH_DEFINES \
      -I"${TOOL_DIR}/shim" -I"${OUT}/gen" -I"${SRC_ROOT}" -I/usr/include/eigen3 \
      -I"${CERES_WASM}/include" -c "${src}" -o "${obj}") > "${obj%.o}.log" 2>&1; then
    printf 'OK\twasm\t%s\n' "${rel}"
  else
    printf 'FAIL\twasm\t%s\t%s\n' "${rel}" "$(grep -m1 error "${obj%.o}.log" | sed "s|${SRC_ROOT}/||g")"
  fi
}
export -f compile_wasm
export OUT SRC_ROOT TOOL_DIR CERES_WASM OBJ
printf '%s\n' "${SRCS[@]}" | xargs -P "${JOBS}" -I{} bash -c 'compile_wasm "$0"' {} >> "${STATUS}"

objs=()
for rel in "${SRCS[@]}"; do [[ -f "${OBJ}/${rel%.*}.o" ]] && objs+=("${OBJ}/${rel%.*}.o"); done
if (cd "${SRC_ROOT}" && em++ -std=c++20 -O2 -msimd128 -w -DJAVASCRIPT -D_USE_MATH_DEFINES \
    -I"${TOOL_DIR}/shim" -I"${OUT}/gen" -I"${SRC_ROOT}" -I/usr/include/eigen3 -I"${CERES_WASM}/include" \
    "${TOOL_DIR}/probe/probe_main.cc" "${TOOL_DIR}/shim_src/parameter-data-shim.cc" "${objs[@]}" \
    "${CERES_WASM}/lib/libceres.a" -sALLOW_MEMORY_GROWTH=1 -sSTACK_SIZE=${WASM_STACK_SIZE:-65536} -sENVIRONMENT=node \
    -o "${OUT}/probe-wasm.js") > "${OUT}/link-wasm.log" 2>&1; then
  printf 'OK\twasm\tlink probe-wasm\n' >> "${STATUS}"
else
  printf 'FAIL\twasm\tlink probe-wasm\t%s\n' "$(grep -m1 -E 'error|undefined' "${OUT}/link-wasm.log")" >> "${STATUS}"
fi
sort -o "${STATUS}" "${STATUS}"
awk -F'\t' '{n[$1]++} END {for (k in n) print k, n[k]}' "${STATUS}"
