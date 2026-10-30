#!/usr/bin/env bash
# Re-compile every object that build_probe.sh built for the C++17 variant with
# `-std=c++17 -Wpedantic -fsyntax-only`, to list C++20-only constructs GCC accepts as extensions
# in C++17 mode (e.g. designated initializers). Requires build_probe.sh to have run first.
# Usage: pedantic_scan.sh <out_dir>   (writes <out_dir>/cxx17-pedantic.log and a summary)
set -euo pipefail
ROOT_REPO="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_ROOT="${ROOT_REPO}/assets/references/xr_reference/official/xr_reference"
TOOL_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT="$(cd "${1:?out dir}" && pwd)"
OV="${OUT}/overlay-cxx17"
LOG="${OUT}/cxx17-pedantic.log"
: > "${LOG}"
for rel in $(awk -F'\t' '$1=="OK" && $2=="cxx17" && $3 !~ /^link/ {print $3}' "${OUT}/build-status.tsv"); do
  src="${SRC_ROOT}/${rel}"
  [[ -f "${OUT}/gen/${rel}" ]] && src="${OUT}/gen/${rel}"
  [[ -f "${OV}/${rel}" ]] && src="${OV}/${rel}"
  (cd "${SRC_ROOT}" && g++ -std=c++17 -Wpedantic -fsyntax-only -D_USE_MATH_DEFINES \
    -I"${TOOL_DIR}/shim" -I"${OV}" -I"${OUT}/gen" -I"${SRC_ROOT}" -I/usr/include/eigen3 \
    -I"${ROOT_REPO}/build/_deps/ceres-solver-src/include" \
    -I"${ROOT_REPO}/build/dev-native/_deps/ceres-solver-build/include" \
    -I"${ROOT_REPO}/build/_deps/ceres-solver-src/internal/ceres/miniglog" "${src}") >> "${LOG}" 2>&1 || \
    echo "FAILED ${rel}" >> "${LOG}"
done
grep -E 'warning: .*(C\+\+20|c\+\+20|ISO C\+\+)' "${LOG}" \
  | sed -E "s|${SRC_ROOT}/||; s|${OUT}/||" | sed -E 's/^([^:]+:[0-9]+):[0-9]+:/\1:/' \
  | sort | uniq -c | sort -rn
grep -c '^FAILED' "${LOG}" || true
