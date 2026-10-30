#!/bin/bash
# Build the U1 measurement tool pyramid_exact against an engine build directory.
#   build_pyramid_exact.sh <engine_build_dir> <out_bin> [<gr8_cpu_shader.cc override>]
# With an override source (e.g. `git show slam-refact-3:.../gr8_cpu_shader.cc`), that file is
# compiled with the engine flags and linked in front of libsr_frontend.a, so the binary measures
# the older pyramid model with everything else unchanged.
set -euo pipefail
B="$1"
OUT="$2"
OVERRIDE="${3:-}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ENGINE="$(cd "$HERE/../../../engine" && pwd)"
TP="$ENGINE/third_party"
CERES_SRC="$(sed -n 's/^SLAM_REFACT_CERES_SOURCE_DIR:PATH=//p' "$B/CMakeCache.txt")"
GTEST_SRC="$(sed -n 's/^SLAM_REFACT_GTEST_SOURCE_DIR:PATH=//p' "$B/CMakeCache.txt")"
FLAGS=(-O3 -DNDEBUG -Wall -Wextra -Werror -fno-exceptions -fno-threadsafe-statics -fno-fast-math
  -ffp-contract=off -march=x86-64 -std=c++17 -DC8_NO_EXCEPT -DEIGEN_FAST_MATH=0 -DJAVASCRIPT)
INC=(-I"$ENGINE/tests/frontend" -I"$ENGINE/include" -I"$ENGINE/src"
  -isystem "$TP/xr_shim" -isystem "$TP/xr_reference" -isystem /usr/include/eigen3
  -isystem "$TP/libcxx_compat/include" -isystem "$CERES_SRC/internal/ceres/miniglog"
  -isystem "$CERES_SRC/include" -isystem "$B/_deps/ceres_solver-build/include"
  -isystem "$GTEST_SRC/googletest/include")
OBJS=()
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
if [ -n "$OVERRIDE" ]; then
  g++ "${FLAGS[@]}" "${INC[@]}" -c "$OVERRIDE" -o "$TMP/override.o"
  OBJS+=("$TMP/override.o")
fi
g++ "${FLAGS[@]}" "${INC[@]}" -c "$HERE/pyramid_exact.cc" -o "$TMP/pyramid_exact.o"
g++ -o "$OUT" "$TMP/pyramid_exact.o" "${OBJS[@]}" "$B/tests/frontend/libsr_frontend_test_util.a" \
  "$B/src/frontend/libsr_frontend.a" "$B/src/config/libsr_config.a" \
  "$B/third_party/libsr_vendor_features.a" "$B/third_party/libsr_vendor_geometry.a" \
  "$B/third_party/libsr_vendor_features.a" "$B/third_party/libsr_vendor_geometry.a" \
  "$B/third_party/libsr_vendor_c8.a" "$B/_deps/ceres_solver-build/lib/libceres.a" \
  "$B/third_party/libsr_vendor_geometry.a" -lz
echo "built $OUT"
