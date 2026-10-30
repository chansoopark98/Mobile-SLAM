#!/usr/bin/env bash
# Build Ceres 2.0 (the release line in the original xr-slam.wasm) for the engine's WASM build.
#
#   source ~/emsdk/emsdk_env.sh   # or let this script source it
#   bash slam_refact/engine/tools/build_wasm_ceres.sh [jobs 1-4, default 2] [2.0.0|2b32b321]
#
# Input : 2.0.0    build/_deps/ceres-2.0.0-src (official release tarball, not downloaded here:
#                  http://ceres-solver.org/ceres-solver-2.0.0.tar.gz,
#                  sha256 10298a1d75ca884aa0507d1abb0e0f04800a92871cd400d4c361b56a777a7603).
#         2b32b321 build/_deps/ceres-2b32b321-src (the reference runtime MODULE.bazel pin, from
#                  fetch_ceres_pin.sh).
# Output: build/slam-refact-wasm-ceres-<source>/{build,install}. Never writes wasm/libs (the VINS
#         product's Ceres 2.2 stays untouched). Configure the engine with
#         -DSLAM_REFACT_CERES_VERSION=2.0 (2.0.0) or additionally
#         -DSLAM_REFACT_CERES_COMMIT=2b32b321242b32f6364108694e0069f87e56326c (pin) to use it; the
#         source's VERSION / SLAM_REFACT_SOURCE file is copied next to CeresConfig.cmake so the
#         engine configure can check the commit.
# Options mirror the original's Ceres configuration (NO_THREADS: only parallel_for_nothreads.cc is
# in the binary; Eigen sparse; no SuiteSparse/CXSparse/LAPACK) and wasm/build_ceres.sh's compile
# flags (-O3 -msimd128), so the only difference to the 2.2 prebuilt is the release.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# Worktrees keep build/ in the main checkout.
if [ ! -d "$ROOT/build/_deps" ] && [ -n "${SLAM_REFACT_MAIN_ROOT:-}" ]; then ROOT=$SLAM_REFACT_MAIN_ROOT; fi
JOBS="${1:-2}"
case "$JOBS" in 1|2|3|4) ;; *) echo "jobs must be 1-4 (got $JOBS)" >&2; exit 2 ;; esac
SOURCE="${2:-2.0.0}"
case "$SOURCE" in 2.0.0|2b32b321) ;; *) echo "source must be 2.0.0 or 2b32b321 (got $SOURCE)" >&2; exit 2 ;; esac
SRC="$ROOT/build/_deps/ceres-$SOURCE-src"
OUT="$ROOT/build/slam-refact-wasm-ceres-$SOURCE"

if ! grep -q "<version>2.0.0</version>" "$SRC/package.xml" 2>/dev/null; then
  echo "Ceres 2.0.0 source not found at $SRC (see the header for the official source)" >&2
  exit 1
fi
if ! command -v emcmake >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "${EMSDK:-$HOME/emsdk}/emsdk_env.sh" >/dev/null
fi

emcmake cmake -S "$SRC" -B "$OUT/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$OUT/install" \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_FLAGS="-O3 -msimd128" \
  -DCMAKE_C_FLAGS="-O3 -msimd128" \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=OFF \
  -DBUILD_DOCUMENTATION=OFF -DPROVIDE_UNINSTALL_TARGET=OFF -DEXPORT_BUILD_DIR=OFF \
  -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DCXSPARSE=OFF -DACCELERATESPARSE=OFF \
  -DLAPACK=OFF -DEIGENSPARSE=ON -DCUSTOM_BLAS=ON -DSCHUR_SPECIALIZATIONS=ON \
  -DCERES_THREADING_MODEL=NO_THREADS \
  -DEigen3_DIR=/usr/share/eigen3/cmake
cmake --build "$OUT/build" --parallel "$JOBS"
cmake --install "$OUT/build"
test -f "$OUT/install/lib/cmake/Ceres/CeresConfig.cmake"
for f in VERSION SLAM_REFACT_SOURCE; do
  if [ -f "$SRC/$f" ]; then cp "$SRC/$f" "$OUT/install/lib/cmake/Ceres/$f"; fi
done
sha256sum "$OUT/install/lib/libceres.a"
echo "Ceres $SOURCE WASM installed at $OUT/install"
