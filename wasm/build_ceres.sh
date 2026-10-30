#!/bin/bash
# Build Ceres Solver for WASM (NO_THREADS, miniglog, Eigen-only)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${SCRIPT_DIR}/libs/ceres_build"
INSTALL_DIR="${SCRIPT_DIR}/libs/ceres"
CERES_VERSION="2.2.0"
CERES_COMMIT="85331393dc0dff09f6fb9903ab0c4bfa3e134b01"
CERES_CACHE="${PROJECT_ROOT}/build/_deps/ceres-solver-src"
CERES_SRC="${CERES_SOURCE_DIR:-$CERES_CACHE}"
JOBS="${MOBILE_SLAM_JOBS:-2}"
[[ "$JOBS" =~ ^[1-4]$ ]] || { echo "MOBILE_SLAM_JOBS must be 1..4" >&2; exit 2; }

# Emscripten
EMSDK_DIR="${EMSDK:-$HOME/emsdk}"
EMSCRIPTEN_DIR="${EMSDK_DIR}/upstream/emscripten"
TOOLCHAIN="${EMSCRIPTEN_DIR}/cmake/Modules/Platform/Emscripten.cmake"

if [ ! -f "$TOOLCHAIN" ]; then
    echo "Error: Emscripten toolchain not found at $TOOLCHAIN"
    exit 1
fi

# Only a missing default cache may be acquired. Never clone over a partial or explicit tree.
if [ ! -e "$CERES_SRC" ] && [ ! -L "$CERES_SRC" ] && [ -z "${CERES_SOURCE_DIR:-}" ]; then
    mkdir -p "$(dirname "$CERES_SRC")"
    git clone --depth 1 --single-branch --branch "$CERES_VERSION" \
        https://github.com/ceres-solver/ceres-solver.git "$CERES_SRC"
fi
VERSION_HEADER="$CERES_SRC/include/ceres/version.h"
if [ ! -f "$CERES_SRC/CMakeLists.txt" ] || [ ! -f "$VERSION_HEADER" ]; then
    echo "Incomplete Ceres source at $CERES_SRC; preserve it and set CERES_SOURCE_DIR to a complete Ceres $CERES_VERSION tree." >&2
    exit 1
fi
for field in MAJOR:2 MINOR:2 REVISION:0; do
    macro="CERES_VERSION_${field%:*}"
    value="$(awk -v key="$macro" '$1 == "#define" && $2 == key { print $3; exit }' "$VERSION_HEADER")"
    if [ "$value" != "${field#*:}" ]; then
        echo "Expected Ceres $CERES_VERSION source at $CERES_SRC; set CERES_SOURCE_DIR to the correct version." >&2
        exit 1
    fi
done
if [ -z "${CERES_SOURCE_DIR:-}" ]; then
    if [ ! -e "$CERES_SRC/.git" ] || [ "$(git -C "$CERES_SRC" rev-parse HEAD)" != "$CERES_COMMIT" ] \
        || ! git -C "$CERES_SRC" diff --quiet "$CERES_COMMIT" --; then
        echo "Ceres cache does not match pinned commit $CERES_COMMIT; preserve it and provide CERES_SOURCE_DIR explicitly." >&2
        exit 1
    fi
fi

CERES_SRC="$(cd "$CERES_SRC" && pwd)"

echo "=== Building Ceres for WASM ==="
echo "Source: $CERES_SRC"
echo "Build: $BUILD_DIR"
echo "Install: $INSTALL_DIR"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

emcmake cmake "$CERES_SRC" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_FLAGS="-O3 -msimd128" \
    -DCMAKE_C_FLAGS="-O3 -msimd128" \
    \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_BENCHMARKS=OFF \
    \
    -DMINIGLOG=ON \
    -DGFLAGS=OFF \
    -DSUITESPARSE=OFF \
    -DCXSPARSE=OFF \
    -DLAPACK=OFF \
    -DEIGENSPARSE=ON \
    -DSCHUR_SPECIALIZATIONS=ON \
    -DCERES_THREADING_MODEL="NO_THREADS" \
    -DPROVIDE_UNINSTALL_TARGET=OFF \
    -DEigen3_DIR="/usr/share/eigen3/cmake"

emmake make -j"$JOBS"
emmake make install

# Fix glog header paths (use Ceres miniglog)
echo "Fixing glog header paths..."
find "$INSTALL_DIR/include" -type f -name '*.h' -exec sed -i 's#glog/logging.h#ceres/internal/miniglog/glog/logging.h#g' {} +

echo "=== Ceres WASM build complete ==="
echo "Library:"
ls -la "$INSTALL_DIR/lib/"*.a 2>/dev/null || echo "(check $INSTALL_DIR/lib/)"
