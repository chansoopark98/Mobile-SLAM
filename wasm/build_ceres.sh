#!/bin/bash
# Build Ceres Solver for WASM (NO_THREADS, miniglog, Eigen-only)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${SCRIPT_DIR}/libs/ceres_build"
INSTALL_DIR="${SCRIPT_DIR}/libs/ceres"
EIGEN_DIR="${SCRIPT_DIR}/libs/opencv/include/opencv4/opencv2"  # fallback

# Emscripten
EMSDK_DIR="${EMSDK:-$HOME/emsdk}"
EMSCRIPTEN_DIR="${EMSDK_DIR}/upstream/emscripten"
TOOLCHAIN="${EMSCRIPTEN_DIR}/cmake/Modules/Platform/Emscripten.cmake"

if [ ! -f "$TOOLCHAIN" ]; then
    echo "Error: Emscripten toolchain not found at $TOOLCHAIN"
    exit 1
fi

# Download Ceres source if not cached
CERES_SRC="${SCRIPT_DIR}/libs/ceres-solver-src"
if [ ! -d "$CERES_SRC" ]; then
    echo "Downloading Ceres Solver 2.2.0..."
    mkdir -p "${SCRIPT_DIR}/libs"
    cd "${SCRIPT_DIR}/libs"
    git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git ceres-solver-src
fi

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
    -DSCHUR_SPECIALIZATIONS=OFF \
    -DCERES_THREADING_MODEL="NO_THREADS" \
    -DPROVIDE_UNINSTALL_TARGET=OFF \
    -DEigen3_DIR="/usr/share/eigen3/cmake"

emmake make -j$(nproc)
emmake make install

# Fix glog header paths (use Ceres miniglog)
echo "Fixing glog header paths..."
find "$INSTALL_DIR/include" -type f -name '*.h' -exec sed -i 's#glog/logging.h#ceres/internal/miniglog/glog/logging.h#g' {} +

echo "=== Ceres WASM build complete ==="
echo "Library:"
ls -la "$INSTALL_DIR/lib/"*.a 2>/dev/null || echo "(check $INSTALL_DIR/lib/)"
