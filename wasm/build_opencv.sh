#!/bin/bash
# Build OpenCV for WASM (minimal modules for VIO)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
OPENCV_VERSION="4.5.5"
OPENCV_COMMIT="dad26339a975b49cfb6c7dbe4bd5276c9dcb36e2"
OPENCV_CACHE="${PROJECT_ROOT}/build/_deps/opencv-${OPENCV_VERSION}-src"
OPENCV_SRC="${OPENCV_SOURCE_DIR:-$OPENCV_CACHE}"
JOBS="${MOBILE_SLAM_JOBS:-2}"
[[ "$JOBS" =~ ^[1-4]$ ]] || { echo "MOBILE_SLAM_JOBS must be 1..4" >&2; exit 2; }
BUILD_DIR="${SCRIPT_DIR}/libs/opencv_build"
INSTALL_DIR="${SCRIPT_DIR}/libs/opencv"

# Emscripten
EMSDK_DIR="${EMSDK:-$HOME/emsdk}"
EMSCRIPTEN_DIR="${EMSDK_DIR}/upstream/emscripten"
TOOLCHAIN="${EMSCRIPTEN_DIR}/cmake/Modules/Platform/Emscripten.cmake"

if [ ! -f "$TOOLCHAIN" ]; then
    echo "Error: Emscripten toolchain not found at $TOOLCHAIN"
    echo "Set EMSDK environment variable or run: source ~/emsdk/emsdk_env.sh"
    exit 1
fi

# Only a missing default cache may be acquired. Existing/explicit source trees are never replaced.
if [ ! -e "$OPENCV_SRC" ] && [ ! -L "$OPENCV_SRC" ] && [ -z "${OPENCV_SOURCE_DIR:-}" ]; then
    mkdir -p "$(dirname "$OPENCV_SRC")"
    git clone --depth 1 --single-branch --branch "$OPENCV_VERSION" \
        https://github.com/opencv/opencv.git "$OPENCV_SRC"
fi
VERSION_HEADER="$OPENCV_SRC/modules/core/include/opencv2/core/version.hpp"
if [ ! -f "$OPENCV_SRC/CMakeLists.txt" ] || [ ! -f "$VERSION_HEADER" ]; then
    echo "Incomplete OpenCV source at $OPENCV_SRC; preserve it and set OPENCV_SOURCE_DIR to a complete OpenCV $OPENCV_VERSION tree." >&2
    exit 1
fi
for field in MAJOR:4 MINOR:5 REVISION:5; do
    macro="CV_VERSION_${field%:*}"
    value="$(awk -v key="$macro" '$1 == "#define" && $2 == key { print $3; exit }' "$VERSION_HEADER")"
    if [ "$value" != "${field#*:}" ]; then
        echo "Expected OpenCV $OPENCV_VERSION source at $OPENCV_SRC; set OPENCV_SOURCE_DIR to the correct version." >&2
        exit 1
    fi
done
if [ -z "${OPENCV_SOURCE_DIR:-}" ]; then
    if [ ! -e "$OPENCV_SRC/.git" ] || [ "$(git -C "$OPENCV_SRC" rev-parse HEAD)" != "$OPENCV_COMMIT" ] \
        || ! git -C "$OPENCV_SRC" diff --quiet "$OPENCV_COMMIT" --; then
        echo "OpenCV cache does not match pinned commit $OPENCV_COMMIT; preserve it and provide OPENCV_SOURCE_DIR explicitly." >&2
        exit 1
    fi
fi

OPENCV_SRC="$(cd "$OPENCV_SRC" && pwd)"

echo "=== Building OpenCV for WASM ==="
echo "Source: $OPENCV_SRC"
echo "Build: $BUILD_DIR"
echo "Install: $INSTALL_DIR"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

emcmake cmake "$OPENCV_SRC" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_FLAGS="-O3 -msimd128" \
    -DCMAKE_C_FLAGS="-O3 -msimd128" \
    \
    -DBUILD_SHARED_LIBS=OFF \
    -DENABLE_PIC=OFF \
    -DCV_TRACE=OFF \
    -DCV_ENABLE_INTRINSICS=ON \
    -DCPU_BASELINE="" \
    -DCPU_DISPATCH="" \
    -DWITH_PTHREADS_PF=OFF \
    -DWITH_TBB=OFF \
    -DWITH_OPENMP=OFF \
    -DWITH_IPP=OFF \
    -DWITH_OPENCL=OFF \
    -DWITH_CUDA=OFF \
    -DWITH_GTK=OFF \
    -DWITH_QT=OFF \
    -DWITH_VTK=OFF \
    -DWITH_FFMPEG=OFF \
    -DWITH_V4L=OFF \
    -DWITH_GSTREAMER=OFF \
    -DWITH_1394=OFF \
    -DWITH_JASPER=OFF \
    -DWITH_TIFF=OFF \
    -DWITH_WEBP=OFF \
    -DWITH_OPENJPEG=OFF \
    -DWITH_OPENEXR=OFF \
    -DWITH_PNG=OFF \
    -DWITH_JPEG=OFF \
    -DWITH_PROTOBUF=OFF \
    -DWITH_QUIRC=OFF \
    -DWITH_EIGEN=OFF \
    \
    -DBUILD_opencv_core=ON \
    -DBUILD_opencv_imgproc=ON \
    -DBUILD_opencv_calib3d=ON \
    -DBUILD_opencv_features2d=ON \
    -DBUILD_opencv_video=ON \
    -DBUILD_opencv_flann=ON \
    \
    -DBUILD_opencv_highgui=OFF \
    -DBUILD_opencv_videoio=OFF \
    -DBUILD_opencv_imgcodecs=OFF \
    -DBUILD_opencv_photo=OFF \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_ml=OFF \
    -DBUILD_opencv_objdetect=OFF \
    -DBUILD_opencv_stitching=OFF \
    -DBUILD_opencv_gapi=OFF \
    -DBUILD_opencv_python2=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_opencv_java=OFF \
    -DBUILD_opencv_js=OFF \
    -DBUILD_opencv_ts=OFF \
    -DBUILD_opencv_apps=OFF \
    \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_DOCS=OFF \
    -DBUILD_ZLIB=ON \
    -DBUILD_ITT=OFF \
    -DINSTALL_C_EXAMPLES=OFF \
    -DINSTALL_PYTHON_EXAMPLES=OFF

emmake make -j"$JOBS"
emmake make install

echo "=== OpenCV WASM build complete ==="
echo "Libraries:"
ls -la "$INSTALL_DIR/lib/"*.a 2>/dev/null || echo "(check $INSTALL_DIR/lib/)"
