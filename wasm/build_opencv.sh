#!/bin/bash
# Build OpenCV for WASM (minimal modules for VIO)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
OPENCV_SRC="${PROJECT_ROOT}/assets/references/AlvaAR/src/libs/opencv"
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
    -DCMAKE_CXX_FLAGS="-O3" \
    -DCMAKE_C_FLAGS="-O3" \
    \
    -DBUILD_SHARED_LIBS=OFF \
    -DENABLE_PIC=OFF \
    -DCV_TRACE=OFF \
    -DCV_ENABLE_INTRINSICS=OFF \
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

emmake make -j$(nproc)
emmake make install

echo "=== OpenCV WASM build complete ==="
echo "Libraries:"
ls -la "$INSTALL_DIR/lib/"*.a 2>/dev/null || echo "(check $INSTALL_DIR/lib/)"
