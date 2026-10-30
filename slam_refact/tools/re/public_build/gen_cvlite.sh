#!/usr/bin/env bash
# Recreate the Bazel genrule outputs of third_party/cvlite/core/BUILD (custom_hal, cvconfig,
# opencl_kernels_core, opencv_modules, version_string) under <gen_dir>/third_party/cvlite/core,
# byte-for-byte following the genrule `cmd`s, so cvlite can be compiled without Bazel.
# Usage: gen_cvlite.sh <gen_dir>
set -euo pipefail
D="${1:?gen dir}/third_party/cvlite/core"
mkdir -p "${D}"
: > "${D}/custom_hal.hpp"
{
  echo '#ifdef C8_OPENCV_OSX_ONLY'
  echo '  #define HAVE_AVFOUNDATION'
  echo '  #define HAVE_COCOA'
  echo '#endif // ifdef C8_OPENCV_OSX_ONLY'
} > "${D}/cvconfig.h"
{
  echo '#include "third_party/cvlite/core/ocl.hpp"'
  echo '#include "third_party/cvlite/core/ocl_genbase.hpp"'
  echo '#include "third_party/cvlite/core/opencl/ocl_defs.hpp"'
} > "${D}/opencl_kernels_core.hpp"
{
  echo '#define HAVE_OPENCV_CALIB3D'
  echo '#define HAVE_OPENCV_CORE'
  echo '#define HAVE_OPENCV_FLANN'
  echo '#define HAVE_OPENCV_FEATURES2D'
  echo '#define HAVE_OPENCV_IMGCODECS'
  echo '#define HAVE_OPENCV_IMGPROC'
  echo '#define HAVE_OPENCV_ML'
  echo '#ifdef C8_OPENCV_OSX_ONLY'
  echo '  #define HAVE_OPENCV_HIGHGUI'
  echo '  #define HAVE_OPENCV_VIDEOIO'
  echo '#endif  // C8_OPENCV_OSX_ONLY'
} > "${D}/opencv_modules.hpp"
echo '"OpenCV 3.2.0"' > "${D}/version_string.inc"
echo "generated cvlite headers in ${D}"
