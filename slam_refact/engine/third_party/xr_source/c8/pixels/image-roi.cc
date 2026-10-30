// Original Author: Nicholas Butko

#include "bzl/inliner/rules2.h"

cc_library {
  hdrs = {
    "image-roi.h",
  };
  deps = {
    "//c8:hmatrix",
    "//c8/geometry:parameterized-geometry",
  };
  visibility = {
    "//visibility:public",
  };
}
cc_end(0x511b5c12);

#include "c8/pixels/image-roi.h"

namespace c8 {}  // namespace c8
