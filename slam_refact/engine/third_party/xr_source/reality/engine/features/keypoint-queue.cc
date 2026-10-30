// Original Author: Nicholas Butko

#include "bzl/inliner/rules2.h"

cc_library {
  hdrs = {
    "keypoint-queue.h",
  };
  deps = {
    ":image-point",
  };
}
cc_end(0xe768e357);

#include "reality/engine/features/keypoint-queue.h"

namespace c8 {}  // namespace c8
