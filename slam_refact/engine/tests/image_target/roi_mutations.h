// Mutations of the ROI warp used by the RM-6 gates (rm6-it-roi-gate.v1.md): input variations that
// are equivalent to a vertical flip, a scale error or a shift inside the ROI draw.

#ifndef SLAM_REFACT_TESTS_IMAGE_TARGET_ROI_MUTATIONS_H_
#define SLAM_REFACT_TESTS_IMAGE_TARGET_ROI_MUTATIONS_H_

#include <functional>
#include <vector>

#include "c8/hmatrix.h"

namespace slam_refact::image_target::testing {

struct Mutation {
  const char* name;
  std::function<c8::HMatrix(const c8::HMatrix&)> apply;
};

inline std::vector<Mutation> Mutations() {
  return {
      {"MF vertical flip",
       [](const c8::HMatrix& w) {
         return c8::HMatrix{{1.f, 0.f, 0.f, 0.f}, {0.f, -1.f, 0.f, 0.f}, {0.f, 0.f, 1.f, 0.f},
                            {0.f, 0.f, 0.f, 1.f}} * w;
       }},
      {"MZ zoom 1.15",
       [](const c8::HMatrix& w) {
         return c8::HMatrix{{1.15f, 0.f, 0.f, 0.f}, {0.f, 1.15f, 0.f, 0.f}, {0.f, 0.f, 1.f, 0.f},
                            {0.f, 0.f, 0.f, 1.f}} * w;
       }},
      {"MH shift +0.06 NDC",
       [](const c8::HMatrix& w) {
         return c8::HMatrix{{1.f, 0.f, 0.f, 0.06f}, {0.f, 1.f, 0.f, 0.f}, {0.f, 0.f, 1.f, 0.f},
                            {0.f, 0.f, 0.f, 1.f}} * w;
       }},
  };
}

}  // namespace slam_refact::image_target::testing

#endif  // SLAM_REFACT_TESTS_IMAGE_TARGET_ROI_MUTATIONS_H_
