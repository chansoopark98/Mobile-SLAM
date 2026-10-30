// ROI list helpers of GlRealityFrame (PUB:reality/engine/features/gl-reality-frame.cc:617-693).

#include "image_target/image_roi_list.h"

#include <cmath>
#include <string>

#include "c8/hmatrix.h"
#include "common/c8_adapters.h"
#include "reality/engine/geometry/image-warp.h"

namespace slam_refact::image_target {

void AddNextDrawRoi(const c8::ImageRoi& roi, ImageRoiList* next) {
  if (next->rois.size() >= static_cast<size_t>(kMaxTrackedImageTargets)) {
    return;  // idx >= roiLayouts_.size()
  }
  next->rois.push_back(roi);
}

void AddNextDrawHiResScans(const PinholeIntrinsics& processing, ImageRoiList* next) {
  const c8_PixelPinholeCameraModel intrinsics = ToC8Intrinsics(processing);
  const int start_at = static_cast<int>(next->rois.size());
  for (int i = start_at; i < kMaxTrackedImageTargets; ++i) {
    const int pyrscale = (i - start_at) + 1;
    // Same expression as the original (std::pow(float, int) evaluates in double).
    const float scale = 1.0f - std::pow(1.44f, 2 + pyrscale);
    const c8::HMatrix roi = c8::glImageTargetWarp(
        intrinsics, intrinsics, c8::HMatrixGen::translation(0.0f, 0.0f, scale));
    c8::ImageRoi r;
    r.source = c8::ImageRoi::Source::HIRES_SCAN;
    r.faceId = -pyrscale;
    r.name = "INTERNAL_HIRES_SCAN_ROI";
    r.warp = roi;
    AddNextDrawRoi(r, next);
  }
}

}  // namespace slam_refact::image_target
