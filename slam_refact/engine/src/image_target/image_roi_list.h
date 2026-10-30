// Pyramid ROI list of the image target path (contract FULL-3, US-013): the opaque public
// slam_refact::ImageRoiList (image_targets.h) carrying c8::ImageRoi values between the 938 output,
// GlueRunner (GlRealityFrame::addNextDrawRoi / addNextDrawHiResScans) and the pyramid draw.

#ifndef SLAM_REFACT_IMAGE_TARGET_IMAGE_ROI_LIST_H_
#define SLAM_REFACT_IMAGE_TARGET_IMAGE_ROI_LIST_H_

#include <vector>

#include "c8/pixels/image-roi.h"
#include "slam_refact/image_targets.h"
#include "slam_refact/types.h"

namespace slam_refact {

struct ImageRoiList {
  std::vector<c8::ImageRoi> rois;
};

namespace image_target {

// GlRealityFrame::addNextDrawRoi (PUB:reality/engine/features/gl-reality-frame.cc:617-623): an ROI
// beyond the kMaxTrackedImageTargets slots is dropped.
void AddNextDrawRoi(const c8::ImageRoi& roi, ImageRoiList* next);

// GlRealityFrame::addNextDrawHiResScans(intrinsics, {0, 0}) (gl-reality-frame.cc:682-693): fills
// the remaining slots with centred HIRES_SCAN ROIs, k = 1.., scale = 1 - 1.44^(2 + k),
// warp = glImageTargetWarp(K, K, translation(0, 0, scale)), faceId -k,
// name "INTERNAL_HIRES_SCAN_ROI". `processing` is the request's processing intrinsics
// (Intrinsics::getProcessingIntrinsics, tracking-controller.cc:317).
void AddNextDrawHiResScans(const PinholeIntrinsics& processing, ImageRoiList* next);

}  // namespace image_target
}  // namespace slam_refact

#endif  // SLAM_REFACT_IMAGE_TARGET_IMAGE_ROI_LIST_H_
