// Closed image-world-tracker of the original image target path (US-015), reconstructed from the
// xr-slam.wasm binary: func994 (scopes "image-world-tracker", "match", "scale-solver"), which the
// binary's ImageTrackingState (func938) calls while VIO is on. The state struct and the call site
// are the vendored patch us015-image-world-tracker.patch (c8::ImageWorldTrackerState,
// c8::ImageWorldTracker); this file is the algorithm. Evidence, line by line:
// slam_refact/docs/spec/11-image-world-tracker.md section 9.
//
// Per localized target and frame:
//   1. lift every feature ray of the frame onto the target surface with the target view pose
//      (public cameraRaysToVisiblePointsInCamera{Planar,Curvy}): 3d points in target units in the
//      camera frame, and the indices of the visible ones;
//   2. join the visible features with the map points MapTracker tracked in this frame
//      (MapTrackerOutput.tracked_ -> WorldMap::getPoint), TRIANGULATED points only;
//   3. fewer than 6 pairs: scale = the public scale guess and
//      worldPose = tam * scaleTranslation(scale, targetViewPose^-1);
//      otherwise the public poseEstimationScale3d (Ceres; Scale3dPoseEstimation residual
//      s * target - tam^-1 * world; CauchyLoss(1e-4)) from the previous scale; a usable solution
//      sets the scale and recomputes worldPose with it, an unusable one keeps both.

#ifndef SLAM_REFACT_IMAGE_TARGET_IMAGE_WORLD_TRACKER_H_
#define SLAM_REFACT_IMAGE_TARGET_IMAGE_WORLD_TRACKER_H_

#include <array>
#include <cstdint>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/imagedetection/detection-image.h"
#include "reality/engine/imagedetection/image-tracking-state.h"

namespace slam_refact::image_target {

// One world map point as 994 reads it after the tracked-map lookup (WorldPoint +0x04 id,
// +0x08 position, +0x40 source).
struct TrackedMapPoint {
  uint32_t slot = 0;
  uint32_t id = 0;
  c8::HPoint3 position;
  int32_t source = 0;  // 1 TRIANGULATED, 2 GROUND, 3 SKY, 4 DEPTH, 5 CLOUD
};

// The map side of 994: 784 passes a copy of MapTrackerOutput.tracked_ (frame point index -> map
// point id, D:83453-83652 / D:84154) and the active WorldMap (b+7232). Find returns false when the
// frame point has no entry, the id is 0, or no point pool holds that id in that slot (634's
// five-pool check); otherwise the first pool's point in TRIANGULATED, GROUND, SKY, DEPTH, CLOUD
// order.
class TrackedPointLookup {
 public:
  virtual ~TrackedPointLookup() = default;
  virtual bool Find(uint32_t frame_idx, TrackedMapPoint* out) const = 0;
};

// 994 param4 head: world point -> tracker coordinates as (p - offset) * scale. 784 passes
// scale 1 and offset 0 (D:84151-84153), i.e. the map coordinates unchanged.
struct MapToTrackerTransform {
  float scale = 1.0f;
  std::array<float, 3> offset{{0.0f, 0.0f, 0.0f}};
};

// func994. Returns the new world pose (also left in st->worldPose).
c8::HMatrix TrackImageWorld(c8::ImageWorldTrackerState* st, const TrackedPointLookup& lookup,
                            const MapToTrackerTransform& xf, const c8::DetectionImage& im,
                            const c8::HMatrix& target_view_pose, const c8::HMatrix& tam,
                            const c8::FrameWithPoints& feats, float scale_guess);

// c8::ImageWorldTracker backed by TrackImageWorld, with the lookup of the current frame. VioTracker
// installs it on its DetectionImageTracker when the image world tracker is enabled and VIO is on.
class ImageWorldTrackerHook final : public c8::ImageWorldTracker {
 public:
  // The lookup must outlive the next DetectionImageTracker::locate call; nullptr = no map points.
  void set_lookup(const TrackedPointLookup* lookup) { lookup_ = lookup; }
  void set_transform(const MapToTrackerTransform& xf) { xf_ = xf; }

  c8::HMatrix track(c8::ImageWorldTrackerState* st, const c8::DetectionImage& im,
                    const c8::HMatrix& targetViewPose, const c8::HMatrix& tamExtrinsic,
                    const c8::FrameWithPoints& feats, float scaleGuess) override;

 private:
  const TrackedPointLookup* lookup_ = nullptr;
  MapToTrackerTransform xf_;
};

}  // namespace slam_refact::image_target

#endif  // SLAM_REFACT_IMAGE_TARGET_IMAGE_WORLD_TRACKER_H_
