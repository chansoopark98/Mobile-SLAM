// Original Author: Scott Pollack

#pragma once

#include <memory>

#include "c8/hmatrix.h"
#include "c8/vector.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/geometry/pose-pnp.h"
#include "reality/engine/imagedetection/detection-image.h"
#include "reality/engine/imagedetection/tracked-image.h"

namespace c8 {

// Reset worldTracker after this number of consecutive untracked frames.
static int constexpr WORLD_RESET_UNTRACKED_FRAMES = 100;

// [slam_refact patch us015-image-world-tracker] The released binary's ImageTrackingState carries a
// world tracker at +4 that the public source lacks (xr-slam.wasm func938/func994; evidence in
// slam_refact/docs/spec/11-image-world-tracker.md section 9). The state is plain data here; the
// algorithm (func994) is implemented outside the vendored tree and reached through
// ImageWorldTracker. Without a tracker (nullptr, the default) the public behavior is unchanged.
struct ImageWorldTrackerState {
  float scale = 1.0f;
  HMatrix worldPose = HMatrixGen::i();
  Vector<HPoint2> rays;
  Vector<HPoint3> searchPtsInCamera;
  Vector<size_t> visible;
  Vector<HPoint3> worldPts;
  Vector<WorldMapPointId> ids;
  Vector<HPoint3> targetPts;

  // Scale is kept (func938 D:109587-109607).
  void reset() {
    worldPose = HMatrixGen::i();
    rays.clear();
    searchPtsInCamera.clear();
    visible.clear();
    worldPts.clear();
    ids.clear();
    targetPts.clear();
  }
};

class ImageWorldTracker {
public:
  virtual ~ImageWorldTracker() = default;
  // func994: returns the new world pose of the target; st->scale holds the new scale.
  virtual HMatrix track(
    ImageWorldTrackerState *st,
    const DetectionImage &im,
    const HMatrix &targetViewPose,
    const HMatrix &tamExtrinsic,
    const FrameWithPoints &feats,
    float scaleGuess) = 0;
};

struct ImageTrackingScratchSpace {
public:
  Vector<uint8_t> localMatchPoseInliers_;
  Vector<uint8_t> globalMatchPoseInliers_;
  Vector<PointMatch> globalMatches_;
  Vector<PointMatch> localMatches_;
  Vector<HPoint3> worldPts_;
  Vector<HPoint2> imTargetRays_;
  Vector<HPoint2> camRays_;
  RobustPoseScratchSpace scratch_;
  RobustP2PScratchSpace p2pScratch_;
  FrameWithPoints featsRayInTargetView_{{}};
  FrameWithPoints featsRayInTargetViewPostFit_{{}};
};

class ImageTrackingState {
public:
  static bool AGGRESSIVELY_FREE_MEMORY;
  static const uint8_t DESCRIPTOR_DISTANCE_THRESHOLD_LOCAL_ORB;
  static const uint8_t DESCRIPTOR_DISTANCE_THRESHOLD_LOCAL_GORB;

  ImageTrackingState(String name) {
    trackedImage_.name = name;
    sc_.reset(new ImageTrackingScratchSpace());
  }

  // Default move
  ImageTrackingState(ImageTrackingState &&) = default;
  ImageTrackingState &operator=(ImageTrackingState &&) = default;

  // Disallow copying
  ImageTrackingState(const ImageTrackingState &) = delete;
  ImageTrackingState &operator=(const ImageTrackingState &) = delete;

  const bool localMode() const { return localMode_; }

  const TrackedImage &trackedImage() const { return trackedImage_; }

  const HMatrix &pose() const { return trackedImage_.pose; }

  const int32_t lastSeen() const { return trackedImage_.lastSeen; }

  const bool everSeen() const { return trackedImage_.everSeen(); }

  // Debug API for visualization only.
  const Vector<uint8_t> &inliers() const { return sc_->localMatchPoseInliers_; }
  const Vector<uint8_t> &globalInliers() const { return sc_->globalMatchPoseInliers_; }
  const Vector<PointMatch> &globalMatches() const { return sc_->globalMatches_; }
  const Vector<PointMatch> &localMatches() const { return sc_->localMatches_; }
  const FrameWithPoints &featsRayInTargetView() const { return sc_->featsRayInTargetView_; }
  const FrameWithPoints &featsRayInTargetViewPostFit() const {
    return sc_->featsRayInTargetViewPostFit_;
  }

  float angleFromLastPose() const { return angleFromLastPose_; }

  const ImageWorldTrackerState &worldTracker() const { return worldTracker_; }

  void track(
    const FrameWithPoints &feats,
    const Vector<FrameWithPoints> &roiFeats,
    DetectionImage &imageTarget,
    RandomNumbers *random,
    const HMatrix &tamExtrinsic,
    ImageWorldTracker *worldTracker = nullptr);

  static float computeScale(
    const HMatrix &aFirst, const HMatrix &aSecond, const HMatrix &bFirst, const HMatrix &bSecond);

  c8::String toString() const noexcept;

private:
  bool worldInit_ = false;
  bool localMode_ = false;
  ImageWorldTrackerState worldTracker_;

  float scaleGuess_ = 0.5f;
  HMatrix lastCamPos_ = HMatrixGen::i();
  TrackedImage trackedImage_;
  float angleFromLastPose_ = 0.0f;

  // Scratch space.
  std::unique_ptr<ImageTrackingScratchSpace> sc_;

  float guessScale(const HMatrix &camPos);

  void updateTrackingState(
    TrackedImage::Status status,
    DetectionImage &im,
    const HMatrix &pose,
    const FrameWithPoints &pyramidOnlyFeats,
    c8_PixelPinholeCameraModel camK,
    const HMatrix &tamExtrinsic,
    ImageWorldTracker *worldTracker);

  void nextTargetViewPose(
    const FrameWithPoints &pyramidFeats,
    const FrameWithPoints &roiFeats,
    DetectionImage &im,
    HMatrix *nextPose,
    bool *found,
    RandomNumbers *random);

  static const FrameWithPoints &featsForTrackedImage(
    const String &name, const FrameWithPoints &feats, const Vector<FrameWithPoints> &roiFeats);
};

}  // namespace c8
