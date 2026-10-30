// Detection: VioTracker::track detection part (func 784, S1 §3.5) around the vendored public
// Gr8Gl CPU half (func 498 detectAndCompute, S1 §3.7).
//
// Copy-out follows the public FeatureDetector::detectFeatures(pyramid, config, pts, rois)
// (archive reality/engine/features/feature-detector.cc:60-80, SOURCE): clear, reserve,
// addImagePixelPoint in detectAndCompute order, setExcludedEdgePixels(edgeThreshold = 21)
// (func 784 r[229] = 21, S1 §2.5 VERIFIED). World SLAM has no ROI (S1 §2.3).

#include <utility>

#include "c8/hpoint.h"
#include "common/stage_timer.h"
#include "frontend/frontend.h"

namespace slam_refact::frontend {

// Gr8Gl::create(1.44f, 21, 2) (func 452: scaleFactor 1.440000057 as double, edgeThreshold 21,
// wtaK 2; S1 §1.3 VERIFIED).
FeatureFrontend::FeatureFrontend() : gr8gl_(c8::Gr8Gl::create(1.44f, 21, 2)) {}

FeatureFrontend::~FeatureFrontend() = default;

void FeatureFrontend::Detect(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
                             c8::FrameWithPoints* frame) {
  StageTimer timer("features");
  // gravityAngleOffset = 0.0f (func 784 -> func 498, S1 §3.5).
  c8::ImagePoints feats = gr8gl_.detectAndCompute(pyr, *frame, cfg, 0.0f);
  DetectStats stats;
  frame->clear();
  frame->reserve(feats.size());
  for (c8::ImagePoint& f : feats) {
    const c8::ImagePointLocation l = f.location();
    stats.num_orb += f.features().orbFeature.has_value() ? 1 : 0;
    stats.num_gorb += f.features().gorbFeature.has_value() ? 1 : 0;
    frame->addImagePixelPoint(c8::HPoint2(l.pt.x, l.pt.y), l.scale, l.angle, l.gravityAngle,
                              std::move(f.mutableFeatures()));
  }
  frame->setExcludedEdgePixels(gr8gl_.edgeThreshold());
  stats.num_keypoints = static_cast<int>(feats.size());
  last_stats_ = stats;
}

void FeatureFrontend::DetectRois(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
                                 const c8::FrameWithPoints& frame,
                                 std::vector<c8::FrameWithPoints>* rois) {
  StageTimer timer("gr8gl-rois");
  c8::Vector<c8::ImagePoints> roi_feats = gr8gl_.detectAndComputeRois(pyr, frame, cfg);
  rois->clear();
  for (const c8::ImagePoints& image_points : roi_feats) {
    rois->emplace_back(frame.intrinsic());
    c8::FrameWithPoints& frame_points = rois->back();
    frame_points.setRoi(pyr.rois[rois->size() - 1].roi);
    frame_points.setXRDevicePose(frame.xrDevicePose());
    frame_points.clear();
    frame_points.reserve(image_points.size());
    for (const c8::ImagePoint& f : image_points) {
      const c8::ImagePointLocation l = f.location();
      frame_points.addImagePixelPoint(c8::HPoint2(l.pt.x, l.pt.y), l.scale, l.angle,
                                      l.gravityAngle, f.features().clone());
    }
    frame_points.setExcludedEdgePixels(0);
  }
}

const DetectStats& FeatureFrontend::last_stats() const { return last_stats_; }

// func 784 dcmp 82457-82480 (S1 §3.5): nKeypoints = FEATURES_PER_FRAME_GL (500); world SLAM has no
// VPS map, so the descriptor type is VioTracker.trackingDescriptorType = 2 -> allGorb; ORB only
// for the lower 45 degrees (gravityPortions[ORB] = SEVENTH | EIGHTH = 192). The +6208 flag
// (numIncreasedFramePoints) is 0 in world SLAM (R17).
c8::DetectionConfig WorldSlamDetectionConfig() {
  c8::DetectionConfig cfg;
  cfg.nKeypoints = c8::FEATURES_PER_FRAME_GL;
  cfg.allGorb = true;
  cfg.gravityPortions[c8::DescriptorType::ORB] =
      static_cast<c8::GravityPortion>(c8::GravityPortion::SEVENTH | c8::GravityPortion::EIGHTH);
  return cfg;
}

}  // namespace slam_refact::frontend
