// ACC-3: floor and horizontal surface detection + hit test over published world points.
//
// Points: the published world points (631: triangulated points with an accepted keyframe view,
// 0-padding removed) are kept by id with their latest position; points not refreshed for
// `max_age_frames` frames are dropped. The published world is gravity aligned (y up), so a
// horizontal surface is a peak of the point height histogram:
//   1. heights of points below the camera (y < camera y - min_below) in bins of `bin`;
//   2. 3-bin smoothed histogram; local maxima with >= min_inliers points are candidates;
//   3. a candidate's height = median of the points within +-2 bins; inliers = those points;
//   4. the floor = the lowest candidate holding >= min_floor_fraction of the below-camera points;
//      the others are reported as further surfaces (tables, steps), highest support first;
//   5. extent = convex hull (x, z) of the inliers; the floor height is smoothed over frames.
// Hit test (screen point -> ray through the processing intrinsics): DETECTED_SURFACE inside a hull,
// ESTIMATED_SURFACE on the floor plane outside its hull, FEATURE_POINT = depth along the ray from
// a Gaussian-weighted (angular sigma 0.02 rad) average of nearby points (public hit-test performer
// estimateZDepth idea, SOURCE archive/reality/engine/hittest/hit-test-performer.cc:84-135).

#ifndef SLAM_REFACT_ENGINE_FLOOR_DETECTOR_H_
#define SLAM_REFACT_ENGINE_FLOOR_DETECTOR_H_

#include <cstdint>
#include <map>
#include <vector>

#include "slam_refact/surfaces.h"
#include "slam_refact/types.h"

namespace slam_refact {

struct FloorDetectorParams {
  float bin = 0.03f;               // height bin, published units
  float min_below = 0.3f;          // floor must be this far below the camera
  float max_below = 4.0f;          // points further below are outliers (histogram range)
  int min_inliers = 15;
  float min_floor_fraction = 0.08f;
  int max_age_frames = 90;
  int max_points = 6000;
  float height_alpha = 0.3f;       // floor height smoothing
};

class FloorDetector {
 public:
  explicit FloorDetector(const FloorDetectorParams& params = {});

  void Reset();
  void SetBin(float bin) { params_.bin = bin; }
  // One published frame (NORMAL): camera centre and its world points.
  void Update(const Vec3f& camera, const std::vector<WorldPointOut>& points);
  const SurfaceState& state() const { return state_; }

  // origin/direction in the published world; mask = OR of HitType bits.
  std::vector<HitResult> HitTest(const Vec3f& origin, const Vec3f& direction, uint32_t mask) const;

 private:
  struct Stored {
    Vec3f p;
    int64_t frame = 0;
  };
  void Detect(const Vec3f& camera);

  FloorDetectorParams params_;
  // Ordered by id: HitTest accumulates floats over the cloud, so the visit order must not depend on the
  // standard library's hash layout (native vs WASM).
  std::map<uint64_t, Stored> cloud_;
  int64_t frame_ = 0;
  SurfaceState state_;
  bool have_height_ = false;
  float smoothed_height_ = 0.f;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_FLOOR_DETECTOR_H_
