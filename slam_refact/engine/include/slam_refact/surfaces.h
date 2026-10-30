// slam_refact public API: floor / horizontal surface detection and hit test (ACC-3,
// contract-changes.md). Not part of the original engine: reference runtime's web runtime exposes hitTest
// types FEATURE_POINT / ESTIMATED_SURFACE / DETECTED_SURFACE, but no web surface source was found
// (slam_refact/docs/reports/accuracy.md §5); these are computed here from the published,
// triangulated world points in the published world frame (y up).

#ifndef SLAM_REFACT_SURFACES_H_
#define SLAM_REFACT_SURFACES_H_

#include <cstdint>
#include <vector>

#include "slam_refact/types.h"

namespace slam_refact {

// A horizontal plane y = height supported by triangulated map points.
struct HorizontalSurface {
  float height = 0.f;
  int inliers = 0;
  float confidence = 0.f;     // 0..1
  std::vector<Vec3f> hull;    // convex hull of the inliers on the plane (y = height), CCW in x-z
  float area = 0.f;           // hull area, published units^2
};

struct SurfaceState {
  bool floor_detected = false;
  HorizontalSurface floor;                 // lowest well-supported horizontal surface
  std::vector<HorizontalSurface> others;   // further horizontal surfaces above the floor
  int cloud_points = 0;                    // points held by the detector
};

enum class HitType : uint8_t {
  kFeaturePoint = 1,
  kEstimatedSurface = 2,  // the floor plane outside its hull
  kDetectedSurface = 4,   // inside the hull of a detected surface
};

struct HitResult {
  HitType type = HitType::kFeaturePoint;
  Vec3f position;
  float distance = 0.f;  // along the ray from the camera centre
  int surface = -1;      // -1 none, 0 floor, k >= 1 others[k - 1]
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_SURFACES_H_
