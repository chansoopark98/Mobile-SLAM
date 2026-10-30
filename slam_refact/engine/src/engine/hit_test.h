// Engine private header: XrController hitTest (spec 06 §2, contract FULL-1). Port of
// PUB:reality/engine/hittest/hit-test-performer.cc (FEATURE_POINT path), the input handling of
// PUB:reality/engine/executor/xr-engine.cc:87-155 (XREngine::query) and the XrQueryResponse
// branch of rewriteCoordinateSystemPostprocess (PUB:reality/engine/geometry/orientation.cc).

#ifndef SLAM_REFACT_ENGINE_HIT_TEST_H_
#define SLAM_REFACT_ENGINE_HIT_TEST_H_

#include <array>
#include <cstdint>
#include <vector>

#include "engine/device_orientation.h"
#include "slam_refact/types.h"
#include "slam_refact/xr_controller.h"

namespace slam_refact {

// The configured coordinate system (CoordinateSystemConfiguration): origin, facing, scale and
// axes / mirror flags as the 704 post-processing reads them.
struct CoordinateSystem {
  Vec3f origin{0.f, 2.f, 0.f};
  QuatWxyz facing;
  float scale = 1.f;
  bool left_handed_axes = false;  // JS leftHandedAxes (axes X_RIGHT_Y_UP_Z_FORWARD)
  bool mirrored_display = false;
};

// What a hit test reads from XREngine's lastCall: the response camera (733-corrected pose in
// internal coordinates), its display projection matrix and the feature set (631 list).
struct HitTestState {
  Vec3f camera_position;
  QuatWxyz camera_rotation;
  std::array<float, 16> projection{};
  std::vector<Vec3f> points;
  // AppContext.deviceOrientation of the request that produced this response (FULL-8): query turns
  // the hit input by it (xr-engine.cc:111-135).
  DeviceOrientation orientation = DeviceOrientation::kPortrait;
};

// One hit in internal coordinates (before 704).
struct InternalHit {
  XrHitType type = XrHitType::kUnspecified;
  Vec3f position;
  float distance = 0.f;
};

// OR of 1 << type over the requested types (performHitTest, hit-test-performer.cc:162-165).
uint32_t HitTypeMask(const std::vector<XrHitType>& included_types);

// containsResultType (hit-test-performer.cc:149-151): an empty mask accepts every type.
bool ContainsResultType(uint32_t mask, XrHitType type);

// XREngine::query input (mirroredDisplay flips x, then the device orientation turns (x, y)) followed by the
// FEATURE_POINT part of performHitTest. Empty when the type is not requested or there is no point.
std::vector<InternalHit> FeaturePointHitTest(const HitTestState& state, float x, float y,
                                             uint32_t mask, bool mirrored_display);

// rewriteCoordinateSystemPostprocess for one hit (orientation.cc:174-179, 364-381, 483-503).
XrHitResult RewriteHit(const CoordinateSystem& coords, const InternalHit& hit);

// The position part of RewriteHit (right-handed z flip, mirror x, recenterAndScale).
Vec3f RewritePosition(const CoordinateSystem& coords, const Vec3f& internal);

// The display ray of (x, y) in internal coordinates as getHitFeaturePoints builds it: the camera
// centre and the point at camera depth 1 on the ray (ACC-3 surfaces extension of XrHitTest).
void DisplayRayInternal(const HitTestState& state, float x, float y, bool mirrored_display,
                        Vec3f* camera_centre, Vec3f* depth_one_point);

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_HIT_TEST_H_
