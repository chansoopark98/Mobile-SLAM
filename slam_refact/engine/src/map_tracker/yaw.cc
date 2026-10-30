// yawOf (519) and replaceYaw (1271), S2 §5.5. Read from the f519 / f1271 wasm disassembly.

#include <cmath>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "map_tracker/map_tracker.h"

namespace slam_refact::map_tracker {

// 519. The binary evaluates R(q) * (0, 0, 1, 1) with SIMD lanes: the camera forward axis in world
// coordinates, (m02 + (m00 * 0 + m01 * 0), m22 + (m20 * 0 + m21 * 0), w = 1). When its horizontal
// length is <= 1e-5 (the axis is vertical) the quaternion is turned by -90 degrees about x
// (q * (w = sqrt(1/2), x = -sqrt(1/2))) and the test is repeated. The public
// Quaternion::yawRadians() is a different function (S2 §5.5).
float YawOf(c8::Quaternion q) {
  constexpr float kSqrtHalf = 0x1.6a09e6p-1f;
  constexpr float kVerticalEps = 0x1.4f8b58p-17f;  // 1e-5
  while (true) {
    const c8::HMatrix r = q.toRotationMat();
    const float x = r(0, 2) + (r(0, 0) * 0.0f + r(0, 1) * 0.0f);
    const float z = r(2, 2) + (r(2, 0) * 0.0f + r(2, 1) * 0.0f);
    const float w = 1.0f;
    const float xw = x / w;
    const float zw = z / w;
    if (!(std::sqrt(zw * zw + (xw * xw + 0.0f)) <= kVerticalEps)) {
      return std::atan2(xw, zw);  // 843 = atan2f
    }
    q = q.times(c8::Quaternion(kSqrtHalf, -kSqrtHalf, 0.0f, 0.0f));
  }
}

// 1271: tilt from the device quaternion, yaw from the pose; the translation is kept.
c8::HMatrix ReplaceYaw(const c8::HMatrix& pose, const c8::Quaternion& q_dev) {
  const float half_delta = (YawOf(c8::Quaternion::fromHMatrix(pose)) - YawOf(q_dev)) * 0.5f;
  const c8::Quaternion yaw(std::cos(half_delta), 0.0f, std::sin(half_delta), 0.0f);
  const c8::Quaternion q = yaw.times(q_dev);
  return c8::cameraMotion(c8::translation(pose), q);
}

}  // namespace slam_refact::map_tracker
