// Conversions between public API value types (include/slam_refact/types.h) and the vendored c8
// types used inside the engine (architecture.md §2.1: the public API never exposes c8 types).
// All conversions are exact field copies; no arithmetic.

#ifndef SLAM_REFACT_COMMON_C8_ADAPTERS_H_
#define SLAM_REFACT_COMMON_C8_ADAPTERS_H_

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "slam_refact/types.h"

namespace slam_refact {

inline HMatrix4f ToHMatrix4f(const c8::HMatrix& h) {
  HMatrix4f out;
  for (int i = 0; i < 16; ++i) {
    out.m[i] = h.data()[i];
  }
  return out;
}

inline QuatWxyz ToQuatWxyz(const c8::Quaternion& q) { return {q.w(), q.x(), q.y(), q.z()}; }

inline c8::Quaternion ToC8Quaternion(const QuatWxyz& q) {
  return c8::Quaternion(q.w, q.x, q.y, q.z);
}

inline PinholeIntrinsics ToPinholeIntrinsics(const c8_PixelPinholeCameraModel& k) {
  return {k.pixelsWidth, k.pixelsHeight, k.centerPointX,
          k.centerPointY, k.focalLengthHorizontal, k.focalLengthVertical};
}

inline c8_PixelPinholeCameraModel ToC8Intrinsics(const PinholeIntrinsics& k) {
  c8_PixelPinholeCameraModel out;
  out.pixelsWidth = k.width;
  out.pixelsHeight = k.height;
  out.centerPointX = k.cx;
  out.centerPointY = k.cy;
  out.focalLengthHorizontal = k.fx;
  out.focalLengthVertical = k.fy;
  return out;
}

}  // namespace slam_refact

#endif  // SLAM_REFACT_COMMON_C8_ADAPTERS_H_
