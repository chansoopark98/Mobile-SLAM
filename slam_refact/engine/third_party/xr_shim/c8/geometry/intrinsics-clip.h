// Subset of class Intrinsics (PUB:c8/geometry/intrinsics.{h,cc}) for the vendored ear code
// (US-019).
//
// The world-SLAM shim c8/geometry/intrinsics.h deliberately has no class Intrinsics (the original
// factory depends on reality.capnp device tables). ears/ear-types.cc needs four capnp-free static
// functions; patches/face-ear-closure.patch (include lines only, shared with c8/map.h in
// tracked-face-state.h) adds `#include "c8/geometry/intrinsics-clip.h"` to
// ear-types.cc and this header defines them. Bodies are copied from intrinsics.cc with the same
// float operations and order (line numbers at each function). No other member exists.
#pragma once

#include <algorithm>
#include <array>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"

namespace c8 {

class Intrinsics {
public:
  // intrinsics.cc:129-166
  static void toClipSpaceLeftHanded(
    const c8_PixelPinholeCameraModel intrinsics,
    float kn,
    float kf,
    float *colMajorProjMat,
    float *invColMajorProjMat) {

    float c00 = 2 * intrinsics.focalLengthHorizontal / intrinsics.pixelsWidth;
    float c11 = 2 * intrinsics.focalLengthVertical / intrinsics.pixelsHeight;

    float c02 = 1.0f - (2.0f * intrinsics.centerPointX + 1.0f) / intrinsics.pixelsWidth;
    float c12 = -1.0f + (2.0f * intrinsics.centerPointY + 1.0f) / intrinsics.pixelsHeight;

    float c22 = (kf + kn) / (kf - kn);
    float c23 = -(2 * kf * kn) / (kf - kn);
    float c32 = 1.0f;

    std::fill(colMajorProjMat, colMajorProjMat + 16, 0.0f);

    colMajorProjMat[0] = c00;
    colMajorProjMat[5] = c11;
    colMajorProjMat[8] = c02;
    colMajorProjMat[9] = c12;
    colMajorProjMat[10] = c22;
    colMajorProjMat[11] = c32;
    colMajorProjMat[14] = c23;

    if (invColMajorProjMat != nullptr) {
      std::fill(invColMajorProjMat, invColMajorProjMat + 16, 0.0f);
      invColMajorProjMat[0] = 1.0f / c00;
      invColMajorProjMat[5] = 1.0f / c11;
      invColMajorProjMat[11] = 1.0f / c23;
      invColMajorProjMat[12] = -c02 / c00;
      invColMajorProjMat[13] = -c12 / c11;
      invColMajorProjMat[14] = 1.0f;
      invColMajorProjMat[15] = -c22 / c23;
    }
  }

  // intrinsics.cc:168-184
  static c8_PixelPinholeCameraModel rotateCropAndScaleIntrinsics(
    const c8_PixelPinholeCameraModel intrinsics, int width, int height) {
    auto k = intrinsics;
    // If the camera feed is rotated with respect to our model, swap x/y.
    if ((width > height) != (k.pixelsWidth > k.pixelsHeight)) {
      k = {
        k.pixelsHeight,           // width
        k.pixelsWidth,            // height
        k.centerPointY,           // centerPointX
        k.centerPointX,           // centerPointY
        k.focalLengthVertical,    // focalLengthHorizontal
        k.focalLengthHorizontal,  // focalLengthVertical
      };
    }

    return Intrinsics::cropAndScaleIntrinsics(k, width, height);
  }

  // intrinsics.cc:186-193
  static HMatrix toClipSpaceMatLeftHanded(
    const c8_PixelPinholeCameraModel intrinsics, float nearClip, float farClip) {
    std::array<float, 16> mat;
    std::array<float, 16> matInv;
    Intrinsics::toClipSpaceLeftHanded(intrinsics, nearClip, farClip, mat.data(), matInv.data());

    return {mat.data(), matInv.data()};
  }

  // intrinsics.cc:245-263
  static c8_PixelPinholeCameraModel cropAndScaleIntrinsics(
    const c8_PixelPinholeCameraModel intrinsics, const int width, const int height) {
    float scale = std::max(
      static_cast<float>(width) / intrinsics.pixelsWidth,
      static_cast<float>(height) / intrinsics.pixelsHeight);

    float centerX =
      (width - 1) / 2.0f + (intrinsics.centerPointX - (intrinsics.pixelsWidth - 1) / 2.0f) * scale;
    float centerY =
      (height - 1) / 2.0f + (intrinsics.centerPointY - (intrinsics.pixelsHeight - 1) / 2.0f) * scale;

    return {
      width,
      height,
      centerX,
      centerY,
      intrinsics.focalLengthHorizontal * scale,
      intrinsics.focalLengthVertical * scale};
  }
};

}  // namespace c8
