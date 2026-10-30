// Device orientation corrections of the XREngine (contract FULL-8, US-023); see device_orientation.h.

#include "engine/device_orientation.h"

#include <utility>

#include "c8/geometry/egomotion.h"

namespace slam_refact {

DeviceOrientation RotationToDeviceOrientation(int rotation_deg) {
  switch (rotation_deg) {
    case 0:
      return DeviceOrientation::kPortrait;
    case 90:
    case -270:
      return DeviceOrientation::kLandscapeLeft;
    case 180:
    case -180:
      return DeviceOrientation::kPortraitUpsideDown;
    case 270:
    case -90:
      return DeviceOrientation::kLandscapeRight;
    default:
      return DeviceOrientation::kUnspecified;
  }
}

bool IsSupportedRotation(int rotation_deg) {
  return RotationToDeviceOrientation(rotation_deg) != DeviceOrientation::kUnspecified;
}

bool IsLandscape(DeviceOrientation o) {
  return o == DeviceOrientation::kLandscapeLeft || o == DeviceOrientation::kLandscapeRight;
}

void PortraitGraphicsSize(DeviceOrientation o, int* texture_width, int* texture_height) {
  if (IsLandscape(o)) {
    std::swap(*texture_width, *texture_height);
  }
}

std::array<float, 16> PortraitToDeviceProjection(DeviceOrientation o, const std::array<float, 16>& in) {
  std::array<float, 16> m = in;
  const float r0x = in[0 + 0 * 4];
  const float r0z = in[0 + 2 * 4];
  const float r1y = in[1 + 1 * 4];
  const float r1z = in[1 + 2 * 4];
  const float nr0z = -r0z;
  const float nr1z = -r1z;
  switch (o) {
    case DeviceOrientation::kLandscapeRight:
      m[0 + 0 * 4] = r1y;
      m[0 + 2 * 4] = r1z;
      m[1 + 1 * 4] = r0x;
      m[1 + 2 * 4] = nr0z;
      break;
    case DeviceOrientation::kPortraitUpsideDown:
      m[0 + 2 * 4] = nr0z;
      m[1 + 2 * 4] = nr1z;
      break;
    case DeviceOrientation::kLandscapeLeft:
      m[0 + 0 * 4] = r1y;
      m[0 + 2 * 4] = nr1z;
      m[1 + 1 * 4] = r0x;
      m[1 + 2 * 4] = r0z;
      break;
    default:
      break;  // Leave as specified.
  }
  return m;
}

void PortraitToDevicePose(DeviceOrientation o, c8::HVector3* t, c8::Quaternion* q) {
  c8::HMatrix rot = c8::HMatrixGen::i();
  switch (o) {
    case DeviceOrientation::kLandscapeRight:
      rot = c8::HMatrixGen::z90();
      break;
    case DeviceOrientation::kPortraitUpsideDown:
      rot = c8::HMatrixGen::z180();
      break;
    case DeviceOrientation::kLandscapeLeft:
      rot = c8::HMatrixGen::z270();
      break;
    default:
      return;  // No correction needed.
  }
  const c8::HMatrix cam = c8::cameraMotion(*t, *q);
  const c8::HMatrix new_cam = cam * rot;
  *q = c8::rotation(new_cam);
  *t = c8::translation(new_cam);
}

void DeviceToPortraitHitInput(DeviceOrientation o, float x, float y, float* out_x, float* out_y) {
  switch (o) {
    case DeviceOrientation::kLandscapeRight:
      *out_x = y;
      *out_y = 1 - x;
      break;
    case DeviceOrientation::kPortraitUpsideDown:
      *out_x = 1 - x;
      *out_y = 1 - y;
      break;
    case DeviceOrientation::kLandscapeLeft:
      *out_x = 1 - y;
      *out_y = x;
      break;
    default:
      *out_x = x;
      *out_y = y;
      break;  // No correction needed.
  }
}

}  // namespace slam_refact
