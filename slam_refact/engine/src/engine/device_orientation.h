// Device orientation corrections of the XREngine (contract FULL-8, US-023). Port of the public
//   reality/app/xr/js/tracking-controller.cc:152-167   rotationToDeviceOrientation
//   reality/engine/geometry/orientation.cc:38-125      updateCameraIntrinsics / updateCameraExtrinsics
//                                                      (portraitToDeviceOrientationPostprocess)
//   reality/engine/geometry/orientation.cc:435-453     deviceToPortraitOrientationPreprocess
//   reality/engine/executor/xr-engine.cc:111-135       XREngine::query input rotation
// The tracker itself always works in the portrait frame of the device: the pyramid renderer turns the
// camera image by the engineInit rotation (stage 1 shader), the IMU is in the device frame, and
// sensorRotationToPortrait is the identity on the web (SOURCE xr-requests.cc:312-315, VERIFIED with the
// probe of func 647 in slam_refact/tools/re/runtime/orient). Only the response is corrected here.

#ifndef SLAM_REFACT_ENGINE_DEVICE_ORIENTATION_H_
#define SLAM_REFACT_ENGINE_DEVICE_ORIENTATION_H_

#include <array>

#include "c8/hmatrix.h"
#include "c8/hvector.h"
#include "c8/quaternion.h"

namespace slam_refact {

// AppContext::DeviceOrientation (app.capnp).
enum class DeviceOrientation { kUnspecified, kPortrait, kLandscapeLeft, kPortraitUpsideDown, kLandscapeRight };

// tracking-controller.cc:152-167: 0 -> PORTRAIT, 90 / -270 -> LANDSCAPE_LEFT, 180 / -180 ->
// PORTRAIT_UPSIDE_DOWN, 270 / -90 -> LANDSCAPE_RIGHT, anything else UNSPECIFIED.
DeviceOrientation RotationToDeviceOrientation(int rotation_deg);

// The values the engine accepts as EngineConfig::rotation_deg (everything that is not UNSPECIFIED).
bool IsSupportedRotation(int rotation_deg);

// LANDSCAPE_LEFT / LANDSCAPE_RIGHT (the graphics texture size is exchanged, the hit input is turned).
bool IsLandscape(DeviceOrientation o);

// deviceToPortraitOrientationPreprocess: landscape exchanges the graphics intrinsics texture size.
void PortraitGraphicsSize(DeviceOrientation o, int* texture_width, int* texture_height);

// updateCameraIntrinsics (orientation.cc:38-70) on a column-major 4x4 projection matrix.
std::array<float, 16> PortraitToDeviceProjection(DeviceOrientation o, const std::array<float, 16>& m);

// updateCameraExtrinsics (orientation.cc:76-125) of the camera pose: cameraMotion(t, q) * z90 / z180 /
// z270 for LANDSCAPE_RIGHT / PORTRAIT_UPSIDE_DOWN / LANDSCAPE_LEFT, unchanged otherwise.
void PortraitToDevicePose(DeviceOrientation o, c8::HVector3* t, c8::Quaternion* q);

// XREngine::query (xr-engine.cc:111-135): the hit test input (x, y) of the displayed feed, after the
// mirror flip, turned into the portrait image of the tracker.
void DeviceToPortraitHitInput(DeviceOrientation o, float x, float y, float* out_x, float* out_y);

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_DEVICE_ORIENTATION_H_
