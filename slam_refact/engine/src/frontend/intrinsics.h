// Frontend private header: K rule (S1 §5.12). Capnp-free port of the public
// c8/camera/device-infos.{h,cc} (device table) and c8/geometry/intrinsics.cc (focal table,
// fallback, cropAndScale).

#ifndef SLAM_REFACT_FRONTEND_INTRINSICS_H_
#define SLAM_REFACT_FRONTEND_INTRINSICS_H_

#include <string>
#include <string_view>

#include "frontend/frontend.h"
#include "slam_refact/types.h"

namespace slam_refact::frontend {

// DeviceInfos::DeviceModel (device-infos.h:34-148); value = position in the enum.
namespace device_infos {
enum DeviceModel : int {
#define SR_DEVICE_MODEL(name, focal) name,
#define SR_DEVICE_ENTRY(manufacturer, model, name)
#include "frontend/device_table.inc"
#undef SR_DEVICE_MODEL
#undef SR_DEVICE_ENTRY
  kNumDeviceModels
};
}  // namespace device_infos

// getModel (device-infos.cc:51-71): strips appended data of Samsung / Huawei model strings.
std::string GetModelBase(const std::string& manufacturer_upper, const std::string& model);

// DeviceInfos::getDeviceModel (device-infos.cc:88-298).
device_infos::DeviceModel GetDeviceModel(std::string_view manufacturer, std::string_view model);

// Intrinsics::getCameraIntrinsics (intrinsics.cc:366-475): {480, 640, 239.5, 319.5, f, f}; models
// without a calibrated case (and out-of-range values) get the default 506.4104 (APPLE_IPHONE_X).
PinholeIntrinsics CameraIntrinsicsForModel(int model);

// Intrinsics::getFallbackCameraIntrinsics (intrinsics.cc:477-497); uses the raw model string.
PinholeIntrinsics FallbackCameraIntrinsics(std::string_view model);

// processing = cropAndScale(capture, level-0 w, h) (intrinsics.cc:275-284 with a pyramid);
// capture = cropAndScale(full, capture geometry turned portrait: w > h swaps, S1 §5.12).
IntrinsicsSet ComputeIntrinsicsSet(std::string_view manufacturer, std::string_view model,
                                   int capture_width, int capture_height, int level0_width,
                                   int level0_height);

// Accuracy mode (ACC-4): focal length at the 480x640 reference of devices whose web camera stream
// was calibrated for this project (not part of the original table); 0 when unknown. Samsung model
// numbers are matched on their 7-character base like the original.
float AccuracyDeviceFocal(std::string_view manufacturer, std::string_view model);

// ComputeIntrinsicsSet with the reference focal replaced by `focal` (same crop and scale rule).
IntrinsicsSet ComputeIntrinsicsSetWithFocal(float focal, int capture_width, int capture_height,
                                            int level0_width, int level0_height);

}  // namespace slam_refact::frontend

#endif  // SLAM_REFACT_FRONTEND_INTRINSICS_H_
