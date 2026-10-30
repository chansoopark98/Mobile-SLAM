// K rule (S1 §5.12).
//
// Ports (SOURCE = public xr_reference/xr_reference@4edfa484):
//   c8/camera/device-infos.cc:34-71, 73-86, 88-298   getModel, buildModelMap, getDeviceModel
//   c8/geometry/intrinsics.cc:245-263                 cropAndScaleIntrinsics
//   c8/geometry/intrinsics.cc:291-309                 getFullFrameIntrinsics (web: no pixel intrinsics)
//   c8/geometry/intrinsics.cc:358-497                 makeIntrinsics, getCameraIntrinsics, fallback
// The binary focal table (data @296576, DeviceModel 2..112) equals this table 111/111 (S1 §5.12
// VERIFIED, re-checked by the F-H9 evidence test). The string -> DeviceModel map of the binary is
// not decoded; it is taken from the public source (INFERRED).

#include "frontend/intrinsics.h"

#include <algorithm>
#include <map>
#include <set>

#include "c8/string/format.h"

namespace slam_refact::frontend {
namespace {

using device_infos::DeviceModel;
using DeviceMap = std::map<std::string, std::map<std::string, DeviceModel>>;

const DeviceMap& GetDeviceMap() {
  static const DeviceMap device_map = [] {
    DeviceMap m;
#define SR_DEVICE_MODEL(name, focal)
#define SR_DEVICE_ENTRY(manufacturer, model, name) \
  m[manufacturer].insert({model, device_infos::name});
#include "frontend/device_table.inc"
#undef SR_DEVICE_MODEL
#undef SR_DEVICE_ENTRY
    return m;
  }();
  return device_map;
}

// buildModelMap (device-infos.cc:73-86): iteration in TreeMap order, first insertion wins.
const std::map<std::string, DeviceModel>& GetModelMap() {
  static const std::map<std::string, DeviceModel> model_map = [] {
    std::map<std::string, DeviceModel> m;
    for (const auto& [manufacturer_name, manufacturer_model_map] : GetDeviceMap()) {
      for (const auto& [model_name, device_enum] : manufacturer_model_map) {
        m.insert({model_name, device_enum});
      }
    }
    return m;
  }();
  return model_map;
}

// getHuaweiModelPrefixes (device-infos.cc:44-47), filled from deviceMap.at("HUAWEI") keys
// (device-infos.cc:273-278) before getModel runs.
const std::set<std::string>& GetHuaweiModelPrefixes() {
  static const std::set<std::string> prefixes = [] {
    std::set<std::string> s;
    for (const auto& [prefix, model] : GetDeviceMap().at("HUAWEI")) {
      s.insert(prefix);
    }
    return s;
  }();
  return prefixes;
}

std::string GetSamsungModelBase(const std::string& model_number) {
  return model_number.substr(0, 7);
}

std::string GetHuaweiModelBase(const std::string& model_number) {
  const auto p = model_number.find('-');
  if (p == std::string::npos) {
    return model_number;
  }
  return model_number.substr(0, p);
}

// makeIntrinsics (intrinsics.cc:358-360).
PinholeIntrinsics MakeIntrinsics(float focal_length) {
  return {480, 640, 239.5f, 319.5f, focal_length, focal_length};
}

constexpr float kFocalByModel[device_infos::kNumDeviceModels] = {
#define SR_DEVICE_MODEL(name, focal) focal,
#define SR_DEVICE_ENTRY(manufacturer, model, name)
#include "frontend/device_table.inc"
#undef SR_DEVICE_MODEL
#undef SR_DEVICE_ENTRY
};

// The default case of getCameraIntrinsics (intrinsics.cc:470-472).
constexpr float kDefaultFocalLength = 506.4104f;

}  // namespace

std::string GetModelBase(const std::string& manufacturer_upper, const std::string& model) {
  if (manufacturer_upper == "SAMSUNG") {
    return GetSamsungModelBase(model);
  } else if (manufacturer_upper == "HUAWEI") {
    return GetHuaweiModelBase(model);
  } else if (manufacturer_upper.empty()) {
    if (model.compare(0, 3, "SM-") == 0) {
      return GetSamsungModelBase(model);
    }
    const auto& huawei_model_prefixes = GetHuaweiModelPrefixes();
    if (huawei_model_prefixes.find(model.substr(0, 3)) != huawei_model_prefixes.end() &&
        (model.length() > 3 && model[3] == '-')) {
      return GetHuaweiModelBase(model);
    }
  }
  return model;
}

device_infos::DeviceModel GetDeviceModel(std::string_view manufacturer_in,
                                         std::string_view model_in) {
  const std::string manufacturer = c8::toUpperCase(std::string(manufacturer_in));
  const std::string model = c8::toUpperCase(GetModelBase(manufacturer, std::string(model_in)));
  if (manufacturer.empty()) {
    const auto& model_map = GetModelMap();
    const auto it_model = model_map.find(model);
    if (it_model != model_map.end()) {
      return it_model->second;
    }
  } else {
    const auto& device_map = GetDeviceMap();
    const auto it_manufacturer = device_map.find(manufacturer);
    if (it_manufacturer != device_map.end() &&
        it_manufacturer->second.find(model) != it_manufacturer->second.end()) {
      return it_manufacturer->second.find(model)->second;
    }
  }
  return device_infos::NOT_SPECIFIED;
}

PinholeIntrinsics CameraIntrinsicsForModel(int model) {
  if (model < 0 || model >= device_infos::kNumDeviceModels) {
    return MakeIntrinsics(kDefaultFocalLength);
  }
  return MakeIntrinsics(kFocalByModel[model]);
}

PinholeIntrinsics FallbackCameraIntrinsics(std::string_view model_in) {
  const std::string model = c8::toUpperCase(std::string(model_in));
  if (model == "IPHONE 12/12PRO") {
    return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_12);
  } else if (model == "IPHONE XSMAX/11PROMAX") {
    return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_XS_MAX);
  } else if (model == "IPHONE X/XS/11PRO") {
    return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_X);
  } else if (model == "IPHONE XR/11") {
    return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_XR);
  } else if (model == "IPHONE 6+/6S+/7+/8+") {
    return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_6PLUS);
  } else if (model == "IPHONE 6/6S/7/8") {
    return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_6);
  } else if (model == "IPHONE 5/5S/5C/SE") {
    return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_5);
  }
  return CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_X);
}

PinholeIntrinsics FullIntrinsicsForDevice(std::string_view manufacturer, std::string_view model) {
  const device_infos::DeviceModel infos_model = GetDeviceModel(manufacturer, model);
  if (infos_model != device_infos::NOT_SPECIFIED) {
    return CameraIntrinsicsForModel(infos_model);
  }
  return FallbackCameraIntrinsics(model);
}

PinholeIntrinsics CropAndScale(const PinholeIntrinsics& k, int w, int h) {
  const float scale = std::max(static_cast<float>(w) / k.width, static_cast<float>(h) / k.height);
  const float center_x = (w - 1) / 2.0f + (k.cx - (k.width - 1) / 2.0f) * scale;
  const float center_y = (h - 1) / 2.0f + (k.cy - (k.height - 1) / 2.0f) * scale;
  return {w, h, center_x, center_y, k.fx * scale, k.fy * scale};
}

IntrinsicsSet ComputeIntrinsicsSet(std::string_view manufacturer, std::string_view model,
                                   int capture_width, int capture_height, int level0_width,
                                   int level0_height) {
  IntrinsicsSet set;
  set.full = FullIntrinsicsForDevice(manufacturer, model);
  const bool swap = capture_width > capture_height;
  set.capture = CropAndScale(set.full, swap ? capture_height : capture_width,
                             swap ? capture_width : capture_height);
  set.processing = CropAndScale(set.capture, level0_width, level0_height);
  return set;
}

float AccuracyDeviceFocal(std::string_view manufacturer_in, std::string_view model_in) {
  const std::string manufacturer = c8::toUpperCase(std::string(manufacturer_in));
  const std::string model = c8::toUpperCase(GetModelBase(manufacturer, std::string(model_in)));
  // Galaxy Z Flip6 (SM-F741*), Chrome back camera stream 480x640: focal 540 at the reference,
  // from the Flip6 S2 room-loop recording (loop closure minimum over a focal sweep, 535-550) and a
  // gyro-vs-optical-flow fit (474-526 raw, about 5 % low on ARCore-calibrated data); INFERRED,
  // slam_refact/docs/reports/accuracy.md §3. ARCore's own 640x480 stream is wider (425 at 480x640)
  // and does not apply to the Chrome stream.
  if (model == "SM-F741") {
    return 540.0f;
  }
  return 0.0f;
}

IntrinsicsSet ComputeIntrinsicsSetWithFocal(float focal, int capture_width, int capture_height,
                                            int level0_width, int level0_height) {
  IntrinsicsSet set;
  set.full = MakeIntrinsics(focal);
  const bool swap = capture_width > capture_height;
  set.capture = CropAndScale(set.full, swap ? capture_height : capture_width,
                             swap ? capture_width : capture_height);
  set.processing = CropAndScale(set.capture, level0_width, level0_height);
  return set;
}

}  // namespace slam_refact::frontend
