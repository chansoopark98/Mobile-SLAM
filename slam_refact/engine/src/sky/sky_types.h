// Sky segmentation / LayersController data types (US-020; docs/spec/09-sky.md).
//
// Plain structs for what the public reference runtime LayersController passes around (PUB:reality/app/xr/js/
// src/layers-controller.ts, layers-controller.cc, semantics-worker.cc, semantics.capnp). No GL, no
// Cap'n Proto, no Worker. Image memory layout everywhere in this module is the GL texture layout of
// the original: row 0 = GL row 0 (the first row of the CPU buffer that was uploaded, or the bottom
// row of a render target, as glReadPixels returns it).
#ifndef SLAM_REFACT_SKY_SKY_TYPES_H_
#define SLAM_REFACT_SKY_SKY_TYPES_H_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace slam_refact::sky {

// semantics-classifier.h: SEMANTICS_INPUT_WIDTH/HEIGHT, SEMANTICS_CLASS_NUM. layers-controller.ts
// feeds the model in portrait only: U8[256 rows, 144 cols] RGBA.
inline constexpr int kModelInputWidth = 144;
inline constexpr int kModelInputHeight = 256;
inline constexpr int kModelClassCount = 20;
// semantics-worker.js MODEL_THRESHOLD: a pixel counts for `percentage` when its sky score is above.
inline constexpr double kModelThreshold = 0.4;  // JS number: compared with the float32 score promoted to double
// layers-controller.ts: `percentage_ > 0.05 && !haveSeenSky_` fires layerfound.
inline constexpr double kLayerFoundPercentage = 0.05;

// RGBA8 image, row-major, 4 bytes per pixel, no row padding.
struct Rgba8Image {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> px;  // width * height * 4

  Rgba8Image() = default;
  Rgba8Image(int w, int h) : width(w), height(h), px(static_cast<size_t>(w) * h * 4, 0) {}
  bool empty() const { return width <= 0 || height <= 0; }
  uint8_t* at(int x, int y) { return &px[(static_cast<size_t>(y) * width + x) * 4]; }
  const uint8_t* at(int x, int y) const { return &px[(static_cast<size_t>(y) * width + x) * 4]; }
};

// Device orientation as the `deviceorientation` event gives it (degrees). getQuaternion() turns it
// into the device rotation (quaternion.ts).
struct DeviceOrientation {
  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.0;
};

struct QuaternionXyzw {  // JS {x, y, z, w}
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

// quaternion.ts getQuaternion (the `|| 0` of the original turns NaN/undefined into 0).
QuaternionXyzw GetQuaternion(double alpha, double beta, double gamma);

struct Position3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// types/layers.ts: DEFAULT_INVERT_LAYER_MASK = false, DEFAULT_EDGE_SMOOTHNESS = 0.0 (the interface
// comment says 1.0; the code constant is what runs).
inline constexpr bool kDefaultInvertLayerMask = false;
inline constexpr double kDefaultEdgeSmoothness = 0.0;

// Coordinates of LayersControllerArgs (types/common.ts Coordinates). `has_*` mirror the optional
// JS fields: an absent origin position keeps the default (0, 2, 0), an absent rotation identity.
struct Coordinates {
  bool has_origin_position = false;
  Position3 origin_position;
  bool has_origin_rotation = false;
  QuaternionXyzw origin_rotation;  // w, x, y, z
  bool left_handed = false;        // axes === 'LEFT_HANDED'
  bool mirrored_display = false;
};

// One layer's options (types/layers.ts LayerOptions).
struct LayerOptions {
  bool invert_layer_mask = kDefaultInvertLayerMask;
  double edge_smoothness = kDefaultEdgeSmoothness;
};

// What configure({...}) accepts, with JS null/undefined as std::optional-like flags.
//   * undefined field: leave as is; null: reset to default (or remove the layer).
struct LayerOptionsPatch {
  bool present = false;  // layers.sky given (non-undefined)
  bool remove = false;   // layers.sky === null
  bool has_invert = false;
  bool invert_is_null = false;
  bool invert = false;
  bool has_smoothness = false;
  bool smoothness_is_null = false;
  double smoothness = 0.0;
};

struct ConfigureArgs {
  bool has_near_clip = false;
  double near_clip = 0.0;
  bool has_far_clip = false;
  double far_clip = 0.0;
  bool has_coordinates = false;
  bool coordinates_null = false;
  Coordinates coordinates;
  bool layers_null = false;  // layers === null: drop every layer
  LayerOptionsPatch sky;
  bool has_debug = false;
  bool debug_null = false;
  bool debug_input_mask = false;
};

// Output of one layer (layers-controller.ts LayerOutput).
struct LayerOutput {
  // The mask texture content: r, g, b = confidence of the layer, flipped when invertLayerMask is set;
  // a = 1. Same memory layout as the original semantics texture (row 0 = GL row 0).
  Rgba8Image texture;
  int texture_width = 0;
  int texture_height = 0;
  double percentage = 0.0;
  // debug.inputMaskImg when configure({debug:{inputMask:true}}): the model output as gray RGBA.
  bool has_input_mask = false;
  Rgba8Image input_mask;
};

// LayersControllerProcessCpuOutput (what onProcessCpu returns when it has a response).
struct LayersFrameOutput {
  bool valid = false;
  Position3 position;
  QuaternionXyzw rotation;
  std::array<float, 16> intrinsics{};  // column-major projection matrix of the response camera
  LayerOutput sky;
};

// A pipeline event of the original (framework.dispatchEvent): `detail_json` is JSON.stringify(detail).
struct SkyEvent {
  std::string name;  // layerloading, layerscanning, layerfound
  std::string detail_json;
};

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_SKY_TYPES_H_
