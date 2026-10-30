// slam_refact public API: basic value types (architecture.md §4.1, §4.2).
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.
// This header never exposes c8 types; internal code converts with src/common/c8_adapters.h.

#ifndef SLAM_REFACT_TYPES_H_
#define SLAM_REFACT_TYPES_H_

#include <cstdint>
#include <variant>

namespace slam_refact {

struct Vec3f {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
};

// Public API quaternions are (w, x, y, z), the order used by the original traces.
// (c8::Quaternion stores x, y, z, w in memory; see c8_adapters.h.)
struct QuatWxyz {
  float w = 1.f;
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
};

// 4x4 matrix, column-major: element (row r, column c) = m[4 * c + r]; translation = m[12..14].
struct HMatrix4f {
  float m[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
};

// XrTrackingState.status (pose.capnp:14-31, S4 §2.6; VERIFIED D:85110-85118).
enum class TrackingStatus : uint16_t {
  kUnspecified = 0,
  kNotAvailable = 1,
  kLimited = 2,
  kNormal = 3,
};

// XrTrackingState.reason (S4 §2.6).
enum class TrackingReason : uint16_t {
  kUnspecified = 0,
  kInitializing = 1,
  kRelocalizing = 2,
  kTooMuchMotion = 3,
  kNotEnoughTexture = 4,
};

// Same field order and widths as c8_PixelPinholeCameraModel (24 bytes).
struct PinholeIntrinsics {
  int width = 0;
  int height = 0;
  float cx = 0.f;
  float cy = 0.f;
  float fx = 0.f;
  float fy = 0.f;
};

// Numeric model of the CPU port of the GPU pyramid shader (S1 §5.1.2, D-9).
enum class ShaderNumericModel {
  kIeee,
  kSwiftShaderApprox,
};

// Value of a process-global parameter (c8 globalParams()) override.
using ParamValue = std::variant<bool, int, float, double>;

// One published world point (S4 §3.10). 0-padding entries have id 0 and confidence 0 (Q10).
struct WorldPointOut {
  uint64_t id = 0;
  Vec3f position;
  float confidence = 0.f;
};

// Per-stage wall time of one tracking step, milliseconds (§4.2).
struct StageTimings {
  double frontend_ms = 0.0;
  double map_tracker_ms = 0.0;
  double map_builder_ms = 0.0;
  double metric_scale_ms = 0.0;
  double glue_ms = 0.0;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_TYPES_H_
