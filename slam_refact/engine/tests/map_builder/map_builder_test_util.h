// Helpers for the WP3 map builder tests: synthetic frames, poses and golden-file readers.

#ifndef SLAM_REFACT_TESTS_MAP_BUILDER_MAP_BUILDER_TEST_UTIL_H_
#define SLAM_REFACT_TESTS_MAP_BUILDER_MAP_BUILDER_TEST_UTIL_H_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/parameter-data.h"
#include "c8/quaternion.h"
#include "common/check.h"
#include "common/tracking_types.h"
#include "config/config_defaults.h"
#include "config/params.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/image-descriptor.h"

namespace slam_refact::map_builder::testing {

// Applies parameter overrides to the process-global globalParams() for one scope and restores
// the values the keys had before, also when an ASSERT_* leaves the test body early (S2 review
// LOW-5), so a failing test cannot leak its configuration into later tests of the same binary.
class ScopedParamOverrides {
 public:
  explicit ScopedParamOverrides(std::vector<std::pair<std::string, ParamValue>> overrides) {
    c8::ParameterData& params = c8::globalParams();
    for (const auto& entry : overrides) {
      const config::ConfigKeyInfo* info = config::FindConfigKey(entry.first.c_str());
      SR_CHECK(info != nullptr && info->default_value.has_value(),
               "ScopedParamOverrides: unknown or untyped key");
      // Current value (getOrSet with the binary default, as every Settings::Get() reads it).
      const ParamValue previous = std::visit(
          [&](auto def) { return ParamValue(params.getOrSet<decltype(def)>(entry.first.c_str(), def)); },
          *info->default_value);
      saved_.emplace_back(entry.first, previous);
    }
    config::ApplyParamOverrides(overrides);
  }
  ~ScopedParamOverrides() { config::ApplyParamOverrides(saved_); }

  ScopedParamOverrides(const ScopedParamOverrides&) = delete;
  ScopedParamOverrides& operator=(const ScopedParamOverrides&) = delete;

 private:
  std::vector<std::pair<std::string, ParamValue>> saved_;
};

inline c8_PixelPinholeCameraModel TestIntrinsics() {
  c8_PixelPinholeCameraModel k;
  k.pixelsWidth = 320;
  k.pixelsHeight = 240;
  k.centerPointX = 159.5f;
  k.centerPointY = 119.5f;
  k.focalLengthHorizontal = 260.0f;
  k.focalLengthVertical = 260.0f;
  return k;
}

// Deterministic 32-byte descriptor for a feature id (xorshift).
inline std::array<uint8_t, 32> Descriptor(uint32_t id) {
  std::array<uint8_t, 32> d;
  uint32_t s = id * 2654435761u + 12345u;
  for (uint8_t& b : d) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    b = static_cast<uint8_t>(s >> 24);
  }
  return d;
}

struct RayFeature {
  float x;
  float y;
  uint8_t scale;
  uint32_t descriptor_id;
};

// A frame with one keypoint per ray; every keypoint carries an ORB and a GORB descriptor.
inline void AddRays(c8::FrameWithPoints* f, const std::vector<RayFeature>& rays) {
  for (const RayFeature& r : rays) {
    c8::FeatureSet set;
    set.orbFeature = c8::OrbFeature(Descriptor(r.descriptor_id));
    set.gorbFeature = c8::GorbFeature(Descriptor(r.descriptor_id ^ 0x5a5a5a5au));
    f->addImageRayPoint(c8::HPoint2(r.x, r.y), r.scale, 0.25f, 0.5f, std::move(set));
  }
}

inline c8::FrameWithPoints MakeFrame(const std::vector<RayFeature>& rays) {
  c8::FrameWithPoints f(TestIntrinsics());
  AddRays(&f, rays);
  return f;
}

// world_from_camera with rotation about y by yaw (radians) and translation t.
inline c8::HMatrix Pose(float yaw, float tx, float ty, float tz) {
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  return c8::HMatrixGen::translation(tx, ty, tz) *
         c8::HMatrix({c, 0.0f, s, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {-s, 0.0f, c, 0.0f},
                     {0.0f, 0.0f, 0.0f, 1.0f}, {c, 0.0f, -s, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f},
                     {s, 0.0f, c, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
}

// Ray of a world point in a camera (x/z, y/z of camera_from_world * X).
inline c8::HPoint2 Project(const c8::HMatrix& world_from_camera, const c8::HPoint3& x) {
  const c8::HPoint3 c = world_from_camera.inv() * x;
  return c8::HPoint2(c.x() / c.z(), c.y() / c.z());
}

inline float BitsToFloat(uint32_t w) {
  float f;
  std::memcpy(&f, &w, 4);
  return f;
}

inline double BitsToDouble(uint64_t w) {
  double d;
  std::memcpy(&d, &w, 8);
  return d;
}

// |ours - ref| <= rel * max(1, |ref|); non-finite references must match exactly in class.
inline bool Near(double ours, double ref, double rel) {
  if (!std::isfinite(ref)) {
    return std::isnan(ref) ? std::isnan(ours) : ours == ref;
  }
  return std::fabs(ours - ref) <= rel * std::fmax(1.0, std::fabs(ref));
}

inline std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

inline std::vector<std::string> Split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == sep) {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

inline std::vector<uint32_t> HexWords(const std::string& csv) {
  std::vector<uint32_t> out;
  if (csv.empty()) {
    return out;
  }
  for (const std::string& t : Split(csv, ',')) {
    out.push_back(static_cast<uint32_t>(std::strtoul(t.c_str(), nullptr, 16)));
  }
  return out;
}

}  // namespace slam_refact::map_builder::testing

#endif  // SLAM_REFACT_TESTS_MAP_BUILDER_MAP_BUILDER_TEST_UTIL_H_
