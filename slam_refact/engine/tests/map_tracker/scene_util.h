// Synthetic scenes for the MapTracker tests: a world map with one keyframe whose points carry
// random ORB/GORB descriptors, and current frames that observe the same points (same descriptors)
// from another camera pose.

#ifndef SLAM_REFACT_TESTS_MAP_TRACKER_SCENE_UTIL_H_
#define SLAM_REFACT_TESTS_MAP_TRACKER_SCENE_UTIL_H_

#include <algorithm>
#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "map/world_map.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/image-descriptor.h"

namespace slam_refact::map_tracker::testing {

inline c8_PixelPinholeCameraModel SceneIntrinsics() {
  c8_PixelPinholeCameraModel k;
  k.pixelsWidth = 320;
  k.pixelsHeight = 240;
  k.centerPointX = 159.5f;
  k.centerPointY = 119.5f;
  k.focalLengthHorizontal = 260.0f;
  k.focalLengthVertical = 260.0f;
  return k;
}

struct Descriptor {
  std::array<uint8_t, 32> orb;
  std::array<uint8_t, 32> gorb;
};

inline std::vector<Descriptor> RandomDescriptors(size_t n, uint32_t seed) {
  std::mt19937 gen(seed);
  std::vector<Descriptor> out(n);
  for (Descriptor& d : out) {
    for (int i = 0; i < 32; ++i) {
      d.orb[i] = static_cast<uint8_t>(gen() & 0xff);
      d.gorb[i] = static_cast<uint8_t>(gen() & 0xff);
    }
  }
  return out;
}

inline c8::FeatureSet Features(const Descriptor& d) {
  c8::FeatureSet set;
  set.orbFeature = c8::OrbFeature(d.orb);
  set.gorbFeature = c8::GorbFeature(d.gorb);
  return set;
}

// Grid of world points in front of the identity camera: rays in [-0.45, 0.45] x [-0.35, 0.35],
// depth 2.5 + 1.0 * ((i * 7) % 5) / 4.
inline std::vector<c8::HPoint3> GridPoints(int nx, int ny) {
  std::vector<c8::HPoint3> pts;
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const int k = j * nx + i;
      const float rx = -0.45f + 0.9f * static_cast<float>(i) / static_cast<float>(nx - 1);
      const float ry = -0.35f + 0.7f * static_cast<float>(j) / static_cast<float>(ny - 1);
      const float d = 2.5f + 1.0f * static_cast<float>((k * 7) % 5) / 4.0f;
      pts.emplace_back(rx * d, ry * d, d);
    }
  }
  return pts;
}

// Frame observing `world` from `world_from_camera`; points outside the image are skipped and their
// indices are not returned. `observed` receives the world index of every frame point.
inline void AddObservations(c8::FrameWithPoints* f, const std::vector<c8::HPoint3>& world,
                            const std::vector<Descriptor>& desc,
                            const c8::HMatrix& world_from_camera, std::vector<size_t>* observed) {
  const c8::HMatrix inv = world_from_camera.inv();
  for (size_t i = 0; i < world.size(); ++i) {
    const c8::HPoint3 pc = inv * world[i];
    if (pc.z() <= 0.1f) {
      continue;
    }
    const c8::HPoint2 ray(pc.x() / pc.z(), pc.y() / pc.z());
    if (ray.x() < -0.58f || ray.x() > 0.58f || ray.y() < -0.43f || ray.y() > 0.43f) {
      continue;
    }
    f->addImageRayPoint(ray, 1, 0.0f, 0.0f, Features(desc[i]));
    if (observed != nullptr) {
      observed->push_back(i);
    }
  }
}

// Map with one keyframe at `kf_pose` and one map point per visible world point.
struct Scene {
  map::WorldMap map;
  map::KeyframeId kf;
  std::vector<map::PointId> ids;  // per world point (null id when not visible in the keyframe)
};

inline void BuildScene(Scene* s, const std::vector<c8::HPoint3>& world,
                       const std::vector<Descriptor>& desc, const c8::HMatrix& kf_pose,
                       map::PointSource source) {
  c8::FrameWithPoints f(SceneIntrinsics(), kf_pose, c8::Quaternion());
  std::vector<size_t> observed;
  AddObservations(&f, world, desc, kf_pose, &observed);
  s->kf = s->map.AddKeyframe(f, 0);
  s->map.SetKeyframePose(s->kf, kf_pose);
  s->ids.assign(world.size(), map::PointId());
  for (size_t k = 0; k < observed.size(); ++k) {
    const map::PointId pid =
        s->map.AddPoint(s->map.keyframe_seq_counter(), world[observed[k]], source);
    s->map.AddObservation(pid, static_cast<uint32_t>(k), s->kf);
    s->ids[observed[k]] = pid;
  }
}

inline TrackerInput MakeInput(const std::vector<c8::HPoint3>& world,
                              const std::vector<Descriptor>& desc,
                              const c8::HMatrix& world_from_camera, int64_t time_ns) {
  const c8::Quaternion q = c8::rotation(world_from_camera);
  TrackerInput in(SceneIntrinsics(), q, time_ns);
  AddObservations(&in.frame_points, world, desc, world_from_camera, nullptr);
  return in;
}

inline float MaxTranslationError(const c8::HMatrix& a, const c8::HMatrix& b) {
  float e = 0.0f;
  for (int r = 0; r < 3; ++r) {
    const float d = a(r, 3) - b(r, 3);
    e = std::max(e, d < 0 ? -d : d);
  }
  return e;
}

inline float MaxRotationError(const c8::HMatrix& a, const c8::HMatrix& b) {
  float e = 0.0f;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      const float d = a(r, c) - b(r, c);
      e = std::max(e, d < 0 ? -d : d);
    }
  }
  return e;
}

}  // namespace slam_refact::map_tracker::testing

#endif  // SLAM_REFACT_TESTS_MAP_TRACKER_SCENE_UTIL_H_
