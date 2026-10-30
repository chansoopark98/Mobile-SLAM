// US-015: the closed image world tracker (func994) reconstructed from xr-slam.wasm and its opt-in
// integration (vendored patch us015-image-world-tracker, diagnostics::SetImageWorldTracker; contract
// FULL-12). Constructed geometry with known answers, label unit. The comparison with the original
// func994 itself is the oracle run of slam_refact/tools/re/iwt_oracle (spec/11 section 9.4), not this
// file.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/pixels/gr8-pyramid.h"
#include "config/params.h"
#include "image_target/image_target_spec.h"
#include "image_target/image_world_tracker.h"
#include "image_target/target_loader.h"
#include "map/world_map.h"
#include "reality/engine/imagedetection/detection-image-tracker.h"
#include "slam_refact/replay_diagnostics.h"
#include "test_util.h"
#include "vio_tracker/tracker_modules.h"
#include "vio_tracker/vio_tracker.h"
#include "vio_tracker/vio_tracker_internal.h"

namespace slam_refact {
namespace {

using image_target::MapToTrackerTransform;
using image_target::TrackedMapPoint;
using image_target::TrackedPointLookup;
using image_target::TrackImageWorld;
using image_target::testing::SyntheticTarget;
using image_target::testing::TestK;
using image_target::testing::TestPinholeK;

constexpr int32_t kTri = 1;
constexpr int32_t kGround = 2;

class MapLookup final : public TrackedPointLookup {
 public:
  std::map<uint32_t, TrackedMapPoint> points;
  bool Find(uint32_t frame_idx, TrackedMapPoint* out) const override {
    const auto it = points.find(frame_idx);
    if (it == points.end()) return false;
    *out = it->second;
    return true;
  }
};

bool SameBits(const c8::HMatrix& a, const c8::HMatrix& b) {
  return std::memcmp(a.data().data(), b.data().data(), sizeof(float) * 16) == 0 &&
         std::memcmp(a.inverseData().data(), b.inverseData().data(), sizeof(float) * 16) == 0;
}

float MaxAbsDiff(const c8::HMatrix& a, const c8::HMatrix& b) {
  float m = 0.0f;
  for (int i = 0; i < 16; ++i) m = std::fmax(m, std::fabs(a.data()[i] - b.data()[i]));
  return m;
}

// A planar target seen by a camera: target view pose P (camera in the model frame whose target
// plane is z = 1), world camera tam, n_on on-target features (map points at their true world
// position for world / target unit ratio k) and n_off background features.
struct PlanarScene {
  PlanarScene(float k, int n_on, int n_off) : k(k) {
    p = c8::HMatrixGen::translation(0.08f, -0.05f, -0.35f) * c8::HMatrixGen::yDegrees(11.0f) *
        c8::HMatrixGen::xDegrees(7.0f);
    tam = c8::HMatrixGen::translation(0.4f, 1.3f, -0.7f) * c8::HMatrixGen::yDegrees(52.0f);
    const c8_PixelPinholeCameraModel cam_k = TestK();
    feats = std::make_unique<c8::FrameWithPoints>(cam_k);
    uint32_t state = 12345u;
    auto uniform = [&state](float lo, float hi) {
      state = state * 1664525u + 1013904223u;
      return lo + (hi - lo) * static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
    };
    for (int i = 0; i < n_on + n_off; ++i) {
      c8::HPoint2 ray;
      if (i < n_on) {
        const c8::HPoint3 xm{uniform(-0.3f, 0.3f), uniform(-0.4f, 0.4f), 1.0f};
        const c8::HPoint3 xc = p.inv() * xm;
        ray = c8::HPoint2(xc.x() / xc.z(), xc.y() / xc.z());
        TrackedMapPoint mp;
        mp.slot = static_cast<uint32_t>(i);
        mp.id = 1000u + static_cast<uint32_t>(i);
        mp.position = tam * c8::HPoint3(k * xc.x(), k * xc.y(), k * xc.z());
        mp.source = kTri;
        lookup.points[static_cast<uint32_t>(i)] = mp;
      } else {
        ray = c8::HPoint2(uniform(-0.6f, 0.6f), uniform(-0.7f, 0.7f));
      }
      const c8::HPoint2 pix(cam_k.focalLengthHorizontal * ray.x() + cam_k.centerPointX,
                            cam_k.centerPointY - cam_k.focalLengthVertical * ray.y());
      feats->addWorldMapPoint(c8::FramePoint(ray, 0, 0.0f, 0.0f), pix, c8::FeatureSet{});
    }
    im = std::make_unique<c8::DetectionImage>("t", c8::TargetWithPoints(TestK()), 0,
                                              c8::PlanarImageGeometry{});
  }
  c8::HMatrix Expected(float s) const { return tam * c8::scaleTranslation(s, p.inv()); }

  float k;
  c8::HMatrix p = c8::HMatrixGen::i();
  c8::HMatrix tam = c8::HMatrixGen::i();
  std::unique_ptr<c8::FrameWithPoints> feats;
  std::unique_ptr<c8::DetectionImage> im;
  MapLookup lookup;
};

TEST(ImageWorldTracker, SolvesTheWorldToTargetScaleFromTrackedMapPoints) {
  PlanarScene sc(0.72f, 40, 20);
  c8::ImageWorldTrackerState st;  // scale 1 (binary temp construction)
  const c8::HMatrix out =
      TrackImageWorld(&st, sc.lookup, {}, *sc.im, sc.p, sc.tam, *sc.feats, 0.5f);
  EXPECT_EQ(st.rays.size(), 60u);
  ASSERT_GE(st.visible.size(), 40u);  // every on-target feature, maybe a background one inside
  EXPECT_EQ(st.worldPts.size(), 40u);
  EXPECT_EQ(st.targetPts.size(), 40u);
  EXPECT_NEAR(st.scale, 0.72f, 1e-4f);
  EXPECT_LT(MaxAbsDiff(out, sc.Expected(0.72f)), 1e-4f);
  EXPECT_TRUE(SameBits(out, st.worldPose));
}

TEST(ImageWorldTracker, FewerThanSixPairsUseTheScaleGuess) {
  PlanarScene sc(0.72f, 5, 10);
  c8::ImageWorldTrackerState st;
  const c8::HMatrix out =
      TrackImageWorld(&st, sc.lookup, {}, *sc.im, sc.p, sc.tam, *sc.feats, 0.61f);
  EXPECT_EQ(st.worldPts.size(), 5u);
  EXPECT_EQ(st.scale, 0.61f);
  EXPECT_TRUE(SameBits(out, sc.Expected(0.61f)));
}

TEST(ImageWorldTracker, OnlyVisibleTriangulatedPointsArePaired) {
  PlanarScene sc(1.15f, 12, 6);
  sc.lookup.points[0].source = kGround;
  sc.lookup.points[1].source = 3;  // SKY
  // A background feature tied to a map point: paired only if it falls inside the target outline.
  TrackedMapPoint far = sc.lookup.points[2];
  far.slot = 900;
  sc.lookup.points[14] = far;
  c8::ImageWorldTrackerState st;
  TrackImageWorld(&st, sc.lookup, {}, *sc.im, sc.p, sc.tam, *sc.feats, 0.5f);
  size_t expected = 0;
  for (size_t idx : st.visible) {
    const auto it = sc.lookup.points.find(static_cast<uint32_t>(idx));
    expected += it != sc.lookup.points.end() && it->second.source == kTri;
  }
  EXPECT_EQ(st.worldPts.size(), expected);
  for (const auto& id : st.ids) {
    EXPECT_NE(id.index(), 0u);
    EXPECT_NE(id.index(), 1u);
  }
}

TEST(ImageWorldTracker, AnUnusableSolveKeepsTheScaleAndTheWorldPose) {
  PlanarScene sc(0.72f, 20, 0);
  auto& raw = sc.lookup.points[4].position.raw();
  raw[3] = 0.0f;  // w = 0: non-finite residual, Ceres FAILURE
  c8::ImageWorldTrackerState st;
  st.scale = 0.5f;
  st.worldPose = c8::HMatrixGen::translation(9.0f, 9.0f, 9.0f);
  const c8::HMatrix before = st.worldPose;
  const c8::HMatrix out =
      TrackImageWorld(&st, sc.lookup, {}, *sc.im, sc.p, sc.tam, *sc.feats, 0.3f);
  EXPECT_EQ(st.scale, 0.5f);
  EXPECT_TRUE(SameBits(out, before));
}

TEST(ImageWorldTracker, MapTransformIsAppliedBeforeTheSolve) {
  PlanarScene sc(0.9f, 25, 0);
  MapToTrackerTransform xf;
  xf.scale = 2.0f;
  xf.offset = {{0.25f, -0.4f, 0.15f}};  // every axis non-zero: the sign of each offset term is seen
  c8::ImageWorldTrackerState st;
  TrackImageWorld(&st, sc.lookup, xf, *sc.im, sc.p, sc.tam, *sc.feats, 0.5f);
  ASSERT_EQ(st.worldPts.size(), 25u);
  for (size_t i = 0; i < st.worldPts.size(); ++i) {
    const c8::HPoint3& w = sc.lookup.points.at(static_cast<uint32_t>(st.ids[i].index())).position;
    EXPECT_EQ(st.worldPts[i].x(), (w.x() - 0.25f) * 2.0f) << i;
    EXPECT_EQ(st.worldPts[i].y(), (w.y() + 0.4f) * 2.0f) << i;
    EXPECT_EQ(st.worldPts[i].z(), (w.z() - 0.15f) * 2.0f) << i;
  }
}

// ---- VioTracker integration (opt-in FULL-12) -------------------------------------------------

c8::DetectionImage LoadTarget(const std::string& name) {
  static const image_target::GrayImage img = SyntheticTarget(480, 640, 7);
  image_target::ImageTargetSpec spec;
  spec.name = name;
  spec.type = image_target::ImageTargetType::kPlanar;
  return image_target::LoadDetectionImage(spec, img.view(), TestPinholeK(),
                                          ShaderNumericModel::kIeee);
}

// A photo of the target from `cam` (target plane z = 1, detection-image-tracker-test.cc image()) and,
// for every photographed target point, its world position for the world camera G * scaleT(k, cam).
struct World {
  World(const c8::TargetWithPoints& src, const c8::HMatrix& cam, const c8::HMatrix& g, float k) {
    const c8_PixelPinholeCameraModel kk = TestK();
    photo = std::make_unique<c8::FrameWithPoints>(kk);
    const c8::HMatrix km = c8::HMatrixGen::intrinsic(kk);
    for (size_t i = 0; i < src.points().size(); ++i) {
      const auto& pt = src.points()[i];
      const c8::HPoint3 wpt{pt.x(), pt.y(), 1.0f};
      if ((cam.inv() * wpt).z() < 0) continue;
      const c8::HPoint2 im = (km * cam.inv() * wpt).flatten();
      if (im.x() < 0 || im.y() < 0 || im.x() >= kk.pixelsWidth || im.y() >= kk.pixelsHeight) continue;
      photo->addImagePixelPoint(im, pt.scale(), pt.angle(), pt.gravityAngle(),
                                src.store().getClone(i));
      world.push_back(g * c8::HPoint3(k * wpt.x(), k * wpt.y(), k * wpt.z()));
    }
    tam = g * c8::scaleTranslation(k, cam);
  }
  std::unique_ptr<c8::FrameWithPoints> photo;
  std::vector<c8::HPoint3> world;
  c8::HMatrix tam = c8::HMatrixGen::i();
};

struct WorldModules : public vio_tracker::TrackerModules {
  const World* cur = nullptr;
  bool with_map = true;
  map::PointSource source = map::PointSource::kTriangulated;  // source of every tracked map point
  map::WorldMap wm;

  void Detect(const c8::Gr8Pyramid&, const c8::DetectionConfig&, c8::FrameWithPoints* f) override {
    if (cur != nullptr) f->clearAndClone(*cur->photo);
  }
  frontend::DetectStats detect_stats() const override { return {}; }
  void TrackMap(TrackerInput*, const FrameState&, MapTrackerOutput* out,
                compat::RandomNumbers*) override {
    out->state = 1;
    out->tracked.clear();
    if (cur == nullptr) return;
    for (size_t i = 0; i < cur->world.size(); ++i) {  // every feature tracked to its true point
      out->tracked[static_cast<uint32_t>(i)] = wm.AddPoint(0, cur->world[i], source);
    }
  }
  void UpdateMap(const TrackerInput&, const MapTrackerOutput&, int64_t, MapBuilderOutput* out) override {
    *out = MapBuilderOutput{};
  }
  c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput&) override {
    return cur != nullptr ? cur->tam : c8::HMatrixGen::i();
  }
  void RecreateMapTrackerAndBuilder() override {}
  void VisibleMapPoints(std::vector<vio_tracker::VisibleMapPoint>* out) const override { out->clear(); }
  const map::WorldMap* active_world_map() const override { return with_map ? &wm : nullptr; }
  int ground_match_count() const override { return 0; }
  int num_keyframes() const override { return 0; }
  std::array<int, 5> num_points_by_source() const override { return {0, 0, 0, 0, 0}; }
};

struct Rig {
  explicit Rig(bool disable_vio = false, bool with_map = true) {
    config::RegisterDefaults();
    auto m = std::make_unique<WorldModules>();
    m->with_map = with_map;
    modules = m.get();
    tracker = vio_tracker::NewVioTrackerWithModules(false, disable_vio, std::move(m));
    pyr.levels.push_back(c8::LevelLayout{});
  }
  void Track(int frame, const World* w, const DetectionImagePtrMap& targets) {
    modules->cur = w;
    TrackingSensorFrame f;
    f.time_ns = 1000000000LL + frame * 33333333LL;
    f.intrinsic = TestK();
    tracker->TrackWithTargets(f, pyr, targets, &rng, &status, &debug);
  }
  const c8::DetectionImageTracker& image_tracker() {
    return vio_tracker::VioTrackerAccess::state(*tracker).image_tracker;
  }
  WorldModules* modules = nullptr;
  std::unique_ptr<vio_tracker::VioTracker> tracker;
  c8::Gr8Pyramid pyr;
  compat::RandomNumbers rng;
  vio_tracker::StatusOut status;
  vio_tracker::TrackDebug debug;
};

// The world tracker is on by default (contract-changes.md WT-DEFAULT-ON); the guards set the value explicitly
// and restore the previous one, so a test never leaves the process-wide flag different from how it found it.
struct ImageWorldTrackerScope {
  explicit ImageWorldTrackerScope(bool on) : previous_(diagnostics::ImageWorldTrackerEnabled()) {
    diagnostics::SetImageWorldTracker(on);
  }
  ~ImageWorldTrackerScope() { diagnostics::SetImageWorldTracker(previous_); }
  ImageWorldTrackerScope(const ImageWorldTrackerScope&) = delete;
  ImageWorldTrackerScope& operator=(const ImageWorldTrackerScope&) = delete;

 private:
  bool previous_;
};
struct ImageWorldTrackerOn : ImageWorldTrackerScope {
  ImageWorldTrackerOn() : ImageWorldTrackerScope(true) {}
};
struct ImageWorldTrackerOff : ImageWorldTrackerScope {
  ImageWorldTrackerOff() : ImageWorldTrackerScope(false) {}
};

// World pose of the target (target plane z = 1 scaled by k, then g): the answer of every world pose
// below, since tam * scaleTranslation(k, cam^-1) = g * scaleTranslation(k, cam * cam^-1) = g.
c8::HMatrix TargetWorld() {
  return c8::HMatrixGen::translation(0.3f, 1.1f, -0.4f) * c8::HMatrixGen::yDegrees(35.0f);
}

// A short orbit (photo pose changes by ~1-2 cm and ~1 degree per frame).
std::vector<std::unique_ptr<World>> Orbit(const c8::DetectionImage& im, float k, int n) {
  std::vector<std::unique_ptr<World>> out;
  for (int i = 0; i < n; ++i) {
    const c8::HMatrix cam = c8::HMatrixGen::translation(0.02f * i, -0.01f * i, 0.0f) *
                            c8::HMatrixGen::yDegrees(1.0f * i);
    out.push_back(std::make_unique<World>(im.framePoints(), cam, TargetWorld(), k));
  }
  return out;
}

// A frame where the target is seen but every map point is non-finite (w = 0): 994 pairs >= 6 points
// and the Ceres solve is unusable, so the world tracker keeps its state. `g` may differ from the
// target world pose of the other frames (the target moved).
std::unique_ptr<World> UnusableSolveFrame(const c8::DetectionImage& im, const c8::HMatrix& g, float k) {
  auto w = std::make_unique<World>(
      im.framePoints(), c8::HMatrixGen::translation(0.1f, -0.05f, 0.0f) * c8::HMatrixGen::yDegrees(5.0f), g, k);
  for (auto& p : w->world) p.raw()[3] = 0.0f;
  return w;
}

std::unique_ptr<World> LostFrame(float k) {  // the camera keeps moving, the photo is empty
  return std::make_unique<World>(c8::TargetWithPoints(TestK()),
                                 c8::HMatrixGen::translation(0.13f, -0.06f, 0.0f), TargetWorld(), k);
}

bool SameLocated(const std::vector<c8::LocatedImage>& a, const std::vector<c8::LocatedImage>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (!SameBits(a[i].pose, b[i].pose) || a[i].width != b[i].width || a[i].height != b[i].height ||
        !SameBits(a[i].targetSpace.pose, b[i].targetSpace.pose) ||
        !SameBits(a[i].targetSpace.worldPose, b[i].targetSpace.worldPose) ||
        a[i].targetSpace.scale != b[i].targetSpace.scale) {
      return false;
    }
  }
  return true;
}

TEST(ImageWorldTrackerVio, OffIgnoresTheMapAndOnWithDisableVioIsThePublicPath) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const auto orbit = Orbit(images.at("t0"), 0.72f, 10);
  ImageWorldTrackerOff explicit_off;  // the default is on: the "off" rigs below need it set explicitly
  Rig off;                     // option off, map present
  Rig nomap(false, false);     // option off, no map at all
  Rig off_dv(true);            // disableVio, option off
  Rig on_dv(true);             // disableVio, option on: 938 c = 0, public latch
  for (int i = 0; i < 10; ++i) {
    off.Track(i, orbit[i].get(), targets);
    nomap.Track(i, orbit[i].get(), targets);
    off_dv.Track(i, orbit[i].get(), targets);
    {
      ImageWorldTrackerOn on;
      on_dv.Track(i, orbit[i].get(), targets);
    }
    EXPECT_TRUE(SameLocated(off.tracker->located_images(), nomap.tracker->located_images())) << i;
    EXPECT_TRUE(SameLocated(off_dv.tracker->located_images(), on_dv.tracker->located_images())) << i;
  }
  ASSERT_FALSE(off.tracker->located_images().empty());
  // Off: the world tracker state is never touched (scale 1, no pairs).
  EXPECT_EQ(off.image_tracker().worldTracker("t0").scale, 1.0f);
  EXPECT_TRUE(off.image_tracker().worldTracker("t0").worldPts.empty());
  EXPECT_TRUE(on_dv.image_tracker().worldTracker("t0").worldPts.empty());
}

TEST(ImageWorldTrackerVio, OnSolvesTheScaleFromTheMapAndFeedsTheScaleGuess) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const float k = 0.72f;
  const auto orbit = Orbit(images.at("t0"), k, 6);
  Rig on;
  Rig off;
  ImageWorldTrackerOn enable;
  for (int i = 0; i < 6; ++i) {
    on.Track(i, orbit[i].get(), targets);
    diagnostics::SetImageWorldTracker(false);
    off.Track(i, orbit[i].get(), targets);
    diagnostics::SetImageWorldTracker(true);
  }
  ASSERT_EQ(on.tracker->located_images().size(), 1u);
  const c8::ImageWorldTrackerState& st = on.image_tracker().worldTracker("t0");
  EXPECT_GE(st.worldPts.size(), 6u);
  EXPECT_NEAR(st.scale, k, 2e-3f * k);
  // 938 copies the solved scale into the public scale guess: located size = frameBounds * scale.
  const c8::LocatedImage& l_on = on.tracker->located_images()[0];
  const c8::LocatedImage& l_off = off.tracker->located_images()[0];
  EXPECT_NEAR(l_on.targetSpace.scale, k, 2e-3f * k);
  EXPECT_NEAR(l_on.width / l_off.width, k / l_off.targetSpace.scale, 2e-3f);
  // The public estimator alone has not moved from 0.5 by much (per-frame tam motion here is small).
  EXPECT_GT(std::fabs(l_off.targetSpace.scale - k), 0.1f);
}

// 938 diff 2/3: the world pose is recomputed on every tracked frame (not latched on the first one) and a lost
// frame publishes it. Frame 0 has no map (fallback: public scale guess 0.5, so its world pose is wrong by
// k / 0.5); later frames solve the scale from the map. The published world pose of the lost frame must be
// the target's true world pose, not the frame-0 one. (Reviewer probe P1 of opus-xhigh-r1-us015-impl.md.)
TEST(ImageWorldTrackerVio, OnLostTargetPublishesTheLatestWorldPoseNotTheFirst) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const float k = 0.72f;
  const auto orbit = Orbit(images.at("t0"), k, 6);
  const auto lost = LostFrame(k);
  Rig on;
  ImageWorldTrackerOn enable;
  on.modules->with_map = false;
  on.Track(0, orbit[5].get(), targets);
  const c8::HMatrix first_world = on.image_tracker().worldTracker("t0").worldPose;
  on.modules->with_map = true;
  for (int i = 1; i < 6; ++i) on.Track(i, orbit[5 - i].get(), targets);
  on.Track(6, lost.get(), targets);
  ASSERT_EQ(on.tracker->located_images().size(), 1u);
  const c8::LocatedImage& l = on.tracker->located_images()[0];
  EXPECT_EQ(l.targetSpace.lastSeen, 1);
  EXPECT_GT(MaxAbsDiff(first_world, TargetWorld()), 1e-2f);  // the frame-0 pose really is different
  EXPECT_LT(MaxAbsDiff(l.targetSpace.worldPose, TargetWorld()), 1e-3f);
  // The target-relative pose of the lost frame follows from that world pose and the camera.
  const c8::HMatrix expected_rel =
      c8::scaleTranslation(1.0f / l.targetSpace.scale, c8::egomotion(l.targetSpace.worldPose, lost->tam));
  EXPECT_LT(MaxAbsDiff(l.targetSpace.pose, expected_rel), 1e-5f);
}

// 938 diff 3 when the world pose is stale: the last tracked frame's solve is unusable and the target moved
// (g2), so the public extrapolation (lastCamPos and the frame's target pose, i.e. g2) and the binary rule
// (worldPose^-1 * tam with the stale world pose, i.e. g) differ.
TEST(ImageWorldTrackerVio, OnLostFrameUsesTheWorldPoseEvenWhenItIsStale) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const float k = 0.72f;
  const auto orbit = Orbit(images.at("t0"), k, 6);
  const c8::HMatrix g2 = c8::HMatrixGen::translation(0.25f, 0.0f, 0.1f) * TargetWorld();
  const auto bad = UnusableSolveFrame(images.at("t0"), g2, k);
  const auto lost = LostFrame(k);
  Rig on;
  ImageWorldTrackerOn enable;
  for (int i = 0; i < 6; ++i) on.Track(i, orbit[5 - i].get(), targets);
  on.Track(6, bad.get(), targets);
  ASSERT_GE(on.image_tracker().worldTracker("t0").worldPts.size(), 6u);
  on.Track(7, lost.get(), targets);
  ASSERT_EQ(on.tracker->located_images().size(), 1u);
  const c8::LocatedImage& l = on.tracker->located_images()[0];
  EXPECT_LT(MaxAbsDiff(l.targetSpace.worldPose, TargetWorld()), 1e-3f);  // stale: still g
  const c8::HMatrix expected_rel =
      c8::scaleTranslation(1.0f / l.targetSpace.scale, c8::egomotion(l.targetSpace.worldPose, lost->tam));
  EXPECT_LT(MaxAbsDiff(l.targetSpace.pose, expected_rel), 1e-5f);
}

// 938 c = !disableVio: with disableVio the option changes nothing, including the published world pose of a
// lost frame (public latch) and the world tracker state. (Reviewer probe P2.)
TEST(ImageWorldTrackerVio, DisableVioLostFrameIsThePublicLatchWithTheOptionOn) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const auto orbit = Orbit(images.at("t0"), 0.72f, 6);
  const auto lost = LostFrame(0.72f);
  Rig off_dv(true);
  Rig on_dv(true);
  for (int i = 0; i < 7; ++i) {
    const World* w = i < 6 ? orbit[5 - i].get() : lost.get();
    {
      ImageWorldTrackerOff off;  // the default is on: the comparison rig needs the option off explicitly
      off_dv.Track(i, w, targets);
    }
    ImageWorldTrackerOn on;
    on_dv.Track(i, w, targets);
    EXPECT_TRUE(SameLocated(off_dv.tracker->located_images(), on_dv.tracker->located_images())) << i;
  }
  ASSERT_EQ(on_dv.tracker->located_images().size(), 1u);
  EXPECT_EQ(on_dv.tracker->located_images()[0].targetSpace.lastSeen, 1);
  EXPECT_TRUE(on_dv.image_tracker().worldTracker("t0").rays.empty());
}

// 938 diff 1: after more than WORLD_RESET_UNTRACKED_FRAMES (100) untracked frames the re-found frame resets the
// world tracker pose to identity before 994; visible only when that solve is unusable. (Reviewer probe P3.)
TEST(ImageWorldTrackerVio, ResetAfterHundredUntrackedFramesShowsOnAnUnusableSolve) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const auto orbit = Orbit(images.at("t0"), 0.72f, 6);
  const auto lost = LostFrame(0.72f);
  const auto bad = UnusableSolveFrame(images.at("t0"), TargetWorld(), 0.72f);
  Rig on;
  ImageWorldTrackerOn enable;
  int f = 0;
  for (int i = 0; i < 6; ++i) on.Track(f++, orbit[5 - i].get(), targets);
  const float solved_scale = on.image_tracker().worldTracker("t0").scale;
  for (int i = 0; i < 101; ++i) on.Track(f++, lost.get(), targets);  // lastSeen reaches 101
  on.Track(f++, bad.get(), targets);
  const c8::ImageWorldTrackerState& st = on.image_tracker().worldTracker("t0");
  ASSERT_GE(st.worldPts.size(), 6u);
  EXPECT_TRUE(SameBits(st.worldPose, c8::HMatrixGen::i()));
  // The reset keeps the scale (D:109589-109607 do not write the scale at +0): the unusable solve leaves the
  // scale of the solved frames before the loss.
  EXPECT_EQ(st.scale, solved_scale);
  // Observable output: the next lost frame publishes that identity world pose.
  on.Track(f++, lost.get(), targets);
  ASSERT_EQ(on.tracker->located_images().size(), 1u);
  EXPECT_EQ(on.tracker->located_images()[0].targetSpace.lastSeen, 1);
  EXPECT_TRUE(SameBits(on.tracker->located_images()[0].targetSpace.worldPose, c8::HMatrixGen::i()));
}

// The reset needs lastSeen > 100 (D:109588 `lastSeen < 101` skips it): an unusable solve on a tracked frame,
// after a short loss, or after exactly 100 untracked frames keeps the previous world pose.
TEST(ImageWorldTrackerVio, NoResetWhileTrackedOrAfterAShortLoss) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const auto orbit = Orbit(images.at("t0"), 0.72f, 6);
  const auto lost = LostFrame(0.72f);
  const auto bad = UnusableSolveFrame(images.at("t0"), TargetWorld(), 0.72f);
  for (int loss : {0, 3, 100}) {
    Rig on;
    ImageWorldTrackerOn enable;
    int f = 0;
    for (int i = 0; i < 6; ++i) on.Track(f++, orbit[5 - i].get(), targets);
    const c8::HMatrix before = on.image_tracker().worldTracker("t0").worldPose;
    for (int i = 0; i < loss; ++i) on.Track(f++, lost.get(), targets);
    on.Track(f++, bad.get(), targets);
    const c8::ImageWorldTrackerState& st = on.image_tracker().worldTracker("t0");
    ASSERT_GE(st.worldPts.size(), 6u) << loss;
    EXPECT_TRUE(SameBits(st.worldPose, before)) << loss;
    EXPECT_LT(MaxAbsDiff(st.worldPose, TargetWorld()), 1e-3f) << loss;
  }
}

// The VioTracker lookup passes the map point source through: GROUND points are never paired.
TEST(ImageWorldTrackerVio, GroundMapPointsAreNotPaired) {
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const auto orbit = Orbit(images.at("t0"), 0.72f, 3);
  Rig on;
  on.modules->source = map::PointSource::kGround;
  ImageWorldTrackerOn enable;
  for (int i = 0; i < 3; ++i) on.Track(i, orbit[i].get(), targets);
  ASSERT_EQ(on.tracker->located_images().size(), 1u);
  const c8::ImageWorldTrackerState& st = on.image_tracker().worldTracker("t0");
  EXPECT_FALSE(st.visible.empty());
  EXPECT_TRUE(st.worldPts.empty());
  EXPECT_EQ(st.scale, on.tracker->located_images()[0].targetSpace.scale);  // fallback: scale = guess
}

}  // namespace
}  // namespace slam_refact
