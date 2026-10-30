// US-013 integration unit tests (contract FULL-3, docs/design/image-target.md §7.4):
//   IT-C1  detection config switch (allOrb + allGorb only with targets), ROI detection, ROI slots
//   IT-C2  active target limit 32 and at most 4 located images per frame
//   938 shares the RNG of 926 and runs after it (D:83064, D:84165).
// Checked-in code only (synthetic targets of test_util.h), label unit.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/pixels/gr8-pyramid.h"
#include "config/params.h"
#include "frontend/frontend.h"
#include "frontend/pyramid_source.h"
#include "frontend/roi_draw.h"
#include "image_target/image_roi_list.h"
#include "image_target/image_target_set.h"
#include "image_target/image_target_spec.h"
#include "image_target/target_loader.h"
#include "reality/engine/imagedetection/detection-image-tracker.h"
#include "slam_refact/glue_runner.h"
#include "test_util.h"
#include "vio_tracker/tracker_modules.h"
#include "vio_tracker/vio_tracker.h"
#include "vio_tracker/vio_tracker_internal.h"

namespace slam_refact {
namespace {

using image_target::testing::SyntheticTarget;
using image_target::testing::TestK;
using image_target::testing::TestPinholeK;
using vio_tracker::TrackerModules;

// detection-image-tracker-test.cc image(): a virtual photo of the target points taken at camPos.
c8::FrameWithPoints Photo(const c8::TargetWithPoints& src, float scale, const c8::HMatrix& cam) {
  const c8_PixelPinholeCameraModel k = TestK();
  c8::FrameWithPoints ret{k};
  const c8::HMatrix kk = c8::HMatrixGen::intrinsic(k);
  const c8::HMatrix target_loc = c8::HMatrixGen::translation(0.0f, 0.0f, scale);
  for (size_t i = 0; i < src.points().size(); ++i) {
    const auto& pt = src.points()[i];
    const c8::HPoint3 wpt = target_loc * c8::HPoint3{pt.x() * scale, pt.y() * scale, 0.0f};
    if ((cam.inv() * wpt).z() < 0) continue;
    const c8::HPoint2 im = (kk * cam.inv() * wpt).flatten();
    if (im.x() < 0 || im.y() < 0 || im.x() >= k.pixelsWidth || im.y() >= k.pixelsHeight) continue;
    ret.addImagePixelPoint(im, pt.scale(), pt.angle(), pt.gravityAngle(), src.store().getClone(i));
  }
  return ret;
}

c8::DetectionImage LoadTarget(const std::string& name) {
  static const image_target::GrayImage img = SyntheticTarget(480, 640, 7);
  image_target::ImageTargetSpec spec;
  spec.name = name;
  spec.type = image_target::ImageTargetType::kPlanar;
  return image_target::LoadDetectionImage(spec, img.view(), TestPinholeK(),
                                          ShaderNumericModel::kIeee);
}

// TrackerModules double: Detect returns a prepared photo, TrackMap draws one RNG number.
struct PhotoModules : public TrackerModules {
  const c8::FrameWithPoints* photo = nullptr;
  c8::DetectionConfig last_cfg;
  int roi_calls = 0;
  std::vector<uint32_t> track_map_draws;
  compat::RandomNumbers* track_map_rng = nullptr;

  void Detect(const c8::Gr8Pyramid&, const c8::DetectionConfig& cfg,
              c8::FrameWithPoints* frame) override {
    last_cfg = cfg;
    if (photo != nullptr) frame->clearAndClone(*photo);
  }
  void DetectRois(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig&, const c8::FrameWithPoints&,
                  std::vector<c8::FrameWithPoints>* rois) override {
    ++roi_calls;
    rois->clear();
    for (size_t i = 0; i < pyr.rois.size(); ++i) rois->emplace_back(TestK());
  }
  frontend::DetectStats detect_stats() const override { return {}; }
  void TrackMap(TrackerInput*, const FrameState&, MapTrackerOutput* out,
                compat::RandomNumbers* rng) override {
    track_map_rng = rng;
    track_map_draws.push_back(rng->nextUnsignedInt());
    out->state = 1;
  }
  void UpdateMap(const TrackerInput&, const MapTrackerOutput&, int64_t,
                 MapBuilderOutput* out) override {
    *out = MapBuilderOutput{};
  }
  c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput&) override {
    return c8::HMatrixGen::i();
  }
  void RecreateMapTrackerAndBuilder() override {}
  void VisibleMapPoints(std::vector<vio_tracker::VisibleMapPoint>* out) const override {
    out->clear();
  }
  int ground_match_count() const override { return 0; }
  int num_keyframes() const override { return 0; }
  std::array<int, 5> num_points_by_source() const override { return {0, 0, 0, 0, 0}; }
};

struct Rig {
  Rig() {
    config::RegisterDefaults();
    auto m = std::make_unique<PhotoModules>();
    modules = m.get();
    tracker = vio_tracker::NewVioTrackerWithModules(false, false, std::move(m));
    pyr.levels.push_back(c8::LevelLayout{});
  }
  c8::HMatrix Track(int k, const DetectionImagePtrMap& targets) {
    TrackingSensorFrame frame;
    frame.time_ns = 1000000000LL + k * 33333333LL;
    frame.intrinsic = TestK();
    return tracker->TrackWithTargets(frame, pyr, targets, &rng, &status, &debug);
  }
  PhotoModules* modules = nullptr;
  std::unique_ptr<vio_tracker::VioTracker> tracker;
  c8::Gr8Pyramid pyr;
  compat::RandomNumbers rng;
  vio_tracker::StatusOut status;
  vio_tracker::TrackDebug debug;
};

// IT-C1: tracker.cc:105-109 / D:82467-82468.
TEST(VioImageTarget, DetectionConfigSwitchesToAllOrbAllGorbOnlyWithTargets) {
  Rig rig;
  DetectionImageMap images;
  images.insert(std::make_pair("t0", LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  rig.Track(0, DetectionImagePtrMap{});
  const c8::DetectionConfig none = rig.modules->last_cfg;
  EXPECT_FALSE(none.allOrb);  // world SLAM default: GORB (trackingDescriptorType 2)
  EXPECT_TRUE(none.allGorb);
  EXPECT_EQ(none.nKeypoints, 500);
  rig.Track(1, targets);
  const c8::DetectionConfig with = rig.modules->last_cfg;
  EXPECT_TRUE(with.allOrb);
  EXPECT_TRUE(with.allGorb);
  EXPECT_EQ(with.nKeypoints, 500);
  EXPECT_EQ(with.gravityPortions[c8::DescriptorType::ORB], none.gravityPortions[c8::DescriptorType::ORB]);
  EXPECT_EQ(rig.modules->roi_calls, 0);  // no pyramid ROI, no "gr8gl-rois"
}

TEST(VioImageTarget, PyramidRoisRunRoiDetection) {
  Rig rig;
  rig.pyr.rois.resize(4);
  rig.Track(0, DetectionImagePtrMap{});
  EXPECT_EQ(rig.modules->roi_calls, 1);
}

// IT-C2 (per frame) and the shared RNG: five identical targets in one photo; global search finds one
// new target per frame (round robin), the located list is capped at 4.
TEST(VioImageTarget, AtMostFourLocatedImagesAndRngSharedWithMapTracker) {
  DetectionImageMap images;
  for (int i = 0; i < 5; ++i) {
    const std::string n = "t" + std::to_string(i);
    images.insert(std::make_pair(n, LoadTarget(n)));
  }
  DetectionImagePtrMap targets;
  for (auto& e : images) targets.insert(std::make_pair(e.first, &e.second));
  const c8::FrameWithPoints photo = Photo(images.at("t0").framePoints(), 1.0f, c8::HMatrixGen::i());
  ASSERT_GT(photo.points().size(), 100u);

  Rig with;
  Rig without;
  with.modules->photo = &photo;
  without.modules->photo = &photo;
  size_t max_located = 0;
  for (int k = 0; k < 8; ++k) {
    const c8::HMatrix pose = with.Track(k, targets);
    without.Track(k, DetectionImagePtrMap{});
    EXPECT_EQ(with.modules->track_map_rng, &with.rng);
    const auto& located = with.tracker->located_images();
    EXPECT_LE(located.size(), 4u) << "frame " << k;
    max_located = std::max(max_located, located.size());
    // applyToLocatedImages: responsive output = Y * tam, so located = Y * raw located.
    (void)pose;
  }
  EXPECT_EQ(max_located, 4u);
  EXPECT_TRUE(without.tracker->located_images().empty());
  // Frame 0: both MapTrackers draw from a fresh generator. From frame 1 on the RNG of the tracker
  // with targets has also been consumed by 938 (robustPnP of the global search) after 926.
  ASSERT_EQ(with.modules->track_map_draws.size(), 8u);
  EXPECT_EQ(with.modules->track_map_draws[0], without.modules->track_map_draws[0]);
  EXPECT_NE(with.modules->track_map_draws[1], without.modules->track_map_draws[1]);
}

TEST(VioImageTarget, ResetImageTrackingClearsLocatedImages) {
  DetectionImageMap images;
  images.insert(std::make_pair("t0", LoadTarget("t0")));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  const c8::FrameWithPoints photo = Photo(images.at("t0").framePoints(), 1.0f, c8::HMatrixGen::i());
  Rig rig;
  rig.modules->photo = &photo;
  rig.Track(0, targets);
  ASSERT_EQ(rig.tracker->located_images().size(), 1u);
  EXPECT_EQ(rig.tracker->located_images()[0].targetSpace.lastSeen, 0);
  rig.tracker->ResetImageTracking();
  EXPECT_TRUE(rig.tracker->located_images().empty());
}

// FULL-7 (RM-4b): the producer of an injected atlas draws the ROI slots; the engine uses them as
// drawn and attaches the same slot layouts / ROIs as on the CPU path.
TEST(VioImageTarget, InjectedAtlasWithRoiSlotsIsUsedAsDrawn) {
  const image_target::GrayImage cam = SyntheticTarget(480, 640, 3);
  frontend::PyramidSource producer(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  ImageRoiList rois;
  image_target::AddNextDrawHiResScans(PinholeIntrinsics{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f},
                                      &rois);
  ASSERT_EQ(rois.rois.size(), 4u);
  const c8::Gr8Pyramid cpu = producer.GetWithRois(PyramidAtlasView{}, RgbaImageView{}, cam.view(), rois.rois);
  const frontend::RgbaAtlas& atlas = producer.last_cpu_atlas();
  ASSERT_EQ(atlas.rows, frontend::RoiAtlasRows(producer.layout()));

  frontend::PyramidSource consumer(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  const PyramidAtlasView view{atlas.rgba.data(), atlas.size, atlas.rows};
  const c8::Gr8Pyramid inj = consumer.GetWithRois(view, RgbaImageView{}, GrayImageView{}, rois.rois);
  // Pixels as given (no copy, no redraw) and the same metadata as the CPU path.
  EXPECT_EQ(inj.data.pixels(), atlas.rgba.data());
  EXPECT_EQ(inj.data.rows(), atlas.rows);
  ASSERT_EQ(inj.rois.size(), cpu.rois.size());
  ASSERT_EQ(inj.levels.size(), cpu.levels.size());
  for (size_t i = 0; i < inj.rois.size(); ++i) {
    EXPECT_EQ(inj.rois[i].layout.r, cpu.rois[i].layout.r);
    EXPECT_EQ(inj.rois[i].layout.c, cpu.rois[i].layout.c);
    EXPECT_EQ(inj.rois[i].layout.w, cpu.rois[i].layout.w);
    EXPECT_EQ(inj.rois[i].layout.h, cpu.rois[i].layout.h);
    EXPECT_EQ(inj.rois[i].layout.rotated, cpu.rois[i].layout.rotated);
    EXPECT_EQ(inj.rois[i].roi.source, cpu.rois[i].roi.source);
    EXPECT_EQ(inj.rois[i].roi.faceId, cpu.rois[i].roi.faceId);
    EXPECT_EQ(inj.rois[i].roi.warp.data(), cpu.rois[i].roi.warp.data());
  }
  // The consumer never drew anything itself.
  EXPECT_TRUE(consumer.last_cpu_atlas().rgba.empty());
  // Without ROIs the injected path is the plain H7 injection (no ROI list).
  const c8::Gr8Pyramid plain = consumer.GetWithRois(view, RgbaImageView{}, GrayImageView{}, {});
  EXPECT_TRUE(plain.rois.empty());
  EXPECT_EQ(plain.data.pixels(), atlas.rgba.data());
}

TEST(VioImageTargetDeathTest, InjectedAtlasWithoutTheRoiRowsIsRejected) {
  frontend::PyramidSource source(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  ImageRoiList rois;
  image_target::AddNextDrawHiResScans(PinholeIntrinsics{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f},
                                      &rois);
  const frontend::PyramidLayout& layout = source.layout();
  ASSERT_LT(layout.rows, frontend::RoiAtlasRows(layout));
  const std::vector<uint8_t> atlas(static_cast<size_t>(layout.texture_size) * layout.rows * 4, 0);
  const PyramidAtlasView short_view{atlas.data(), layout.texture_size, layout.rows};
  EXPECT_DEATH(source.GetWithRois(short_view, RgbaImageView{}, GrayImageView{}, rois.rois),
               "ROI slots");
  // The same atlas is fine while no ROI is requested (world SLAM only: rows >= layout.rows).
  EXPECT_TRUE(source.GetWithRois(short_view, RgbaImageView{}, GrayImageView{}, {}).rois.empty());
}

// IT-C1 pyramid side: HIRES scans fill the 4 slots; without ROIs the pyramid is unchanged.
TEST(VioImageTarget, RoiSlotsAreDrawnAndAttachedToThePyramid) {
  const image_target::GrayImage cam = SyntheticTarget(480, 640, 3);
  frontend::PyramidSource plain(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  frontend::PyramidSource with(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  const c8::Gr8Pyramid p0 = plain.Get(PyramidAtlasView{}, RgbaImageView{}, cam.view());
  const std::vector<uint8_t> atlas0 = plain.last_cpu_atlas().rgba;
  const int rows0 = plain.last_cpu_atlas().rows;
  EXPECT_TRUE(p0.rois.empty());
  const c8::Gr8Pyramid p_empty = with.GetWithRois(PyramidAtlasView{}, RgbaImageView{}, cam.view(), {});
  EXPECT_TRUE(p_empty.rois.empty());
  EXPECT_EQ(with.last_cpu_atlas().rgba, atlas0);  // no ROI: same bytes as Get

  ImageRoiList rois;
  image_target::AddNextDrawHiResScans(PinholeIntrinsics{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f},
                                      &rois);
  ASSERT_EQ(rois.rois.size(), 4u);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(rois.rois[i].source, c8::ImageRoi::Source::HIRES_SCAN);
    EXPECT_EQ(rois.rois[i].faceId, -static_cast<int>(i) - 1);
  }
  const c8::Gr8Pyramid p = with.GetWithRois(PyramidAtlasView{}, RgbaImageView{}, cam.view(), rois.rois);
  ASSERT_EQ(p.rois.size(), 4u);
  const frontend::PyramidLayout& layout = with.layout();
  const frontend::RgbaAtlas& atlas = with.last_cpu_atlas();
  EXPECT_EQ(atlas.rows, frontend::RoiAtlasRows(layout));
  EXPECT_GT(atlas.rows, rows0);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(p.rois[i].layout.r, layout.roi_slots[i].r);
    EXPECT_EQ(p.rois[i].layout.c, layout.roi_slots[i].c);
    // Each slot holds camera content (R > 0 somewhere) and stage-3 output (B score somewhere).
    int lit = 0, scored = 0;
    const frontend::LevelLayout& s = layout.roi_slots[i];
    for (int y = s.r; y < s.r + s.h; ++y) {
      for (int x = s.c; x < s.c + s.w; ++x) {
        const uint8_t* px = &atlas.rgba[(static_cast<size_t>(y) * atlas.size + x) * 4];
        lit += px[0] > 0;
        scored += px[2] > 0;
      }
    }
    EXPECT_GT(lit, s.w * s.h / 2) << "slot " << i;
    EXPECT_GT(scored, 0) << "slot " << i;
  }
  // Level 0 R (camera content away from the ROI rows) is unchanged by the ROI draw.
  const frontend::LevelLayout& l0 = layout.levels[0];
  for (int y = l0.r; y < l0.r + l0.h; y += 7) {
    for (int x = l0.c; x < l0.c + l0.w; x += 5) {
      const size_t at = (static_cast<size_t>(y) * atlas.size + x) * 4;
      ASSERT_EQ(atlas.rgba[at], atlas0[at]);
    }
  }
}

std::string TargetJson(const std::string& name) {
  return "{\"type\":\"PLANAR\",\"properties\":{\"left\":0,\"top\":0,\"width\":120,\"height\":160,"
         "\"isRotated\":false,\"originalWidth\":120,\"originalHeight\":160},\"imagePath\":\"" +
         name + ".png\",\"name\":\"" + name + "\"}";
}

// Review m3: the original keeps the loaded image when a target of the same name is loaded again
// (c8EmAsm_processNewDetectionImageTexture: detectionImages_.insert, tracking-controller.cc:483).
TEST(ImageTargetSetTest, SameNameWithAnotherImagePathKeepsTheLoadedImage) {
  const image_target::GrayImage lum1 = SyntheticTarget(120, 160, 5);
  const image_target::GrayImage lum2 = SyntheticTarget(120, 160, 99);
  const auto json = [](const std::string& name, const std::string& path) {
    return "{\"type\":\"PLANAR\",\"properties\":{\"left\":0,\"top\":0,\"width\":120,\"height\":160,"
           "\"isRotated\":false,\"originalWidth\":120,\"originalHeight\":160},\"imagePath\":\"" +
           path + "\",\"name\":\"" + name + "\"}";
  };
  ImageTargetSet set;
  const PinholeIntrinsics k = TestPinholeK();
  ASSERT_EQ(set.Set({{json("a", "one.png"), lum1.view()}}, k, ShaderNumericModel::kIeee), "");
  const c8::DetectionImage* first = &set.at("a");
  const size_t points_one = first->framePoints().points().size();
  ASSERT_GT(points_one, 100u);
  // A different image under the same name would have another keypoint set (control).
  image_target::ImageTargetSpec spec;
  spec.name = "a";
  spec.type = image_target::ImageTargetType::kPlanar;
  const size_t points_two = image_target::LoadDetectionImage(spec, lum2.view(), k, ShaderNumericModel::kIeee)
                                .framePoints().points().size();
  ASSERT_NE(points_one, points_two);
  // Same name, other imagePath, other pixels: nothing is replaced (no new object, same keypoints).
  EXPECT_EQ(set.Set({{json("a", "two.png"), lum2.view()}}, k, ShaderNumericModel::kIeee), "");
  EXPECT_EQ(&set.at("a"), first);
  EXPECT_EQ(set.at("a").framePoints().points().size(), points_one);
  // Setting the first path again changes nothing either; a new name is added next to it.
  EXPECT_EQ(set.Set({{json("a", "one.png"), lum1.view()}, {json("b", "b.png"), lum2.view()}}, k,
                    ShaderNumericModel::kIeee), "");
  EXPECT_EQ(&set.at("a"), first);
  EXPECT_EQ(set.names(), (std::vector<std::string>{"a", "b"}));
  // A name that is no longer configured is unloaded and loads fresh when it comes back.
  EXPECT_EQ(set.Set({{json("b", "b.png"), lum2.view()}}, k, ShaderNumericModel::kIeee), "");
  EXPECT_EQ(set.names(), (std::vector<std::string>{"b"}));
  EXPECT_EQ(set.Set({{json("a", "two.png"), lum2.view()}}, k, ShaderNumericModel::kIeee), "");
  EXPECT_EQ(set.at("a").framePoints().points().size(), points_two);
}

EngineConfig PortraitConfig() {
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  return e;
}

// IT-C2: kMaxActiveImageTargets; errors leave the set unchanged.
TEST(GlueImageTargets, ActiveLimitDuplicatesCurvyAndUnload) {
  const image_target::GrayImage lum = SyntheticTarget(120, 160, 5);
  GlueRunner glue(PortraitConfig(), GlueConfig{});
  std::vector<ImageTargetInput> in;
  for (int i = 0; i < 33; ++i) in.push_back({TargetJson("t" + std::to_string(i)), lum.view()});
  EXPECT_NE(glue.SetImageTargets(in), "");
  EXPECT_TRUE(glue.image_target_names().empty());
  in.pop_back();
  EXPECT_EQ(glue.SetImageTargets(in), "");
  EXPECT_EQ(glue.image_target_names().size(), 32u);
  std::vector<ImageTargetInput> dup = {{TargetJson("a"), lum.view()}, {TargetJson("a"), lum.view()}};
  EXPECT_NE(glue.SetImageTargets(dup), "");
  EXPECT_EQ(glue.image_target_names().size(), 32u);
  // A CYLINDER target without its cylinder* properties is a JSON error (valid curvy targets load since US-014,
  // image_target_curvy_synth_test.cc).
  std::string curvy = TargetJson("c");
  curvy.replace(curvy.find("PLANAR"), 6, "CYLINDER");
  EXPECT_NE(glue.SetImageTargets({{curvy, lum.view()}}), "");
  EXPECT_EQ(glue.image_target_names().size(), 32u);
  EXPECT_NE(glue.SetImageTargets({{TargetJson("x"), GrayImageView{}}}), "");
  EXPECT_EQ(glue.SetImageTargets({{TargetJson("t3"), lum.view()}, {TargetJson("t7"), lum.view()}}), "");
  EXPECT_EQ(glue.image_target_names(), (std::vector<std::string>{"t3", "t7"}));
  glue.UnloadImageTargets({"t3", "missing"});
  EXPECT_EQ(glue.image_target_names(), (std::vector<std::string>{"t7"}));
  glue.ClearImageTargets();
  EXPECT_TRUE(glue.image_target_names().empty());
}

}  // namespace
}  // namespace slam_refact
