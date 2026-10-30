// US-015 review F-1: the default-off output of the patched DetectionImageTracker on a sequence where the
// target IS detected, tracked, lost (10 and 105 frames) and found again. Every LocatedImage field and the
// TrackedImage state are printed as float bits. Built twice from this one source: against the patched
// vendored headers (target sr_iwt_offpath_dump, no world tracker installed = the engine default) and against
// the unpatched public files (manual build, slam_refact/tools/re/iwt_oracle/offpath_ab.py); the two outputs
// must be byte-identical. Development tool, not a test; dev synthetic data only.
//
//   sr_iwt_offpath_dump > out.txt

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "c8/hmatrix.h"
#include "compat/random_numbers.h"
#include "image_target/image_target_spec.h"
#include "image_target/target_loader.h"
#include "reality/engine/imagedetection/detection-image-tracker.h"
#include "test_util.h"

namespace {

using slam_refact::image_target::testing::SyntheticTarget;
using slam_refact::image_target::testing::TestK;
using slam_refact::image_target::testing::TestPinholeK;

uint32_t Bits(float f) {
  uint32_t u;
  std::memcpy(&u, &f, 4);
  return u;
}

void PrintH(const char* tag, const c8::HMatrix& h) {
  std::printf(" %s", tag);
  for (float v : h.data()) std::printf(" %08" PRIx32, Bits(v));
  for (float v : h.inverseData()) std::printf(" %08" PRIx32, Bits(v));
}

// A virtual photo of the target points (plane z = 1) from `cam` (detection-image-tracker-test.cc image()).
c8::FrameWithPoints Photo(const c8::TargetWithPoints& src, const c8::HMatrix& cam) {
  const c8_PixelPinholeCameraModel k = TestK();
  c8::FrameWithPoints ret{k};
  const c8::HMatrix km = c8::HMatrixGen::intrinsic(k);
  for (size_t i = 0; i < src.points().size(); ++i) {
    const auto& pt = src.points()[i];
    const c8::HPoint3 wpt{pt.x(), pt.y(), 1.0f};
    if ((cam.inv() * wpt).z() < 0) continue;
    const c8::HPoint2 im = (km * cam.inv() * wpt).flatten();
    if (im.x() < 0 || im.y() < 0 || im.x() >= k.pixelsWidth || im.y() >= k.pixelsHeight) continue;
    ret.addImagePixelPoint(im, pt.scale(), pt.angle(), pt.gravityAngle(), src.store().getClone(i));
  }
  return ret;
}

}  // namespace

int main() {
  const slam_refact::image_target::GrayImage img = SyntheticTarget(480, 640, 7);
  slam_refact::image_target::ImageTargetSpec spec;
  spec.name = "t0";
  spec.type = slam_refact::image_target::ImageTargetType::kPlanar;
  DetectionImageMap images;
  images.insert(std::make_pair(std::string("t0"), slam_refact::image_target::LoadDetectionImage(
                                                      spec, img.view(), TestPinholeK(),
                                                      slam_refact::ShaderNumericModel::kIeee)));
  DetectionImagePtrMap targets{{"t0", &images.at("t0")}};
  // k = 4: the world camera moves 4-7 cm per tracked frame, above the public 3e-2 baseline threshold, so
  // the public scale estimator updates (image-tracking-state.cc computeScale).
  const float k = 4.0f;
  const c8::HMatrix g = c8::HMatrixGen::translation(0.3f, 1.1f, -0.4f) * c8::HMatrixGen::yDegrees(35.0f);
  c8::DetectionImageTracker tracker;
  slam_refact::compat::RandomNumbers rng;
  // Phases: 40 tracked frames, 10 lost, 20 tracked, 105 lost (lastSeen passes 100), 15 tracked.
  const int phases[][2] = {{40, 1}, {10, 0}, {20, 1}, {105, 0}, {15, 1}};
  int frame = 0;
  for (const auto& ph : phases) {
    for (int i = 0; i < ph[0]; ++i, ++frame) {
      const float t = 0.07f * static_cast<float>(frame % 40) - 1.4f;
      const c8::HMatrix cam = c8::HMatrixGen::translation(0.1f * t, -0.04f * t, -0.1f + 0.06f * t * t) *
                              c8::HMatrixGen::yDegrees(4.0f * t) * c8::HMatrixGen::xDegrees(2.0f * t);
      const c8::HMatrix tam = g * c8::scaleTranslation(k, cam);
      const c8::FrameWithPoints feats =
          ph[1] != 0 ? Photo(images.at("t0").framePoints(), cam) : c8::FrameWithPoints(TestK());
      const std::vector<c8::LocatedImage> located = tracker.locate(feats, {}, tam, targets, &rng);
      std::printf("frame %d feats %zu located %zu\n", frame, feats.points().size(), located.size());
      for (const c8::LocatedImage& l : located) {
        const c8::TrackedImage& ti = l.targetSpace;
        std::printf(" li %s status %d lastSeen %d firstSeen %d w %08" PRIx32 " h %08" PRIx32
                    " scale %08" PRIx32,
                    ti.name.c_str(), static_cast<int>(ti.status), ti.lastSeen, ti.firstSeen,
                    Bits(l.width), Bits(l.height), Bits(ti.scale));
        PrintH("pose", l.pose);
        PrintH("tpose", ti.pose);
        PrintH("wpose", ti.worldPose);
        std::printf("\n");
      }
    }
  }
  return 0;
}
