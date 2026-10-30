// Contract change US015-CC-1: the CPU planar ROI draw (frontend::DrawRoisIntoAtlas, kSwiftShaderApprox) against the ORIGINAL's GPU
// readback. Fixture tests/data/frontend/roi-draw/seq-a-f10-f31.bin.gz (gen_roi_draw_golden.py): per case the `mvp` uniforms of the
// IMAGE_TARGET and HIRES_SCAN slot draws and, per window of a slot, the original's R bytes and the camera pixels it samples. The
// cases cover a quad that leaves the slot on three sides and ends inside it on the fourth (slanted left and right edges, where the
// clipped polygon, not the unclipped vertices, sets the span) and the 1/16 sub-pixel texUv of a HIRES_SCAN interior.
// Also: the general-vertex quad equals the identity quad for mvp = identity, and quads outside the modelled domain report it.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/pixels/image-roi.h"
#include "frontend/frontend.h"
#include "frontend/gr8_cpu_shader.h"
#include "frontend/roi_draw.h"
#include "gtest/gtest.h"
#include "test_util.h"

namespace slam_refact::frontend {
namespace {

using testing::Json;

constexpr char kMagic[] = "SRROI1\n";

c8::HMatrix FromColumnMajor(const float* d) {  // c8::HMatrix::data() order
  return c8::HMatrix{{d[0], d[4], d[8], d[12]}, {d[1], d[5], d[9], d[13]}, {d[2], d[6], d[10], d[14]}, {d[3], d[7], d[11], d[15]}};
}

struct Fixture {
  std::vector<uint8_t> bytes;
  Json head;
  size_t blob_start = 0;
};

bool LoadFixture(Fixture* f) {
  if (!testing::ReadGzipFile(testing::TestDataDir() + "/frontend/roi-draw/seq-a-f10-f31.bin.gz", &f->bytes)) return false;
  const size_t magic = sizeof(kMagic) - 1;
  if (f->bytes.size() < magic || std::memcmp(f->bytes.data(), kMagic, magic) != 0) return false;
  size_t eol = magic;
  while (eol < f->bytes.size() && f->bytes[eol] != '\n') ++eol;
  std::string error;
  if (!Json::Parse(std::string(f->bytes.begin() + static_cast<long>(magic), f->bytes.begin() + static_cast<long>(eol)), &f->head, &error)) return false;
  f->blob_start = eol + 1;
  return true;
}

TEST(RoiDrawGolden, PlanarSlotsMatchTheOriginalsGpuReadback) {
  Fixture fx;
  ASSERT_TRUE(LoadFixture(&fx));
  const int cam_w = static_cast<int>(fx.head["camera"][0].number());
  const int cam_h = static_cast<int>(fx.head["camera"][1].number());
  const PyramidLayout layout = ComputePyramidLayout(cam_w, cam_h);
  size_t pos = fx.blob_start;
  long compared = 0;
  for (size_t ci = 0; ci < fx.head["cases"].size(); ++ci) {
    const Json& cs = fx.head["cases"][ci];
    SCOPED_TRACE("frame " + std::to_string(static_cast<int>(cs["frame"].number())));
    // The camera image: zero except the pasted crops (everything a window samples lies inside its crop).
    std::vector<uint8_t> gray(static_cast<size_t>(cam_w) * cam_h, 0);
    struct Window {
      int slot, x, y, w, h;
      size_t expected;
    };
    std::vector<Window> windows;
    for (size_t wi = 0; wi < cs["windows"].size(); ++wi) {
      const Json& jw = cs["windows"][wi];
      const int cx = static_cast<int>(jw["crop"][0].number()), cy = static_cast<int>(jw["crop"][1].number());
      const int cw = static_cast<int>(jw["crop"][2].number()), ch = static_cast<int>(jw["crop"][3].number());
      ASSERT_LE(pos + static_cast<size_t>(cw) * ch, fx.bytes.size());
      for (int y = 0; y < ch; ++y) {
        std::memcpy(&gray[static_cast<size_t>(cy + y) * cam_w + cx], &fx.bytes[pos + static_cast<size_t>(y) * cw], static_cast<size_t>(cw));
      }
      pos += static_cast<size_t>(cw) * ch;
      Window w{static_cast<int>(jw["slot"].number()), static_cast<int>(jw["x"].number()), static_cast<int>(jw["y"].number()),
               static_cast<int>(jw["w"].number()), static_cast<int>(jw["h"].number()), pos};
      pos += static_cast<size_t>(w.w) * w.h;
      ASSERT_LE(pos, fx.bytes.size());
      windows.push_back(w);
    }
    std::vector<uint8_t> rgba(gray.size() * 4);
    for (size_t i = 0; i < gray.size(); ++i) {
      rgba[4 * i] = rgba[4 * i + 1] = rgba[4 * i + 2] = gray[i];
      rgba[4 * i + 3] = 255;
    }
    const RgbaImageView camera{rgba.data(), cam_w, cam_h, cam_w * 4};
    std::vector<c8::ImageRoi> rois;
    for (size_t ri = 0; ri < cs["rois"].size(); ++ri) {
      float m[16];
      for (int k = 0; k < 16; ++k) m[k] = static_cast<float>(cs["rois"][ri]["mvp"][static_cast<size_t>(k)].number());
      c8::ImageRoi roi;
      roi.source = static_cast<int>(cs["rois"][ri]["source"].number()) == 3 ? c8::ImageRoi::Source::HIRES_SCAN : c8::ImageRoi::Source::IMAGE_TARGET;
      roi.warp = FromColumnMajor(m);
      rois.push_back(roi);
    }
    const CpuGr8Pyramid cpu(layout, ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox);
    RgbaAtlas atlas;
    cpu.Build(camera, 0, &atlas);
    DrawRoisIntoAtlas(layout, cpu, ShaderNumericModel::kSwiftShaderApprox, camera, 0, rois, &atlas);
    for (const Window& w : windows) {
      const LevelLayout& slot = layout.roi_slots[static_cast<size_t>(w.slot)];
      long diff = 0, nonzero = 0;
      for (int y = 0; y < w.h; ++y) {
        for (int x = 0; x < w.w; ++x) {
          const uint8_t got = atlas.rgba[(static_cast<size_t>(slot.r + w.y + y) * atlas.size + (slot.c + w.x + x)) * 4];
          const uint8_t want = fx.bytes[w.expected + static_cast<size_t>(y) * w.w + x];
          diff += got != want;
          nonzero += want != 0;
        }
      }
      EXPECT_EQ(diff, 0) << "slot " << w.slot << " window (" << w.x << "," << w.y << " " << w.w << "x" << w.h << ")";
      EXPECT_GT(nonzero, 0) << "window covers nothing: the case is vacuous";
      compared += static_cast<long>(w.w) * w.h;
    }
  }
  EXPECT_EQ(pos, fx.bytes.size());
  EXPECT_GT(compared, 50000);
}

// mvp = identity: the general-vertex quad is the identity quad of every pyramid stage (bit equality with SwiftShaderQuadTexUv).
TEST(RoiDrawGolden, IdentityMvpEqualsTheIdentityQuad) {
  const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  struct Vp {
    int c, r, w, h, rot;
  };
  for (const Vp& v : {Vp{3, 650, 252, 336, 0}, Vp{258, 650, 252, 336, 90}, Vp{0, 0, 720, 960, 0}, Vp{339, 720, 160, 214, -90}}) {
    const std::vector<float> want = SwiftShaderQuadTexUv(v.c, v.r, v.w, v.h, v.rot, 1024);
    std::vector<float> got;
    std::vector<uint8_t> covered;
    ASSERT_TRUE(SwiftShaderMvpQuadTexUv(v.c, v.r, v.w, v.h, identity, v.rot, 1024, &got, &covered));
    ASSERT_EQ(got.size(), want.size());
    EXPECT_EQ(std::memcmp(got.data(), want.data(), got.size() * sizeof(float)), 0) << v.w << "x" << v.h << " rot " << v.rot;
    for (uint8_t c : covered) ASSERT_EQ(c, 1);
  }
}

// A quad inside the viewport covers only its interior; one outside the modelled domain is reported, not drawn.
TEST(RoiDrawGolden, QuadDomain) {
  std::vector<float> uv;
  std::vector<uint8_t> covered;
  const float half[16] = {0.5f, 0, 0, 0, 0, 0.5f, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  ASSERT_TRUE(SwiftShaderMvpQuadTexUv(0, 0, 64, 64, half, 0, 1024, &uv, &covered));
  long n = 0;
  for (uint8_t c : covered) n += c;
  EXPECT_GT(n, 32 * 32 - 80);
  EXPECT_LT(n, 32 * 32 + 80);
  EXPECT_EQ(covered[0], 0);
  EXPECT_EQ(covered[32 * 64 + 32], 1);
  const float behind[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1};  // w = -1
  EXPECT_FALSE(SwiftShaderMvpQuadTexUv(0, 0, 64, 64, behind, 0, 1024, &uv, &covered));
  const float flat[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // collapsed quad
  EXPECT_FALSE(SwiftShaderMvpQuadTexUv(0, 0, 64, 64, flat, 0, 1024, &uv, &covered));
}

}  // namespace
}  // namespace slam_refact::frontend
