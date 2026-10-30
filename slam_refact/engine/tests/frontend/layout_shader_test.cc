// WP1 gates F-LAY, F-FAST, F-H9 (unit part) and frontend config constants
// (architecture.md §6.6.1; S1 §2.2, §4, §5.2, §8.3 H9).

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "config/settings.h"
#include "frontend/frontend.h"
#include "frontend/gr8_cpu_shader.h"
#include "frontend/intrinsics.h"
#include "gtest/gtest.h"
#include "reality/engine/features/gr8gl-data.h"
#include "test_util.h"

namespace slam_refact::frontend {
namespace {

using testing::Json;

// ---- F-LAY: ComputePyramidLayout vs layouts.jsonl (6 camera sizes) ----

TEST(FrontendLayout, MatchesSpecLayouts) {
  std::string text;
  ASSERT_TRUE(testing::ReadTextFile(testing::TestDataDir() + "/frontend/layouts.jsonl", &text));
  std::vector<std::string> lines;
  size_t start = 0;
  while (start < text.size()) {
    const size_t end = text.find('\n', start);
    const std::string line = text.substr(start, end == std::string::npos ? std::string::npos
                                                                          : end - start);
    if (!line.empty()) {
      lines.push_back(line);
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  ASSERT_EQ(lines.size(), 6u);
  for (const std::string& line : lines) {
    Json j;
    std::string error;
    ASSERT_TRUE(Json::Parse(line, &j, &error)) << error;
    const int w = static_cast<int>(j["src"][0].number());
    const int h = static_cast<int>(j["src"][1].number());
    SCOPED_TRACE(std::to_string(w) + "x" + std::to_string(h));
    const PyramidLayout layout = ComputePyramidLayout(w, h);
    EXPECT_EQ(layout.texture_size, static_cast<int>(j["tex"][0].number()));
    EXPECT_EQ(layout.texture_size, static_cast<int>(j["tex"][1].number()));
    EXPECT_EQ(layout.filtered_w, static_cast<int>(j["filtered"][0].number()));
    EXPECT_EQ(layout.filtered_h, static_cast<int>(j["filtered"][1].number()));
    EXPECT_EQ(layout.rotate_input, w > h);
    ASSERT_EQ(j["levels"].size(), 4u);
    for (int i = 0; i < 4; ++i) {
      const Json& l = j["levels"][i];
      EXPECT_EQ(layout.levels[i].r, static_cast<int>(l["r"].number())) << "level " << i;
      EXPECT_EQ(layout.levels[i].c, static_cast<int>(l["c"].number())) << "level " << i;
      EXPECT_EQ(layout.levels[i].w, static_cast<int>(l["w"].number())) << "level " << i;
      EXPECT_EQ(layout.levels[i].h, static_cast<int>(l["h"].number())) << "level " << i;
      EXPECT_EQ(layout.levels[i].rotated, l["rotated"].number() != 0) << "level " << i;
    }
    ASSERT_EQ(j["rois"].size(), 4u);
    for (int i = 0; i < 4; ++i) {
      const Json& l = j["rois"][i];
      EXPECT_EQ(layout.roi_slots[i].r, static_cast<int>(l["r"].number())) << "roi " << i;
      EXPECT_EQ(layout.roi_slots[i].c, static_cast<int>(l["c"].number())) << "roi " << i;
      EXPECT_EQ(layout.roi_slots[i].w, static_cast<int>(l["w"].number())) << "roi " << i;
      EXPECT_EQ(layout.roi_slots[i].h, static_cast<int>(l["h"].number())) << "roi " << i;
    }
    // Row-count DECISION: levels plus the gutter below the lowest level.
    int bottom = 0;
    for (const LevelLayout& l : layout.levels) {
      bottom = std::max(bottom, l.r + l.h);
    }
    EXPECT_EQ(layout.rows, bottom + kAtlasGutter);
  }
}

TEST(FrontendLayout, StandardPortraitRowsCoverLevels) {
  const PyramidLayout layout = ComputePyramidLayout(480, 640);
  EXPECT_EQ(layout.rows, 650);  // L2 bottom 339 + 308 = 647, + gutter 3
  EXPECT_EQ(kPyramidSizeDefault, 1024);
}

// ---- F-FAST: shader scoreGr8 == textbook FAST-9 on 2,000,000 inputs (S1 §5.2) ----

float Fast9Textbook(const float d[16]) {
  float best = -1e30f;
  for (int sign = 0; sign < 2; ++sign) {
    for (int k = 0; k < 16; ++k) {
      float m = 1e30f;
      for (int j = 0; j < 9; ++j) {
        const float v = d[(k + j) % 16];
        m = std::min(m, sign ? -v : v);
      }
      best = std::max(best, m);
    }
  }
  return best;
}

TEST(FrontendFast, ShaderScoreEqualsTextbookFast9) {
  // Seed 20261015 as the spec lane (S1 §8.1 V-syn-FAST); bytes from the raw mt19937 output so the
  // stream does not depend on a standard-library distribution. One third of the trials carry a
  // bright or dark contiguous arc of 6..13 points (corner-like inputs).
  std::mt19937 rng(20261015u);
  const auto byte = [&rng]() { return static_cast<int>(rng() >> 24); };
  constexpr long kTrials = 2000000;
  long mismatches = 0;
  for (long t = 0; t < kTrials; ++t) {
    float d[16];
    const float c = byte() / 255.0f;
    for (int i = 0; i < 16; ++i) {
      d[i] = byte() / 255.0f - c;
    }
    if (t % 3 == 0) {
      const int start = byte() % 16;
      const int len = 6 + byte() % 8;
      const float s = (byte() & 1) ? 1.0f : -1.0f;
      for (int i = 0; i < len; ++i) {
        d[(start + i) % 16] = s * (byte() + 1) / 255.0f;
      }
    }
    if (ShaderFast9Score(d) != Fast9Textbook(d)) {
      ++mismatches;
    }
  }
  EXPECT_EQ(mismatches, 0) << "of " << kTrials;
}

// ---- F-U1 (unit): texUv varying model vs SwiftShader readback spot values ----

// Bits read back by tools/re/spec_frontend/varying_probe.mjs (Chrome 144.0.7559.96, ANGLE Vulkan
// on SwiftShader Subzero; build/slam-refact-20261016/pyramid/varying-probe). Both quad triangles,
// the diagonal, the corners and a rotated viewport are covered; the full readback is the F-U1
// evidence gate.
TEST(FrontendVarying, SwiftShaderQuadTexUvSpotValues) {
  struct Spot {
    int c, r, w, h, rotation, x, y;
    uint32_t u, v;
  };
  constexpr Spot kSpots[] = {
      {0, 0, 720, 960, 0, 0, 0, 0x3A360B61u, 0x3A088889u},
      {0, 0, 720, 960, 0, 719, 959, 0x3F7FD27Eu, 0x3F7FDDDFu},
      {0, 0, 720, 960, 0, 360, 480, 0x3F002D83u, 0x3F002223u},
      {0, 0, 720, 960, 0, 100, 800, 0x3E0EEEEFu, 0x3F557778u},
      {0, 0, 720, 960, 0, 600, 100, 0x3F5582D9u, 0x3DD66667u},
      {0, 0, 720, 960, 0, 359, 479, 0x3EFFA4FBu, 0x3EFFBBBDu},
      {486, 3, 444, 333, -90, 486, 3, 0x3F7F9D99u, 0x3A939A86u},
      {486, 3, 444, 333, -90, 929, 335, 0x3AC4D000u, 0x3F7FB633u},
      {486, 3, 444, 333, -90, 708, 169, 0x3F000000u, 0x3F0049CDu},
      {486, 3, 444, 333, -90, 496, 303, 0x3DC7E148u, 0x3CC1BAD0u},
      {486, 3, 444, 333, -90, 886, 23, 0x3F703D80u, 0x3F66EB3Fu},
      {720, 339, 160, 214, 0, 720, 339, 0x3B4CCCCDu, 0x3B191F1Bu},
      {720, 339, 160, 214, 0, 879, 552, 0x3F7F3333u, 0x3F7F66E2u},
      {720, 339, 160, 214, 0, 800, 446, 0x3F00CCCDu, 0x3F009920u},
      {720, 339, 160, 214, 0, 725, 539, 0x3D0CCCCDu, 0x3F6FD9B9u},
      {720, 339, 160, 214, 0, 870, 342, 0x3F70CCCDu, 0x3C85FB38u},
  };
  for (const Spot& s : kSpots) {
    const std::vector<float> uv =
        SwiftShaderQuadTexUv(s.c, s.r, s.w, s.h, s.rotation, kPyramidSizeDefault);
    const size_t i = (static_cast<size_t>(s.y - s.r) * s.w + (s.x - s.c)) * 2;
    uint32_t u = 0, v = 0;
    std::memcpy(&u, &uv[i], 4);
    std::memcpy(&v, &uv[i + 1], 4);
    EXPECT_EQ(u, s.u) << "viewport " << s.w << "x" << s.h << " pixel " << s.x << "," << s.y;
    EXPECT_EQ(v, s.v) << "viewport " << s.w << "x" << s.h << " pixel " << s.x << "," << s.y;
  }
}

// ---- F-H9 (unit): constant tables vs the binary (binary-tables.json, S1 §8.3 H9) ----

class FrontendTables : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(testing::ReadJsonFile(testing::TestDataDir() + "/frontend/binary-tables.json",
                                      &tables_));
    ASSERT_EQ(tables_["wasm_sha256"].str(),
              "780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422");
  }
  Json tables_;
};

std::vector<uint8_t> FeatureLutBytes() {
  std::vector<uint8_t> out;
  for (int k = 0; k < 512; ++k) {
    for (int a = 0; a < 36; ++a) {
      out.push_back(static_cast<uint8_t>(c8::gr8gldata::FEATURE_LUT[k][a].first));
      out.push_back(static_cast<uint8_t>(c8::gr8gldata::FEATURE_LUT[k][a].second));
    }
  }
  return out;
}

TEST_F(FrontendTables, FeatureLutMatchesBinary) {
  const std::vector<uint8_t> lut = FeatureLutBytes();
  ASSERT_EQ(lut.size(), static_cast<size_t>(tables_["feature_lut"]["bytes"].number()));
  const auto digest = testing::Sha256(lut.data(), lut.size());
  EXPECT_EQ(testing::Hex(digest.data(), digest.size()), tables_["feature_lut"]["sha256"].str());
  EXPECT_EQ(tables_["feature_lut"]["addresses"].size(), 1u);
}

TEST(FrontendSha256, KnownVectors) {
  const auto empty = testing::Sha256(nullptr, 0);
  EXPECT_EQ(testing::Hex(empty.data(), 32),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  const char* abc = "abc";
  const auto d = testing::Sha256(reinterpret_cast<const uint8_t*>(abc), 3);
  EXPECT_EQ(testing::Hex(d.data(), 32),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST_F(FrontendTables, SubpixelParamsMatchBinary) {
  const std::array<float, 6> q = SubpixelParams();
  const Json& floats = tables_["subpixel_q"]["floats"];
  ASSERT_EQ(floats.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    const float expected = static_cast<float>(floats[i].number());
    uint32_t a = 0, b = 0;
    std::memcpy(&a, &q[i], 4);
    std::memcpy(&b, &expected, 4);
    EXPECT_EQ(a, b) << "subpixelQ[" << i << "] " << q[i] << " vs " << expected;
  }
}

TEST_F(FrontendTables, FocalTableMatchesBinarySpotValues) {
  const Json& ft = tables_["focal_table"];
  EXPECT_EQ(static_cast<int>(ft["entries"].number()), 111);
  EXPECT_EQ(static_cast<int>(ft["enum_range"][0].number()), 2);
  EXPECT_EQ(static_cast<int>(ft["enum_range"][1].number()), 112);
  EXPECT_EQ(ft["mismatches"].size(), 0u);
  // The table covers enum values 2..112 of a 113-value enum.
  EXPECT_EQ(device_infos::kNumDeviceModels, 113);
  EXPECT_EQ(static_cast<int>(ft["GOOGLE_PIXEL7"][0].number()), device_infos::GOOGLE_PIXEL7);
  EXPECT_EQ(CameraIntrinsicsForModel(device_infos::GOOGLE_PIXEL7).fx,
            static_cast<float>(ft["GOOGLE_PIXEL7"][1].number()));
  EXPECT_EQ(static_cast<int>(ft["APPLE_IPHONE_X(default)"][0].number()),
            device_infos::APPLE_IPHONE_X);
  EXPECT_EQ(CameraIntrinsicsForModel(device_infos::APPLE_IPHONE_X).fx,
            static_cast<float>(ft["APPLE_IPHONE_X(default)"][1].number()));
  // 99 calibrated cases (public MODEL_TO_INTRINSIC) + default for the rest.
  EXPECT_EQ(static_cast<int>(ft["public_cases"].number()), 99);
}

// Config key strings read by the frontend getters (func 459, 900) vs the recovered defaults.
TEST_F(FrontendTables, FrontendConfigKeys) {
  const Json& keys = tables_["config_key_strings"];
  EXPECT_EQ(keys["31309"].str(), "GlRealityFrame.PyramidSize");
  struct Expected {
    const char* address;
    const char* key;
  };
  const Expected expected[] = {
      {"20112", "VioTracker.numIncreasedFramePoints"},
      {"20005", "VioTracker.cloudMapRelocNumPoints"},
      {"33330", "VioTracker.trackingDescriptorType"},
      {"24709", "VioTracker.disableMapBuilder"},
      {"21521", "VioTracker.maxFirstSuccessfulDeviceVpsWaitTimeNanos"},
  };
  // Every VioTracker key is present in the recovered table with the S1 §4 default.
  std::vector<std::string> registered;
#define SR_CONFIG_BEGIN(prefix)
#define SR_CONFIG_KEY(prefix, field, key, kind, ctype, def, getter, owner, read) \
  registered.push_back(key);
#define SR_CONFIG_END(prefix)
#define SR_CONFIG_UNRECOVERED(prefix, key, func)
#include "config/config_defaults.inc"
#undef SR_CONFIG_BEGIN
#undef SR_CONFIG_KEY
#undef SR_CONFIG_END
#undef SR_CONFIG_UNRECOVERED
  for (const Expected& e : expected) {
    EXPECT_EQ(keys[e.address].str(), e.key);
    EXPECT_NE(std::find(registered.begin(), registered.end(), e.key), registered.end()) << e.key;
  }
  const config::VioTrackerSettings defaults;
  EXPECT_EQ(defaults.num_increased_frame_points, 1000);
  EXPECT_EQ(defaults.cloud_map_reloc_num_points, 500);
  EXPECT_EQ(defaults.tracking_descriptor_type, 2);
  EXPECT_FALSE(defaults.disable_map_builder);
  EXPECT_EQ(defaults.max_first_successful_device_vps_wait_time_nanos, 1e10f);
  // GlRealityFrame.PyramidSize is not in the recovered table; the frontend uses its default.
  EXPECT_EQ(std::find(registered.begin(), registered.end(), "GlRealityFrame.PyramidSize"),
            registered.end());
  EXPECT_EQ(kPyramidSizeDefault, 1024);
}

}  // namespace
}  // namespace slam_refact::frontend
