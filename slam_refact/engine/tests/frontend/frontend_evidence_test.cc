// WP1 evidence gates (label evidence; registered only when SLAM_REFACT_EVIDENCE_ROOT exists):
//   F-H2  binary func 917+918 vs engine CPU half, TUM c2 (92 levels) + Flip6 c1 (92 levels)
//   F-H3  captured gray -> stage 2+3 BIT_EXACT, 46 frames
//   F-H4  captured level 0 -> levels 1-3 + stage 2/3, 46 frames
//   F-H7  injected pyramid -> production detection path == binary CPU half, 46 frames
//   F-H8  GetQuaternion vs every captured stageFrame call
//   F-H9  FEATURE_LUT / subpixelQ / focal table / key strings vs the wasm data segments
//   F-U1  SwiftShader texUv varying model vs the WebGL readback (10 viewports, bit equality)
//   F-H5, F-H6 (REPORT) full CPU pyramid vs capture; verdicts are printed, not asserted
// Inputs: build/slam-refact-20261015/spec/frontend/capture/{tum-room1-full/c2,flip6-s2/c1},
// the binary golden written by tests/frontend/gen_binary_gr8_golden.sh under
// build/slam-refact-20261015/wp1/binary-gr8/, and build/slam-refact-20261015/inputs/xr-slam.wasm.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "c8/geometry/device-pose.h"
#include "c8/hmatrix.h"
#include "common/c8_adapters.h"
#include "frontend/frontend.h"
#include "frontend/gr8_cpu_shader.h"
#include "frontend/intrinsics.h"
#include "frontend/pyramid_source.h"
#include "gate_util.h"
#include "gtest/gtest.h"
#include "reality/engine/features/gr8gl-data.h"
#include "test_util.h"

namespace slam_refact::frontend {
namespace {

using testing::Json;

struct Scenario {
  const char* capture_dir;  // relative to the evidence root
  const char* golden_dir;   // binary_gr8 golden, relative to the evidence root
  const char* manufacturer;
  const char* model;
  int cam_w;
  int cam_h;
};

// TUM: UA "Pixel 7" (Google); Flip6: reduced UA model "K" (S1 §5.12, architecture.md §6.2).
const Scenario kScenarios[] = {
    {"spec/frontend/capture/tum-room1-full/c2", "wp1/binary-gr8/tum-room1-full-c2", "Google",
     "Pixel 7", 240, 320},
    {"spec/frontend/capture/flip6-s2/c1", "wp1/binary-gr8/flip6-s2-c1", "", "K", 480, 640},
};

std::string Path(const std::string& rel) { return testing::EvidenceRoot() + "/" + rel; }

std::string CapName(int k, const char* ext) {
  char name[64];
  std::snprintf(name, sizeof(name), "cap-%03d%s", k, ext);
  return name;
}

struct Capture {
  int capture;
  int input_frame;
  c8::Quaternion q;
};

bool LoadMeta(const Scenario& s, Json* meta, std::vector<Capture>* caps) {
  if (!testing::ReadJsonFile(Path(s.capture_dir) + "/meta.json", meta)) {
    return false;
  }
  caps->clear();
  const Json& list = (*meta)["captures"];
  for (size_t i = 0; i < list.size(); ++i) {
    if (list[i]["inputFrame"].is_null()) {
      continue;
    }
    const Json& q = list[i]["stageArgs"]["qwxyz"];
    caps->push_back({static_cast<int>(list[i]["capture"].number()),
                     static_cast<int>(list[i]["inputFrame"].number()),
                     c8::Quaternion(static_cast<float>(q[0].number()),
                                    static_cast<float>(q[1].number()),
                                    static_cast<float>(q[2].number()),
                                    static_cast<float>(q[3].number()))});
  }
  return true;
}

// ---- F-H2, F-H3, F-H4, F-H7 over all captures ----

TEST(FrontendEvidence, CpuHalfShaderAndDetectionGates) {
  int h2_levels = 0, h2_exact = 0, h3_frames = 0, h3_exact = 0, h4_frames = 0, h4_pass = 0;
  int h7_frames = 0, h7_exact = 0;
  for (const Scenario& s : kScenarios) {
    SCOPED_TRACE(s.capture_dir);
    Json meta;
    std::vector<Capture> caps;
    ASSERT_TRUE(LoadMeta(s, &meta, &caps));
    const int rows = static_cast<int>(meta["rows"].number());
    ASSERT_EQ(caps.size(), 23u);
    const PyramidLayout layout = ComputePyramidLayout(s.cam_w, s.cam_h);
    const IntrinsicsSet k = ComputeIntrinsicsSet(s.manufacturer, s.model, s.cam_w, s.cam_h,
                                                 layout.levels[0].w, layout.levels[0].h);
    const CpuGr8Pyramid cpu(layout, ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox);
    FeatureFrontend frontend;
    for (const Capture& c : caps) {
      SCOPED_TRACE("capture " + std::to_string(c.capture));
      RgbaAtlas captured;
      ASSERT_TRUE(testing::LoadCaptureAtlas(
          Path(s.capture_dir) + "/" + CapName(c.capture, ".rgba.gz"), rows, &captured));
      Json golden;
      ASSERT_TRUE(testing::ReadJsonFile(Path(s.golden_dir) + "/" + CapName(c.capture, ".json"),
                                        &golden))
          << "run tests/frontend/gen_binary_gr8_golden.sh first";

      // F-H2.
      const c8::Gr8Pyramid pyr = PyramidFromInjectedAtlas(
          PyramidAtlasView{captured.rgba.data(), captured.size, captured.rows}, layout);
      std::string detail;
      const int bad = testing::CompareCpuHalfWithBinary(testing::CpuHalfPerLevel(pyr), golden,
                                                        &detail);
      h2_levels += 4;
      h2_exact += 4 - bad;
      EXPECT_EQ(bad, 0) << "F-H2 " << detail;

      // F-H7: production detection path on the injected pyramid.
      c8::FrameWithPoints frame(ToC8Intrinsics(k.processing), c8::HMatrixGen::i(),
                                c8::xrRotationFromDeviceRotation(c.q));
      frontend.Detect(pyr, WorldSlamDetectionConfig(), &frame);
      const auto expected = testing::ExpectedBasePoints(golden, pyr);
      bool same = frame.pixels().size() == expected.size();
      for (size_t i = 0; same && i < expected.size(); ++i) {
        same = frame.pixels()[i].x() == expected[i].x && frame.pixels()[i].y() == expected[i].y &&
               static_cast<int>(frame.points()[i].scale()) == expected[i].scale;
      }
      ++h7_frames;
      h7_exact += same ? 1 : 0;
      EXPECT_TRUE(same) << "F-H7";

      // F-H3.
      RgbaAtlas s23 = captured;
      cpu.RunStage2And3(&s23);
      const testing::AtlasStats st3 = testing::CompareLevels(s23, captured, layout);
      ++h3_frames;
      h3_exact += testing::Stage23BitExact(st3) ? 1 : 0;
      EXPECT_TRUE(testing::Stage23BitExact(st3)) << "F-H3";

      // F-H4.
      RgbaAtlas rebuilt = captured;
      cpu.RunFromLevel0(&rebuilt);
      const testing::AtlasStats st4 = testing::CompareLevels(rebuilt, captured, layout);
      ++h4_frames;
      h4_pass += testing::RebuildPass(st4) ? 1 : 0;
      EXPECT_TRUE(testing::RebuildPass(st4))
          << "F-H4 R " << st4.identical_fraction(0) << " G " << st4.identical_fraction(1) << " B "
          << st4.identical_fraction(2) << " A " << st4.identical_fraction(3);
    }
  }
  std::printf(
      "WP1-EVIDENCE F-H2 bit_exact_levels=%d/%d F-H3 bit_exact_frames=%d/%d F-H4 pass_frames=%d/%d "
      "F-H7 exact_frames=%d/%d\n",
      h2_exact, h2_levels, h3_exact, h3_frames, h4_pass, h4_frames, h7_exact, h7_frames);
  EXPECT_EQ(h2_levels, 184);
}

// ---- F-H8: every captured stageFrame call ----

TEST(FrontendEvidence, QuaternionMatchesEveryStageFrameCall) {
  int total = 0, equal = 0;
  for (const Scenario& s : kScenarios) {
    Json meta;
    ASSERT_TRUE(testing::ReadJsonFile(Path(s.capture_dir) + "/meta.json", &meta));
    const Json& calls = meta["stageCalls"];
    int scenario_equal = 0;
    for (size_t i = 0; i < calls.size(); ++i) {
      const Json& a = calls[i]["args"];
      // (texture, w, h, rotation, alpha, beta, gamma, qw, qx, qy, qz, ...) S1 §2.1.
      const c8::Quaternion q = GetQuaternion(a[4].number(), a[5].number(), a[6].number());
      const bool same = q.w() == static_cast<float>(a[7].number()) &&
                        q.x() == static_cast<float>(a[8].number()) &&
                        q.y() == static_cast<float>(a[9].number()) &&
                        q.z() == static_cast<float>(a[10].number());
      scenario_equal += same ? 1 : 0;
      EXPECT_TRUE(same) << s.capture_dir << " call " << calls[i]["call"].number();
    }
    std::printf("WP1-EVIDENCE F-H8 %s equal=%d/%zu\n", s.capture_dir, scenario_equal, calls.size());
    total += static_cast<int>(calls.size());
    equal += scenario_equal;
  }
  EXPECT_EQ(equal, total);
  EXPECT_GE(total, 41);
}

// ---- F-H9: constant tables vs the wasm data segments ----

uint32_t Leb(const std::vector<uint8_t>& b, size_t* i) {
  uint32_t r = 0;
  int shift = 0;
  while (true) {
    const uint8_t x = b[(*i)++];
    r |= static_cast<uint32_t>(x & 0x7f) << shift;
    shift += 7;
    if (x < 0x80) {
      return r;
    }
  }
}

int32_t Sleb(const std::vector<uint8_t>& b, size_t* i) {
  int64_t r = 0;
  int shift = 0;
  while (true) {
    const uint8_t x = b[(*i)++];
    r |= static_cast<int64_t>(x & 0x7f) << shift;
    shift += 7;
    if (x < 0x80) {
      if (x & 0x40) {
        r -= int64_t{1} << shift;
      }
      return static_cast<int32_t>(r);
    }
  }
}

// Active data segments (flag 0, i32.const offset) of the wasm data section (id 11).
std::vector<std::pair<uint32_t, std::vector<uint8_t>>> DataSegments(const std::vector<uint8_t>& w) {
  std::vector<std::pair<uint32_t, std::vector<uint8_t>>> segs;
  size_t i = 8;
  while (i < w.size()) {
    const uint8_t id = w[i++];
    const uint32_t size = Leb(w, &i);
    const size_t start = i;
    if (id == 11) {
      size_t j = i;
      const uint32_t n = Leb(w, &j);
      for (uint32_t s = 0; s < n; ++s) {
        const uint32_t flag = Leb(w, &j);
        if (flag != 0 || w[j] != 0x41) {
          return {};
        }
        ++j;
        const int32_t addr = Sleb(w, &j);
        if (w[j] != 0x0b) {
          return {};
        }
        ++j;
        const uint32_t len = Leb(w, &j);
        segs.push_back({static_cast<uint32_t>(addr),
                        std::vector<uint8_t>(w.begin() + j, w.begin() + j + len)});
        j += len;
      }
    }
    i = start + size;
  }
  return segs;
}

bool Mem(const std::vector<std::pair<uint32_t, std::vector<uint8_t>>>& segs, uint32_t addr,
         size_t n, std::vector<uint8_t>* out) {
  for (const auto& [a, b] : segs) {
    if (a <= addr && addr + n <= a + b.size()) {
      out->assign(b.begin() + (addr - a), b.begin() + (addr - a) + n);
      return true;
    }
  }
  return false;
}

TEST(FrontendEvidence, TablesMatchWasmDataSegments) {
  std::string text;
  ASSERT_TRUE(testing::ReadTextFile(Path("inputs/xr-slam.wasm"), &text));
  const std::vector<uint8_t> wasm(text.begin(), text.end());
  const auto digest = testing::Sha256(wasm.data(), wasm.size());
  ASSERT_EQ(testing::Hex(digest.data(), 32),
            "780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422");
  const auto segs = DataSegments(wasm);
  ASSERT_FALSE(segs.empty());

  // FEATURE_LUT @151808, 512 x 36 int8 pairs.
  std::vector<uint8_t> lut;
  ASSERT_TRUE(Mem(segs, 151808, 36864, &lut));
  int lut_mismatch = 0;
  for (int k = 0; k < 512; ++k) {
    for (int a = 0; a < 36; ++a) {
      const auto& p = c8::gr8gldata::FEATURE_LUT[k][a];
      lut_mismatch += static_cast<int8_t>(lut[(k * 36 + a) * 2]) != p.first;
      lut_mismatch += static_cast<int8_t>(lut[(k * 36 + a) * 2 + 1]) != p.second;
    }
  }
  EXPECT_EQ(lut_mismatch, 0);

  // subpixelQ @92068, 6 floats.
  std::vector<uint8_t> qbytes;
  ASSERT_TRUE(Mem(segs, 92068, 24, &qbytes));
  const std::array<float, 6> q = SubpixelParams();
  EXPECT_EQ(std::memcmp(qbytes.data(), q.data(), 24), 0);

  // Focal table @296576: DeviceModel 2..112, 111 floats.
  std::vector<uint8_t> fbytes;
  ASSERT_TRUE(Mem(segs, 296576, 111 * 4, &fbytes));
  int focal_mismatch = 0;
  for (int m = 2; m <= 112; ++m) {
    float binary = 0.f;
    std::memcpy(&binary, &fbytes[(m - 2) * 4], 4);
    const float engine = CameraIntrinsicsForModel(m).fx;
    if (std::memcmp(&binary, &engine, 4) != 0) {
      ++focal_mismatch;
      ADD_FAILURE() << "DeviceModel " << m << ": binary " << binary << " engine " << engine;
    }
  }
  EXPECT_EQ(focal_mismatch, 0);

  // Config key strings read by func 459 / func 900.
  const std::pair<uint32_t, const char*> keys[] = {
      {31309, "GlRealityFrame.PyramidSize"},
      {20112, "VioTracker.numIncreasedFramePoints"},
      {20005, "VioTracker.cloudMapRelocNumPoints"},
      {33330, "VioTracker.trackingDescriptorType"},
      {24709, "VioTracker.disableMapBuilder"},
      {21521, "VioTracker.maxFirstSuccessfulDeviceVpsWaitTimeNanos"},
  };
  for (const auto& [addr, key] : keys) {
    std::vector<uint8_t> s;
    ASSERT_TRUE(Mem(segs, addr, std::strlen(key) + 1, &s)) << key;
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(s.data())), key);
  }
  std::printf("WP1-EVIDENCE F-H9 lut_mismatch=%d subpixelQ_equal=%d focal_mismatch=%d/111\n",
              lut_mismatch, std::memcmp(qbytes.data(), q.data(), 24) == 0 ? 1 : 0,
              focal_mismatch);
}

// ---- F-U1: texUv varying model vs the full SwiftShader readback (S1 §5.1.3) ----

TEST(FrontendEvidence, SwiftShaderVaryingModelMatchesReadback) {
  const std::string dir = Path("spec/frontend/gl-semantics/varying-swiftshader");
  Json meta;
  ASSERT_TRUE(testing::ReadJsonFile(dir + "/varying.json", &meta))
      << "run tools/re/spec_frontend/varying_probe.mjs first";
  ASSERT_EQ(meta["browserVersion"].str(), "144.0.7559.96");
  const Json& cases = meta["cases"];
  long total = 0, equal = 0;
  for (size_t i = 0; i < cases.size(); ++i) {
    const Json& vp = cases[i]["vp"];
    const int c = static_cast<int>(vp[0].number()), r = static_cast<int>(vp[1].number());
    const int w = static_cast<int>(vp[2].number()), h = static_cast<int>(vp[3].number());
    std::string raw;
    ASSERT_TRUE(testing::ReadTextFile(dir + "/case-" + std::to_string(i) + ".f32", &raw));
    ASSERT_EQ(raw.size(), static_cast<size_t>(w) * h * 16);
    const std::vector<float> uv = SwiftShaderQuadTexUv(
        c, r, w, h, static_cast<int>(cases[i]["rot"].number()), kPyramidSizeDefault);
    long case_equal = 0;
    for (size_t p = 0; p < static_cast<size_t>(w) * h; ++p) {
      // Readback RGBA32F: R = texUv.x, G = texUv.y; compare the bits.
      case_equal += std::memcmp(raw.data() + p * 16, &uv[p * 2], 8) == 0 ? 1 : 0;
    }
    std::printf("WP1-EVIDENCE F-U1 %s bit_equal=%ld/%d\n", cases[i]["name"].str().c_str(),
                case_equal, w * h);
    EXPECT_EQ(case_equal, static_cast<long>(w) * h) << cases[i]["name"].str();
    total += static_cast<long>(w) * h;
    equal += case_equal;
  }
  std::printf("WP1-EVIDENCE F-U1 bit_equal_pixels=%ld/%ld\n", equal, total);
  EXPECT_EQ(cases.size(), 10u);
}

// ---- F-H5 / F-H6 (REPORT): full CPU pyramid from the camera frame vs the capture ----

struct KeyRec {
  bool has_orb;
  bool has_gorb;
  std::array<uint8_t, 32> orb;
  std::array<uint8_t, 32> gorb;
};
using KeyMap = std::map<std::tuple<int, long, long>, KeyRec>;

KeyMap DetectKeys(FeatureFrontend* frontend, const c8::Gr8Pyramid& pyr,
                  const PinholeIntrinsics& k, const c8::Quaternion& device, int per_level[4]) {
  c8::FrameWithPoints frame(ToC8Intrinsics(k), c8::HMatrixGen::i(),
                            c8::xrRotationFromDeviceRotation(device));
  frontend->Detect(pyr, WorldSlamDetectionConfig(), &frame);
  KeyMap out;
  for (int i = 0; i < 4; ++i) {
    per_level[i] = 0;
  }
  const auto& store = frame.store();
  for (size_t i = 0; i < frame.points().size(); ++i) {
    const int scale = frame.points()[i].scale();
    ++per_level[scale];
    KeyRec r{};
    r.has_orb = store.has<c8::OrbFeature>(i);
    r.has_gorb = store.has<c8::GorbFeature>(i);
    if (r.has_orb) {
      std::memcpy(r.orb.data(), store.get<c8::OrbFeature>(i).data(), 32);
    }
    if (r.has_gorb) {
      std::memcpy(r.gorb.data(), store.get<c8::GorbFeature>(i).data(), 32);
    }
    out[{scale, std::lround(frame.pixels()[i].x() * 1000.0),
         std::lround(frame.pixels()[i].y() * 1000.0)}] = r;
  }
  return out;
}

TEST(FrontendEvidence, ReportFullCpuPyramidVsCapture) {
  // tolerances-v2.json full_cpu_pyramid_vs_capture (fixed before Flip6 was compared, S1 §8.2).
  constexpr double kRMin = 0.985, kGMin = 0.985, kCandJMin = 0.75, kCountRelMax = 0.1;
  constexpr long kRMaxDiff = 2, kGMaxDiff = 2;
  for (const Scenario& s : kScenarios) {
    Json meta;
    std::vector<Capture> caps;
    ASSERT_TRUE(LoadMeta(s, &meta, &caps));
    const int rows = static_cast<int>(meta["rows"].number());
    const std::string dataset = meta["input"]["dataset"].str();
    const bool tum = meta["scenario"].str().rfind("tum", 0) == 0;
    Json index;
    ASSERT_TRUE(testing::ReadJsonFile(dataset + (tum ? "/input.json" : "/record.json"), &index));
    const Json& frames = tum ? index["frames"] : index["recording"]["frames"];
    PyramidSource source(s.cam_w, s.cam_h, ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox);
    const IntrinsicsSet k = ComputeIntrinsicsSet(s.manufacturer, s.model, s.cam_w, s.cam_h,
                                                 source.layout().levels[0].w,
                                                 source.layout().levels[0].h);
    FeatureFrontend frontend;
    int pass = 0, evaluated = 0, atlas_exact = 0;
    std::vector<double> kp_jaccard, desc_identical;
    for (const Capture& c : caps) {
      int w = 0, h = 0;
      std::vector<uint8_t> gray;
      const std::string pgm =
          dataset + "/mav0/cam0/data/" + frames[static_cast<size_t>(c.input_frame)]["file"].str();
      ASSERT_TRUE(testing::ReadPgm(pgm, &w, &h, &gray)) << pgm;
      ASSERT_EQ(w, s.cam_w);
      ASSERT_EQ(h, s.cam_h);
      RgbaAtlas captured;
      ASSERT_TRUE(testing::LoadCaptureAtlas(
          Path(s.capture_dir) + "/" + CapName(c.capture, ".rgba.gz"), rows, &captured));
      const c8::Gr8Pyramid mine = source.Get({}, {}, GrayImageView{gray.data(), w, h, w});
      const testing::AtlasStats st =
          testing::CompareLevels(source.last_cpu_atlas(), captured, source.layout());
      int count_mine[4], count_cap[4];
      const KeyMap km = DetectKeys(&frontend, mine, k.processing, c.q, count_mine);
      const c8::Gr8Pyramid cap_pyr = PyramidFromInjectedAtlas(
          PyramidAtlasView{captured.rgba.data(), captured.size, captured.rows}, source.layout());
      const KeyMap kc = DetectKeys(&frontend, cap_pyr, k.processing, c.q, count_cap);
      double rel = 0.0;
      for (int i = 0; i < 4; ++i) {
        rel = std::max(rel, std::abs(count_mine[i] - count_cap[i]) /
                                static_cast<double>(std::max(count_cap[i], 1)));
      }
      const bool ok = st.identical_fraction(0) >= kRMin && st.max_diff[0] <= kRMaxDiff &&
                      st.identical_fraction(1) >= kGMin && st.max_diff[1] <= kGMaxDiff &&
                      st.candidate_jaccard() >= kCandJMin && rel <= kCountRelMax;
      pass += ok ? 1 : 0;
      ++evaluated;
      // Every byte of the CPU atlas rows, gutters included (S1 §9 U1); the readback rows below
      // PyramidLayout::rows hold only the cleared ROI slots, which the CPU half never reads.
      const std::vector<uint8_t>& cpu_bytes = source.last_cpu_atlas().rgba;
      atlas_exact += cpu_bytes.size() <= captured.rgba.size() &&
                             std::equal(cpu_bytes.begin(), cpu_bytes.end(), captured.rgba.begin())
                         ? 1
                         : 0;
      long both = 0, pairs = 0, identical = 0;
      for (const auto& [key, a] : km) {
        const auto it = kc.find(key);
        if (it == kc.end()) {
          continue;
        }
        ++both;
        if (a.has_orb && it->second.has_orb) {
          ++pairs;
          identical += a.orb == it->second.orb;
        }
        if (a.has_gorb && it->second.has_gorb) {
          ++pairs;
          identical += a.gorb == it->second.gorb;
        }
      }
      const long uni = static_cast<long>(km.size() + kc.size()) - both;
      kp_jaccard.push_back(uni ? static_cast<double>(both) / uni : 1.0);
      desc_identical.push_back(pairs ? static_cast<double>(identical) / pairs : 1.0);
      std::printf(
          "WP1-REPORT F-H5 %s frame=%d R=%.5f G=%.5f maxdiff=%ld/%ld/%ld/%ld candJ=%.4f "
          "countRel=%.4f %s | F-H6 kpJ=%.4f descIdentical=%.4f\n",
          s.capture_dir, c.input_frame, st.identical_fraction(0), st.identical_fraction(1),
          st.max_diff[0], st.max_diff[1], st.max_diff[2], st.max_diff[3], st.candidate_jaccard(),
          rel, ok ? "PASS" : "FAIL", kp_jaccard.back(), desc_identical.back());
    }
    std::stable_sort(kp_jaccard.begin(), kp_jaccard.end());
    std::stable_sort(desc_identical.begin(), desc_identical.end());
    std::printf(
        "WP1-REPORT F-H5 %s pass=%d/%d verdict=%s atlas_bit_exact=%d/%d | F-H6 kpJ "
        "min/med/max=%.3f/%.3f/%.3f descIdentical med=%.3f\n",
        s.capture_dir, pass, evaluated, pass == evaluated ? "PASS" : "FAIL", atlas_exact, evaluated,
        kp_jaccard.front(),
        kp_jaccard[kp_jaccard.size() / 2], kp_jaccard.back(),
        desc_identical[desc_identical.size() / 2]);
    // REPORT gate: the evaluation must cover every capture; the verdict is recorded, not asserted.
    EXPECT_EQ(evaluated, 23);
  }
}

}  // namespace
}  // namespace slam_refact::frontend
