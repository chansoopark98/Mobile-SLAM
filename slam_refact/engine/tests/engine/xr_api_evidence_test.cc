// XrController API evidence tests against the original SDK (contract FULL-1, spec 06 §1.3, §2.3,
// §4.3; label evidence). Inputs are TSV extracts (slam_refact/tools/harness/oracle/
// extract_api_oracle.py) of
//   - every original reference run's reality.intrinsics (build/slam-refact-20261015/reference/runs)
//     and of the api-oracle runs (build/slam-refact-full-20261019/oracle, incl. a 360x640 display),
//   - XrController.hitTest pairs recorded inside one onUpdate callback with the same callback's
//     reality fields (flip6-s2 api1: default coordinates; api2: leftHandedAxes + mirroredDisplay,
//     origin (0.5, 1.5, -1), yaw-45 facing, 360x640 display),
//   - reality.lighting of every fresh callback (flip6-s2 api1).
// Tolerances are the ones registered in spec 06 before the runs; they are not changed here.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/homography.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "engine/display_intrinsics.h"
#include "engine/hit_test.h"
#include "engine/lighting.h"
#include "engine/startup_recenter.h"
#include "frontend/frontend.h"
#include "frontend/pyramid_source.h"
#include "slam_refact/engine_config.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/xr_controller.h"

namespace slam_refact {
namespace {

const std::string kOracleDir = SR_FULL_ORACLE_DIR;
const std::string kRepoRoot = SR_REPO_ROOT;

std::vector<std::vector<std::string>> ReadTsv(const std::string& path) {
  std::ifstream in(path);
  EXPECT_TRUE(in.good()) << path;
  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string c;
    while (std::getline(ss, c, '\t')) cols.push_back(c);
    rows.push_back(cols);
  }
  return rows;
}

float F(const std::string& s) { return std::stof(s); }

// ------------------------------------------------------------------------------------------------
// I-1: display intrinsics of every recorded original run, |error| <= 1e-6 per element.

void CheckIntrinsicsFile(const std::string& path, int* rows_checked) {
  for (const std::vector<std::string>& r : ReadTsv(path)) {
    ASSERT_EQ(r.size(), 26u);
    EngineConfig e;
    e.manufacturer = r[2] == "-" ? "" : r[2];
    e.model = r[3];
    e.capture_width = std::stoi(r[4]);
    e.capture_height = std::stoi(r[5]);
    const GraphicsIntrinsics g{std::stoi(r[6]), std::stoi(r[7]), F(r[8]), F(r[9])};
    const std::array<float, 16> m = DisplayProjection(CaptureIntrinsicsFor(e), g);
    for (int i = 0; i < 16; ++i) {
      EXPECT_NEAR(m[i], std::stod(r[10 + i]), 1e-6) << r[1] << " element " << i;
    }
    ++*rows_checked;
  }
}

TEST(XrApiEvidence, IntrinsicsMatchEveryOriginalRun) {
  int rows = 0;
  CheckIntrinsicsFile(kOracleDir + "/tsv-intrinsics/intrinsics.tsv", &rows);
  CheckIntrinsicsFile(kOracleDir + "/tsv-intrinsics-oracle/intrinsics.tsv", &rows);
  std::printf("[I-1] intrinsics rows checked: %d\n", rows);
  EXPECT_GE(rows, 60);
}

// ------------------------------------------------------------------------------------------------
// H-1..H-4: hit test pairs.

// Inverse of the 704 chain for positions: p = undo(o^-1 np / s).
Vec3f InversePosition(const CoordinateSystem& c, const Vec3f& pub) {
  const float s = c.scale > 0.f ? c.scale : 1.f;
  const c8::Quaternion oq =
      c8::groundPlaneRotation(c8::Quaternion(c.facing.w, c.facing.x, c.facing.y, c.facing.z));
  const c8::HMatrix o = c8::cameraMotion(c8::HVector3(c.origin.x, c.origin.y, c.origin.z), oq);
  const c8::HPoint3 p = o.inv() * c8::HPoint3(pub.x, pub.y, pub.z);
  float x = p.x() / s, y = p.y() / s, z = p.z() / s;
  if (c.mirrored_display) x = -x;
  if (!c.left_handed_axes) z = -z;
  return Vec3f{x, y, z};
}

// Inverse of the 704 chain for the camera rotation: q' = oq^-1 nq, then undo mirror / handedness.
QuatWxyz InverseRotation(const CoordinateSystem& c, const QuatWxyz& pub) {
  const c8::Quaternion oq =
      c8::groundPlaneRotation(c8::Quaternion(c.facing.w, c.facing.x, c.facing.y, c.facing.z));
  const c8::Quaternion q = oq.inverse().times(c8::Quaternion(pub.w, pub.x, pub.y, pub.z));
  float w = q.w(), x = q.x(), y = q.y(), z = q.z();
  if (c.mirrored_display) {
    y = -y;
    z = -z;
  }
  if (!c.left_handed_axes) {
    w = -w;
    z = -z;
  }
  return QuatWxyz{w, x, y, z};
}

struct HitStats {
  int samples = 0;
  int feature_queries = 0;     // queries whose original result has a FEATURE_POINT hit
  int empty_surface_queries = 0;
  int count_mismatch = 0;
  int type_mismatch = 0;
  int pos_fail = 0;
  int dist_fail = 0;
  int rot_fail = 0;
  int rot_sign_diff = 0;
  double max_pos_rel = 0.0;
  double max_dist_rel = 0.0;
};

HitStats CheckHits(const std::string& path) {
  HitStats st;
  CoordinateSystem coords;
  HitTestState state;
  std::vector<std::vector<std::string>> rows = ReadTsv(path);
  for (size_t i = 0; i < rows.size(); ++i) {
    const std::vector<std::string>& r = rows[i];
    if (r[0] == "C") {
      coords.origin = Vec3f{F(r[1]), F(r[2]), F(r[3])};
      coords.facing = QuatWxyz{F(r[4]), F(r[5]), F(r[6]), F(r[7])};
      coords.scale = F(r[8]);
      coords.left_handed_axes = r[9] == "1";
      coords.mirrored_display = r[10] == "1";
    } else if (r[0] == "S") {
      ++st.samples;
      state.camera_position = InversePosition(coords, Vec3f{F(r[3]), F(r[4]), F(r[5])});
      state.camera_rotation = InverseRotation(coords, QuatWxyz{F(r[6]), F(r[7]), F(r[8]), F(r[9])});
      for (int k = 0; k < 16; ++k) state.projection[static_cast<size_t>(k)] = F(r[10 + static_cast<size_t>(k)]);
      const int n = std::stoi(r[26]);
      state.points.clear();
      for (int k = 0; k < n; ++k) {
        const std::vector<std::string>& p = rows[++i];
        state.points.push_back(InversePosition(coords, Vec3f{F(p[1]), F(p[2]), F(p[3])}));
      }
    } else if (r[0] == "Q") {
      const float x = F(r[2]), y = F(r[3]);
      const uint32_t mask = static_cast<uint32_t>(std::stoul(r[4]));
      const size_t n_orig = std::stoul(r[5]);
      std::vector<XrHitResult> mine;
      for (const InternalHit& h : FeaturePointHitTest(state, x, y, mask, coords.mirrored_display)) {
        mine.push_back(RewriteHit(coords, h));
      }
      if (mine.size() != n_orig) {
        ++st.count_mismatch;
        ADD_FAILURE() << "hit count " << mine.size() << " vs original " << n_orig << " (sample row " << i << ")";
        continue;
      }
      if (n_orig == 0) {
        if (mask != 0 && !ContainsResultType(mask, XrHitType::kFeaturePoint)) ++st.empty_surface_queries;
        continue;
      }
      for (size_t k = 0; k < n_orig; ++k) {
        const size_t b = 6 + 9 * k;
        const int type = std::stoi(r[b]);
        const XrHitResult& m = mine[k];
        if (static_cast<int>(m.type) != type) {
          ++st.type_mismatch;
          continue;
        }
        if (type == 1) ++st.feature_queries;
        const double ox = std::stod(r[b + 1]), oy = std::stod(r[b + 2]), oz = std::stod(r[b + 3]);
        const double d = std::stod(r[b + 8]);
        const double tol = 1e-3 * std::max(1.0, d);
        const double dp = std::sqrt((m.position.x - ox) * (m.position.x - ox) +
                                    (m.position.y - oy) * (m.position.y - oy) +
                                    (m.position.z - oz) * (m.position.z - oz));
        const double dd = std::fabs(m.distance - d);
        st.max_pos_rel = std::max(st.max_pos_rel, dp / std::max(1.0, d));
        st.max_dist_rel = std::max(st.max_dist_rel, dd / std::max(1.0, d));
        if (dp > tol) ++st.pos_fail;
        if (dd > tol) ++st.dist_fail;
        const float rw = F(r[b + 4]), rx = F(r[b + 5]), ry = F(r[b + 6]), rz = F(r[b + 7]);
        if (m.rotation.w != rw || m.rotation.x != rx || m.rotation.y != ry || m.rotation.z != rz) ++st.rot_fail;
        if (std::signbit(m.rotation.w) != std::signbit(rw) || std::signbit(m.rotation.z) != std::signbit(rz)) {
          ++st.rot_sign_diff;
        }
      }
    }
  }
  return st;
}

void ExpectHitGates(const char* label, const HitStats& st) {
  std::printf("[H %s] samples=%d feature_hits=%d empty_surface_queries=%d count_mismatch=%d "
              "type_mismatch=%d pos_fail=%d dist_fail=%d rot_fail=%d rot_sign_diff=%d "
              "max_pos_rel=%.3g max_dist_rel=%.3g\n",
              label, st.samples, st.feature_queries, st.empty_surface_queries, st.count_mismatch,
              st.type_mismatch, st.pos_fail, st.dist_fail, st.rot_fail, st.rot_sign_diff,
              st.max_pos_rel, st.max_dist_rel);
  EXPECT_EQ(st.count_mismatch, 0);
  EXPECT_EQ(st.type_mismatch, 0);
  EXPECT_EQ(st.pos_fail, 0);   // H-1
  EXPECT_EQ(st.dist_fail, 0);  // H-1
  EXPECT_EQ(st.rot_fail, 0);   // H-2
  EXPECT_GT(st.empty_surface_queries, 0);  // H-3
  EXPECT_GE(st.feature_queries, 20);       // H-4
}

TEST(XrApiEvidence, HitTestMatchesOriginalDefaultCoordinates) {
  ExpectHitGates("api1", CheckHits(kOracleDir + "/tsv-api1/hits.tsv"));
}

TEST(XrApiEvidence, HitTestMatchesOriginalLeftHandedMirroredOriginFacing) {
  ExpectHitGates("api2", CheckHits(kOracleDir + "/tsv-api2/hits.tsv"));
}

// ------------------------------------------------------------------------------------------------
// Lighting (spec 06 §4.3, RM-3a gate rm3a-lighting-gate.v1 under
// build/slam-refact-full-20261019/preregistration/, registered before these tests ran).
//
// Rows (lighting.tsv): L callback frame exposure temperature prev_frame prev_pgm_path. The
// exposure of the original is bit-compared with
//   - the registered association (callback k = the previous fresh callback's input frame, a CPU
//     pyramid port `swiftshader-approx` that is byte-identical to the original's SwiftShader GPU
//     pyramid): G-L1a (api1) / G-L1b (api3), on the registered subset (i < 100 or i % 25 == 0);
//     reported as it was registered, characterised by its one expected exception;
//   - the engine pipeline (GlueRunner + Engine, staged order of the original): G-L2 (api1, all
//     1009 callbacks) / G-L3 (api3, all 600 callbacks).
// Tolerance is 0 (integer score over an integer product, float32 bit equality).

struct LightRow {
  int callback = 0;
  int frame = 0;
  float exposure = 0.f;
  float temperature = 0.f;
  int prev = 0;
  std::string prev_path;
};

std::vector<LightRow> ReadLighting(const std::string& path) {
  std::vector<LightRow> out;
  for (const std::vector<std::string>& r : ReadTsv(path)) {
    EXPECT_EQ(r.size(), 7u) << path;
    LightRow row;
    row.callback = std::stoi(r[1]);
    row.frame = std::stoi(r[2]);
    row.exposure = F(r[3]);
    row.temperature = F(r[4]);
    row.prev = std::stoi(r[5]);
    row.prev_path = r[6];
    out.push_back(row);
  }
  return out;
}

std::vector<uint8_t> ReadPgm(const std::string& path, int w, int h) {
  std::ifstream in(path, std::ios::binary);
  EXPECT_TRUE(in.good()) << path;
  std::string magic;
  int pw = 0, ph = 0, maxv = 0;
  in >> magic >> pw >> ph >> maxv;
  in.get();
  EXPECT_EQ(magic, "P5");
  EXPECT_EQ(pw, w);
  EXPECT_EQ(ph, h);
  EXPECT_EQ(maxv, 255);
  std::vector<uint8_t> px(static_cast<size_t>(w) * static_cast<size_t>(h));
  in.read(reinterpret_cast<char*>(px.data()), static_cast<std::streamsize>(px.size()));
  return px;
}

bool SameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

struct LightingDataset {
  const char* label;
  std::string tsv;
  int width;
  int height;
  size_t rows;                  // registered row count
  size_t registered_subset;     // registered subset size (i < 100 or i % 25 == 0)
  EngineConfig engine;
};

LightingDataset Api1() {
  LightingDataset d{"api1", kOracleDir + "/tsv-api1/lighting.tsv", 480, 640, 1009, 137, {}};
  d.engine.capture_width = 480;
  d.engine.capture_height = 640;
  d.engine.model = "K";
  d.engine.pyramid_model = ShaderNumericModel::kSwiftShaderApprox;
  return d;
}

LightingDataset Api3() {
  LightingDataset d{"api3", kOracleDir + "/tsv-api3/lighting.tsv", 240, 320, 600, 120, {}};
  d.engine.capture_width = 240;
  d.engine.capture_height = 320;
  d.engine.manufacturer = "Google";
  d.engine.model = "Pixel 7";
  d.engine.pyramid_model = ShaderNumericModel::kSwiftShaderApprox;
  return d;
}

// <dir of the row's frame files>/NNNNN.pgm; the directory is the one the extractor wrote.
std::string FramePath(const std::vector<LightRow>& rows, int frame) {
  const std::string& any = rows.front().prev_path;
  char name[32];
  std::snprintf(name, sizeof(name), "%05d.pgm", frame);
  return kRepoRoot + "/" + any.substr(0, any.rfind('/') + 1) + name;
}

// Callback numbers where the two exposure sequences differ bitwise (comparator of every gate).
std::vector<int> BitMismatches(const std::vector<float>& got, const std::vector<LightRow>& want) {
  std::vector<int> bad;
  EXPECT_EQ(got.size(), want.size());
  for (size_t i = 0; i < std::min(got.size(), want.size()); ++i) {
    if (!SameBits(got[i], want[i].exposure)) bad.push_back(want[i].callback);
  }
  return bad;
}

// All callback numbers unless `max` is given (the gates print every mismatch; only the mutation
// summaries are shortened).
std::string List(const std::vector<int>& v, size_t max = SIZE_MAX) {
  std::string s;
  for (size_t i = 0; i < v.size() && i < max; ++i) s += (i ? "," : "") + std::to_string(v[i]);
  if (v.size() > max) s += ",... (" + std::to_string(v.size()) + " total)";
  return s;
}

// Engine pipeline: the original's staged order (bootstrap = copy of frame 0, then the input frame
// of every fresh callback, shifted by `frame_shift` for the mutation) through GlueRunner + Engine
// with configure({enableLighting: true}). The glue pipeline decides which frame a response comes
// from (1-frame lag, startup re-publication at callback kInitialFramesUntilRecenter + 1, S4 §3.8).
struct PipelineRun {
  std::vector<float> exposure;
  int temperature_6500 = 0;
  int has_lighting = 0;
};

PipelineRun RunPipeline(const LightingDataset& d, const std::vector<LightRow>& rows, size_t n,
                        int frame_shift) {
  GlueRunner glue(d.engine, GlueConfig{});
  XrConfigureOptions o;
  o.enable_lighting = true;
  EXPECT_TRUE(glue.Configure(o).ok());
  int64_t t = 1000000000;
  std::vector<uint8_t> gray = ReadPgm(FramePath(rows, 0), d.width, d.height);
  glue.ProcessFrame(t, t, GrayImageView{gray.data(), d.width, d.height, d.width});  // bootstrap
  PipelineRun run;
  for (size_t k = 0; k < n; ++k) {
    t += 33000000;
    gray = ReadPgm(FramePath(rows, rows[k].frame + frame_shift), d.width, d.height);
    const PublishedFrame f =
        glue.ProcessFrame(t, t, GrayImageView{gray.data(), d.width, d.height, d.width});
    run.exposure.push_back(f.lighting.exposure);
    if (f.has_lighting) ++run.has_lighting;
    if (f.has_lighting && f.lighting.temperature == kWebLightingTemperature) ++run.temperature_6500;
  }
  return run;
}

void ExpectPipelineGate(const char* gate, const LightingDataset& d) {
  const std::vector<LightRow> rows = ReadLighting(d.tsv);
  ASSERT_EQ(rows.size(), d.rows);
  for (size_t i = 0; i < rows.size(); ++i) {
    ASSERT_EQ(rows[i].callback, static_cast<int>(i) + 1);
    ASSERT_EQ(rows[i].temperature, kWebLightingTemperature);
  }
  const PipelineRun run = RunPipeline(d, rows, rows.size(), 0);
  const std::vector<int> bad = BitMismatches(run.exposure, rows);
  const int checked = static_cast<int>(rows.size());
  std::printf("[%s %s] callbacks=%d bit_exact=%d temperature_6500=%d has_lighting=%d mismatches:[%s]\n",
              gate, d.label, checked, checked - static_cast<int>(bad.size()), run.temperature_6500,
              run.has_lighting, List(bad).c_str());
  EXPECT_TRUE(bad.empty()) << "mismatching callbacks: " << List(bad);
  EXPECT_EQ(run.temperature_6500, checked);
  EXPECT_EQ(run.has_lighting, checked);
}

// G-L2: engine pipeline, api1, all 1009 callbacks.
TEST(XrApiEvidence, LightingMatchesOriginalBitExact) {
  ExpectPipelineGate("G-L2", Api1());
}

// G-L3: engine pipeline, api3 (TUM 240x320: another pyramid layout), all 600 callbacks.
TEST(XrApiEvidence, LightingMatchesOriginalBitExactTum) {
  ExpectPipelineGate("G-L3", Api3());
}

// Registered association: exposure of the previous fresh callback's frame (variant 0) or, for the
// mutation M3, of the callback's own input frame (variant 1), on the registered subset.
struct RuleRun {
  int checked = 0;
  int exact = 0;
  std::vector<int> mismatches;
};

RuleRun RunRegisteredRule(const LightingDataset& d, const std::vector<LightRow>& rows,
                          bool own_frame) {
  frontend::PyramidSource source(d.width, d.height, frontend::ShaderParams{},
                                 ShaderNumericModel::kSwiftShaderApprox);
  std::map<int, float> cache;
  RuleRun run;
  for (size_t i = 0; i < rows.size(); ++i) {
    if (!(i < 100 || i % 25 == 0)) continue;  // registered subset
    const int frame = own_frame ? rows[i].frame : rows[i].prev;
    if (!cache.count(frame)) {
      const std::vector<uint8_t> gray = ReadPgm(FramePath(rows, frame), d.width, d.height);
      cache[frame] = EstimateExposure(source.Get(
          PyramidAtlasView{}, RgbaImageView{}, GrayImageView{gray.data(), d.width, d.height, d.width}));
    }
    ++run.checked;
    if (SameBits(rows[i].exposure, cache[frame])) {
      ++run.exact;
    } else {
      run.mismatches.push_back(rows[i].callback);
    }
  }
  return run;
}

// G-L1: the registered gate (every sampled callback bit-equal) is reported as registered. Its
// single known exception is the startup re-publication: the 5th processed frame rebuilds the core
// (kInitialFramesUntilRecenter), so callback kInitialFramesUntilRecenter + 1 repeats the 5th
// response instead of using the previous callback's frame. The test asserts that characterisation
// (sample count, mismatch set exactly {6}, callback 6 = callback 5 in the original); it does not
// turn the registered gate into a pass.
void ExpectRegisteredRuleCharacterisation(const char* gate, const LightingDataset& d) {
  const std::vector<LightRow> rows = ReadLighting(d.tsv);
  ASSERT_EQ(rows.size(), d.rows);
  for (size_t i = 0; i < rows.size(); ++i) ASSERT_EQ(rows[i].callback, static_cast<int>(i) + 1);
  ASSERT_EQ(rows[0].prev_path, FramePath(rows, rows[0].prev).substr(kRepoRoot.size() + 1))
      << "frame path convention of the extract";
  const RuleRun run = RunRegisteredRule(d, rows, false);
  std::printf("[%s %s] registered rule gate: checked=%d bit_exact=%d -> %s (registered gate needs "
              "every sampled callback); mismatches:[%s]\n",
              gate, d.label, run.checked, run.exact, run.exact == run.checked ? "PASS" : "FAIL",
              List(run.mismatches).c_str());
  EXPECT_EQ(static_cast<size_t>(run.checked), d.registered_subset);
  const int first_republished = kInitialFramesUntilRecenter + 1;
  EXPECT_EQ(run.mismatches, std::vector<int>{first_republished});
  // The original publishes the 5th response again at that callback.
  EXPECT_TRUE(SameBits(rows[static_cast<size_t>(first_republished - 1)].exposure,
                       rows[static_cast<size_t>(first_republished - 2)].exposure));
}

TEST(XrApiEvidence, LightingRegisteredAssociationReport) {
  ExpectRegisteredRuleCharacterisation("G-L1a", Api1());
}

TEST(XrApiEvidence, LightingRegisteredAssociationReportTum) {
  ExpectRegisteredRuleCharacterisation("G-L1b", Api3());
}

// ---- null-runs and mutations of the lighting gate (rm3a-lighting-gate.v1 §5) ----

// N1: the comparator applied to the original data against itself finds nothing.
TEST(XrApiEvidence, LightingGateNullRunComparatorSelfIdentity) {
  for (const LightingDataset& d : {Api1(), Api3()}) {
    const std::vector<LightRow> rows = ReadLighting(d.tsv);
    std::vector<float> same;
    for (const LightRow& r : rows) same.push_back(r.exposure);
    const std::vector<int> bad = BitMismatches(same, rows);
    std::printf("[N1 %s] rows=%zu mismatches=%zu\n", d.label, rows.size(), bad.size());
    EXPECT_EQ(rows.size(), d.rows);
    EXPECT_TRUE(bad.empty());
  }
}

// N2 / N2b: the engine pipeline is deterministic (same input, same bits), first 60 callbacks.
void ExpectPipelineDeterminism(const char* id, const LightingDataset& d) {
  const std::vector<LightRow> rows = ReadLighting(d.tsv);
  const PipelineRun a = RunPipeline(d, rows, 60, 0);
  const PipelineRun b = RunPipeline(d, rows, 60, 0);
  ASSERT_EQ(a.exposure.size(), 60u);
  int diff = 0;
  for (size_t i = 0; i < 60; ++i) diff += SameBits(a.exposure[i], b.exposure[i]) ? 0 : 1;
  std::printf("[%s %s] callbacks=60 differing_between_two_runs=%d\n", id, d.label, diff);
  EXPECT_EQ(diff, 0);
}

TEST(XrApiEvidence, LightingGateNullRunPipelineDeterminism) {
  ExpectPipelineDeterminism("N2", Api3());
}

TEST(XrApiEvidence, LightingGateNullRunPipelineDeterminismApi1) {
  ExpectPipelineDeterminism("N2b", Api1());
}

// M1: input frames shifted by one must be detected (api3 first 60 callbacks).
TEST(XrApiEvidence, LightingGateMutationShiftedInputFrames) {
  const LightingDataset d = Api3();
  const std::vector<LightRow> rows = ReadLighting(d.tsv);
  const PipelineRun shifted = RunPipeline(d, rows, 60, 1);
  const std::vector<LightRow> first60(rows.begin(), rows.begin() + 60);
  const std::vector<int> bad = BitMismatches(shifted.exposure, first60);
  std::printf("[M1 api3] frame+1 input: mismatching callbacks=%zu of 60 [%s]\n", bad.size(),
              List(bad, 12).c_str());
  EXPECT_GE(bad.size(), 1u);
}

// M2: one expected value moved by one ulp is pinpointed (api1).
TEST(XrApiEvidence, LightingGateMutationOneUlp) {
  const LightingDataset d = Api1();
  const std::vector<LightRow> rows = ReadLighting(d.tsv);
  std::vector<float> got;
  for (const LightRow& r : rows) got.push_back(r.exposure);
  const size_t at = 500;
  got[at] = std::nextafter(got[at], 1.0f);
  const std::vector<int> bad = BitMismatches(got, rows);
  std::printf("[M2 api1] one ulp at callback %d: mismatches=[%s]\n", rows[at].callback,
              List(bad).c_str());
  EXPECT_EQ(bad, std::vector<int>{rows[at].callback});
}

// M3 / M3b: the wrong association rule (own input frame instead of the previous callback's) must
// break the G-L1 characterisation (mismatch set != {6}).
void ExpectWrongRuleDetected(const char* id, const LightingDataset& d) {
  const std::vector<LightRow> rows = ReadLighting(d.tsv);
  const RuleRun run = RunRegisteredRule(d, rows, true);
  std::printf("[%s %s] own-frame rule: checked=%d mismatches=%zu\n", id, d.label, run.checked,
              run.mismatches.size());
  EXPECT_NE(run.mismatches, std::vector<int>{kInitialFramesUntilRecenter + 1});
  EXPECT_GT(run.mismatches.size(), 1u);
}

TEST(XrApiEvidence, LightingGateMutationWrongAssociationRule) {
  ExpectWrongRuleDetected("M3", Api1());
}

TEST(XrApiEvidence, LightingGateMutationWrongAssociationRuleTum) {
  ExpectWrongRuleDetected("M3b", Api3());
}

}  // namespace
}  // namespace slam_refact
