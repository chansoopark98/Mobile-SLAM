// WP4 VioTracker gates (architecture.md §6.6.4): V-MP, V-XR, V-H1/H2, V-H3, V-ST, V-3DOF,
// V-LATCH, plus Q10 world-point padding, SetDisableVio partial reset and TrackDebug wiring.
// Oracles: tests/data/vio-tracker/golden-motion-prediction.json, probe-tail.json (spec lane S4)
// and wp4-tail-golden.json (same func784 WAT emulation, per-case vectors; gen_tail_golden.py).
// MapTracker / MapBuilder / frontend are test doubles behind the TrackerModules seam.

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "c8/geometry/device-pose.h"
#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "config/params.h"
#include "metric_scale/metric_scale_estimator.h"
#include "vio_tracker/tracker_modules.h"
#include "vio_tracker/vio_tracker.h"
#include "vio_tracker/vio_tracker_internal.h"
#include "../metric_scale/wp4_json.h"

namespace slam_refact::vio_tracker {
namespace {

const wp4_json::Value& Data(const std::string& name) {
  static std::map<std::string, wp4_json::Value> cache;
  auto it = cache.find(name);
  if (it == cache.end()) {
    it = cache.emplace(name, wp4_json::Load(std::string(SR_TEST_DATA_DIR) + "/vio-tracker/" + name)).first;
  }
  return it->second;
}

c8::HMatrix FromArrays(const wp4_json::Value& m, const wp4_json::Value& inv) {
  float a[16];
  float b[16];
  for (int i = 0; i < 16; ++i) {
    a[i] = m[i].f();
    b[i] = inv[i].f();
  }
  return c8::HMatrix(a, b);
}

c8::Quaternion QuatXyzw(const wp4_json::Value& q) {
  return c8::Quaternion(q[3].f(), q[0].f(), q[1].f(), q[2].f());
}

// wat_emu.camera_motion: T(p) R(q) in double, stored as f32 (forward and inverse).
c8::HMatrix CameraMotionF32(const wp4_json::Value& p, const wp4_json::Value& q) {
  const double x = q[0].d(), y = q[1].d(), z = q[2].d(), w = q[3].d();
  const double r[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                          {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                          {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
  const double t[3] = {p[0].d(), p[1].d(), p[2].d()};
  float m[16] = {0};
  float inv[16] = {0};
  for (int c = 0; c < 3; ++c) {
    for (int rr = 0; rr < 3; ++rr) {
      m[4 * c + rr] = static_cast<float>(r[rr][c]);
      inv[4 * c + rr] = static_cast<float>(r[c][rr]);
    }
  }
  for (int rr = 0; rr < 3; ++rr) {
    m[12 + rr] = static_cast<float>(t[rr]);
    inv[12 + rr] = static_cast<float>(-(r[0][rr] * t[0] + r[1][rr] * t[1] + r[2][rr] * t[2]));
  }
  m[15] = inv[15] = 1.f;
  return c8::HMatrix(m, inv);
}

void ExpectMatrixNear(const c8::HMatrix& got, const wp4_json::Value& fwd, const wp4_json::Value* inv,
                      double tol, const std::string& what) {
  for (int i = 0; i < 16; ++i) {
    EXPECT_LE(std::fabs(got.data()[i] - fwd[i].d()), tol) << what << " m[" << i << "]";
  }
  if (inv != nullptr) {
    const c8::HMatrix gi = got.inv();
    for (int i = 0; i < 16; ++i) {
      EXPECT_LE(std::fabs(gi.data()[i] - (*inv)[i].d()), tol) << what << " inv[" << i << "]";
    }
  }
}

// ---- test double of the collaborators ----
struct FakeModules : public TrackerModules {
  // What the doubles report.
  c8::HMatrix tam = c8::HMatrixGen::i();
  int state = 1;
  bool frame_tracker_ok = false;
  MapBuilderOutput next_mb;
  std::vector<VisibleMapPoint> visible;
  frontend::DetectStats stats{11, 4, 7};
  // Call counters.
  int detect_calls = 0, track_calls = 0, update_calls = 0, apply_calls = 0, recreate_calls = 0;
  c8::DetectionConfig last_cfg;
  int64_t last_frame_ms = 0;
  FrameState last_prediction;

  void Detect(const c8::Gr8Pyramid&, const c8::DetectionConfig& cfg, c8::FrameWithPoints*) override {
    ++detect_calls;
    last_cfg = cfg;
  }
  frontend::DetectStats detect_stats() const override { return stats; }
  void TrackMap(TrackerInput*, const FrameState& predicted, MapTrackerOutput* out,
                compat::RandomNumbers*) override {
    ++track_calls;
    last_prediction = predicted;
    out->state = state;
    out->frame_tracker_ok = frame_tracker_ok;
  }
  void UpdateMap(const TrackerInput&, const MapTrackerOutput&, int64_t frame_ms, MapBuilderOutput* out) override {
    ++update_calls;
    last_frame_ms = frame_ms;
    *out = next_mb;
  }
  c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput&) override {
    ++apply_calls;
    return tam;
  }
  void RecreateMapTrackerAndBuilder() override { ++recreate_calls; }
  void VisibleMapPoints(std::vector<VisibleMapPoint>* out) const override { *out = visible; }
  int ground_match_count() const override { return 23; }
  int num_keyframes() const override { return 5; }
  std::array<int, 5> num_points_by_source() const override { return {1, 2, 3, 4, 0}; }
};

struct Harness {
  explicit Harness(bool estimate_scale = false, bool disable_vio = false) {
    config::RegisterDefaults();
    auto f = std::make_unique<FakeModules>();
    fake = f.get();
    tracker = NewVioTrackerWithModules(estimate_scale, disable_vio, std::move(f));
    pyr.levels.push_back(c8::LevelLayout{});
  }
  c8::HMatrix Track(int64_t t_ns, c8::Quaternion device = c8::Quaternion(1, 0, 0, 0),
                    std::vector<TrackingSensorEvent> events = {}) {
    TrackingSensorFrame frame;
    frame.time_ns = t_ns;
    frame.device_pose = device;
    frame.sensor_events = std::move(events);
    return tracker->Track(frame, pyr, &rng, &status, &debug);
  }
  FakeModules* fake = nullptr;
  std::unique_ptr<VioTracker> tracker;
  c8::Gr8Pyramid pyr;
  compat::RandomNumbers rng;
  StatusOut status;
  TrackDebug debug;
};

// ---- V-MP: MotionPrediction, 200 cases, quaternion components and position <= 1e-5 ----
TEST(VioTrackerGolden, VMpMotionPrediction) {
  config::RegisterDefaults();
  const wp4_json::Value& cases = Data("golden-motion-prediction.json")["cases"];
  ASSERT_EQ(cases.size(), 200u);
  int default_cases = 0;
  for (size_t c = 0; c < cases.size(); ++c) {
    const wp4_json::Value& in = cases[c]["inputs"];
    const wp4_json::Value& out = cases[c]["outputs_emulated"];
    FrameState prev;
    prev.pose = CameraMotionF32(in["prev_cam_p"], in["prev_cam_q_xyzw"]);
    prev.velocity = c8::HVector3(in["prev_velocity"][0].f(), in["prev_velocity"][1].f(), in["prev_velocity"][2].f());
    prev.time_ns = in["t_prev_ns"].ll();
    prev.xr_device_rotation = QuatXyzw(in["prev_device_xr_xyzw"]);
    FrameRecord now;
    now.time_ns = in["t_cur_ns"].ll();
    now.xr_device_rotation = c8::xrRotationFromDeviceRotation(QuatXyzw(in["device_pose_raw_xyzw"]));
    c8::Quaternion q;
    const float gci = in["gravityCorrectedInterpolation"].f();
    const float drag = in["aboveHorizonDrag"].f();
    const FrameState pred = PredictMotionWithParams(prev, now, gci, drag, &q);
    const wp4_json::Value& gq = out["pred_q_xyzw"];
    const double dot = q.x() * gq[0].d() + q.y() * gq[1].d() + q.z() * gq[2].d() + q.w() * gq[3].d();
    const double sgn = dot < 0 ? -1.0 : 1.0;
    EXPECT_LE(std::fabs(sgn * q.x() - gq[0].d()), 1e-5) << c;
    EXPECT_LE(std::fabs(sgn * q.y() - gq[1].d()), 1e-5) << c;
    EXPECT_LE(std::fabs(sgn * q.z() - gq[2].d()), 1e-5) << c;
    EXPECT_LE(std::fabs(sgn * q.w() - gq[3].d()), 1e-5) << c;
    for (int i = 0; i < 3; ++i) {
      EXPECT_LE(std::fabs(pred.pose.data()[12 + i] - out["pred_p"][i].d()), 1e-5) << c;
    }
    for (int i = 0; i < 16; ++i) {
      EXPECT_LE(std::fabs(pred.pose.data()[i] - out["pred_matrix_colmajor"][i].d()), 1e-5) << c << " m" << i;
    }
    EXPECT_EQ(pred.time_ns, now.time_ns);
    if (gci == 0.0f && drag == 1.0f) {  // binary defaults: the public PredictMotion gives the same
      ++default_cases;
      const FrameState p2 = PredictMotion(prev, now);
      for (int i = 0; i < 16; ++i) EXPECT_EQ(p2.pose.data()[i], pred.pose.data()[i]) << c;
    }
  }
  EXPECT_EQ(default_cases, 50);
}

// ---- V-XR: xrRotationFromDeviceRotation <= 1e-6 ----
TEST(VioTrackerGolden, VXrRotation) {
  const wp4_json::Value& cases = Data("wp4-tail-golden.json")["xr_rotation"];
  ASSERT_EQ(cases.size(), 30u);
  for (size_t c = 0; c < cases.size(); ++c) {
    const c8::Quaternion q = c8::xrRotationFromDeviceRotation(QuatXyzw(cases[c]["device_pose_raw_xyzw"]));
    const wp4_json::Value& g = cases[c]["xr_xyzw"];
    EXPECT_LE(std::fabs(q.x() - g[0].d()), 1e-6) << c;
    EXPECT_LE(std::fabs(q.y() - g[1].d()), 1e-6) << c;
    EXPECT_LE(std::fabs(q.z() - g[2].d()), 1e-6) << c;
    EXPECT_LE(std::fabs(q.w() - g[3].d()), 1e-6) << c;
  }
}

// ---- V-H1 / V-H2: yaw offset on the first frame, Y*P output <= 1e-5 ----
TEST(VioTrackerGolden, VH1H2YawOffsetAndOutput) {
  const wp4_json::Value& cases = Data("wp4-tail-golden.json")["yaw_output"];
  ASSERT_EQ(cases.size(), 30u);
  for (size_t c = 0; c < cases.size(); ++c) {
    Harness h;
    h.fake->tam = FromArrays(cases[c]["tam1"], cases[c]["tam1_inv"]);
    const c8::HMatrix out1 = h.Track(1050000000);
    ExpectMatrixNear(out1, cases[c]["out1"], nullptr, 1e-5, "H1 out case " + std::to_string(c));
    ExpectMatrixNear(VioTrackerAccess::state(*h.tracker).yaw_offset, cases[c]["yaw_offset"],
                     &cases[c]["yaw_offset_inv"], 1e-5, "H1 Y case " + std::to_string(c));
    EXPECT_TRUE(h.tracker->yaw_offset_initialized());
    h.fake->tam = FromArrays(cases[c]["tam2"], cases[c]["tam2_inv"]);
    const c8::HMatrix out2 = h.Track(1100000000);
    ExpectMatrixNear(out2, cases[c]["out2"], nullptr, 1e-5, "H2 out case " + std::to_string(c));
  }
}

// ---- V-H3: velocity / acceleration / history (relative <= 1e-5, incl. H3b) ----
TEST(VioTrackerGolden, VH3History) {
  const wp4_json::Value& cases = Data("wp4-tail-golden.json")["velocity"];
  ASSERT_EQ(cases.size(), 30u);
  auto rel = [](double a, double b) { return std::fabs(a - b) / std::max(1.0, std::fabs(b)); };
  for (size_t c = 0; c < cases.size(); ++c) {
    const wp4_json::Value& g = cases[c];
    for (int jump = 0; jump < 2; ++jump) {
      FrameState hist[2];
      hist[1].pose = FromArrays(g["prev_pose"], g["prev_pose_inv"]);
      hist[1].velocity = c8::HVector3(g["v_prev"][0].f(), g["v_prev"][1].f(), g["v_prev"][2].f());
      hist[1].time_ns = g["t_prev"].ll();
      FrameRecord now;
      now.time_ns = g["t_cur"].ll();
      UpdateHistory(hist, FromArrays(g["pose"], g["pose_inv"]), now, jump != 0);
      const wp4_json::Value& e = g[jump ? "jump" : "normal"];
      const float v[3] = {hist[1].velocity.x(), hist[1].velocity.y(), hist[1].velocity.z()};
      const float a[3] = {hist[1].acceleration.x(), hist[1].acceleration.y(), hist[1].acceleration.z()};
      for (int i = 0; i < 3; ++i) {
        EXPECT_LE(rel(v[i], e["velocity"][i].d()), 1e-5) << c << " jump " << jump;
        EXPECT_LE(rel(a[i], e["acceleration"][i].d()), 1e-5) << c << " jump " << jump;
      }
      for (int i = 0; i < 16; ++i) {
        EXPECT_LE(rel(hist[1].pose.data()[i], e["hist1_pose"][i].d()), 1e-5) << c << " jump " << jump;
      }
      EXPECT_EQ(hist[0].time_ns, g["t_prev"].ll());
      EXPECT_EQ(hist[1].time_ns, g["t_cur"].ll());
    }
  }
}

// ---- V-ST: status_table (100 rows) exact ----
TEST(VioTrackerGolden, VStStatusTable) {
  const wp4_json::Value& rows = Data("probe-tail.json")["status_table"];
  ASSERT_EQ(rows.size(), 100u);
  const int64_t now = 10050000000LL;
  for (size_t i = 0; i < rows.size(); ++i) {
    const wp4_json::Value& r = rows[i];
    LossStateMachine m;
    LossStateMachineAccess::Set(&m, static_cast<int>(r["state_in"].ll()), static_cast<int>(r["lost_in"].ll()),
                                static_cast<int>(r["frames_in"].ll()), now - r["since_state_ns"].ll());
    m.Update(r["ra"].ll() != 0, now);
    EXPECT_EQ(m.state(), r["state_out"].ll()) << i;
    EXPECT_EQ(m.lost_frames(), r["lost_out"].ll()) << i;
    EXPECT_EQ(m.frames(), r["frames_out"].ll()) << i;
    const StatusOut st = ComposeStatus(StatusFlags{}, m.state());
    EXPECT_EQ(static_cast<int>(st.status), r["status"].ll()) << i;
    EXPECT_EQ(static_cast<int>(st.reason), r["reason"].ll()) << i;
    EXPECT_EQ(m.status(), st.status) << i;
  }
}

// Exact boundaries of the WAT constants (2e9 ns with >=, ratio 0.2f with <).
TEST(VioTracker, LossStateBoundaries) {
  LossStateMachine m;
  LossStateMachineAccess::Set(&m, 1, 0, 3, 0);
  m.Update(true, 1999999999);
  EXPECT_EQ(m.state(), 1);
  LossStateMachineAccess::Set(&m, 1, 0, 3, 0);
  m.Update(true, 2000000000);
  EXPECT_EQ(m.state(), 0);
  LossStateMachineAccess::Set(&m, 3, 1, 4, 0);  // after update 1/5 = 0.2 -> not < 0.2 -> 2
  m.Update(true, 2000000000);
  EXPECT_EQ(m.state(), 2);
  EXPECT_EQ(LossStateMachineAccess::enter_ns(m), 2000000000);
}

// ---- V-ST: flags_table (12 rows) exact, through Track (ok = MapTracker TRACKING) ----
TEST(VioTrackerGolden, VStFlagsTable) {
  const wp4_json::Value& rows = Data("probe-tail.json")["flags_table"];
  ASSERT_EQ(rows.size(), 12u);
  for (size_t i = 0; i < rows.size(); ++i) {
    const wp4_json::Value& r = rows[i];
    Harness h;
    StatusFlags& f = VioTrackerAccess::state(*h.tracker).flags;
    const bool set = r["set"].ll() != 0;
    const bool ack = r["ack"].ll() != 0;
    switch (r["flag"].ll()) {
      case 6376: f.scale_pending = set; f.scale_ready = ack; break;
      case 6378: f.vps_map_pending = set; f.map_built = ack; break;
      default: f.wait_first_ok = set; f.got_first_ok = ack; break;
    }
    h.Track(10050000000LL);
    EXPECT_EQ(static_cast<int>(h.status.status), r["status"].ll()) << i;
    EXPECT_EQ(static_cast<int>(h.status.reason), r["reason"].ll()) << i;
  }
}

// ---- V-3DOF: disableVio rotation <= 1e-5, position exactly 0 ----
TEST(VioTrackerGolden, V3DofDisableVio) {
  const wp4_json::Value& cases = Data("wp4-tail-golden.json")["disable_vio"];
  ASSERT_EQ(cases.size(), 30u);
  for (size_t c = 0; c < cases.size(); ++c) {
    Harness h(false, true);
    const c8::HMatrix out = h.Track(1000000000, QuatXyzw(cases[c]["device_pose_raw_xyzw"]));
    const c8::HMatrix& tam = h.tracker->tam_extrinsic();
    const c8::HMatrix tinv = tam.inv();
    for (int i : {0, 1, 2, 4, 5, 6, 8, 9, 10}) {
      EXPECT_LE(std::fabs(tam.data()[i] - cases[c]["tam"][i].d()), 1e-5) << c;
      EXPECT_LE(std::fabs(tinv.data()[i] - cases[c]["tam_inv"][i].d()), 1e-5) << c;
    }
    for (int i : {12, 13, 14}) {
      EXPECT_EQ(tam.data()[i], 0.0f) << c;
      EXPECT_EQ(tinv.data()[i], 0.0f) << c;
      EXPECT_EQ(out.data()[i], 0.0f) << c;
    }
    EXPECT_EQ(h.fake->track_calls, 0);
    EXPECT_EQ(h.fake->update_calls, 0);
    EXPECT_EQ(h.fake->detect_calls, 1);  // detection still runs (S4 §3.1 D before E)
  }
  // disableVio from the first frame: ok stays false -> LIMITED after 2 s (S4 §3.12, INFERRED).
  Harness h(false, true);
  int64_t t = 1000000000;
  for (int k = 0; k < 80; ++k, t += 50000000) h.Track(t);
  EXPECT_EQ(h.status.status, TrackingStatus::kLimited);
}

// ---- V-LATCH: metric latch <= 1e-5 (Y re-alignment, M, output) ----
TEST(VioTrackerGolden, VLatchMetricTransform) {
  const wp4_json::Value& cases = Data("wp4-tail-golden.json")["metric_latch"];
  ASSERT_EQ(cases.size(), 10u);
  for (size_t c = 0; c < cases.size(); ++c) {
    const wp4_json::Value& g = cases[c];
    const c8::HMatrix tam = FromArrays(g["tam"], g["tam_inv"]);
    const float s = g["mse_scale"].f();
    EXPECT_EQ(s, g["latched_scale"].f()) << c;
    const c8::HMatrix y = YawOffsetFromPose(tam);
    ExpectMatrixNear(y, g["yaw_offset"], &g["yaw_offset_inv"], 1e-5, "Y " + std::to_string(c));
    const c8::HMatrix m = MetricTransform(s, y, tam);
    ExpectMatrixNear(m, g["metric"], &g["metric_inv"], 1e-5, "M " + std::to_string(c));
    ExpectMatrixNear(MetricOutput(m, tam), g["out"], nullptr, 1e-5, "out " + std::to_string(c));
    EXPECT_EQ(g["status"][0].ll(), 3);
  }
}

// estimateScale through VioTracker with the real MSE on the MS-P8 scenario: INITIALIZING until the
// latch frame (= MSE first accept, 253), latch once, metric output afterwards (Q11).
TEST(VioTracker, MetricLatchThroughTrack) {
  Harness h(true, false);
  const int64_t base = 5000000000LL;
  int64_t imu_k = 0;
  int latch_frame = -1;
  for (int k = 0; k < 300; ++k) {
    const double tf = k / 30.0;
    std::vector<TrackingSensorEvent> ev;
    while (imu_k / 60.0 <= tf + 1e-9 && ev.size() < 63) {
      const double ti = imu_k / 60.0;
      const double th = 0.3 * std::sin(0.7 * ti);
      const float r[9] = {static_cast<float>(std::cos(th)), 0.f, static_cast<float>(std::sin(th)), 0.f, 1.f, 0.f,
                          static_cast<float>(-std::sin(th)), 0.f, static_cast<float>(std::cos(th))};
      const double am[3] = {0.5 * -1.69 * std::sin(1.3 * ti) * 0.8, 0.5 * -4.41 * 0.3 * std::sin(2.1 * ti + 0.4),
                            0.5 * -0.81 * std::cos(0.9 * ti) * 0.6};
      const int64_t tns = base + static_cast<int64_t>(std::floor(ti * 1e9 + 0.5));
      ImuEvent lin{ImuEvent::Kind::kLinearAcceleration, tns, 16666667, 0, 0, 0};
      lin.x = static_cast<float>(r[0] * am[0] + r[3] * am[1] + r[6] * am[2]);
      lin.y = static_cast<float>(r[1] * am[0] + r[4] * am[1] + r[7] * am[2]);
      lin.z = static_cast<float>(r[2] * am[0] + r[5] * am[1] + r[8] * am[2]);
      ev.push_back(lin);
      ev.push_back(ImuEvent{ImuEvent::Kind::kGyroscope, tns, 16666667, 0.f,
                            static_cast<float>(0.21 * std::cos(0.7 * ti) * 180 / M_PI), 0.f});
      ++imu_k;
    }
    const double th = 0.3 * std::sin(0.7 * tf);
    const float c = static_cast<float>(std::cos(th));
    const float s = static_cast<float>(std::sin(th));
    const float p[3] = {static_cast<float>(std::sin(1.3 * tf) * 0.8), static_cast<float>(0.3 * std::sin(2.1 * tf + 0.4)),
                        static_cast<float>(std::cos(0.9 * tf) * 0.6)};
    const float m[16] = {c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, p[0], p[1], p[2], 1};
    const float inv[16] = {c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0,
                           static_cast<float>(-(static_cast<double>(c) * p[0] + -static_cast<double>(s) * p[2])),
                           -p[1],
                           static_cast<float>(-(static_cast<double>(s) * p[0] + static_cast<double>(c) * p[2])), 1};
    h.fake->tam = c8::HMatrix(m, inv);
    const c8::HMatrix out = h.Track(base + static_cast<int64_t>(std::floor(tf * 1e9 + 0.5)), c8::Quaternion(1, 0, 0, 0),
                                    std::move(ev));
    if (!h.tracker->scale_latched()) {
      EXPECT_EQ(h.status.status, TrackingStatus::kLimited) << k;
      EXPECT_EQ(h.status.reason, TrackingReason::kInitializing) << k;
    } else {
      if (latch_frame < 0) {
        latch_frame = k;
        EXPECT_NEAR(h.tracker->latched_scale(), 0.5037921667098999, 1e-3 * 0.5037921667098999);
        EXPECT_EQ(out.data()[12], 0.0f);  // latch frame: camera xz moved to the origin
        EXPECT_EQ(out.data()[14], 0.0f);
      }
      EXPECT_EQ(h.status.reason, TrackingReason::kUnspecified) << k;
      const auto& st = VioTrackerAccess::state(*h.tracker);
      const c8::HMatrix expect = MetricOutput(st.metric_transform, st.tam_extrinsic);
      for (int i = 0; i < 16; ++i) EXPECT_EQ(out.data()[i], expect.data()[i]) << k;
      EXPECT_TRUE(h.debug.scale_latched);
    }
  }
  EXPECT_EQ(latch_frame, 253);
  // map_reset releases the latch and resets M (D:85651-85667).
  h.fake->next_mb.map_reset = true;
  h.Track(base + 400000000000LL);
  EXPECT_FALSE(h.tracker->scale_latched());
  EXPECT_EQ(h.tracker->latched_scale(), 0.0f);
}

// ---- Q10: world points sized by the unfiltered count, 0-padding ----
VisibleMapPoint Pt(uint32_t id, map::PointState st, map::PointSource src, float x) {
  VisibleMapPoint p;
  p.key_id = id;
  p.point_id = id;
  p.state = st;
  p.source = src;
  p.position = c8::HPoint3(x, 2.f * x, 3.f);
  p.tracking_hits = 3;
  p.tracking_count = 4;
  return p;
}

// S2 review LOW-2: a limit < 1 would never end the GetWorldPoints stride search.
TEST(VioTrackerDeathTest, MaxPointsBelowOneAborts) {
  Harness h;
  h.Track(1000000000);
  std::vector<WorldPointOut> out;
  EXPECT_DEATH(
      {
        config::ApplyParamOverrides({{"MapTracker.maxPoints", ParamValue(0)}});
        h.tracker->GetWorldPoints(&out);
      },
      "maxPoints must be >= 1");
}

TEST(VioTracker, Q10WorldPointPadding) {
  Harness h;
  h.Track(1000000000);  // Y = I for an identity tam
  using S = map::PointState;
  using P = map::PointSource;
  h.fake->visible = {Pt(7, S::kTenured, P::kTriangulated, 1.f), Pt(8, S::kDead, P::kTriangulated, 2.f),
                     Pt(9, S::kNew, P::kGround, 3.f), Pt(10, S::kNew, P::kTriangulated, 4.f)};
  std::vector<WorldPointOut> out;
  h.tracker->GetWorldPoints(&out);
  ASSERT_EQ(out.size(), 4u);
  EXPECT_EQ(out[0].id, 7u);
  EXPECT_FLOAT_EQ(out[0].position.x, 1.f);
  EXPECT_FLOAT_EQ(out[0].position.y, 2.f);
  EXPECT_FLOAT_EQ(out[0].confidence, 0.75f);
  EXPECT_EQ(out[1].id, 10u);
  for (size_t i = 2; i < 4; ++i) {
    EXPECT_EQ(out[i].id, 0u);
    EXPECT_EQ(out[i].position.x, 0.f);
    EXPECT_EQ(out[i].position.y, 0.f);
    EXPECT_EQ(out[i].position.z, 0.f);
    EXPECT_EQ(out[i].confidence, 0.f);
  }
  // More than MapTracker.maxPoints (400): stride on the id, exact count, no padding.
  h.fake->visible.clear();
  for (uint32_t id = 1; id <= 900; ++id) h.fake->visible.push_back(Pt(id, S::kNew, P::kTriangulated, 1.f));
  h.tracker->GetWorldPoints(&out);
  ASSERT_EQ(out.size(), 300u);  // stride 3: ids 3, 6, ..., 900
  EXPECT_EQ(out.front().id, 3u);
  EXPECT_EQ(out.back().id, 900u);
}

// ---- SetDisableVio (476): only turning it on recreates MapTracker + MapBuilder ----
TEST(VioTracker, SetDisableVioPartialReset) {
  Harness h;
  h.tracker->SetDisableVio(false);
  EXPECT_EQ(h.fake->recreate_calls, 0);
  h.tracker->SetDisableVio(true);
  EXPECT_EQ(h.fake->recreate_calls, 1);
  h.Track(1000000000);
  EXPECT_EQ(h.fake->track_calls, 0);
  h.tracker->SetDisableVio(true);
  EXPECT_EQ(h.fake->recreate_calls, 2);
  h.tracker->SetDisableVio(false);
  EXPECT_EQ(h.fake->recreate_calls, 2);
  h.Track(1050000000);
  EXPECT_EQ(h.fake->track_calls, 1);
}

// ---- 784 wiring: DetectionConfig, frame ms, prediction input, TrackDebug sources ----
TEST(VioTracker, TrackWiringAndDebug) {
  Harness h;
  h.fake->state = 0;
  h.fake->frame_tracker_ok = true;
  h.fake->next_mb.keyframe_created = true;
  h.Track(1234567890123LL);
  EXPECT_EQ(h.fake->last_cfg.nKeypoints, 500);
  EXPECT_TRUE(h.fake->last_cfg.allGorb);
  EXPECT_FALSE(h.fake->last_cfg.allOrb);
  EXPECT_EQ(h.fake->last_cfg.gravityPortions[c8::DescriptorType::ORB], 0xC0);
  EXPECT_EQ(h.fake->last_frame_ms, 1234567);
  EXPECT_EQ(h.fake->last_prediction.time_ns, 1234567890123LL);
  EXPECT_EQ(h.debug.map_tracker_state, 0);
  EXPECT_TRUE(h.debug.frame_tracker_ok);
  EXPECT_TRUE(h.debug.ok);  // ok = state == 1 || frameTrackerOk
  EXPECT_TRUE(h.debug.keyframe_created);
  EXPECT_EQ(h.debug.frames, 1);
  EXPECT_EQ(h.debug.num_keypoints, 11);
  EXPECT_EQ(h.debug.num_orb, 4);
  EXPECT_EQ(h.debug.num_gorb, 7);
  EXPECT_EQ(h.debug.num_keyframes, 5);
  EXPECT_EQ(h.debug.num_points_by_source[3], 4);
  EXPECT_EQ(h.debug.ground_matches, 23);
  EXPECT_TRUE(h.debug.yaw_offset_initialized);
  for (int i = 0; i < 16; ++i) {
    EXPECT_EQ(h.debug.predicted_pose.data()[i], h.tracker->last_prediction().pose.data()[i]);
  }
  EXPECT_EQ(h.status.status, TrackingStatus::kNormal);
  EXPECT_EQ(h.status.reason, TrackingReason::kUnspecified);
  // First frame prediction: hist[1] initial (I, v = 0, identity quaternion) -> rotation = xr
  // device rotation, position 0 (S4 §3.2).
  const c8::HMatrix& pp = h.tracker->last_prediction().pose;
  EXPECT_EQ(pp.data()[12], 0.f);
  EXPECT_EQ(pp.data()[13], 0.f);
  EXPECT_EQ(pp.data()[14], 0.f);
}

}  // namespace
}  // namespace slam_refact::vio_tracker
