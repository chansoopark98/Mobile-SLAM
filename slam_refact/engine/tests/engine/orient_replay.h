// Exact-input replay of a run-orient.mjs recording of the ORIGINAL SDK through GlueRunner (US-023): shared by the REPORT driver
// (orient_replay_main.cc) and the pose gate (orient_gate_test.cc). Reads the extract_orient.py files of the recording
// (tsv/device.tsv, stages.tsv, raw-events.tsv, packets.tsv) and the served frames (served-NNNNN.pgm, run-orient.mjs --save-frames)
// and replays every stageFrame call of the original in order:
//   the DeviceMotion packets of the event queue set before the stage (timestamps = the handler times the original recorded),
//   the stage's deviceorientation (alpha, beta, gamma), the camera frame the original staged (CPU pyramid port,
//   swiftshader-approx; the pyramid gate P shows it equals the original GPU atlas), frame time = TrackingSensorFrame.timeNanos.

#ifndef SLAM_REFACT_TESTS_ENGINE_ORIENT_REPLAY_H_
#define SLAM_REFACT_TESTS_ENGINE_ORIENT_REPLAY_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "slam_refact/glue_runner.h"
#include "test_util.h"

namespace slam_refact::orient_replay {

struct TraceRow {
  int frame = 0;
  bool fresh = false;
  std::string status;
  float p[3] = {0, 0, 0};
  float q[4] = {1, 0, 0, 0};  // w, x, y, z
  int world_points = 0;
};

// Mutations of the replayed input (instrument checks of the pose gate); kNone for the gate itself.
enum class Mutation {
  kNone,
  kRotationPlus90,       // engineInit rotation +90 (orientation correction and pyramid turn wrong)
  kEventTimesPlus1Ms,    // every DeviceMotion receive time 1 ms later (a one-packet shift of the IMU binding at 60 Hz would be 16 ms)
  kSkipOnePacket,        // the DeviceMotion packet 7 of the stream dropped
  kOrientationAlphaPlusHalfDegree,  // deviceorientation alpha + 0.5 deg
};

struct Options {
  int rotation_override = -1000;
  Mutation mutation = Mutation::kNone;
  int max_frame = 1 << 30;  // stop after the stage of this input frame
};

struct Outcome {
  bool ok = false;
  std::string error;
  int skipped_samples = 0;
  int width = 0, height = 0, rotation = 0;
  std::string manufacturer, model;
  std::vector<TraceRow> rows;
};

inline std::vector<std::vector<std::string>> Tsv(const std::string& path, bool* ok) {
  std::string text;
  *ok = frontend::testing::ReadTextFile(path, &text);
  std::vector<std::vector<std::string>> rows;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> f;
    size_t p = 0;
    while (true) {
      const size_t q = line.find('\t', p);
      f.push_back(line.substr(p, q == std::string::npos ? std::string::npos : q - p));
      if (q == std::string::npos) break;
      p = q + 1;
    }
    rows.push_back(std::move(f));
  }
  return rows;
}

inline double Dbl(const std::string& s) { return std::strtod(s.c_str(), nullptr); }
inline uint32_t HexBits(const std::string& s) { return static_cast<uint32_t>(std::stoul(s, nullptr, 16)); }
inline float BitsF(uint32_t b) { float f; std::memcpy(&f, &b, 4); return f; }
inline uint32_t FBits(float f) { uint32_t b; std::memcpy(&b, &f, 4); return b; }

inline Outcome Replay(const std::string& rec, const Options& opt) {
  namespace fe = slam_refact::frontend;
  Outcome out;
  bool ok = false;
  std::map<std::string, std::string> device;
  for (const auto& r : Tsv(rec + "/tsv/device.tsv", &ok)) if (r.size() >= 2) device[r[0]] = r[1];
  if (!ok) { out.error = "device.tsv missing"; return out; }
  int w = 0, h = 0, rot = 0;
  {
    std::istringstream in(device.at("engine_init"));
    in >> w >> h >> rot;
  }
  if (opt.rotation_override != -1000) rot = opt.rotation_override;
  if (opt.mutation == Mutation::kRotationPlus90) rot = (rot + 90) % 360;
  struct Ev { int kind; int64_t ts, interval; uint32_t xb, yb, zb; };
  struct Packet { double interval_ms, ax, ay, az, alpha, beta, gamma; };
  std::vector<std::vector<Ev>> msgs;
  for (const auto& r : Tsv(rec + "/tsv/raw-events.tsv", &ok)) {
    const size_t m = static_cast<size_t>(std::stoi(r[0]));
    if (m >= msgs.size()) msgs.resize(m + 1);
    msgs[m].push_back({std::stoi(r[2]), std::stoll(r[3]), std::stoll(r[4]), HexBits(r[5]), HexBits(r[6]), HexBits(r[7])});
  }
  std::vector<Packet> packets;
  for (const auto& r : Tsv(rec + "/tsv/packets.tsv", &ok)) packets.push_back({Dbl(r[1]), Dbl(r[2]), Dbl(r[3]), Dbl(r[4]), Dbl(r[5]), Dbl(r[6]), Dbl(r[7])});
  const auto stages = Tsv(rec + "/tsv/stages.tsv", &ok);
  if (!ok) { out.error = "stages.tsv missing"; return out; }

  EngineConfig cfg;
  cfg.capture_width = w;
  cfg.capture_height = h;
  cfg.rotation_deg = rot;
  cfg.manufacturer = device.at("manufacturer");
  cfg.model = device.at("model");
  cfg.pyramid_model = ShaderNumericModel::kSwiftShaderApprox;
  GlueConfig glue_cfg;  // origin (0, 2, 0), startup recenter after 5 processed frames, as the page-runtime session
  GlueRunner glue(cfg, glue_cfg);
  CameraProjectionUpdate u;
  u.pixel_rect_width = std::stoi(device.at("served_width"));
  u.pixel_rect_height = std::stoi(device.at("served_height"));
  u.near_clip_plane = 0.01f;
  u.far_clip_plane = 1000.0f;
  u.origin = Vec3f{0.f, 2.f, 0.f};
  u.facing = QuatWxyz{1.f, 0.f, 0.f, 0.f};
  glue.UpdateCameraProjectionMatrix(u);

  size_t cursor = 0;
  int packet_no = 0;
  std::vector<uint8_t> gray0;
  auto load_frame = [&](int index, std::vector<uint8_t>* g) {
    char name[64];
    std::snprintf(name, sizeof(name), "/served-%05d.pgm", index);
    int fw = 0, fh = 0;
    if (!fe::testing::ReadPgm(rec + name, &fw, &fh, g)) { out.error = std::string("missing ") + name; return false; }
    if (fw != w || fh != h) { out.error = "served frame size differs from the engineInit size"; return false; }
    return true;
  };
  if (!load_frame(0, &gray0)) return out;
  for (const auto& st : stages) {
    const size_t msg = static_cast<size_t>(std::stoi(st[1]));
    const int frame = std::stoi(st[2]);
    if (frame > opt.max_frame) break;
    const double video_s = Dbl(st[3]), frame_s = Dbl(st[4]);
    const int64_t time_ns_arg = static_cast<int64_t>(Dbl(st[5]));
    if (msg < msgs.size()) {
      const std::vector<Ev>& ev = msgs[msg];
      for (size_t i = 0; i + 2 < ev.size(); i += 3) {
        const Ev &lin = ev[i], &acc = ev[i + 1], &gyr = ev[i + 2];
        DeviceMotionSample s;
        s.receive_time_ns = acc.ts + (opt.mutation == Mutation::kEventTimesPlus1Ms ? 1000000 : 0);
        s.acceleration = Vec3f{BitsF(lin.xb), BitsF(lin.yb), BitsF(lin.zb)};
        s.acceleration_including_gravity = Vec3f{BitsF(acc.xb), BitsF(acc.yb), BitsF(acc.zb)};
        s.rotation_rate_abg = Vec3f{BitsF(gyr.xb), BitsF(gyr.yb), BitsF(gyr.zb)};
        bool found = false;
        size_t best = cursor;
        for (size_t c = cursor; c < std::min(packets.size(), cursor + 3); ++c) {
          const Packet& p = packets[c];
          if (FBits(static_cast<float>(p.ax)) == acc.xb && FBits(static_cast<float>(p.ay)) == acc.yb && FBits(static_cast<float>(p.az)) == acc.zb &&
              FBits(static_cast<float>(p.alpha)) == gyr.xb && FBits(static_cast<float>(p.beta)) == gyr.yb && FBits(static_cast<float>(p.gamma)) == gyr.zb) {
            s.interval_ms = p.interval_ms;
            best = c;
            found = true;
          }
        }
        if (!found) { ++out.skipped_samples; continue; }
        cursor = best;
        if (opt.mutation == Mutation::kSkipOnePacket && packet_no++ == 7) continue;
        glue.PushDeviceMotion(s);
      }
    }
    DeviceOrientationSample o;
    o.alpha_deg = Dbl(st[6]) + (opt.mutation == Mutation::kOrientationAlphaPlusHalfDegree ? 0.5 : 0.0);
    o.beta_deg = Dbl(st[7]);
    o.gamma_deg = Dbl(st[8]);
    glue.PushDeviceOrientation(o);
    std::vector<uint8_t> gray;
    const std::vector<uint8_t>* g = &gray0;
    if (frame >= 0) {
      if (!load_frame(frame, &gray)) return out;
      g = &gray;
    }
    const int64_t frame_time_ns = static_cast<int64_t>(frame_s * 1e9);
    const PublishedFrame pf = glue.ProcessFrame(frame_time_ns != 0 ? frame_time_ns : time_ns_arg, static_cast<int64_t>(video_s * 1e9),
                                                GrayImageView{g->data(), w, h, w});
    if (frame < 0) continue;
    TraceRow t;
    t.frame = frame;
    t.fresh = !pf.repeated;
    t.status = pf.status == TrackingStatus::kNormal ? "NORMAL" : pf.status == TrackingStatus::kLimited ? "LIMITED"
               : pf.status == TrackingStatus::kNotAvailable ? "NOT_AVAILABLE" : "UNSPECIFIED";
    t.p[0] = pf.position.x; t.p[1] = pf.position.y; t.p[2] = pf.position.z;
    t.q[0] = pf.rotation.w; t.q[1] = pf.rotation.x; t.q[2] = pf.rotation.y; t.q[3] = pf.rotation.z;
    t.world_points = static_cast<int>(pf.world_points.size());
    out.rows.push_back(t);
  }
  out.ok = true;
  out.width = w; out.height = h; out.rotation = rot;
  out.manufacturer = cfg.manufacturer; out.model = cfg.model;
  return out;
}

}  // namespace slam_refact::orient_replay

#endif  // SLAM_REFACT_TESTS_ENGINE_ORIENT_REPLAY_H_
