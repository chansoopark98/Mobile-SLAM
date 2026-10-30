// US-023 / US-024 gates against recordings of the ORIGINAL SDK (preregistered in
// build/slam-refact-full-20261019/preregistration/orient-us024-gate.v<N>.md and orient-us023-gate.v<N>.md; protocol
// slam_refact/docs/design/preregistration-protocol.md). The held gate does not name its version: the caller designates it
// (SLAM_REFACT_ORIENT_GATE=<gate_id>.v<N>). One source, three targets:
//
//   sr_tests_orient_gate          label unit      instrument checks only: null-runs and mutation detection on a
//                                                 synthetic recording that the test builds itself (no original data).
//                                                 With SLAM_REFACT_ORIENT_REPORT_DIR=<recording dir> it also prints the
//                                                 comparison of that recording as a REPORT (never for the registered v2-*
//                                                 recordings: refused).
//   sr_tests_orient_gate_held     label held      the gates: engine vs the registered recordings. Fails closed (HELD)
//                                                 until the manifest records the coordinator-designated review and
//                                                 SLAM_REFACT_ORIENT_REVIEWER equals the sha256 of the registered
//                                                 designation file (shared device sr_held; protocol §12.5). Environment:
//                                                 SLAM_REFACT_ORIENT_GATE (required), SLAM_REFACT_ORIENT_REVIEWER (required),
//                                                 SLAM_REFACT_ORIENT_PREREG_DIR / SLAM_REFACT_ORIENT_REC_DIR (optional;
//                                                 default: the directories compiled in by CMake).
//
// The comparators (identical in all modes):
//   S  sensors      raw capnp event queue of every stage -> engine chain (GlueRunner: DeviceMotion samples -> SensorIntake
//                   -> func 647 port, is_ios from the manufacturer) vs the sorted frame->sensorEvents the ORIGINAL func 647
//                   produced (probe build, tools/re/runtime/orient/instrument_647_wat.py). Bit-exact (exact-class).
//   S' sensors647   the same with the decoded raw events passed straight to frontend::PrepareTrackingSensorEvents.
//   K  intrinsics   reality.intrinsics of the original (16 floats) vs GlueRunner::PublishedFrame::intrinsics for the
//                   recorded device, capture size, rotation and graphics size. Bit-exact.
//   P  pyramid      the CPU shader port (swiftshader-approx) on the served frame vs the original GPU atlas readback, all
//                   four channels of every level rectangle. Bit-exact.

#include <gtest/gtest.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "frontend/frontend.h"
#include "frontend/pyramid_source.h"
#include "../support/sr_held.h"
#include "frontend/sensor_intake.h"
#include "orient_replay.h"
#include "slam_refact/glue_runner.h"
#include "test_util.h"

namespace slam_refact {
namespace {

namespace fe = frontend;
using fe::testing::Hex;
using fe::testing::ReadGzipFile;
using fe::testing::ReadPgm;
using fe::testing::ReadTextFile;
using fe::testing::Sha256;

// ------------------------------------------------------------------------------------------------
// Recording (the TSV files of extract_orient.py)

struct RawEv {
  int msg = 0;
  int kind = 0;  // capnp numbering: accel 1, gyro 2, mag 3, lin 4
  int64_t ts = 0;
  int64_t interval = 0;
  uint32_t xb = 0, yb = 0, zb = 0;
};
struct OutEv {
  int call = 0;
  int kind = 0;  // post-647 numbering
  int64_t ts = 0;
  int64_t interval = 0;
  uint32_t xb = 0, yb = 0, zb = 0;
};
struct Packet {
  double interval_ms = 0, ax = 0, ay = 0, az = 0, alpha = 0, beta = 0, gamma = 0;
};
struct IntrinsicsRow {
  int tex_w = 0, tex_h = 0, orientation = 0;
  std::array<uint32_t, 16> bits{};
};
struct CaptureRow {
  int capture = 0, input_frame = -1, rotation = 0, cam_w = 0, cam_h = 0;
  std::string raw_sha256, served;
};
struct Recording {
  std::string dir;
  std::map<std::string, std::string> device;
  std::vector<std::vector<RawEv>> msgs;  // per capnp event queue message
  std::vector<std::vector<OutEv>> calls;  // per func 647 call
  std::vector<Packet> packets;
  std::vector<IntrinsicsRow> intrinsics;
  std::vector<CaptureRow> captures;
};

double Dbl(const std::string& s) { return std::strtod(s.c_str(), nullptr); }  // std::stod throws on the denormal -1e-320

uint32_t ParseHex(const std::string& s) { return static_cast<uint32_t>(std::stoul(s, nullptr, 16)); }

float BitsToFloat(uint32_t b) {
  float f;
  std::memcpy(&f, &b, sizeof(f));
  return f;
}
uint32_t FloatToBits(float f) {
  uint32_t b;
  std::memcpy(&b, &f, sizeof(b));
  return b;
}

std::vector<std::vector<std::string>> ReadTsv(const std::string& path, bool* ok) {
  std::string text;
  *ok = ReadTextFile(path, &text);
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

bool LoadRecording(const std::string& dir, Recording* rec, std::string* err) {
  rec->dir = dir;
  const std::string t = dir + "/tsv/";
  bool ok = false;
  for (const auto& r : ReadTsv(t + "device.tsv", &ok)) {
    if (r.size() >= 2) rec->device[r[0]] = r[1];
  }
  if (!ok) { *err = "device.tsv missing in " + dir; return false; }
  for (const auto& r : ReadTsv(t + "raw-events.tsv", &ok)) {
    RawEv e;
    e.msg = std::stoi(r[0]);
    e.kind = std::stoi(r[2]);
    e.ts = std::stoll(r[3]);
    e.interval = std::stoll(r[4]);
    e.xb = ParseHex(r[5]);
    e.yb = ParseHex(r[6]);
    e.zb = ParseHex(r[7]);
    if (static_cast<size_t>(e.msg) >= rec->msgs.size()) rec->msgs.resize(static_cast<size_t>(e.msg) + 1);
    rec->msgs[static_cast<size_t>(e.msg)].push_back(e);
  }
  if (!ok) { *err = "raw-events.tsv missing"; return false; }
  for (const auto& r : ReadTsv(t + "ev647.tsv", &ok)) {
    const int call = std::stoi(r[0]);
    if (static_cast<size_t>(call) >= rec->calls.size()) rec->calls.resize(static_cast<size_t>(call) + 1);
    if (std::stoi(r[1]) < 0) continue;  // a call without events
    OutEv e;
    e.call = call;
    e.kind = std::stoi(r[2]);
    e.ts = std::stoll(r[3]);
    e.interval = std::stoll(r[4]);
    e.xb = ParseHex(r[5]);
    e.yb = ParseHex(r[6]);
    e.zb = ParseHex(r[7]);
    rec->calls[static_cast<size_t>(call)].push_back(e);
  }
  if (!ok) { *err = "ev647.tsv missing"; return false; }
  for (const auto& r : ReadTsv(t + "packets.tsv", &ok)) {
    rec->packets.push_back({Dbl(r[1]), Dbl(r[2]), Dbl(r[3]), Dbl(r[4]), Dbl(r[5]), Dbl(r[6]), Dbl(r[7])});
  }
  for (const auto& r : ReadTsv(t + "intrinsics.tsv", &ok)) {
    IntrinsicsRow row;
    row.tex_w = std::stoi(r[0]);
    row.tex_h = std::stoi(r[1]);
    row.orientation = std::stoi(r[2]);
    for (size_t k = 0; k < 16; ++k) row.bits[k] = ParseHex(r[3 + k]);
    rec->intrinsics.push_back(row);
  }
  for (const auto& r : ReadTsv(t + "captures.tsv", &ok)) {
    CaptureRow c;
    c.capture = std::stoi(r[0]);
    c.input_frame = r[1] == "null" ? -1 : std::stoi(r[1]);
    c.rotation = std::stoi(r[2]);
    c.cam_w = std::stoi(r[3]);
    c.cam_h = std::stoi(r[4]);
    c.raw_sha256 = r[5];
    c.served = r[6];
    rec->captures.push_back(c);
  }
  return true;
}

// ------------------------------------------------------------------------------------------------
// Comparison results

struct Result {
  bool pass = true;
  int compared = 0;       // units compared (events / matrix floats / pixels, see detail)
  int mismatches = 0;
  std::string detail;
  void Fail(const std::string& why) {
    pass = false;
    if (detail.size() < 2000) detail += why + "\n";
  }
};

// ------------------------------------------------------------------------------------------------
// S, S': sensors

using EventList = std::vector<ImuEvent>;

bool SameEvent(const ImuEvent& e, const OutEv& o) {
  return static_cast<int>(e.kind) == o.kind && e.event_time_ns == o.ts && e.interval_ns == o.interval &&
         FloatToBits(e.x) == o.xb && FloatToBits(e.y) == o.yb && FloatToBits(e.z) == o.zb;
}
bool SameList(const EventList& e, const std::vector<OutEv>& o) {
  if (e.size() != o.size()) return false;
  for (size_t i = 0; i < e.size(); ++i) {
    if (!SameEvent(e[i], o[i])) return false;
  }
  return true;
}

// Mutations of the engine side, applied to the unit under test (never to the comparator). The null run is kNone.
enum class SensorMutation {
  kNone,
  kIosTreatedAsAndroid,   // manufacturer "" instead of the recorded one
  kAndroidTreatedAsIos,   // manufacturer "Apple" instead of the recorded one
  kIntervalNotScaled,     // iOS interval x1000 missing: interval / 1000 on every event
  kWrongAxisSign,         // accelerometer / linear acceleration: all three signs flipped
  kOneUlpOnOneEvent,      // x of event 0 of message 3 moved by one ulp
  kDropOneEvent,          // the last event of message 3 removed
  kSwapEqualTimeEvents,   // events 0 and 1 of message 3 exchanged (same time, different kind)
  kZeroSignFlip,          // the sign bit of the first negated zero component (0x80000000) of message 3 cleared: the engine would say +0
};

// Mutation helper: the first component equal to -0 (0x80000000) becomes +0.
void ClearFirstNegatedZero(EventList* l) {
  for (ImuEvent& e : *l) {
    for (float* c : {&e.x, &e.y, &e.z}) {
      if (FloatToBits(*c) == 0x80000000u) {
        *c = 0.0f;
        return;
      }
    }
  }
}

struct SensorRun {
  std::vector<EventList> per_message;  // engine events per queue message
};

// Reconstructs the DeviceMotion samples of message m from its raw events (push order lin, accel, gyro of one
// dispatch) and the dispatched packets (the interval number). Returns false with a reason when the queue does not
// follow the sensors.ts push pattern.
bool SamplesOfMessage(const Recording& rec, size_t m, size_t* packet_cursor, std::vector<DeviceMotionSample>* out,
                      std::string* why) {
  const std::vector<RawEv>& ev = rec.msgs[m];
  if (ev.size() % 3 != 0) { *why = "message " + std::to_string(m) + ": event count not a multiple of 3"; return false; }
  for (size_t i = 0; i < ev.size(); i += 3) {
    const RawEv &lin = ev[i], &acc = ev[i + 1], &gyr = ev[i + 2];
    if (lin.kind != 4 || acc.kind != 1 || gyr.kind != 2 || lin.ts != acc.ts || acc.ts != gyr.ts) {
      *why = "message " + std::to_string(m) + ": events do not follow the push order linear acceleration, accelerometer, gyroscope";
      return false;
    }
    DeviceMotionSample s;
    s.receive_time_ns = acc.ts;
    s.acceleration = Vec3f{BitsToFloat(lin.xb), BitsToFloat(lin.yb), BitsToFloat(lin.zb)};
    s.acceleration_including_gravity = Vec3f{BitsToFloat(acc.xb), BitsToFloat(acc.yb), BitsToFloat(acc.zb)};
    s.rotation_rate_abg = Vec3f{BitsToFloat(gyr.xb), BitsToFloat(gyr.yb), BitsToFloat(gyr.zb)};
    // The packet: the cursor's or the next one, whose acceleration, rotation rate and interval produce the recorded
    // raw values (a repeated bootstrap packet keeps the cursor).
    // Several packets can carry the same numbers (the IMU repeats samples; identical interval too): the last matching
    // one within the window is taken, which is equivalent for the engine, and the window never moves backwards.
    bool found = false;
    size_t best = *packet_cursor;
    for (size_t cand = *packet_cursor; cand < std::min(rec.packets.size(), *packet_cursor + 3); ++cand) {
      const Packet& p = rec.packets[cand];
      if (FloatToBits(static_cast<float>(p.ax)) == acc.xb && FloatToBits(static_cast<float>(p.ay)) == acc.yb &&
          FloatToBits(static_cast<float>(p.az)) == acc.zb && FloatToBits(static_cast<float>(p.alpha)) == gyr.xb &&
          FloatToBits(static_cast<float>(p.beta)) == gyr.yb && FloatToBits(static_cast<float>(p.gamma)) == gyr.zb &&
          fe::IntervalMsToNanos(p.interval_ms) == acc.interval) {
        s.interval_ms = p.interval_ms;
        best = cand;
        found = true;
      }
    }
    if (!found) { *why = "message " + std::to_string(m) + ": no dispatched packet explains event triple " + std::to_string(i / 3); return false; }
    *packet_cursor = best;
    out->push_back(s);
  }
  return true;
}

class CaptureCore final : public TrackerCore {
 public:
  explicit CaptureCore(std::vector<EventList>* sink) : sink_(sink) {}
  TrackerStep Track(const FrameInput& in) override {
    sink_->push_back(in.imu_events);
    TrackerStep s;
    s.status = TrackingStatus::kNormal;
    return s;
  }

 private:
  std::vector<EventList>* sink_;
};

// The engine chain: GlueRunner (SensorIntake, is_ios from the manufacturer) per message; the core sees the events of
// the staged frame one stage later (pipeline of two), so one extra stage flushes the last message.
bool EngineSensorRun(const Recording& rec, SensorMutation mut, SensorRun* run, std::string* why) {
  std::string manufacturer = rec.device.at("manufacturer");
  if (mut == SensorMutation::kIosTreatedAsAndroid) manufacturer = "";
  if (mut == SensorMutation::kAndroidTreatedAsIos) manufacturer = "Apple";
  EngineConfig cfg;
  cfg.capture_width = 480;  // the injected core ignores the geometry
  cfg.capture_height = 640;
  cfg.manufacturer = manufacturer;
  cfg.model = rec.device.at("model");
  GlueConfig glue_cfg;
  glue_cfg.frames_until_recenter = 0;
  std::vector<EventList> sink;
  GlueRunner glue(cfg, glue_cfg, [&sink](const EngineConfig&) -> std::unique_ptr<TrackerCore> {
    return std::make_unique<CaptureCore>(&sink);
  });
  uint8_t pixel = 0;
  size_t cursor = 0;
  for (size_t m = 0; m < rec.msgs.size(); ++m) {
    std::vector<DeviceMotionSample> samples;
    if (!SamplesOfMessage(rec, m, &cursor, &samples, why)) return false;
    for (const DeviceMotionSample& s : samples) glue.PushDeviceMotion(s);
    glue.ProcessFrame(1000 + static_cast<int64_t>(m), 1000 + static_cast<int64_t>(m), GrayImageView{&pixel, 1, 1, 1});
  }
  glue.ProcessFrame(999999, 999999, GrayImageView{&pixel, 1, 1, 1});  // flush
  // sink[k] = events of message k (the first stage is staged first and processed at the second ProcessFrame).
  run->per_message = std::move(sink);
  // Mutations of the produced events.
  auto& pm = run->per_message;
  if (mut == SensorMutation::kIntervalNotScaled) {
    for (EventList& l : pm) for (ImuEvent& e : l) e.interval_ns /= 1000;
  } else if (mut == SensorMutation::kWrongAxisSign) {
    for (EventList& l : pm) for (ImuEvent& e : l) {
      if (e.kind == ImuEvent::Kind::kAccelerometer || e.kind == ImuEvent::Kind::kLinearAcceleration) {
        e.x = -e.x;  // all three signs flipped: the recorded rule is wrong on the axes it changes and on the one it keeps
        e.y = -e.y;
        e.z = -e.z;
      }
    }
  } else if (pm.size() > 3 && !pm[3].empty()) {
    if (mut == SensorMutation::kOneUlpOnOneEvent) {
      uint32_t b = FloatToBits(pm[3][0].x) + 1u;
      pm[3][0].x = BitsToFloat(b);
    } else if (mut == SensorMutation::kDropOneEvent) {
      pm[3].pop_back();
    } else if (mut == SensorMutation::kSwapEqualTimeEvents && pm[3].size() > 1) {
      std::swap(pm[3][0], pm[3][1]);
    } else if (mut == SensorMutation::kZeroSignFlip) {
      ClearFirstNegatedZero(&pm[3]);
    }
  }
  return true;
}

// Greedy in-order matching of the original func 647 calls to the engine messages. `max_unmatched` messages (the
// bootstrap / startup-recenter / in-flight ones the original never processed) may stay unmatched; every original
// call must match a distinct message, in order.
Result CompareSensorLists(const Recording& rec, const std::vector<EventList>& engine, int max_unmatched) {
  Result r;
  size_t m = 0;
  int matched = 0;
  for (size_t c = 0; c < rec.calls.size(); ++c) {
    const std::vector<OutEv>& o = rec.calls[c];
    size_t k = m;
    while (k < engine.size() && !SameList(engine[k], o)) ++k;
    if (k == engine.size()) {
      r.Fail("original func 647 call " + std::to_string(c) + " (" + std::to_string(o.size()) + " events) matches no engine message at or after " + std::to_string(m));
      if (r.mismatches == 0 && m < engine.size()) {  // first miss: show the nearest candidate event by event
        const EventList& e = engine[m];
        for (size_t i = 0; i < std::max(e.size(), o.size()); ++i) {
          char buf[400];
          std::snprintf(buf, sizeof(buf), "   [%zu] engine %s original %s", i,
                        i < e.size() ? (std::to_string(static_cast<int>(e[i].kind)) + " t=" + std::to_string(e[i].event_time_ns) + " iv=" + std::to_string(e[i].interval_ns) + " " + std::to_string(FloatToBits(e[i].x)) + "," + std::to_string(FloatToBits(e[i].y)) + "," + std::to_string(FloatToBits(e[i].z))).c_str() : "-",
                        i < o.size() ? (std::to_string(o[i].kind) + " t=" + std::to_string(o[i].ts) + " iv=" + std::to_string(o[i].interval) + " " + std::to_string(o[i].xb) + "," + std::to_string(o[i].yb) + "," + std::to_string(o[i].zb)).c_str() : "-");
          r.Fail(buf);
        }
      }
      ++r.mismatches;
      continue;
    }
    m = k + 1;
    ++matched;
    r.compared += static_cast<int>(o.size());
  }
  const int unmatched = static_cast<int>(engine.size()) - matched;
  if (unmatched > max_unmatched) r.Fail("unmatched engine messages " + std::to_string(unmatched) + " > " + std::to_string(max_unmatched));
  if (rec.calls.empty() || matched < static_cast<int>(rec.calls.size())) r.pass = false;
  r.detail += "calls=" + std::to_string(rec.calls.size()) + " matched=" + std::to_string(matched) + " engine_messages=" + std::to_string(engine.size()) +
              " events_compared=" + std::to_string(r.compared) + "\n";
  return r;
}

Result CompareSensors(const Recording& rec, SensorMutation mut, int max_unmatched) {
  SensorRun run;
  std::string why;
  Result r;
  if (!EngineSensorRun(rec, mut, &run, &why)) { r.Fail("S: " + why); return r; }
  return CompareSensorLists(rec, run.per_message, max_unmatched);
}

// S': decoded raw events straight into PrepareTrackingSensorEvents.
Result CompareSensors647(const Recording& rec, SensorMutation mut, int max_unmatched) {
  bool ios = false;
  {
    std::string u = rec.device.at("manufacturer");
    for (char& ch : u) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    ios = u == "APPLE";
  }
  if (mut == SensorMutation::kIosTreatedAsAndroid) ios = false;
  if (mut == SensorMutation::kAndroidTreatedAsIos) ios = true;
  std::vector<EventList> engine;
  for (const std::vector<RawEv>& msg : rec.msgs) {
    std::vector<RawSensorEvent> raw;
    for (const RawEv& e : msg) {
      RawSensorEvent re;
      re.kind = static_cast<RawSensorKind>(e.kind);
      re.timestamp_ns = e.ts;
      re.interval_ns = e.interval;
      re.x = BitsToFloat(e.xb);
      re.y = BitsToFloat(e.yb);
      re.z = BitsToFloat(e.zb);
      raw.push_back(re);
    }
    engine.push_back(fe::PrepareTrackingSensorEvents(raw, ios));
  }
  if (mut == SensorMutation::kIntervalNotScaled) {
    for (EventList& l : engine) for (ImuEvent& e : l) e.interval_ns /= 1000;
  } else if (engine.size() > 3 && !engine[3].empty()) {
    if (mut == SensorMutation::kOneUlpOnOneEvent) engine[3][0].x = BitsToFloat(FloatToBits(engine[3][0].x) + 1u);
    if (mut == SensorMutation::kDropOneEvent) engine[3].pop_back();
    if (mut == SensorMutation::kSwapEqualTimeEvents && engine[3].size() > 1) std::swap(engine[3][0], engine[3][1]);
    if (mut == SensorMutation::kZeroSignFlip) ClearFirstNegatedZero(&engine[3]);
  }
  return CompareSensorLists(rec, engine, max_unmatched);
}

// ------------------------------------------------------------------------------------------------
// K: intrinsics

enum class IntrinsicsMutation { kNone, kOrientationIgnored, kWrongDevice, kOneUlp, kZeroSignFlip };

Result CompareIntrinsics(const Recording& rec, IntrinsicsMutation mut) {
  Result r;
  if (rec.intrinsics.empty()) { r.Fail("K: no intrinsics rows"); return r; }
  int w = 0, h = 0, rot = 0;
  {
    std::istringstream in(rec.device.at("engine_init"));
    in >> w >> h >> rot;
  }
  EngineConfig cfg;
  cfg.capture_width = w;
  cfg.capture_height = h;
  cfg.rotation_deg = mut == IntrinsicsMutation::kOrientationIgnored ? 0 : rot;
  cfg.manufacturer = rec.device.at("manufacturer");
  cfg.model = mut == IntrinsicsMutation::kWrongDevice ? "Pixel 7" : rec.device.at("model");
  GlueConfig glue_cfg;
  glue_cfg.frames_until_recenter = 0;
  std::vector<EventList> sink;
  GlueRunner glue(cfg, glue_cfg, [&sink](const EngineConfig&) -> std::unique_ptr<TrackerCore> {
    return std::make_unique<CaptureCore>(&sink);
  });
  // What the page sent: updateCameraProjectionMatrix({cam: {pixelRectWidth, pixelRectHeight, 0.01, 1000}, origin, facing}).
  CameraProjectionUpdate u;
  u.pixel_rect_width = std::stoi(rec.device.at("served_width"));
  u.pixel_rect_height = std::stoi(rec.device.at("served_height"));
  u.near_clip_plane = 0.01f;
  u.far_clip_plane = 1000.0f;
  u.origin = Vec3f{0.f, 2.f, 0.f};
  u.facing = QuatWxyz{1.f, 0.f, 0.f, 0.f};
  glue.UpdateCameraProjectionMatrix(u);
  uint8_t pixel = 0;
  std::array<float, 16> published{};
  for (int k = 0; k < 3; ++k) published = glue.ProcessFrame(1000 + k, 1000 + k, GrayImageView{&pixel, 1, 1, 1}).intrinsics;
  const IntrinsicsRow& row = rec.intrinsics.front();
  bool zero_flipped = false;
  for (size_t k = 0; k < 16; ++k) {
    uint32_t got = FloatToBits(published[k]);
    if (mut == IntrinsicsMutation::kOneUlp && k == 0) got += 1u;
    // 0-sign mutation: the sign bit of the first element that is a zero is toggled (the engine would publish the other zero).
    if (mut == IntrinsicsMutation::kZeroSignFlip && (published[k] == 0.0f) && !zero_flipped) {
      got ^= 0x80000000u;
      zero_flipped = true;
    }
    ++r.compared;
    if (got != row.bits[k]) {
      ++r.mismatches;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "K: m[%zu] engine %08x original %08x", k, got, row.bits[k]);
      r.Fail(buf);
    }
  }
  r.detail += "floats_compared=" + std::to_string(r.compared) + " mismatches=" + std::to_string(r.mismatches) + "\n";
  return r;
}

// ------------------------------------------------------------------------------------------------
// P: pyramid

enum class PyramidMutation { kNone, kWrongRotation, kOnePixelOneLevel };

Result ComparePyramids(const Recording& rec, PyramidMutation mut) {
  Result r;
  if (rec.captures.empty()) { r.Fail("P: no captures"); return r; }
  int compared_frames = 0;
  for (const CaptureRow& c : rec.captures) {
    if (c.served == "-" || c.input_frame < 0) continue;
    int w = 0, h = 0;
    std::vector<uint8_t> gray;
    if (!ReadPgm(rec.dir + "/" + c.served, &w, &h, &gray)) { r.Fail("P: cannot read " + c.served); continue; }
    if (w != c.cam_w || h != c.cam_h) { r.Fail("P: served frame size differs from the engineInit size"); continue; }
    fe::RgbaAtlas captured;
    if (!fe::testing::LoadCaptureAtlas(rec.dir + "/" + [&] { char b[40]; std::snprintf(b, sizeof(b), "cap-%03d.rgba.gz", c.capture); return std::string(b); }(), 660, &captured)) {
      r.Fail("P: cannot read capture " + std::to_string(c.capture));
      continue;
    }
    const int rotation = mut == PyramidMutation::kWrongRotation ? (c.rotation + 90) % 360 : c.rotation;
    fe::PyramidSource source(c.cam_w, c.cam_h, fe::ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox, rotation);
    source.Get({}, {}, GrayImageView{gray.data(), w, h, w});
    fe::RgbaAtlas mine = source.last_cpu_atlas();
    if (mut == PyramidMutation::kOnePixelOneLevel) {
      const fe::LevelLayout& l0 = source.layout().levels[0];
      const size_t o = (static_cast<size_t>(l0.r + l0.h / 2) * mine.size + static_cast<size_t>(l0.c + l0.w / 2)) * 4;
      mine.rgba[o] = static_cast<uint8_t>(mine.rgba[o] + 1);
    }
    const fe::testing::AtlasStats st = fe::testing::CompareLevels(mine, captured, source.layout());
    ++compared_frames;
    r.compared += static_cast<int>(st.pixels) * 4;
    for (int chn = 0; chn < 4; ++chn) {
      const long bad = st.pixels - st.equal[static_cast<size_t>(chn)];
      if (bad != 0) {
        r.mismatches += static_cast<int>(bad);
        char buf[200];
        std::snprintf(buf, sizeof(buf), "P: capture %d frame %d channel %d: %ld of %ld pixels differ (max diff %ld)", c.capture, c.input_frame, chn, bad, st.pixels, st.max_diff[static_cast<size_t>(chn)]);
        r.Fail(buf);
      }
    }
  }
  if (compared_frames == 0) r.Fail("P: no frame compared");
  r.detail += "frames=" + std::to_string(compared_frames) + " channel_values_compared=" + std::to_string(r.compared) + " mismatches=" + std::to_string(r.mismatches) + "\n";
  return r;
}

// ------------------------------------------------------------------------------------------------
// R: exact-input replay of the recording vs the original's published pose (US-023)

constexpr int kPoseWindow = 120;      // input frames 0..119
constexpr double kPoseTol = 1e-5;     // per component of the position and of the rotation quaternion
constexpr int kPoseMinCompared = 110;  // of the 120 frames of the window

struct OrigRow {
  bool fresh = false;
  bool has_pose = false;
  std::string status;
  double p[3] = {0, 0, 0};
  double q[4] = {1, 0, 0, 0};
};

bool LoadOriginalTrace(const std::string& path, std::map<int, OrigRow>* rows) {
  std::string text;
  if (!ReadTextFile(path, &text)) return false;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    fe::testing::Json j;
    if (!fe::testing::Json::Parse(line, &j, nullptr)) return false;
    OrigRow r;
    r.fresh = j["fresh"].number() != 0.0 || j["status"].str() != "";
    r.status = j["status"].str();
    const fe::testing::Json& p = j["p"];
    const fe::testing::Json& q = j["q_wxyz"];
    if (!p.is_null() && !q.is_null()) {
      r.has_pose = true;
      for (size_t k = 0; k < 3; ++k) r.p[k] = p[k].number();
      for (size_t k = 0; k < 4; ++k) r.q[k] = q[k].number();
    }
    (*rows)[static_cast<int>(j["i"].number())] = r;
  }
  return true;
}

Result ComparePose(const Recording& rec, orient_replay::Mutation mut) {
  Result r;
  std::map<int, OrigRow> orig;
  if (!LoadOriginalTrace(rec.dir + "/trace.jsonl", &orig)) { r.Fail("R: original trace unreadable"); return r; }
  orient_replay::Options opt;
  opt.mutation = mut;
  opt.max_frame = kPoseWindow - 1;
  const orient_replay::Outcome eng = orient_replay::Replay(rec.dir, opt);
  if (!eng.ok) { r.Fail("R: replay failed: " + eng.error); return r; }
  double max_p = 0, max_q = 0;
  int compared = 0, status_diff = 0;
  int first_orig = -1, first_eng = -1;
  for (const orient_replay::TraceRow& e : eng.rows) {
    if (e.frame >= kPoseWindow) continue;
    auto it = orig.find(e.frame);
    if (it == orig.end() || !it->second.has_pose || !it->second.fresh || !e.fresh) continue;
    const OrigRow& o = it->second;
    ++compared;
    if (o.status != e.status) { ++status_diff; r.Fail("R: frame " + std::to_string(e.frame) + " status " + e.status + " vs original " + o.status); }
    double dp = 0;
    for (int k = 0; k < 3; ++k) dp = std::max(dp, std::fabs(static_cast<double>(e.p[k]) - o.p[k]));
    double dq_plus = 0, dq_minus = 0;
    for (int k = 0; k < 4; ++k) {
      dq_plus = std::max(dq_plus, std::fabs(static_cast<double>(e.q[k]) - o.q[k]));
      dq_minus = std::max(dq_minus, std::fabs(static_cast<double>(e.q[k]) + o.q[k]));
    }
    const double dq = std::min(dq_plus, dq_minus);  // q and -q are the same rotation
    max_p = std::max(max_p, dp);
    max_q = std::max(max_q, dq);
    const bool moved_e = std::fabs(e.p[0]) > 1e-6 || std::fabs(e.p[1] - 2.0) > 1e-6 || std::fabs(e.p[2]) > 1e-6;
    const bool moved_o = std::fabs(o.p[0]) > 1e-6 || std::fabs(o.p[1] - 2.0) > 1e-6 || std::fabs(o.p[2]) > 1e-6;
    if (moved_e && first_eng < 0) first_eng = e.frame;
    if (moved_o && first_orig < 0) first_orig = e.frame;
    if (dp > kPoseTol || dq > kPoseTol) {
      ++r.mismatches;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "R: frame %d position diff %.3g rotation diff %.3g", e.frame, dp, dq);
      r.Fail(buf);
    }
  }
  r.compared = compared;
  if (compared < kPoseMinCompared) r.Fail("R: only " + std::to_string(compared) + " of " + std::to_string(kPoseWindow) + " frames compared (< " + std::to_string(kPoseMinCompared) + ")");
  if (first_eng != first_orig) r.Fail("R: first translation frame engine " + std::to_string(first_eng) + " original " + std::to_string(first_orig));
  char buf[200];
  std::snprintf(buf, sizeof(buf), "frames=%d status_diff=%d max_position_diff=%.3g max_rotation_diff=%.3g first_translation engine=%d original=%d\n", compared, status_diff, max_p, max_q, first_eng, first_orig);
  r.detail += buf;
  return r;
}

#ifndef SR_ORIENT_HELD
// ------------------------------------------------------------------------------------------------
// Synthetic recording (instrument checks): an independent second reading of func 647 (the decompile, dcmp
// f_xx / func647: isIos = upper(manufacturer) == "APPLE"; per event: interval iOS x1000, Android 16000000 ->
// 16666667; accel / lin accel: iOS (x, y) negated, Android z negated; gyro (x, y) negated; kind 1 -> 1, 2 -> 3,
// 3 -> 2, 4 -> 4; sort by (time, kind, x, y, z)).

std::vector<OutEv> Reference647(const std::vector<RawEv>& msg, bool ios, int call) {
  std::vector<OutEv> out;
  for (const RawEv& e : msg) {
    // Independent reading of f647.wat:1567-1582 (registered prediction, orient-us024-gate.v2): the identity rotation is a multiply-add
    // followed by the constant (0, 0, 0, 1), so a raw zero of either sign leaves it as +0 (`+ 0.0f`); the sign rule below is an
    // f32.neg after that, so a negated zero is 0x80000000.
    float x = BitsToFloat(e.xb) + 0.0f, y = BitsToFloat(e.yb) + 0.0f, z = BitsToFloat(e.zb) + 0.0f;
    int64_t interval = ios ? static_cast<int64_t>(static_cast<uint64_t>(e.interval) * 1000u) : (e.interval == 16000000 ? 16666667 : e.interval);
    int kind = 0;
    if (e.kind == 1 || e.kind == 4) {
      if (ios) { x = -x; y = -y; } else { z = -z; }
      kind = e.kind == 1 ? 1 : 4;
    } else if (e.kind == 2) {
      x = -x;
      y = -y;
      kind = 3;
    } else if (e.kind == 3) {
      kind = 2;
    }
    out.push_back({call, kind, e.ts, interval, FloatToBits(x), FloatToBits(y), FloatToBits(z)});
  }
  std::sort(out.begin(), out.end(), [](const OutEv& a, const OutEv& b) {
    if (a.ts != b.ts) return a.ts < b.ts;
    if (a.kind != b.kind) return a.kind < b.kind;
    if (BitsToFloat(a.xb) != BitsToFloat(b.xb)) return BitsToFloat(a.xb) < BitsToFloat(b.xb);
    if (BitsToFloat(a.yb) != BitsToFloat(b.yb)) return BitsToFloat(a.yb) < BitsToFloat(b.yb);
    return BitsToFloat(a.zb) < BitsToFloat(b.zb);
  });
  return out;
}

class Lcg {
 public:
  explicit Lcg(uint32_t s) : s_(s) {}
  double Uniform(double lo, double hi) {
    s_ = s_ * 1664525u + 1013904223u;
    return lo + (hi - lo) * (static_cast<double>(s_ >> 8) / 16777216.0);
  }

 private:
  uint32_t s_;
};

Recording SyntheticRecording(bool ios, uint32_t seed) {
  Recording rec;
  rec.dir = "synthetic";
  rec.device = {{"manufacturer", ios ? "Apple" : "Google"}, {"model", ios ? "iPhone13,4" : "Pixel 7"}, {"os", ios ? "iOS" : "Android"},
                {"engine_init", "480 640 0"}, {"served_width", "480"}, {"served_height", "640"}};
  Lcg rng(seed);
  const double intervals[] = {16.0, 16.67, 0.016667, 20.32, 8.33, 33.4, 0.008};
  int64_t t = 1000000000;
  size_t pk = 0;
  for (int m = 0; m < 24; ++m) {
    std::vector<RawEv> msg;
    const int n_packets = 1 + m % 3;
    for (int p = 0; p < n_packets; ++p) {
      Packet pkt{intervals[pk % 7], rng.Uniform(-9, 9), rng.Uniform(-9, 9), rng.Uniform(-9, 9), rng.Uniform(-90, 90),
                 rng.Uniform(-90, 90), rng.Uniform(-90, 90)};
      rec.packets.push_back(pkt);
      ++pk;
      t += 16000000 + static_cast<int64_t>(rng.Uniform(0, 2000000));
      const int64_t interval = fe::IntervalMsToNanos(pkt.interval_ms);
      auto bits = [](double v) { return FloatToBits(static_cast<float>(v)); };
      msg.push_back({m, 4, t, interval, FloatToBits(0.f), FloatToBits(0.f), FloatToBits(0.f)});
      msg.push_back({m, 1, t, interval, bits(pkt.ax), bits(pkt.ay), bits(pkt.az)});
      msg.push_back({m, 2, t, interval, bits(pkt.alpha), bits(pkt.beta), bits(pkt.gamma)});
    }
    rec.msgs.push_back(msg);
  }
  // "original" output: calls for messages 1..22 (the first and the last stay unprocessed, as in a real recording).
  int call = 0;
  for (size_t m = 1; m + 1 < rec.msgs.size(); ++m) rec.calls.push_back(Reference647(rec.msgs[m], ios, call++));
  return rec;
}

// Synthetic K recording: the reality.intrinsics the original would publish, computed by hand from the K rule
// (focal at 480x640, crop / scale, toClipSpace): m00 = 2 fx / W', m11 = 2 fy / H', m02 = 1 - (2 cx + 1) / W',
// m12 = -1 + (2 cy + 1) / H', m10 = (f+n)/(n-f), m11... for the iPhone X default focal 506.4104 and a 480x640
// display; landscape reorders as updateCameraIntrinsics.
Recording SyntheticIntrinsics(int rotation) {
  Recording rec;
  const bool landscape = rotation == 90 || rotation == 270 || rotation == -90;
  rec.device = {{"manufacturer", ""}, {"model", "K"}, {"os", "Android"},
                {"engine_init", landscape ? "640 480 " + std::to_string(rotation) : "480 640 " + std::to_string(rotation)},
                {"served_width", landscape ? "640" : "480"}, {"served_height", landscape ? "480" : "640"}};
  const float f = 506.4104f, W = 480.f, H = 640.f, n = 0.01f, far = 1000.f;
  float m00 = 2 * f / W, m11 = 2 * f / H, m02 = 1.0f - (2.0f * 239.5f + 1.0f) / W, m12 = -1.0f + (2.0f * 319.5f + 1.0f) / H;
  const float m22 = (far + n) / (n - far), m23 = 2 * far * n / (n - far);
  // updateCameraIntrinsics negates with `-` (orientation.cc:50-51): a negated +0 skew term is 0x80000000 (registered prediction).
  auto neg = [](float v) { return -v; };
  if (rotation == 90) { const float r0x = m00, r0z = m02, r1y = m11, r1z = m12; m00 = r1y; m02 = neg(r1z); m11 = r0x; m12 = r0z; }
  if (rotation == 270 || rotation == -90) { const float r0x = m00, r0z = m02, r1y = m11, r1z = m12; m00 = r1y; m02 = r1z; m11 = r0x; m12 = neg(r0z); }
  if (rotation == 180) { m02 = neg(m02); m12 = neg(m12); }
  IntrinsicsRow row;
  row.bits[0] = FloatToBits(m00);
  row.bits[5] = FloatToBits(m11);
  row.bits[8] = FloatToBits(m02);
  row.bits[9] = FloatToBits(m12);
  row.bits[10] = FloatToBits(m22);
  row.bits[11] = FloatToBits(-1.f);
  row.bits[14] = FloatToBits(m23);
  rec.intrinsics.push_back(row);
  return rec;
}

#endif  // !SR_ORIENT_HELD

// ------------------------------------------------------------------------------------------------
// Start of a held run (protocol §12.5, §12.6). The release decision, the one-run marker and the seed policy are the shared
// device sr_held (tests/support/sr_held.h, docs/design/sr-held-design.md); this file adds the check of the recorded inputs.
//
// Order (review N-1 of orient-gates.v2, C-U24-3): every check that can refuse the run happens BEFORE the marker is created, so a
// refusal never uses up the one run of a gate:
//   1. sr_held::CheckRelease  (env value = sha256 of the designation file, manifest, review, pins, gate file; no side effect)
//   2. the recorded inputs     (SHA256SUMS of each recording directory = manifest held.input_sums; every listed file hashes to its line)
//   3. sr_held::HeldGate       (the same checks again, then the O_EXCL marker; the code never removes it)
// The gate (which gate id and version, which preregistration directory, which recordings) is designated by the caller; this source
// names none. The manifest registers, besides the sr_held fields, held.input_sums = {<recording dir>: <sha256 of its SHA256SUMS>}.

namespace sh = slam_refact::sr_held;

// "<gate_id>.v<N>" -> gate_id, N. A name the caller designated; anything else is refused.
bool SplitGateName(const std::string& name, std::string* gate_id, int* version) {
  const size_t p = name.rfind(".v");
  if (p == std::string::npos || p == 0 || p + 2 >= name.size()) return false;
  const std::string digits = name.substr(p + 2);
  if (digits.size() > 6 || digits[0] == '0' || !std::all_of(digits.begin(), digits.end(), [](char c) { return c >= '0' && c <= '9'; })) return false;
  *gate_id = name.substr(0, p);
  *version = std::atoi(digits.c_str());
  return sh::IsId(*gate_id);
}

struct HeldRunSpec {
  std::string prereg_dir;                 // manifest, designation file, review file, gate file, markers
  std::string rec_dir;                    // the recordings: <rec_dir>/<input dir>/SHA256SUMS
  std::string gate_name;                  // "<gate_id>.v<N>", designated by the caller
  std::string env_value;                  // sha256 of the designation file (SLAM_REFACT_ORIENT_REVIEWER)
  std::vector<std::string> pins;          // files whose sha256 manifest held.pins registers (the held binary)
  std::vector<std::string> input_dirs;    // every recording directory the gate reads (duplicates allowed)
};

struct HeldRun {
  bool ok = false;
  std::string code;                       // "" when ok; an sr_held refusal code, "ARGS" or "INPUT"
  std::string why;
  std::vector<std::string> log;
};

// Protocol §12.5 (f): the SHA256SUMS of one recording directory is the registered one and every file it lists hashes to its line.
// Returns "" when the directory is verified, else the reason. No side effect.
std::string VerifyRecordingInputs(const std::string& rec_dir, const std::string& d, const sh::Json& input_sums, std::vector<std::string>* log) {
  if (!sh::IsBareName(d)) return "recording directory name is not a bare name: " + d;
  const sh::Json* registered = input_sums.Get(d);
  if (registered == nullptr || registered->type != sh::Json::Type::kString || !sh::IsHex64(registered->text)) {
    return "manifest held.input_sums does not register " + d;
  }
  const std::string dir = rec_dir + "/" + d;
  std::string sums, actual_sums;
  if (!sh::ReadFile(dir + "/SHA256SUMS", &sums)) return "SHA256SUMS missing in " + d;
  actual_sums = testutil::Sha256Hex(sums);
  if (actual_sums != registered->text) return "SHA256SUMS of " + d + " differs from the manifest (held.input_sums)";
  std::istringstream in(sums);
  std::string line;
  int files = 0;
  while (std::getline(in, line)) {
    const size_t sp = line.find("  ");
    if (sp != 64 || !sh::IsHex64(line.substr(0, sp)) || sp + 2 >= line.size()) return "malformed line in the SHA256SUMS of " + d;
    const std::string name = line.substr(sp + 2);
    if (name[0] == '/' || name.find("..") != std::string::npos) return "SHA256SUMS of " + d + " lists a path outside the directory: " + name;
    std::string actual;
    if (!sh::Sha256File(dir + "/" + name, &actual)) return "input missing or not a regular file: " + d + "/" + name;
    if (actual != line.substr(0, sp)) return "input differs from SHA256SUMS: " + d + "/" + name;
    ++files;
  }
  if (files == 0) return "SHA256SUMS of " + d + " lists no file";
  log->push_back("[ORIENT-INPUT] " + d + " files verified=" + std::to_string(files) + " SHA256SUMS sha256=" + actual_sums);
  return "";
}

HeldRun StartHeldRun(const HeldRunSpec& spec) {
  HeldRun run;
  auto fail = [&run](const std::string& code, const std::string& why) {
    run.ok = false;
    run.code = code;
    run.why = why;
    return run;
  };
  sh::Request rq;
  rq.prereg_dir = spec.prereg_dir;
  rq.kind = "main";
  rq.env_value = spec.env_value;
  rq.pins = spec.pins;
  if (!SplitGateName(spec.gate_name, &rq.gate_id, &rq.version)) {
    return fail("ARGS", "HELD: the gate is not designated as <gate_id>.v<N> (SLAM_REFACT_ORIENT_GATE='" + spec.gate_name + "')");
  }
  // 1. every release condition, no side effect
  const sh::Outcome check = sh::CheckRelease(rq);
  for (const std::string& l : check.log) run.log.push_back(l);
  if (!check.ok) return fail(check.code, check.why);
  // 2. the recorded inputs (f): before the marker
  std::string text, err;
  sh::Json manifest;
  if (!sh::ReadFile(spec.prereg_dir + "/" + sh::ManifestName(rq), &text) || !sh::ParseJson(text, &manifest, &err)) {
    return fail("INPUT", "manifest unreadable after the release check");
  }
  const sh::Json* held = manifest.Get("held");
  const sh::Json* input_sums = held == nullptr ? nullptr : held->Get("input_sums");
  if (input_sums == nullptr || input_sums->type != sh::Json::Type::kObject) return fail("INPUT", "manifest held.input_sums is missing");
  std::vector<std::string> verified;
  for (const std::string& d : spec.input_dirs) {
    if (std::find(verified.begin(), verified.end(), d) != verified.end()) continue;
    const std::string why = VerifyRecordingInputs(spec.rec_dir, d, *input_sums, &run.log);
    if (!why.empty()) return fail("INPUT", why);
    verified.push_back(d);
  }
  // 3. the marker (HeldGate repeats the release checks, then creates the marker with O_EXCL)
  const sh::Outcome claimed = sh::HeldGate(rq);
  if (!claimed.ok) return fail(claimed.code, claimed.why);
  run.log.push_back("[ORIENT-INTEGRITY] first-run marker created for " + spec.gate_name + ": " + claimed.marker);
  run.ok = true;
  return run;
}

// ------------------------------------------------------------------------------------------------
// Tests

#ifndef SR_ORIENT_HELD

// Self-check of the independent reading of func 647 against the engine on the synthetic recording: this is the null
// run of the comparator (engine vs the independent reading, both sides correct) for iOS and Android.
TEST(OrientGateInstrument, NullRunSensorsIosAndAndroid) {
  for (bool ios : {true, false}) {
    const Recording rec = SyntheticRecording(ios, ios ? 11u : 12u);
    const Result s = CompareSensors(rec, SensorMutation::kNone, 3);
    EXPECT_TRUE(s.pass) << s.detail;
    EXPECT_EQ(s.mismatches, 0);
    EXPECT_GT(s.compared, 100);
    const Result s2 = CompareSensors647(rec, SensorMutation::kNone, 3);
    EXPECT_TRUE(s2.pass) << s2.detail;
    std::printf("[null-run sensors ios=%d] %s%s", ios, s.detail.c_str(), s2.detail.c_str());
  }
}

TEST(OrientGateInstrument, SensorMutationsAreDetected) {
  const Recording ios = SyntheticRecording(true, 21u);
  const Recording android = SyntheticRecording(false, 22u);
  struct Case { const Recording* rec; SensorMutation m; const char* name; };
  const Case cases[] = {
      {&ios, SensorMutation::kIosTreatedAsAndroid, "iOS packets processed with the Android rules"},
      {&android, SensorMutation::kAndroidTreatedAsIos, "Android packets processed with the iOS rules"},
      {&ios, SensorMutation::kIntervalNotScaled, "iOS interval x1000 missing"},
      {&android, SensorMutation::kIntervalNotScaled, "Android interval scale changed"},
      {&ios, SensorMutation::kWrongAxisSign, "acceleration signs of the wrong axes (iOS)"},
      {&android, SensorMutation::kWrongAxisSign, "acceleration signs of the wrong axes (Android)"},
      {&ios, SensorMutation::kOneUlpOnOneEvent, "one ulp on one event x (sub-resolution)"},
      {&android, SensorMutation::kOneUlpOnOneEvent, "one ulp on one event x (sub-resolution, Android)"},
      {&ios, SensorMutation::kDropOneEvent, "one event dropped"},
      {&ios, SensorMutation::kSwapEqualTimeEvents, "two events of the same time exchanged"},
      {&ios, SensorMutation::kZeroSignFlip, "sign bit of one negated zero cleared (-0 -> +0, sub-resolution of a value compare)"},
      {&android, SensorMutation::kZeroSignFlip, "sign bit of one negated zero cleared (Android)"},
  };
  for (const Case& c : cases) {
    const Result a = CompareSensors(*c.rec, c.m, 3);
    EXPECT_FALSE(a.pass) << "S missed: " << c.name;
    const Result b = CompareSensors647(*c.rec, c.m, 3);
    if (c.m != SensorMutation::kWrongAxisSign) {
      EXPECT_FALSE(b.pass) << "S' missed: " << c.name;
    }
    std::printf("[mutation sensors] %-60s S=%s S'=%s\n", c.name, a.pass ? "PASS(undetected)" : "FAIL(detected)",
                c.m == SensorMutation::kWrongAxisSign ? "n/a" : (b.pass ? "PASS(undetected)" : "FAIL(detected)"));
  }
}

TEST(OrientGateInstrument, IntrinsicsNullRunAndMutations) {
  for (int rot : {0, 90, 180, 270}) {
    const Recording rec = SyntheticIntrinsics(rot);
    const Result r = CompareIntrinsics(rec, IntrinsicsMutation::kNone);
    EXPECT_TRUE(r.pass) << "rotation " << rot << "\n" << r.detail;
    std::printf("[null-run intrinsics rotation=%d] %s", rot, r.detail.c_str());
    EXPECT_FALSE(CompareIntrinsics(rec, IntrinsicsMutation::kOneUlp).pass) << rot;
    EXPECT_FALSE(CompareIntrinsics(rec, IntrinsicsMutation::kWrongDevice).pass) << rot;
    if (rot != 0) {
      // With the bit channel the 180 degree correction is visible too: it flips the sign bit of the zero skew terms m02 / m12.
      EXPECT_FALSE(CompareIntrinsics(rec, IntrinsicsMutation::kOrientationIgnored).pass) << rot;
    }
    EXPECT_FALSE(CompareIntrinsics(rec, IntrinsicsMutation::kZeroSignFlip).pass) << "zero sign " << rot;
  }
}

// Pyramid comparator: the engine's own CPU atlas as the "capture" is a null run (zero differences by construction)
// and the three mutations must be detected; no original data is read.
TEST(OrientGateInstrument, PyramidNullRunAndMutations) {
  const std::string dir = ::testing::TempDir() + "/orient_gate_pyramid";
  ::mkdir(dir.c_str(), 0775);
  struct Geo { int w, h, rot; };
  for (const Geo& g : {Geo{480, 640, 0}, Geo{640, 480, 90}, Geo{640, 480, 270}, Geo{480, 640, 180}}) {
    std::vector<uint8_t> gray(static_cast<size_t>(g.w) * g.h);
    uint32_t s = 99;
    for (uint8_t& v : gray) { s = s * 1664525u + 1013904223u; v = static_cast<uint8_t>(40 + ((s >> 16) & 0x9F)); }
    {
      std::ofstream pgm(dir + "/served-00001.pgm", std::ios::binary);
      pgm << "P5\n" << g.w << " " << g.h << "\n255\n";
      pgm.write(reinterpret_cast<const char*>(gray.data()), static_cast<std::streamsize>(gray.size()));
    }
    fe::PyramidSource source(g.w, g.h, fe::ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox, g.rot);
    source.Get({}, {}, GrayImageView{gray.data(), g.w, g.h, g.w});
    fe::RgbaAtlas atlas = source.last_cpu_atlas();
    atlas.rgba.resize(static_cast<size_t>(1024) * 660 * 4, 0);
    {
      gzFile gz = gzopen((dir + "/cap-000.rgba.gz").c_str(), "wb");
      ASSERT_NE(gz, nullptr);
      ASSERT_EQ(gzwrite(gz, atlas.rgba.data(), static_cast<unsigned>(atlas.rgba.size())), static_cast<int>(atlas.rgba.size()));
      gzclose(gz);
    }
    Recording rec;
    rec.dir = dir;
    rec.captures.push_back({0, 1, g.rot, g.w, g.h, "", "served-00001.pgm"});
    const Result null_run = ComparePyramids(rec, PyramidMutation::kNone);
    EXPECT_TRUE(null_run.pass) << null_run.detail;
    std::printf("[null-run pyramid %dx%d rotation=%d] %s", g.w, g.h, g.rot, null_run.detail.c_str());
    EXPECT_FALSE(ComparePyramids(rec, PyramidMutation::kOnePixelOneLevel).pass) << "one level on one pixel (sub-resolution)";
    EXPECT_FALSE(ComparePyramids(rec, PyramidMutation::kWrongRotation).pass) << "rotation changed by 90";
  }
}

// The start of a held run (StartHeldRun) on synthetic files: no engine, no original data. The release logic itself (every bypass of
// protocol §12.5) is the conformance of the shared device (tests/support/sr_held_conformance.cc, 198 cases); what this file adds is the
// ORDER of the checks (review N-1 / C-U24-3): a refusal of any kind leaves no marker, so it does not use up the one run of the gate.
struct HeldFixture {
  std::string root, prereg, rec, exe, gate_name, marker;
  HeldRunSpec spec;
};

void PutFile(const std::string& path, const std::string& body) {
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  std::ofstream f(path, std::ios::binary);
  f << body;
}

std::string Sha256Text(const std::string& body) { return testutil::Sha256Hex(body); }

// Two recording directories d1, d2, each with a SHA256SUMS over a.json and tsv/x.tsv.
void PutRecording(const std::string& rec, const std::string& d, const std::string& salt) {
  const std::string a = "{\"a\":\"" + d + salt + "\"}\n", x = "i\tj\n" + d + salt + "\n";
  PutFile(rec + "/" + d + "/a.json", a);
  PutFile(rec + "/" + d + "/tsv/x.tsv", x);
  PutFile(rec + "/" + d + "/SHA256SUMS", Sha256Text(a) + "  a.json\n" + Sha256Text(x) + "  tsv/x.tsv\n");
}

HeldFixture MakeHeldFixture(const std::string& tag, int version) {
  HeldFixture f;
  f.root = ::testing::TempDir() + "/orient_held_" + tag + "_" + std::to_string(::getpid());
  std::filesystem::remove_all(f.root);
  f.prereg = f.root + "/prereg";
  f.rec = f.root + "/rec";
  f.exe = f.root + "/exe.bin";
  const std::string v = std::to_string(version);
  f.gate_name = "g1.v" + v;
  PutRecording(f.rec, "d1", "");
  PutRecording(f.rec, "d2", "");
  PutFile(f.exe, "held binary bytes");
  const std::string gate_md = "the gate file", review_md = "the review";
  PutFile(f.prereg + "/g1.v" + v + ".md", gate_md);
  PutFile(f.prereg + "/g1.v" + v + ".review.md", review_md);
  std::string sums1, sums2;
  sh::ReadFile(f.rec + "/d1/SHA256SUMS", &sums1);
  sh::ReadFile(f.rec + "/d2/SHA256SUMS", &sums2);
  auto manifest = [&](const std::string& designation_sha) {
    return "{\"gate_id\":\"g1\",\"version\":" + v + ",\"file\":\"g1.v" + v + ".md\",\"sha256\":\"" + Sha256Text(gate_md) +
           "\",\"created_at\":\"2026-01-01T00:00:00+09:00\",\"held\":{\"kinds\":[\"main\"],\"pins\":{\"exe.bin\":\"" +
           Sha256Text("held binary bytes") + "\"},\"inputs\":{},\"seed\":false,\"input_sums\":{\"d1\":\"" + Sha256Text(sums1) +
           "\",\"d2\":\"" + Sha256Text(sums2) + "\"}},\"review\":{\"reviewer\":\"test\",\"at\":\"2026-01-02T00:00:00+09:00\",\"file\":\"g1.v" + v +
           ".review.md\",\"sha256\":\"" + Sha256Text(review_md) + "\",\"designation_file\":\"g1.v" + v +
           ".review-designation.json\",\"designation_file_sha256\":\"" + designation_sha + "\"},\"first_real_run_at\":null}";
  };
  // The designation pins the manifest without its review block, so it can be computed before the designation's own sha256 is known.
  sh::Json parsed;
  std::string err, canon;
  EXPECT_TRUE(sh::ParseJson(manifest(std::string(64, '0')), &parsed, &err)) << err;
  EXPECT_TRUE(sh::CanonManifestSha(parsed, &canon));
  const std::string designation = "{\"gate\":\"g1.v" + v + "\",\"first_real_run_permitted\":true,\"pinned\":{\"gate_md_sha256\":\"" +
                                  Sha256Text(gate_md) + "\",\"review_file_sha256\":\"" + Sha256Text(review_md) +
                                  "\",\"manifest_before_review_sha256\":\"" + canon + "\"}}";
  PutFile(f.prereg + "/g1.v" + v + ".review-designation.json", designation);
  PutFile(f.prereg + "/g1.v" + v + ".manifest.json", manifest(Sha256Text(designation)));
  f.marker = f.prereg + "/g1.v" + v + ".first-run.main.marker";
  f.spec.prereg_dir = f.prereg;
  f.spec.rec_dir = f.rec;
  f.spec.gate_name = f.gate_name;
  f.spec.env_value = Sha256Text(designation);
  f.spec.pins = {f.exe};
  f.spec.input_dirs = {"d1", "d2", "d1"};
  return f;
}

bool Exists(const std::string& path) {
  struct stat st;
  return ::lstat(path.c_str(), &st) == 0;
}

TEST(OrientGateInstrument, HeldRunStartsOnceAndTheMarkerIsExclusive) {
  const HeldFixture f = MakeHeldFixture("once", 1);
  const HeldRun first = StartHeldRun(f.spec);
  EXPECT_TRUE(first.ok) << first.code << ": " << first.why;
  EXPECT_TRUE(Exists(f.marker));
  // The fixture directory name carries the process id (two parallel runs must not share it), so the log shows the fixed label
  // <fixture> in its place: the log of two runs of this binary is the same. A failure message above keeps the real path.
  auto labelled = [&f](std::string s) {
    for (size_t p = s.find(f.root); p != std::string::npos; p = s.find(f.root, p)) s.replace(p, f.root.size(), "<fixture>");
    return s;
  };
  for (const std::string& l : first.log) std::printf("[held run] %s\n", labelled(l).c_str());
  const HeldRun second = StartHeldRun(f.spec);
  EXPECT_FALSE(second.ok);
  EXPECT_EQ(second.code, "MARKER_EXISTS");
  std::printf("[held run] second start: %s: %s\n", second.code.c_str(), labelled(second.why).c_str());
}

// C-U24-3: a mismatch of the recorded inputs (f) refuses the run BEFORE the marker exists. After the input is repaired the same gate
// still starts: the refusal did not use up its one run.
TEST(OrientGateInstrument, InputMismatchRefusesBeforeTheMarkerAndKeepsTheRun) {
  struct Case {
    const char* name;
    const char* expect;  // substring of the reason
    std::function<void(HeldFixture*)> damage, repair;
  };
  const Case cases[] = {
      {"a listed file edited", "input differs from SHA256SUMS: d1/a.json",
       [](HeldFixture* f) { PutFile(f->rec + "/d1/a.json", "tampered"); },
       [](HeldFixture* f) { PutFile(f->rec + "/d1/a.json", "{\"a\":\"d1\"}\n"); }},
      {"a SHA256SUMS edited", "SHA256SUMS of d2 differs from the manifest",
       [](HeldFixture* f) {
         std::string s;
         sh::ReadFile(f->rec + "/d2/SHA256SUMS", &s);
         PutFile(f->rec + "/d2/SHA256SUMS", s + "\n");
       },
       [](HeldFixture* f) {
         std::string s;
         sh::ReadFile(f->rec + "/d2/SHA256SUMS", &s);
         PutFile(f->rec + "/d2/SHA256SUMS", s.substr(0, s.size() - 1));
       }},
      {"a listed file removed", "input missing or not a regular file: d2/tsv/x.tsv",
       [](HeldFixture* f) { std::filesystem::rename(f->rec + "/d2/tsv/x.tsv", f->rec + "/d2/x.tsv.moved"); },
       [](HeldFixture* f) { std::filesystem::rename(f->rec + "/d2/x.tsv.moved", f->rec + "/d2/tsv/x.tsv"); }},
      {"a recording the manifest does not register", "does not register d3",
       [](HeldFixture* f) {
         PutRecording(f->rec, "d3", "");
         f->spec.input_dirs.push_back("d3");
       },
       [](HeldFixture* f) { f->spec.input_dirs.pop_back(); }},
  };
  int n = 0;
  for (const Case& c : cases) {
    HeldFixture f = MakeHeldFixture("input" + std::to_string(n), 1 + n);
    ++n;
    c.damage(&f);
    const HeldRun refused = StartHeldRun(f.spec);
    EXPECT_FALSE(refused.ok) << c.name;
    EXPECT_EQ(refused.code, "INPUT") << c.name << ": " << refused.why;
    EXPECT_NE(refused.why.find(c.expect), std::string::npos) << c.name << ": '" << refused.why << "' does not contain '" << c.expect << "'";
    EXPECT_FALSE(Exists(f.marker)) << c.name << ": the refusal created the one-run marker";
    std::printf("[held run] %-44s refused (%s), marker %s\n", c.name, refused.why.c_str(), Exists(f.marker) ? "CREATED" : "absent");
    c.repair(&f);
    const HeldRun started = StartHeldRun(f.spec);
    EXPECT_TRUE(started.ok) << c.name << " (after repair): " << started.code << ": " << started.why;
    EXPECT_TRUE(Exists(f.marker)) << c.name;
  }
}

// A SHA256SUMS line that leaves the recording directory is refused even when the list itself is the registered one.
TEST(OrientGateInstrument, InputListCannotLeaveTheRecordingDirectory) {
  const std::string root = ::testing::TempDir() + "/orient_held_paths_" + std::to_string(::getpid());
  std::filesystem::remove_all(root);
  const std::string sums = Sha256Text("x") + "  ../x\n";
  PutFile(root + "/d9/SHA256SUMS", sums);
  PutFile(root + "/x", "x");
  sh::Json registered;
  std::string err;
  ASSERT_TRUE(sh::ParseJson("{\"d9\":\"" + Sha256Text(sums) + "\"}", &registered, &err)) << err;
  std::vector<std::string> log;
  const std::string why = VerifyRecordingInputs(root, "d9", registered, &log);
  EXPECT_NE(why.find("outside the directory"), std::string::npos) << why;
  EXPECT_NE(VerifyRecordingInputs(root, "../d9", registered, &log).find("bare name"), std::string::npos);
  // An empty list verifies nothing: refused, even though it is the registered (empty) file.
  PutFile(root + "/d8/SHA256SUMS", "");
  ASSERT_TRUE(sh::ParseJson("{\"d8\":\"" + Sha256Text("") + "\"}", &registered, &err)) << err;
  EXPECT_NE(VerifyRecordingInputs(root, "d8", registered, &log).find("lists no file"), std::string::npos);
}

// Any other refusal (env, gate designation, pinned binary) also happens before the marker.
TEST(OrientGateInstrument, ReleaseRefusalLeavesNoMarker) {
  struct Case {
    const char* name;
    const char* code;
    std::function<void(HeldFixture*)> damage;
  };
  const Case cases[] = {
      {"environment value not set", "ENV_MISSING", [](HeldFixture* f) { f->spec.env_value = ""; }},
      {"environment value is not the designation sha256", "ENV_MISMATCH", [](HeldFixture* f) { f->spec.env_value = std::string(64, '0'); }},
      {"gate not designated as <gate_id>.v<N>", "ARGS", [](HeldFixture* f) { f->spec.gate_name = "g1"; }},
      {"gate designated with a leading zero version", "ARGS", [](HeldFixture* f) { f->spec.gate_name = "g1.v01"; }},
      {"the held binary differs from the registered one", "PIN_MISMATCH", [](HeldFixture* f) { PutFile(f->exe, "rebuilt"); }},
      {"another gate designated", "MANIFEST_UNREADABLE", [](HeldFixture* f) { f->spec.gate_name = "g1.v99"; }},
  };
  int n = 0;
  for (const Case& c : cases) {
    HeldFixture f = MakeHeldFixture("rel" + std::to_string(n++), 1);
    c.damage(&f);
    const HeldRun refused = StartHeldRun(f.spec);
    EXPECT_FALSE(refused.ok) << c.name;
    EXPECT_EQ(refused.code, c.code) << c.name << ": " << refused.why;
    EXPECT_FALSE(Exists(f.marker)) << c.name << ": the refusal created the one-run marker";
    std::printf("[held run] %-52s %s\n", c.name, refused.code.c_str());
  }
}

// Loader side of the bit-exact channel null-run (tools/re/runtime/orient/channel_null_run.mjs, run in headless Chrome through the same
// code that carries the func 647 probe and reality.intrinsics): +0, -0, denormals, infinities arrive unchanged in the Recording.
TEST(OrientGateInstrument, ChannelNullRunKeepsEveryBitPattern) {
  const char* patterns[16] = {"00000000", "80000000", "00000001", "80000001", "007fffff", "807fffff", "00800000", "80800000", "3f800000", "bf800000",
                              "c08ccccd", "7f7fffff", "ff7fffff", "7f800000", "ff800000", "3dcccccd"};
  Recording rec;
  std::string err;
  ASSERT_TRUE(LoadRecording(fe::testing::TestDataDir() + "/orient/channel-null-run", &rec, &err)) << err;
  ASSERT_EQ(rec.calls.size(), 1u);
  ASSERT_EQ(rec.calls[0].size(), 5u);
  for (size_t j = 0; j < 5; ++j) {
    const OutEv& e = rec.calls[0][j];
    EXPECT_EQ(e.xb, ParseHex(patterns[3 * j])) << j;
    EXPECT_EQ(e.yb, ParseHex(patterns[3 * j + 1])) << j;
    EXPECT_EQ(e.zb, ParseHex(patterns[3 * j + 2])) << j;
  }
  ASSERT_EQ(rec.intrinsics.size(), 1u);
  for (size_t k = 0; k < 16; ++k) EXPECT_EQ(rec.intrinsics[0].bits[k], ParseHex(patterns[k])) << k;
  std::printf("[channel null-run] 15 probe floats and 16 intrinsics floats (+0, -0, denormals, inf, ordinary) read back unchanged\n");
}

// REPORT mode on an exploration recording (never on the registered ones).
TEST(OrientGateReport, ExploreDirectory) {
  const char* dir = std::getenv("SLAM_REFACT_ORIENT_REPORT_DIR");
  if (dir == nullptr || *dir == '\0') GTEST_SKIP() << "SLAM_REFACT_ORIENT_REPORT_DIR not set";
  const std::string d = dir;
  const size_t slash = d.find_last_of('/');
  const std::string base = slash == std::string::npos ? d : d.substr(slash + 1);
  ASSERT_NE(base.rfind("v2-", 0), 0u) << "REPORT mode refuses the registered recordings (v2-*)";
  Recording rec;
  std::string err;
  ASSERT_TRUE(LoadRecording(d, &rec, &err)) << err;
  const Result s = CompareSensors(rec, SensorMutation::kNone, 6);
  const Result s2 = CompareSensors647(rec, SensorMutation::kNone, 6);
  const Result k = CompareIntrinsics(rec, IntrinsicsMutation::kNone);
  const Result p = ComparePyramids(rec, PyramidMutation::kNone);
  std::printf("[REPORT %s]\n S : %s%s S': %s%s K : %s%s P : %s%s", base.c_str(), s.pass ? "PASS\n" : "FAIL\n", s.detail.c_str(),
              s2.pass ? "PASS\n" : "FAIL\n", s2.detail.c_str(), k.pass ? "PASS\n" : "FAIL\n", k.detail.c_str(), p.pass ? "PASS\n" : "FAIL\n", p.detail.c_str());
  if (std::getenv("SLAM_REFACT_ORIENT_REPORT_POSE") != nullptr) {  // needs the served frames of the whole window
    const Result pose = ComparePose(rec, orient_replay::Mutation::kNone);
    std::printf(" R : %s%s", pose.pass ? "PASS\n" : "FAIL\n", pose.detail.c_str());
    struct M { orient_replay::Mutation m; const char* name; };
    for (const M& m : {M{orient_replay::Mutation::kRotationPlus90, "engineInit rotation + 90"},
                       M{orient_replay::Mutation::kEventTimesPlus1Ms, "every DeviceMotion receive time + 1 ms"},
                       M{orient_replay::Mutation::kSkipOnePacket, "one DeviceMotion packet dropped"},
                       M{orient_replay::Mutation::kOrientationAlphaPlusHalfDegree, "deviceorientation alpha + 0.5 deg"}}) {
      const Result mr = ComparePose(rec, m.m);
      std::printf(" R mutation %-45s %s | %s", m.name, mr.pass ? "UNDETECTED" : "detected", mr.detail.substr(0, mr.detail.find('\n') + 1).c_str());
    }
  }
}

#else  // SR_ORIENT_HELD

struct HeldPlan {
  const char* name;                         // log label only; the gate id and version are designated by the caller
  std::vector<std::string> sensor_dirs;     // registered recordings with a func 647 probe (relative to the recording root)
  std::vector<std::string> intrinsics_dirs;
  std::vector<std::string> pyramid_dirs;
  std::vector<std::string> pose_dirs;
};

#ifndef SR_ORIENT_PREREG_DIR
#error "SR_ORIENT_PREREG_DIR"
#endif
#ifndef SR_ORIENT_REC_DIR
#error "SR_ORIENT_REC_DIR"
#endif

std::string EnvOr(const char* name, const char* fallback) {
  const char* v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

// The held gates. Nothing that can refuse the run follows the marker (StartHeldRun); the expensive part (engine vs the registered
// recordings) runs after it.
void RunHeld(const HeldPlan& plan) {
  HeldRunSpec spec;
  spec.prereg_dir = EnvOr("SLAM_REFACT_ORIENT_PREREG_DIR", SR_ORIENT_PREREG_DIR);
  spec.rec_dir = EnvOr("SLAM_REFACT_ORIENT_REC_DIR", SR_ORIENT_REC_DIR);
  spec.gate_name = EnvOr("SLAM_REFACT_ORIENT_GATE", "");
  spec.env_value = EnvOr("SLAM_REFACT_ORIENT_REVIEWER", "");
  spec.pins = {sh::SelfExe()};
  for (const std::vector<std::string>* dirs : {&plan.sensor_dirs, &plan.intrinsics_dirs, &plan.pyramid_dirs, &plan.pose_dirs}) {
    spec.input_dirs.insert(spec.input_dirs.end(), dirs->begin(), dirs->end());
  }
  const HeldRun run = StartHeldRun(spec);
  for (const std::string& l : run.log) std::printf("%s\n", l.c_str());
  ASSERT_TRUE(run.ok) << run.code << ": " << run.why;
  const char* gate = spec.gate_name.c_str();
  bool all = true;
  auto load = [&](const std::string& d, Recording* rec) {
    std::string err;
    const bool ok = LoadRecording(spec.rec_dir + "/" + d, rec, &err);
    EXPECT_TRUE(ok) << err;
    all = all && ok;
    return ok;
  };
  for (const std::string& d : plan.sensor_dirs) {
    Recording rec;
    if (!load(d, &rec)) continue;
    const Result s = CompareSensors(rec, SensorMutation::kNone, 6);
    const Result s2 = CompareSensors647(rec, SensorMutation::kNone, 6);
    std::printf("[GATE %s %s] S : %s %sS': %s %s", gate, d.c_str(), s.pass ? "PASS" : "FAIL", s.detail.c_str(), s2.pass ? "PASS" : "FAIL", s2.detail.c_str());
    all = all && s.pass && s2.pass;
  }
  for (const std::string& d : plan.intrinsics_dirs) {
    Recording rec;
    if (!load(d, &rec)) continue;
    const Result k = CompareIntrinsics(rec, IntrinsicsMutation::kNone);
    std::printf("[GATE %s %s] K : %s %s", gate, d.c_str(), k.pass ? "PASS" : "FAIL", k.detail.c_str());
    all = all && k.pass;
  }
  for (const std::string& d : plan.pyramid_dirs) {
    Recording rec;
    if (!load(d, &rec)) continue;
    const Result p = ComparePyramids(rec, PyramidMutation::kNone);
    std::printf("[GATE %s %s] P : %s %s", gate, d.c_str(), p.pass ? "PASS" : "FAIL", p.detail.c_str());
    all = all && p.pass;
  }
  for (const std::string& d : plan.pose_dirs) {
    Recording rec;
    if (!load(d, &rec)) continue;
    const Result pose = ComparePose(rec, orient_replay::Mutation::kNone);
    std::printf("[GATE %s %s] R : %s %s", gate, d.c_str(), pose.pass ? "PASS" : "FAIL", pose.detail.c_str());
    all = all && pose.pass;
  }
  EXPECT_TRUE(all);
}

TEST(OrientGateHeld, Us024) {
  RunHeld({"us024", {SR_ORIENT_US024_SENSOR_DIRS}, {SR_ORIENT_US024_K_DIRS}, {}, {}});
}
TEST(OrientGateHeld, Us023) {
  RunHeld({"us023", {SR_ORIENT_US023_SENSOR_DIRS}, {SR_ORIENT_US023_K_DIRS}, {SR_ORIENT_US023_PYRAMID_DIRS}, {SR_ORIENT_US023_POSE_DIRS}});
}

#endif  // SR_ORIENT_HELD

}  // namespace
}  // namespace slam_refact
