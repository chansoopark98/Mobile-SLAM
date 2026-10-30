#include "trace_writer.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

namespace slam_refact::harness {
namespace {

void Append(std::string* s, const char* key, const std::string& value) {
  if (s->size() > 1) s->push_back(',');
  s->push_back('"');
  s->append(key);
  s->append("\":");
  s->append(value);
}

std::string Bool(bool b) { return b ? "true" : "false"; }

std::string Vec(const float* v, int n) {
  std::string s = "[";
  for (int k = 0; k < n; ++k) {
    if (k > 0) s.push_back(',');
    s.append(JsNumber(static_cast<double>(v[k])));
  }
  s.push_back(']');
  return s;
}

}  // namespace

std::string JsNumber(double v) {
  if (!std::isfinite(v)) return "null";
  if (v == 0.0) return "0";  // also -0 (JSON.stringify(-0) == "0")
  // Shortest round-trip significand (ECMA-262 Number::toString picks the shortest k).
  char buf[40];
  int k = 1;
  for (; k <= 17; ++k) {
    std::snprintf(buf, sizeof(buf), "%.*e", k - 1, v);
    if (std::strtod(buf, nullptr) == v) break;
  }
  // buf = [-]d[.ddd]e[+-]xx
  std::string text(buf);
  const bool negative = text[0] == '-';
  if (negative) text.erase(0, 1);
  const size_t e = text.find('e');
  std::string digits = text.substr(0, 1) + (e > 1 ? text.substr(2, e - 2) : std::string());
  while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
  const int n = std::atoi(text.c_str() + e + 1) + 1;  // decimal point position
  const int kd = static_cast<int>(digits.size());
  std::string out = negative ? "-" : "";
  if (kd <= n && n <= 21) {
    out += digits + std::string(static_cast<size_t>(n - kd), '0');
  } else if (0 < n && n <= 21) {
    out += digits.substr(0, static_cast<size_t>(n)) + "." + digits.substr(static_cast<size_t>(n));
  } else if (-6 < n && n <= 0) {
    out += "0." + std::string(static_cast<size_t>(-n), '0') + digits;
  } else {
    out += digits.substr(0, 1);
    if (kd > 1) out += "." + digits.substr(1);
    out += (n - 1 >= 0 ? "e+" : "e-") + std::to_string(std::abs(n - 1));
  }
  return out;
}

std::string JsFixed(double v, int digits) {
  // Number.prototype.toFixed picks the larger n on an exact tie (|v|·10^digits ends in exactly .5), i.e. it
  // rounds ties away from zero; printf rounds exact ties to even (0.15625 -> "0.1562" vs JS "0.1563").
  // glibc prints the exact decimal expansion, so a tie shows as digit 5 followed only by zeros.
  char exact[1200];
  std::snprintf(exact, sizeof(exact), "%.*f", digits + 1100, v);
  const char* dot = std::strchr(exact, '.');
  bool tie = dot != nullptr && dot[digits + 1] == '5';
  for (const char* p = dot ? dot + digits + 2 : exact; tie && *p != '\0'; ++p) tie = *p == '0';
  if (tie) {
    const double step = std::pow(10.0, -digits);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", digits, v + (v < 0 ? -0.5 : 0.5) * step);
    return JsNumber(std::strtod(buf, nullptr));
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return JsNumber(std::strtod(buf, nullptr));
}

const char* StatusName(TrackingStatus s) {
  switch (s) {
    case TrackingStatus::kUnspecified: return "UNSPECIFIED";
    case TrackingStatus::kNotAvailable: return "NOT_AVAILABLE";
    case TrackingStatus::kLimited: return "LIMITED";
    case TrackingStatus::kNormal: return "NORMAL";
  }
  return "UNSPECIFIED";
}

const char* ReasonName(TrackingReason r) {
  switch (r) {
    case TrackingReason::kUnspecified: return "UNSPECIFIED";
    case TrackingReason::kInitializing: return "INITIALIZING";
    case TrackingReason::kRelocalizing: return "RELOCALIZING";
    case TrackingReason::kTooMuchMotion: return "TOO_MUCH_MOTION";
    case TrackingReason::kNotEnoughTexture: return "NOT_ENOUGH_TEXTURE";
  }
  return "UNSPECIFIED";
}

TraceWriter::~TraceWriter() {
  if (trace_ != nullptr) std::fclose(trace_);
  if (samples_ != nullptr) std::fclose(samples_);
}

bool TraceWriter::Open(const std::string& out_dir, int world_point_sample_every, std::string* error) {
  sample_every_ = world_point_sample_every;
  trace_ = std::fopen((out_dir + "/trace.jsonl").c_str(), "wx");
  samples_ = std::fopen((out_dir + "/worldpoints-sampled.jsonl").c_str(), "wx");
  if (trace_ == nullptr || samples_ == nullptr) {
    *error = "cannot create trace files in " + out_dir + " (existing files are never overwritten)";
    return false;
  }
  return true;
}

void TraceWriter::WriteDropped(const FrameRow& frame) {
  std::string s = "{";
  Append(&s, "i", std::to_string(frame.index));
  Append(&s, "t", JsNumber(frame.t));
  Append(&s, "videoTimeUs", std::to_string((frame.video_time_ns + 500) / 1000));
  for (const char* k : {"publishMs", "acceptedMs"}) Append(&s, k, "null");
  Append(&s, "fresh", "false");
  for (const char* k : {"callbackMs", "latencyMs", "status", "reason", "p", "q_wxyz", "worldPoints",
                        "wpAdded", "wpRemoved", "wpMeanConfidence", "xrGpuStageMs", "xrCpuStageMs"}) {
    Append(&s, k, "null");
  }
  s.append("}\n");
  std::fputs(s.c_str(), trace_);
  ++rows_;
}

void TraceWriter::WriteProcessed(const FrameRow& frame, const PublishedFrame& out) {
  ++processed_;
  // page-runtime.js onUpdate: world point id churn and mean confidence over the published list.
  const std::vector<WorldPointOut>& wp = out.world_points;
  std::set<uint64_t> ids;
  double cs = 0.0;
  int added = 0;
  for (const WorldPointOut& pt : wp) {
    ids.insert(pt.id);
    cs += static_cast<double>(pt.confidence);
    if (last_ids_.count(pt.id) == 0) ++added;
  }
  int removed = 0;
  for (uint64_t id : last_ids_) {
    if (ids.count(id) == 0) ++removed;
  }
  last_ids_ = ids;
  const std::string conf = wp.empty() ? "null" : JsNumber(cs / static_cast<double>(wp.size()));

  const float pose[7] = {out.position.x, out.position.y, out.position.z, out.rotation.w,
                         out.rotation.x, out.rotation.y, out.rotation.z};
  bool finite = true;
  for (float v : pose) finite = finite && std::isfinite(v);

  std::string s = "{";
  Append(&s, "i", std::to_string(frame.index));
  Append(&s, "t", JsNumber(frame.t));
  Append(&s, "videoTimeUs", std::to_string((frame.video_time_ns + 500) / 1000));
  for (const char* k : {"publishMs", "acceptedMs"}) Append(&s, k, "null");
  Append(&s, "fresh", "true");
  for (const char* k : {"callbackMs", "latencyMs"}) Append(&s, k, "null");
  Append(&s, "status", std::string("\"") + StatusName(out.status) + "\"");
  Append(&s, "reason", std::string("\"") + ReasonName(out.reason) + "\"");
  Append(&s, "p", finite ? Vec(pose, 3) : "null");
  Append(&s, "q_wxyz", finite ? Vec(pose + 3, 4) : "null");
  Append(&s, "worldPoints", std::to_string(wp.size()));
  Append(&s, "wpAdded", std::to_string(added));
  Append(&s, "wpRemoved", std::to_string(removed));
  Append(&s, "wpMeanConfidence", conf);
  for (const char* k : {"xrGpuStageMs", "xrCpuStageMs"}) Append(&s, k, "null");
  // Engine-only debug fields (architecture.md §6.3).
  if (out.metric) {  // accuracy mode ACC-2 only (absent from faithful traces)
    Append(&s, "srMetricScale", JsNumber(out.metric_scale));
    Append(&s, "srMetricSigmaLog", JsNumber(out.metric_sigma_log));
    Append(&s, "srMetricInitialized", Bool(out.metric_initialized));
    Append(&s, "srMetricLagS", JsNumber(out.metric_lag_s));
  }
  if (have_surfaces_) {  // accuracy mode ACC-3 only
    std::string f = "{";
    Append(&f, "detected", Bool(surfaces_.floor_detected));
    Append(&f, "height", surfaces_.floor_detected ? JsNumber(surfaces_.floor.height) : "null");
    Append(&f, "inliers", std::to_string(surfaces_.floor_detected ? surfaces_.floor.inliers : 0));
    Append(&f, "area", surfaces_.floor_detected ? JsNumber(surfaces_.floor.area) : "null");
    Append(&f, "others", std::to_string(surfaces_.others.size()));
    Append(&f, "cloud", std::to_string(surfaces_.cloud_points));
    Append(&s, "srFloor", f + "}");
    if (center_hits_.empty()) {
      Append(&s, "srHitCenter", "null");
    } else {
      const HitResult& h = center_hits_.front();
      std::string hs = "{";
      Append(&hs, "type", std::to_string(static_cast<int>(h.type)));
      Append(&hs, "distance", JsNumber(h.distance));
      Append(&hs, "y", JsNumber(h.position.y));
      Append(&s, "srHitCenter", hs + "}");
    }
    have_surfaces_ = false;
  }
  Append(&s, "srRepeated", Bool(out.repeated));
  Append(&s, "srSourceFrame", std::to_string(out.source_frame_index));
  if (out.step.has_value()) {
    const TrackerStep& st = *out.step;
    Append(&s, "srMapTrackerState", std::to_string(st.map_tracker_state));
    Append(&s, "srFrameTrackerOk", Bool(st.frame_tracker_ok));
    Append(&s, "srLossState", std::to_string(st.loss_state));
    Append(&s, "srKeyframes", std::to_string(st.num_keyframes));
    std::string by_source = "[";
    for (int k = 0; k < 5; ++k) {
      if (k > 0) by_source.push_back(',');
      by_source += std::to_string(st.num_points_by_source[k]);
    }
    Append(&s, "srPointsBySource", by_source + "]");
    Append(&s, "srGroundMatches", std::to_string(st.ground_matches));
    Append(&s, "srMapReset", Bool(st.map_reset));
    Append(&s, "srKeyframeCreated", Bool(st.keyframe_created));
    Append(&s, "srScaleLatched", Bool(st.scale_latched));
    Append(&s, "srLatchedScale", JsNumber(static_cast<double>(st.latched_scale)));
    Append(&s, "srKeypoints", std::to_string(st.num_keypoints));
    std::string ms = "{";
    Append(&ms, "frontend", JsNumber(st.timings.frontend_ms));
    Append(&ms, "mapTracker", JsNumber(st.timings.map_tracker_ms));
    Append(&ms, "mapBuilder", JsNumber(st.timings.map_builder_ms));
    Append(&ms, "metricScale", JsNumber(st.timings.metric_scale_ms));
    Append(&ms, "glue", JsNumber(st.timings.glue_ms));
    Append(&s, "srStageMs", ms + "}");
  } else {
    for (const char* k : {"srMapTrackerState", "srFrameTrackerOk", "srLossState", "srKeyframes",
                          "srPointsBySource", "srGroundMatches", "srMapReset", "srKeyframeCreated",
                          "srScaleLatched", "srLatchedScale", "srKeypoints", "srStageMs"}) {
      Append(&s, k, "null");
    }
  }
  s.append("}\n");
  std::fputs(s.c_str(), trace_);
  ++rows_;

  // page-runtime.js: sample every `sample_every_` fresh callbacks (freshCount % N == 1).
  if (sample_every_ > 0 && processed_ % sample_every_ == 1) {
    std::string p = "{";
    Append(&p, "frameIndex", std::to_string(frame.index));
    Append(&p, "videoTime", JsNumber(static_cast<double>(frame.video_time_ns) / 1e9));
    std::string pts = "[";
    for (size_t k = 0; k < wp.size(); ++k) {
      if (k > 0) pts.push_back(',');
      pts += "[" + std::to_string(wp[k].id) + "," + JsFixed(wp[k].confidence, 4) + "," +
             JsFixed(wp[k].position.x, 5) + "," + JsFixed(wp[k].position.y, 5) + "," +
             JsFixed(wp[k].position.z, 5) + "]";
    }
    Append(&p, "points", pts + "]");
    p.append("}\n");
    std::fputs(p.c_str(), samples_);
  }
}

bool TraceWriter::Close(std::string* error) {
  const bool ok = std::fclose(trace_) == 0 && std::fclose(samples_) == 0;
  trace_ = nullptr;
  samples_ = nullptr;
  if (!ok) *error = "error closing trace files";
  return ok;
}

void TraceWriter::SetSurfaces(const SurfaceState& state, const std::vector<HitResult>& center_hits) {
  have_surfaces_ = true;
  surfaces_ = state;
  center_hits_ = center_hits;
}

}  // namespace slam_refact::harness
