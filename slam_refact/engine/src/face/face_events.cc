// FaceController event emission (US-018/US-019). See face_events.h; line references are to
// PUB:reality/app/xr/js/src/face-controller.ts.

#include "face/face_events.h"

#include <algorithm>
#include <cstdio>
#include <utility>

namespace slam_refact::face {
namespace {

constexpr char kPrefix[] = "facecontroller.";

// Float32 values printed with 9 significant digits (round-trips the float the JS reads).
void AppendNumber(double v, std::string* out) {
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.9g", v);
  out->append(buf);
}

void AppendXyz(const Position32f& p, std::string* out) {
  out->append("{\"x\":");
  AppendNumber(p.x, out);
  out->append(",\"y\":");
  AppendNumber(p.y, out);
  out->append(",\"z\":");
  AppendNumber(p.z, out);
  out->append("}");
}

void AppendBool(const char* key, bool v, std::string* out) {
  out->append(",\"").append(key).append("\":").append(v ? "true" : "false");
}

std::string IdDetail(int32_t id) { return "{\"id\":" + std::to_string(id) + "}"; }

// face-controller.ts:236-243 reduceAttachmentPoint: name -> {position} (rotation is commented
// out in the original). A repeated name keeps its first key position and takes the last value, as
// a JS object assignment does.
void AppendAttachmentPoints(const std::vector<AttachmentPointOut>& points, std::string* out) {
  std::vector<std::pair<std::string, Position32f>> entries;
  for (const auto& a : points) {
    const std::string name = AttachmentJsName(a.name);
    auto it = std::find_if(entries.begin(), entries.end(),
                           [&name](const auto& e) { return e.first == name; });
    if (it == entries.end()) {
      entries.emplace_back(name, a.position);
    } else {
      it->second = a.position;
    }
  }
  for (size_t i = 0; i < entries.size(); ++i) {
    if (i > 0) {
      out->append(",");
    }
    out->append("\"").append(entries[i].first).append("\":{\"position\":");
    AppendXyz(entries[i].second, out);
    out->append("}");
  }
}

// Point names of EarPointStateKeys (:79-82, key minus "Found").
constexpr const char* kEarPointNames[6] = {"leftLobe",  "leftCanal",  "leftHelix",
                                           "rightLobe", "rightCanal", "rightHelix"};

bool EarPointValue(const FaceMsg& f, int i) {
  switch (i) {
    case 0:
      return f.left_lobe_found;
    case 1:
      return f.left_canal_found;
    case 2:
      return f.left_helix_found;
    case 3:
      return f.right_lobe_found;
    case 4:
      return f.right_canal_found;
    default:
      return f.right_helix_found;
  }
}

}  // namespace

std::string FaceDataJson(const FaceMsg& f) {
  // mapFaces (:245-293) minus the EarPointStateKeys deleted by removeExtraEarData.
  std::string out;
  out.append("{\"id\":").append(std::to_string(f.id));
  out.append(",\"transform\":{\"position\":");
  AppendXyz(f.transform.position, &out);
  out.append(",\"rotation\":{\"w\":");
  AppendNumber(f.transform.rotation.w, &out);
  out.append(",\"x\":");
  AppendNumber(f.transform.rotation.x, &out);
  out.append(",\"y\":");
  AppendNumber(f.transform.rotation.y, &out);
  out.append(",\"z\":");
  AppendNumber(f.transform.rotation.z, &out);
  out.append("},\"scale\":");
  AppendNumber(f.transform.scale, &out);
  out.append(",\"scaledWidth\":");
  AppendNumber(f.transform.scaled_width, &out);
  out.append(",\"scaledHeight\":");
  AppendNumber(f.transform.scaled_height, &out);
  out.append(",\"scaledDepth\":");
  AppendNumber(f.transform.scaled_depth, &out);
  out.append("},\"vertices\":[");
  for (size_t i = 0; i < f.vertices.size(); ++i) {
    if (i > 0) {
      out.append(",");
    }
    AppendXyz(f.vertices[i], &out);
  }
  out.append("],\"normals\":[");
  for (size_t i = 0; i < f.normals.size(); ++i) {
    if (i > 0) {
      out.append(",");
    }
    AppendXyz(f.normals[i], &out);
  }
  out.append("],\"uvsInCameraFrame\":[");
  for (size_t i = 0; i < f.uvs_in_camera_frame.size(); ++i) {
    if (i > 0) {
      out.append(",");
    }
    out.append("{\"u\":");
    AppendNumber(f.uvs_in_camera_frame[i].u, &out);
    out.append(",\"v\":");
    AppendNumber(f.uvs_in_camera_frame[i].v, &out);
    out.append("}");
  }
  // Face attachment points first, then the ear ones (getEarAttachmentPoints().reduce(...)).
  out.append("],\"attachmentPoints\":{");
  std::vector<AttachmentPointOut> all = f.attachment_points;
  all.insert(all.end(), f.ear_attachment_points.begin(), f.ear_attachment_points.end());
  AppendAttachmentPoints(all, &out);
  out.append("}");
  AppendBool("mouthOpen", f.mouth_open, &out);
  AppendBool("leftEyeOpen", f.left_eye_open, &out);
  AppendBool("rightEyeOpen", f.right_eye_open, &out);
  AppendBool("leftEyebrowRaised", f.left_eyebrow_raised, &out);
  AppendBool("rightEyebrowRaised", f.right_eyebrow_raised, &out);
  out.append(",\"interpupillaryDistanceInMM\":");
  AppendNumber(f.interpupillary_distance_in_mm, &out);
  AppendBool("leftEarFound", f.left_ear_found, &out);
  AppendBool("rightEarFound", f.right_ear_found, &out);
  out.append("}");
  return out;
}

std::string ModelGeometryJson(const ModelGeometryMsg& m) {
  // modelGeometry_ (:843-860).
  std::string out;
  out.append("{\"maxDetections\":").append(std::to_string(m.max_detections));
  out.append(",\"pointsPerDetection\":").append(std::to_string(m.points_per_detection));
  out.append(",\"indices\":[");
  for (size_t i = 0; i < m.indices.size(); ++i) {
    if (i > 0) {
      out.append(",");
    }
    out.append("{\"a\":").append(std::to_string(m.indices[i].a));
    out.append(",\"b\":").append(std::to_string(m.indices[i].b));
    out.append(",\"c\":").append(std::to_string(m.indices[i].c)).append("}");
  }
  out.append("],\"uvs\":[");
  for (size_t i = 0; i < m.uvs.size(); ++i) {
    if (i > 0) {
      out.append(",");
    }
    out.append("{\"u\":");
    AppendNumber(m.uvs[i].u, &out);
    out.append(",\"v\":");
    AppendNumber(m.uvs[i].v, &out);
    out.append("}");
  }
  out.append("]}");
  return out;
}

std::vector<FaceEvent> FaceEventEmitter::OnAttach(const ModelGeometryMsg& model_geometry) {
  const std::string detail = ModelGeometryJson(model_geometry);
  return {{std::string(kPrefix) + "faceloading", detail},
          {std::string(kPrefix) + "facescanning", detail}};
}

void FaceEventEmitter::AddTimer(double due_ms, TimerKind kind, bool left, int32_t face_id) {
  timers_.push_back({due_ms, next_seq_++, kind, left, face_id});
}

std::vector<FaceEvent> FaceEventEmitter::AdvanceTo(double now_ms) {
  std::vector<FaceEvent> out;
  while (true) {
    auto it = std::min_element(timers_.begin(), timers_.end(), [](const Timer& a, const Timer& b) {
      return a.due_ms < b.due_ms || (a.due_ms == b.due_ms && a.seq < b.seq);
    });
    if (it == timers_.end() || it->due_ms > now_ms) {
      break;
    }
    const Timer t = *it;
    timers_.erase(it);
    auto state = face_state_.find(t.face_id);
    if (t.kind == TimerKind::kWinkExpire) {
      // maybeWinkOrBlinkStarted timeout (:381-385).
      if (state != face_state_.end()) {
        (t.left ? state->second.left_eye_winked : state->second.right_eye_winked) = false;
      }
      continue;
    }
    // maybeWinkOrBlinkEnded timeout (:390-401).
    if (state == face_state_.end()) {
      continue;
    }
    FaceState& s = state->second;
    const std::string detail = IdDetail(t.face_id);
    const bool left_winked = s.left_eye_winked.value_or(false);
    const bool right_winked = s.right_eye_winked.value_or(false);
    const bool key_winked = (t.left ? s.left_eye_winked : s.right_eye_winked).value_or(false);
    if (left_winked && right_winked) {
      out.push_back({std::string(kPrefix) + "blinked", detail});
    } else if (key_winked) {
      // winkKey.toLowerCase(): lefteyewinked / righteyewinked.
      out.push_back(
          {std::string(kPrefix) + (t.left ? "lefteyewinked" : "righteyewinked"), detail});
    }
    s.left_eye_winked = false;
    s.right_eye_winked = false;
  }
  return out;
}

void FaceEventEmitter::EmitFaceStateUpdates(const FaceMsg& face, double now_ms,
                                            std::vector<FaceEvent>* out) {
  // maybeEmitFaceStateUpdates (:403-491).
  FaceState& stored = face_state_[face.id];  // created with interpupillaryDistanceInMM 0
  const std::string detail = IdDetail(face.id);

  struct Expression {
    std::optional<bool>* stored;
    bool value;
    const char* on;
    const char* off;
    int eye;  // 0: not an eye key, 1: left, 2: right
  };
  const Expression expressions[5] = {
      {&stored.mouth_open, face.mouth_open, "mouthopened", "mouthclosed", 0},
      {&stored.left_eye_open, face.left_eye_open, "lefteyeopened", "lefteyeclosed", 1},
      {&stored.right_eye_open, face.right_eye_open, "righteyeopened", "righteyeclosed", 2},
      {&stored.left_eyebrow_raised, face.left_eyebrow_raised, "lefteyebrowraised",
       "lefteyebrowlowered", 0},
      {&stored.right_eyebrow_raised, face.right_eyebrow_raised, "righteyebrowraised",
       "righteyebrowlowered", 0},
  };
  for (const auto& e : expressions) {
    if (*e.stored == std::optional<bool>(e.value)) {
      continue;
    }
    out->push_back({std::string(kPrefix) + (e.value ? e.on : e.off), detail});
    if (e.eye != 0) {
      const bool left = e.eye == 1;
      if (e.value) {
        AddTimer(now_ms + 100.0, TimerKind::kWinkOrBlinkEnded, left, face.id);
      } else {
        (left ? stored.left_eye_winked : stored.right_eye_winked) = true;
        AddTimer(now_ms + 750.0, TimerKind::kWinkExpire, left, face.id);
      }
    }
    *e.stored = e.value;
  }

  // BuildIf.EARS_20230911 (enabled in the shipped xr-face.js).
  const bool ears[2] = {face.left_ear_found, face.right_ear_found};
  std::optional<bool>* stored_ears[2] = {&stored.left_ear_found, &stored.right_ear_found};
  for (int i = 0; i < 2; ++i) {
    if (*stored_ears[i] == std::optional<bool>(ears[i])) {
      continue;
    }
    out->push_back({std::string(kPrefix) + (ears[i] ? "earfound" : "earlost"),
                    "{\"id\":" + std::to_string(face.id) + ",\"ear\":\"" +
                        (i == 0 ? "left" : "right") + "\"}"});
    *stored_ears[i] = ears[i];
  }
  for (int i = 0; i < 6; ++i) {
    const bool value = EarPointValue(face, i);
    if (stored.ear_points[i] == std::optional<bool>(value)) {
      continue;
    }
    out->push_back({std::string(kPrefix) + (value ? "earpointfound" : "earpointlost"),
                    "{\"id\":" + std::to_string(face.id) + ",\"point\":\"" + kEarPointNames[i] +
                        "\"}"});
    stored.ear_points[i] = value;
  }

  if (stored.interpupillary_distance_in_mm != face.interpupillary_distance_in_mm) {
    std::string ipd = "{\"interpupillaryDistance\":";
    AppendNumber(face.interpupillary_distance_in_mm, &ipd);
    ipd.append(",\"id\":").append(std::to_string(face.id)).append("}");
    out->push_back({std::string(kPrefix) + "interpupillarydistance", ipd});
    stored.interpupillary_distance_in_mm = face.interpupillary_distance_in_mm;
  }
}

std::vector<FaceEvent> FaceEventEmitter::OnProcessCpu(const FaceResponse* response,
                                                      double now_ms) {
  std::vector<FaceEvent> out = AdvanceTo(now_ms);
  if (response == nullptr) {
    return out;
  }
  // onProcessCpu (:1043-1058).
  for (const auto& f : response->faces) {
    if (f.status == FaceTrackingStatus::kLost) {
      out.push_back({std::string(kPrefix) + "facelost", IdDetail(f.id)});
    } else if (f.status == FaceTrackingStatus::kFound) {
      out.push_back({std::string(kPrefix) + "facefound", FaceDataJson(f)});
      EmitFaceStateUpdates(f, now_ms, &out);
    } else if (f.status == FaceTrackingStatus::kUpdated) {
      out.push_back({std::string(kPrefix) + "faceupdated", FaceDataJson(f)});
      EmitFaceStateUpdates(f, now_ms, &out);
    }
  }
  return out;
}

void FaceEventEmitter::OnDetach() { face_state_.clear(); }

}  // namespace slam_refact::face
