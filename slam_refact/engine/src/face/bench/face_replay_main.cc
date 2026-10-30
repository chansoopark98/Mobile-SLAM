// Face/ear replay: RGBA frames through FaceController + FaceEventEmitter, JSON lines out
// (US-018/US-019; docs/spec/08-face-ear.md §8.4). Used for the comparison with the original
// xr-face runtime (slam_refact/tools/re/runtime/run-face-reference.mjs records the camera texture
// of every staged frame; this tool replays those exact pixels).
//
// Usage:
//   sr_face_replay --repo <root> --frames <dir> --out <jsonl> [--ears 0|1] [--mirrored 0|1]
//                  [--axes right|left] [--focal <px>] [--max-detections N] [--rendered-dir <dir>]
// <dir>/meta.json {"width","height","count"[,"timestamps_ms":[...]]}, frame-%04d.rgba (RGBA8,
// row 0 = top). One JSON object per staged frame: index, response (null before the first),
// debug detections, faces (FaceMsg fields), events; --rendered-dir writes the 384x384 ROI render
// of each processed frame (rendered-%04d.rgba).

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "face/face_controller.h"
#include "face/face_events.h"
#include "face/face_models.h"

namespace slam_refact::face {
namespace {

void Num(double v, std::string* out) {
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.9g", v);
  out->append(buf);
}

void Xyz(const Position32f& p, std::string* out) {
  out->append("[");
  Num(p.x, out);
  out->append(",");
  Num(p.y, out);
  out->append(",");
  Num(p.z, out);
  out->append("]");
}

void Detection(const DetectedPointsMsg& d, std::string* out) {
  out->append("{\"confidence\":");
  Num(d.confidence, out);
  out->append(",\"viewport\":[");
  Num(d.viewport_x, out);
  out->append(",");
  Num(d.viewport_y, out);
  out->append(",");
  Num(d.viewport_w, out);
  out->append(",");
  Num(d.viewport_h, out);
  out->append("],\"corners\":[");
  for (int i = 0; i < 4; ++i) {
    out->append(i ? ",[" : "[");
    Num(d.corners[i][0], out);
    out->append(",");
    Num(d.corners[i][1], out);
    out->append("]");
  }
  out->append("],\"renderTexToImageTex\":[");
  for (int i = 0; i < 16; ++i) {
    if (i) {
      out->append(",");
    }
    Num(d.render_tex_to_image_tex[i], out);
  }
  out->append("],\"points\":[");
  for (size_t i = 0; i < d.points.size(); ++i) {
    if (i) {
      out->append(",");
    }
    Xyz(d.points[i], out);
  }
  out->append("]}");
}

bool ReadInt(const std::string& json, const std::string& key, int* out) {
  const auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) {
    return false;
  }
  const auto colon = json.find(':', pos);
  if (colon == std::string::npos) {
    return false;
  }
  *out = std::atoi(json.c_str() + colon + 1);
  return true;
}

std::vector<double> ReadTimestamps(const std::string& json) {
  std::vector<double> out;
  const auto pos = json.find("\"timestamps_ms\"");
  if (pos == std::string::npos) {
    return out;
  }
  auto p = json.find('[', pos);
  const auto end = json.find(']', pos);
  if (p == std::string::npos || end == std::string::npos) {
    return out;
  }
  ++p;
  while (p < end) {
    char* next = nullptr;
    const double v = std::strtod(json.c_str() + p, &next);
    if (next == json.c_str() + p) {
      break;
    }
    out.push_back(v);
    p = static_cast<size_t>(next - json.c_str());
    while (p < end && (json[p] == ',' || json[p] == ' ' || json[p] == '\n')) {
      ++p;
    }
  }
  return out;
}

int Run(int argc, char** argv) {
  std::string repo = ".";
  std::string frames_dir;
  std::string out_path;
  std::string rendered_dir;
  bool ears = false;
  bool mirrored = false;
  bool right_handed = true;
  float focal = 0.0f;
  int max_detections = 1;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const std::string v = argv[i + 1];
    if (k == "--repo") {
      repo = v;
    } else if (k == "--frames") {
      frames_dir = v;
    } else if (k == "--out") {
      out_path = v;
    } else if (k == "--rendered-dir") {
      rendered_dir = v;
    } else if (k == "--ears") {
      ears = v != "0";
    } else if (k == "--mirrored") {
      mirrored = v != "0";
    } else if (k == "--axes") {
      right_handed = v != "left";
    } else if (k == "--focal") {
      focal = std::strtof(v.c_str(), nullptr);
    } else if (k == "--max-detections") {
      max_detections = std::atoi(v.c_str());
    } else {
      std::fprintf(stderr, "unknown argument %s\n", k.c_str());
      return 2;
    }
  }
  if (frames_dir.empty() || out_path.empty()) {
    std::fprintf(stderr, "--frames and --out are required\n");
    return 2;
  }
  std::ifstream meta_file(frames_dir + "/meta.json");
  std::stringstream meta;
  meta << meta_file.rdbuf();
  int width = 0;
  int height = 0;
  int count = 0;
  if (!ReadInt(meta.str(), "width", &width) || !ReadInt(meta.str(), "height", &height) ||
      !ReadInt(meta.str(), "count", &count) || width <= 0 || height <= 0 || count <= 0) {
    std::fprintf(stderr, "bad %s/meta.json\n", frames_dir.c_str());
    return 1;
  }
  const std::vector<double> timestamps = ReadTimestamps(meta.str());

  std::vector<uint8_t> detect_bytes;
  std::vector<uint8_t> mesh_bytes;
  std::vector<uint8_t> ear_bytes;
  std::string error;
  if (!ReadFileBytes(repo + "/" + kFaceDetectModelPath, &detect_bytes, &error) ||
      !ReadFileBytes(repo + "/" + kFaceMeshModelPath, &mesh_bytes, &error) ||
      !ReadFileBytes(repo + "/" + kFaceEarModelPath, &ear_bytes, &error)) {
    std::fprintf(stderr, "model file missing: %s\n", error.c_str());
    return 1;
  }

  FaceController controller;
  FaceOptions opts = DefaultFaceOptions();
  opts.enable_ears = ears;
  opts.max_detections = max_detections;
  opts.has_debug = true;
  opts.debug.detections = true;
  opts.debug.rendered_img = !rendered_dir.empty();
  opts.coordinates.mirrored_display = mirrored;
  opts.coordinates.axes = right_handed ? Axes::kRightHanded : Axes::kLeftHanded;
  controller.UpdateFaceOptions(opts);
  FaceCameraEnvironment env;
  env.camera_width = width;
  env.camera_height = height;
  env.display_width = width;
  env.display_height = height;
  if (focal > 0.0f) {
    env.device_intrinsics.focalLengthHorizontal = focal;
    env.device_intrinsics.focalLengthVertical = focal;
  }
  controller.UpdateCameraEnvironment(env);
  FaceEventEmitter emitter;
  std::ofstream out(out_path);
  {
    const auto attach = emitter.OnAttach(controller.GetFrameworkData());
    for (const auto& e : attach) {
      out << "{\"attach_event\":\"" << e.name << "\"}\n";
    }
  }
  if (!controller.LoadFaceDetectModel(detect_bytes, &error) ||
      !controller.LoadFaceMeshModel(mesh_bytes, &error) ||
      (ears && !controller.LoadEarModel(ear_bytes, &error))) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }

  std::vector<uint8_t> bytes;
  for (int i = 0; i < count; ++i) {
    char name[64];
    std::snprintf(name, sizeof(name), "/frame-%04d.rgba", i);
    if (!ReadFileBytes(frames_dir + name, &bytes, &error) ||
        bytes.size() != static_cast<size_t>(4 * width * height)) {
      std::fprintf(stderr, "bad frame %s\n", name);
      return 1;
    }
    if (!controller.StageFrame({bytes.data(), width, height, 4 * width}, &error)) {
      std::fprintf(stderr, "%s\n", error.c_str());
      return 1;
    }
    const FaceResponse* r = controller.ProcessStagedFrame();
    const double now = i < static_cast<int>(timestamps.size()) ? timestamps[i] : 50.0 * i;
    const auto events = emitter.OnProcessCpu(r, now);
    std::string line = "{\"index\":" + std::to_string(i) + ",\"now_ms\":";
    Num(now, &line);
    if (r == nullptr) {
      line.append(",\"response\":null");
    } else {
      line.append(",\"response\":{\"faces\":[");
      for (size_t f = 0; f < r->faces.size(); ++f) {
        const auto& face = r->faces[f];
        if (f) {
          line.append(",");
        }
        line.append("{\"id\":" + std::to_string(face.id) +
                    ",\"status\":" + std::to_string(static_cast<int>(face.status)) +
                    ",\"earVertices\":[");
        for (size_t v = 0; v < face.ear_vertices.size(); ++v) {
          if (v) {
            line.append(",");
          }
          Xyz(face.ear_vertices[v], &line);
        }
        line.append("]}");
      }
      line.append("],\"globalDetections\":[");
      for (size_t d = 0; d < r->debug.detections.faces.size(); ++d) {
        if (d) {
          line.append(",");
        }
        Detection(r->debug.detections.faces[d], &line);
      }
      line.append("],\"meshDetections\":[");
      for (size_t d = 0; d < r->debug.detections.meshes.size(); ++d) {
        if (d) {
          line.append(",");
        }
        Detection(r->debug.detections.meshes[d], &line);
      }
      line.append("],\"intrinsics\":[");
      for (int k = 0; k < 16; ++k) {
        if (k) {
          line.append(",");
        }
        Num(r->camera.intrinsic[k], &line);
      }
      line.append("]}");
      if (!rendered_dir.empty() && r->debug.has_rendered_img && controller.last_timings().processed) {
        std::snprintf(name, sizeof(name), "/rendered-%04d.rgba", i);
        std::ofstream img(rendered_dir + name, std::ios::binary);
        img.write(reinterpret_cast<const char*>(r->debug.rendered_img.pixels.data()),
                  static_cast<std::streamsize>(r->debug.rendered_img.pixels.size()));
      }
    }
    line.append(",\"events\":[");
    for (size_t e = 0; e < events.size(); ++e) {
      if (e) {
        line.append(",");
      }
      line.append("{\"name\":\"" + events[e].name + "\",\"detail\":" + events[e].detail_json + "}");
    }
    const auto& t = controller.last_timings();
    line.append("],\"timing_ms\":{\"render\":");
    Num(t.render_ms, &line);
    line.append(",\"track\":");
    Num(t.track_ms, &line);
    line.append(",\"ears\":");
    Num(t.ears_ms, &line);
    line.append(",\"response\":");
    Num(t.response_ms, &line);
    line.append("}}");
    out << line << "\n";
  }
  return 0;
}

}  // namespace
}  // namespace slam_refact::face

int main(int argc, char** argv) { return slam_refact::face::Run(argc, argv); }
