#include "bundle_reader.h"

#include <zlib.h>

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace slam_refact::harness {
namespace {

constexpr int kAtlasWidth = 1024;  // S1 §2.2 pyramid texture size

std::vector<std::string> SplitTabs(const std::string& line) {
  std::vector<std::string> out;
  size_t start = 0;
  while (true) {
    const size_t tab = line.find('\t', start);
    out.push_back(line.substr(start, tab == std::string::npos ? std::string::npos : tab - start));
    if (tab == std::string::npos) return out;
    start = tab + 1;
  }
}

bool ParseInt64(const std::string& s, int64_t* v) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long long x = std::strtoll(s.c_str(), &end, 10);
  if (errno != 0 || *end != '\0') return false;
  *v = static_cast<int64_t>(x);
  return true;
}

bool ParseDouble(const std::string& s, double* v) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double x = std::strtod(s.c_str(), &end);
  if (errno != 0 || *end != '\0') return false;
  *v = x;
  return true;
}

bool ParseVec3(const std::vector<std::string>& f, size_t at, Vec3f* v) {
  double x = 0.0, y = 0.0, z = 0.0;
  if (!ParseDouble(f[at], &x) || !ParseDouble(f[at + 1], &y) || !ParseDouble(f[at + 2], &z)) {
    return false;
  }
  // W3C doubles enter the frozen sample types as float (sensor_types.h).
  *v = Vec3f{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
  return true;
}

bool ReadLines(const std::string& path, std::vector<std::string>* lines, std::string* error) {
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty()) lines->push_back(line);
  }
  return true;
}

bool ParseSensorRow(const std::string& line, SensorRow* row) {
  // phase t_ns aIG(3) lin(3|-) rate(3) interval_ms orientation(3)
  const std::vector<std::string> f = SplitTabs(line);
  if (f.size() != 15) return false;
  if (f[0] != "bootstrap" && f[0] != "run") return false;
  row->bootstrap = f[0] == "bootstrap";
  int64_t t_ns = 0;
  if (!ParseInt64(f[1], &t_ns)) return false;
  row->motion.receive_time_ns = t_ns;
  row->orientation.receive_time_ns = t_ns;
  Vec3f v;
  if (!ParseVec3(f, 2, &v)) return false;
  row->motion.acceleration_including_gravity = v;
  if (f[5] == "-") {
    if (f[6] != "-" || f[7] != "-") return false;
    row->motion.acceleration.reset();
  } else {
    if (!ParseVec3(f, 5, &v)) return false;
    row->motion.acceleration = v;
  }
  if (!ParseVec3(f, 8, &v)) return false;
  row->motion.rotation_rate_abg = v;
  if (!ParseDouble(f[11], &row->motion.interval_ms)) return false;
  return ParseDouble(f[12], &row->orientation.alpha_deg) &&
         ParseDouble(f[13], &row->orientation.beta_deg) &&
         ParseDouble(f[14], &row->orientation.gamma_deg);
}

bool ParseFrameRow(const std::string& line, FrameRow* row) {
  // kind index time_ns video_time_ns t process image pyramid [frame_time_ns]
  const std::vector<std::string> f = SplitTabs(line);
  if (f.size() != 8 && f.size() != 9) return false;
  if (f.size() == 9 && !ParseInt64(f[8], &row->frame_time_ns)) return false;
  if (f[0] != "bootstrap" && f[0] != "frame") return false;
  row->bootstrap = f[0] == "bootstrap";
  int64_t index = 0;
  if (!ParseInt64(f[1], &index) || !ParseInt64(f[2], &row->time_ns) ||
      !ParseInt64(f[3], &row->video_time_ns) || !ParseDouble(f[4], &row->t)) {
    return false;
  }
  row->index = static_cast<int>(index);
  if (f[5] != "0" && f[5] != "1") return false;
  row->process = f[5] == "1";
  row->image = f[6];
  row->pyramid = f[7] == "-" ? std::string() : f[7];
  return !row->image.empty();
}

bool ConfigInt(const std::map<std::string, std::string>& c, const char* key, int* v) {
  const auto it = c.find(key);
  int64_t x = 0;
  if (it == c.end() || !ParseInt64(it->second, &x)) return false;
  *v = static_cast<int>(x);
  return true;
}

std::vector<float> ConfigFloats(const std::map<std::string, std::string>& c, const char* key) {
  std::vector<float> out;
  const auto it = c.find(key);
  if (it == c.end()) return out;
  std::istringstream in(it->second);
  std::string tok;
  while (in >> tok) {
    double d = 0.0;
    if (!ParseDouble(tok, &d)) return {};
    out.push_back(static_cast<float>(d));
  }
  return out;
}

}  // namespace

bool LoadBundle(const std::string& dir, ReplayBundle* out, std::string* error) {
  out->dir = dir;
  std::vector<std::string> lines;
  if (!ReadLines(dir + "/engine-config.tsv", &lines, error)) return false;
  for (const std::string& line : lines) {
    const size_t tab = line.find('\t');
    if (tab == std::string::npos) {
      *error = "engine-config.tsv: no tab in '" + line + "'";
      return false;
    }
    out->config[line.substr(0, tab)] = line.substr(tab + 1);
  }
  for (const char* key : {"capture_width", "capture_height", "rotation_deg", "manufacturer", "model",
                          "estimate_scale", "disable_vio", "pyramid_model", "origin", "facing_wxyz",
                          "left_handed_axes", "mirrored_display", "world_point_sample_every"}) {
    if (out->config.find(key) == out->config.end()) {
      *error = std::string("engine-config.tsv: missing key ") + key;
      return false;
    }
  }
  lines.clear();
  if (!ReadLines(dir + "/sensors.tsv", &lines, error)) return false;
  bool seen_run = false;
  int64_t last_run_ns = 0;
  for (size_t k = 0; k < lines.size(); ++k) {
    SensorRow row;
    if (!ParseSensorRow(lines[k], &row)) {
      *error = "sensors.tsv: malformed line " + std::to_string(k + 1);
      return false;
    }
    if (row.bootstrap && seen_run) {
      *error = "sensors.tsv: bootstrap row after run rows";
      return false;
    }
    if (!row.bootstrap) {
      if (seen_run && row.motion.receive_time_ns < last_run_ns) {
        *error = "sensors.tsv: run rows out of time order at line " + std::to_string(k + 1);
        return false;
      }
      seen_run = true;
      last_run_ns = row.motion.receive_time_ns;
    }
    out->sensors.push_back(row);
  }
  lines.clear();
  if (!ReadLines(dir + "/frames.tsv", &lines, error)) return false;
  for (size_t k = 0; k < lines.size(); ++k) {
    FrameRow row;
    if (!ParseFrameRow(lines[k], &row) || row.bootstrap != (k == 0)) {
      *error = "frames.tsv: malformed line " + std::to_string(k + 1) +
               " (the bootstrap row must be first and unique)";
      return false;
    }
    if (k > 0 && row.time_ns <= out->frames.back().time_ns) {
      *error = "frames.tsv: frame times not increasing at line " + std::to_string(k + 1);
      return false;
    }
    out->frames.push_back(row);
  }
  if (out->frames.size() < 2) {
    *error = "frames.tsv: no input frames";
    return false;
  }
  return true;
}

EngineConfig ReplayBundle::MakeEngineConfig() const {
  EngineConfig cfg;
  int v = 0;
  cfg.estimate_scale = ConfigInt(config, "estimate_scale", &v) && v != 0;
  cfg.disable_vio = ConfigInt(config, "disable_vio", &v) && v != 0;
  ConfigInt(config, "capture_width", &cfg.capture_width);
  ConfigInt(config, "capture_height", &cfg.capture_height);
  ConfigInt(config, "rotation_deg", &cfg.rotation_deg);
  cfg.manufacturer = config.at("manufacturer");
  cfg.model = config.at("model");
  cfg.pyramid_model = config.at("pyramid_model") == "ieee" ? ShaderNumericModel::kIeee
                                                           : ShaderNumericModel::kSwiftShaderApprox;
  // Optional: "fx fy cx cy" of the capture image (diagnostic K override, EngineConfig).
  const std::vector<float> k = ConfigFloats(config, "intrinsics");
  if (k.size() == 4) {
    PinholeIntrinsics in;
    in.width = cfg.capture_width;
    in.height = cfg.capture_height;
    in.fx = k[0];
    in.fy = k[1];
    in.cx = k[2];
    in.cy = k[3];
    cfg.intrinsics_override = in;
  }
  // Optional: "Key=b:1;Key=i:3;Key=f:0.5;Key=d:0.1" process-global parameter overrides.
  const auto params = config.find("params");
  if (params != config.end()) {
    std::istringstream in(params->second);
    std::string item;
    while (std::getline(in, item, ';')) {
      const size_t eq = item.find('=');
      if (eq == std::string::npos || eq + 3 > item.size() || item[eq + 2] != ':') continue;
      const std::string key = item.substr(0, eq);
      const char type = item[eq + 1];
      const std::string value = item.substr(eq + 3);
      switch (type) {
        case 'b': cfg.param_overrides.emplace_back(key, ParamValue(value == "1")); break;
        case 'i': cfg.param_overrides.emplace_back(key, ParamValue(std::stoi(value))); break;
        case 'f': cfg.param_overrides.emplace_back(key, ParamValue(std::stof(value))); break;
        case 'd': cfg.param_overrides.emplace_back(key, ParamValue(std::stod(value))); break;
        default: break;
      }
    }
  }
  return cfg;
}

GlueConfig ReplayBundle::MakeGlueConfig() const {
  GlueConfig cfg;
  const std::vector<float> o = ConfigFloats(config, "origin");
  if (o.size() == 3) cfg.origin = Vec3f{o[0], o[1], o[2]};
  const std::vector<float> q = ConfigFloats(config, "facing_wxyz");
  if (q.size() == 4) cfg.facing = QuatWxyz{q[0], q[1], q[2], q[3]};
  int v = 0;
  cfg.left_handed_axes = ConfigInt(config, "left_handed_axes", &v) && v != 0;
  cfg.mirrored_display = ConfigInt(config, "mirrored_display", &v) && v != 0;
  return cfg;
}

bool ReadPgm(const std::string& path, std::vector<uint8_t>* pixels, int* width, int* height,
             std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  std::string magic;
  long long w = 0, h = 0, maxval = 0;
  // Header tokens may be separated by whitespace and '#' comments.
  auto next_token = [&in](std::string* tok) {
    tok->clear();
    int c = in.get();
    while (c != EOF) {
      if (c == '#') {
        while (c != EOF && c != '\n') c = in.get();
      } else if (!std::isspace(c)) {
        break;
      }
      c = in.get();
    }
    while (c != EOF && !std::isspace(c)) {
      tok->push_back(static_cast<char>(c));
      c = in.get();
    }
    return !tok->empty();
  };
  std::string tw, th, tm;
  if (!next_token(&magic) || !next_token(&tw) || !next_token(&th) || !next_token(&tm) ||
      magic != "P5") {
    *error = path + ": not a binary PGM";
    return false;
  }
  w = std::strtoll(tw.c_str(), nullptr, 10);
  h = std::strtoll(th.c_str(), nullptr, 10);
  maxval = std::strtoll(tm.c_str(), nullptr, 10);
  if (w <= 0 || h <= 0 || maxval != 255) {
    *error = path + ": unsupported PGM header";
    return false;
  }
  pixels->resize(static_cast<size_t>(w * h));
  in.read(reinterpret_cast<char*>(pixels->data()), static_cast<std::streamsize>(pixels->size()));
  if (in.gcount() != static_cast<std::streamsize>(pixels->size())) {
    *error = path + ": truncated PGM";
    return false;
  }
  *width = static_cast<int>(w);
  *height = static_cast<int>(h);
  return true;
}

bool ReadGzipAtlas(const std::string& path, std::vector<uint8_t>* rgba, int* rows,
                   std::string* error) {
  gzFile f = gzopen(path.c_str(), "rb");
  if (f == nullptr) {
    *error = "cannot open " + path;
    return false;
  }
  rgba->clear();
  std::vector<uint8_t> chunk(1 << 20);
  int n = 0;
  while ((n = gzread(f, chunk.data(), static_cast<unsigned>(chunk.size()))) > 0) {
    rgba->insert(rgba->end(), chunk.begin(), chunk.begin() + n);
  }
  int errnum = Z_OK;
  const char* msg = gzerror(f, &errnum);
  gzclose(f);
  if (n < 0 || (errnum != Z_OK && errnum != Z_STREAM_END)) {
    *error = path + ": gzip error " + (msg != nullptr ? msg : "");
    return false;
  }
  const size_t row_bytes = static_cast<size_t>(4 * kAtlasWidth);
  if (rgba->empty() || rgba->size() % row_bytes != 0) {
    *error = path + ": atlas size is not a multiple of 1024 RGBA texels";
    return false;
  }
  *rows = static_cast<int>(rgba->size() / row_bytes);
  return true;
}

bool LoadImages(const FrameRow& frame, LoadedImages* out, std::string* error) {
  if (!ReadPgm(frame.image, &out->gray, &out->width, &out->height, error)) return false;
  out->atlas.clear();
  out->atlas_rows = 0;
  if (!frame.pyramid.empty()) {
    return ReadGzipAtlas(frame.pyramid, &out->atlas, &out->atlas_rows, error);
  }
  return true;
}

FrameInput MakeFrameInput(const FrameRow& frame, const LoadedImages& images) {
  FrameInput in;
  // 647: frameTimestampNanos when non-zero, else timestampNanos (the binding time).
  in.time_ns = frame.frame_time_ns != 0 ? frame.frame_time_ns : frame.time_ns;
  in.video_time_ns = frame.video_time_ns;
  in.gray = GrayImageView{images.gray.data(), images.width, images.height, images.width};
  if (!images.atlas.empty()) {
    in.pyramid = PyramidAtlasView{images.atlas.data(), kAtlasWidth, images.atlas_rows};
  }
  return in;
}

}  // namespace slam_refact::harness
