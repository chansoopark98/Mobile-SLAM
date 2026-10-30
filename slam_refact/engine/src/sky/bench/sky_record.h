// Reader of a run-sky-reference.mjs record (tools/sky/make_replay_input.py output, replay.txt).
// Bench / test only: the original runtime's recorded inputs and outputs (docs/spec/09-sky.md §6).
#ifndef SLAM_REFACT_SKY_BENCH_SKY_RECORD_H_
#define SLAM_REFACT_SKY_BENCH_SKY_RECORD_H_

#include <array>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "sky/sky_types.h"

namespace slam_refact::sky {

struct SkyRecord {
  std::string dir;
  int video_w = 0, video_h = 0, orientation = 0, canvas_w = 0, canvas_h = 0, processed = 0;
  bool mirrored = false;
  bool invert = false;
  double edge_smoothness = 0.0, near_clip = -1.0, far_clip = -1.0;
  struct Frame {
    int k = 0, w = 0, h = 0;
    std::string file;
    DeviceOrientation o;
  };
  struct Event {
    int staged = 0;
    std::string name;
    double percentage = 0.0;
  };
  struct Invoke {
    int k = 0, sent = 0, answered = 0;
    QuaternionXyzw rot;
    double percentage = 0.0;
    std::string img, data;
  };
  struct Resized {
    int k = 0, cols = 0, rows = 0;
    std::string file;
    QuaternionXyzw rot;
  };
  struct Update {
    int k = 0, idx = 0, tex_w = 0, tex_h = 0;
    std::string file;
    double percentage = 0.0;
    std::array<float, 16> intrinsics{};
    QuaternionXyzw rot;
    Position3 pos;
  };
  std::vector<Frame> frames;
  std::vector<Event> events;
  std::vector<Invoke> invokes;
  std::vector<Resized> resized;
  std::vector<Update> updates;

  bool ReadBytes(const std::string& rel, std::vector<uint8_t>* out) const {
    std::ifstream f(dir + "/" + rel, std::ios::binary);
    if (!f) {
      return false;
    }
    out->assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
  }
  bool ReadFloats(const std::string& rel, std::vector<float>* out) const {
    std::vector<uint8_t> b;
    if (!ReadBytes(rel, &b) || b.size() % 4 != 0) {
      return false;
    }
    out->resize(b.size() / 4);
    std::copy(b.begin(), b.end(), reinterpret_cast<uint8_t*>(out->data()));
    return true;
  }
  bool ReadImage(const std::string& rel, int w, int h, Rgba8Image* out) const {
    std::vector<uint8_t> b;
    if (!ReadBytes(rel, &b) || b.size() != static_cast<size_t>(w) * h * 4) {
      return false;
    }
    out->width = w;
    out->height = h;
    out->px = std::move(b);
    return true;
  }

  static bool Load(const std::string& dir, SkyRecord* rec, std::string* error) {
    std::ifstream f(dir + "/replay.txt");
    if (!f) {
      *error = "cannot open " + dir + "/replay.txt";
      return false;
    }
    rec->dir = dir;
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream s(line);
      std::string tag;
      s >> tag;
      if (tag == "meta") {
        int m = 0;
        s >> rec->video_w >> rec->video_h >> rec->orientation >> rec->canvas_w >> rec->canvas_h >>
            rec->processed >> m;
        rec->mirrored = m != 0;
      } else if (tag == "config") {
        int inv = 0;
        s >> inv >> rec->edge_smoothness >> rec->near_clip >> rec->far_clip;
        rec->invert = inv != 0;
      } else if (tag == "frame") {
        Frame fr;
        s >> fr.k >> fr.w >> fr.h >> fr.file >> fr.o.alpha >> fr.o.beta >> fr.o.gamma;
        rec->frames.push_back(fr);
      } else if (tag == "event") {
        Event e;
        s >> e.staged >> e.name;
        if (e.name == "layerfound") {
          s >> e.percentage;
        }
        rec->events.push_back(e);
      } else if (tag == "invoke") {
        Invoke v;
        s >> v.k >> v.sent >> v.answered >> v.rot.x >> v.rot.y >> v.rot.z >> v.rot.w >>
            v.percentage >> v.img >> v.data;
        rec->invokes.push_back(v);
      } else if (tag == "resized") {
        Resized r;
        s >> r.k >> r.cols >> r.rows >> r.file >> r.rot.x >> r.rot.y >> r.rot.z >> r.rot.w;
        rec->resized.push_back(r);
      } else if (tag == "update") {
        Update u;
        s >> u.k >> u.idx >> u.file >> u.tex_w >> u.tex_h >> u.percentage;
        for (float& v : u.intrinsics) {
          s >> v;
        }
        s >> u.rot.x >> u.rot.y >> u.rot.z >> u.rot.w >> u.pos.x >> u.pos.y >> u.pos.z;
        rec->updates.push_back(u);
      }
    }
    if (rec->frames.empty()) {
      *error = "no frames in " + dir + "/replay.txt";
      return false;
    }
    return true;
  }
};

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_BENCH_SKY_RECORD_H_
