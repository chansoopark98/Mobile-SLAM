// US-015: engine side of the func994 oracle comparison. Reads the flat cases written by
// slam_refact/tools/re/iwt_oracle/make_cases.py --flat, runs image_target::TrackImageWorld on each
// and prints the same fields run_f994.mjs reads from the original xr-slam.wasm func994
// (compare with slam_refact/tools/re/iwt_oracle/compare.py). Development tool, not a test.
//
//   sr_iwt_replay CASES.txt > ENGINE.txt

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "c8/geometry/parameterized-geometry.h"
#include "c8/hmatrix.h"
#include "image_target/image_world_tracker.h"
#include "reality/engine/imagedetection/detection-image.h"
#include "reality/engine/imagedetection/target-point.h"

namespace {

using slam_refact::image_target::MapToTrackerTransform;
using slam_refact::image_target::TrackedMapPoint;
using slam_refact::image_target::TrackedPointLookup;

[[noreturn]] void Fail(const std::string& what) {
  std::fprintf(stderr, "sr_iwt_replay: %s\n", what.c_str());
  std::exit(2);
}

class Reader {
 public:
  explicit Reader(std::istream& in) : in_(in) {}
  void Expect(const char* tag) {
    std::string t;
    if (!(in_ >> t) || t != tag) Fail(std::string("expected '") + tag + "', got '" + t + "'");
  }
  float F() { return Get<float>(); }
  int I() { return Get<int>(); }
  std::string S() { return Get<std::string>(); }

 private:
  template <typename T>
  T Get() {
    T v{};
    if (!(in_ >> v)) Fail("truncated input");
    return v;
  }
  std::istream& in_;
};

c8::HMatrix ReadH(Reader& r) {
  float m[16];
  float inv[16];
  for (float& v : m) v = r.F();
  for (float& v : inv) v = r.F();
  const int no_invert = r.I();
  const int invert_failed = r.I();
  if (invert_failed != 0) Fail("invertFailed matrices are not supported by this tool");
  return c8::HMatrix(m, inv, no_invert != 0);
}

c8_PixelPinholeCameraModel ReadK(Reader& r) {
  c8_PixelPinholeCameraModel k;
  k.pixelsWidth = r.I();
  k.pixelsHeight = r.I();
  k.centerPointX = r.F();
  k.centerPointY = r.F();
  k.focalLengthHorizontal = r.F();
  k.focalLengthVertical = r.F();
  return k;
}

struct Pt {
  uint32_t id;
  float pos[4];
  int32_t source;
};

// The five point pools of the oracle memory (634: first pool whose slot holds the id).
class CaseLookup final : public TrackedPointLookup {
 public:
  std::vector<Pt> pools[5];
  std::vector<std::pair<uint32_t, std::pair<uint32_t, uint32_t>>> tracked;  // frame idx -> slot, id

  bool Find(uint32_t frame_idx, TrackedMapPoint* out) const override {
    for (const auto& [key, v] : tracked) {
      if (key != frame_idx) continue;
      const auto [slot, id] = v;
      if (id == 0) return false;
      for (const auto& pool : pools) {
        if (slot < pool.size() && pool[slot].id == id) {
          const Pt& p = pool[slot];
          out->slot = slot;
          out->id = p.id;
          out->position = c8::HPoint3(std::array<float, 4>{p.pos[0], p.pos[1], p.pos[2], p.pos[3]});
          out->source = p.source;
          return true;
        }
      }
      return false;
    }
    return false;
  }
};

void PrintH(const char* tag, const c8::HMatrix& h) {
  std::printf("%s", tag);
  for (float v : h.data()) std::printf(" %.9g", v);
  for (float v : h.inverseData()) std::printf(" %.9g", v);
  std::printf("\n");
}

void PrintP3(const char* tag, const c8::Vector<c8::HPoint3>& v) {
  std::printf("%s %zu", tag, v.size());
  for (const auto& p : v) {
    for (float x : p.raw()) std::printf(" %.9g", x);
  }
  std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) Fail("usage: sr_iwt_replay CASES.txt");
  std::ifstream file(argv[1]);
  if (!file) Fail(std::string("cannot open ") + argv[1]);
  Reader r(file);
  r.Expect("iwt-cases");
  if (r.I() != 1) Fail("unsupported format version");
  const int n_cases = r.I();
  for (int c = 0; c < n_cases; ++c) {
    r.Expect("case");
    const std::string name = r.S();
    r.Expect("state");
    c8::ImageWorldTrackerState st;
    st.scale = r.F();
    st.worldPose = ReadH(r);
    CaseLookup lookup;
    for (int p = 0; p < 5; ++p) {
      r.Expect("pool");
      if (r.I() != p) Fail("pool order");
      const int n = r.I();
      for (int k = 0; k < n; ++k) {
        r.Expect("pt");
        Pt pt;
        pt.id = static_cast<uint32_t>(r.I());
        for (float& v : pt.pos) v = r.F();
        pt.source = r.I();
        lookup.pools[p].push_back(pt);
      }
    }
    r.Expect("norm");
    MapToTrackerTransform xf;
    xf.scale = r.F();
    for (float& v : xf.offset) v = r.F();
    r.Expect("tracked");
    const int n_tracked = r.I();
    for (int k = 0; k < n_tracked; ++k) {
      r.Expect("t");
      const uint32_t key = static_cast<uint32_t>(r.I());
      const uint32_t slot = static_cast<uint32_t>(r.I());
      const uint32_t id = static_cast<uint32_t>(r.I());
      lookup.tracked.push_back({key, {slot, id}});
    }
    r.Expect("image");
    const int type = r.I();
    const c8_PixelPinholeCameraModel target_k = ReadK(r);
    c8::CurvyImageGeometry geom;
    geom.radius = r.F();
    geom.height = r.F();
    geom.activationRegion.left = r.F();
    geom.activationRegion.right = r.F();
    geom.activationRegion.top = r.F();
    geom.activationRegion.bottom = r.F();
    geom.srcRows = r.I();
    geom.srcCols = r.I();
    geom.isCone = r.I() != 0;
    geom.radiusBottom = r.F();
    r.Expect("pose");
    const c8::HMatrix pose = ReadH(r);
    r.Expect("tam");
    const c8::HMatrix tam = ReadH(r);
    r.Expect("frame");
    const int n_feats = r.I();
    const c8_PixelPinholeCameraModel cam_k = ReadK(r);
    c8::FrameWithPoints feats(cam_k);
    for (int k = 0; k < n_feats; ++k) {
      r.Expect("f");
      const float rx = r.F();
      const float ry = r.F();
      const float px = r.F();
      const float py = r.F();
      const float pw = r.F();
      feats.addWorldMapPoint(c8::FramePoint(c8::HPoint2(rx, ry), 0, 0.0f, 0.0f),
                             c8::HPoint2(std::array<float, 3>{px, py, pw}), c8::FeatureSet{});
    }
    r.Expect("guess");
    const float guess = r.F();

    std::unique_ptr<c8::DetectionImage> im;
    if (type == 2) {
      im = std::make_unique<c8::DetectionImage>(name, c8::TargetWithPoints(target_k), 0, geom, geom,
                                                c8::HMatrixGen::i());
    } else {
      im = std::make_unique<c8::DetectionImage>(name, c8::TargetWithPoints(target_k), 0,
                                                c8::PlanarImageGeometry{});
    }
    const c8::HMatrix out = slam_refact::image_target::TrackImageWorld(&st, lookup, xf, *im, pose,
                                                                      tam, feats, guess);
    std::printf("case %s\n", name.c_str());
    std::printf("scale %.9g\n", st.scale);
    PrintH("out", out);
    PrintH("state_pose", st.worldPose);
    std::printf("rays %zu", st.rays.size());
    for (const auto& p : st.rays) {
      for (float x : p.raw()) std::printf(" %.9g", x);
    }
    std::printf("\n");
    PrintP3("search", st.searchPtsInCamera);
    std::printf("visible %zu", st.visible.size());
    for (size_t v : st.visible) std::printf(" %zu", v);
    std::printf("\n");
    PrintP3("world", st.worldPts);
    std::printf("ids %zu", st.ids.size());
    for (const auto& id : st.ids) std::printf(" %zu %" PRIu32, id.index(), id.id());
    std::printf("\n");
    PrintP3("target", st.targetPts);
  }
  return 0;
}
