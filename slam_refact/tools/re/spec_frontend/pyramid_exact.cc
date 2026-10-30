// U1 measurement: engine CPU pyramid (camera frame -> full atlas) vs every captured original GPU
// pyramid of one capture set, plus the keypoint sets detected on both (F-H6 style).
//
//   pyramid_exact <capture_dir> <cam_w> <cam_h> <ieee|swiftshader> <manufacturer> <model>
//
// Prints one JSON line per capture and a summary line. "atlas_equal" compares every byte of the
// CPU atlas (rows 0 .. layout.rows-1 of the readback, x size x 4: levels, gutters and the top of
// the ROI band; the readback's remaining ROI rows are never read by the CPU half);
// R/G/B/A fractions are over the level regions (CompareLevels). Built
// against the engine libraries (commands in build/slam-refact-20261016/pyramid/commands.txt); the
// "before" binary links the slam-refact-3 gr8_cpu_shader.o in front of libsr_frontend.a.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "c8/geometry/device-pose.h"
#include "c8/hmatrix.h"
#include "common/c8_adapters.h"
#include "frontend/frontend.h"
#include "frontend/intrinsics.h"
#include "frontend/pyramid_source.h"
#include "reality/engine/features/gr8gl-data.h"
#include "test_util.h"

namespace sf = slam_refact::frontend;
namespace st = slam_refact::frontend::testing;

namespace {

using KeyMap = std::map<std::tuple<int, long, long>, std::array<uint8_t, 32>>;

KeyMap DetectKeys(sf::FeatureFrontend* frontend, const c8::Gr8Pyramid& pyr,
                  const slam_refact::PinholeIntrinsics& k, const c8::Quaternion& device) {
  c8::FrameWithPoints frame(slam_refact::ToC8Intrinsics(k), c8::HMatrixGen::i(),
                            c8::xrRotationFromDeviceRotation(device));
  frontend->Detect(pyr, sf::WorldSlamDetectionConfig(), &frame);
  KeyMap out;
  const auto& store = frame.store();
  for (size_t i = 0; i < frame.points().size(); ++i) {
    std::array<uint8_t, 32> d{};
    if (store.has<c8::OrbFeature>(i)) {
      std::memcpy(d.data(), store.get<c8::OrbFeature>(i).data(), 32);
    }
    out[{static_cast<int>(frame.points()[i].scale()), std::lround(frame.pixels()[i].x() * 1000.0),
         std::lround(frame.pixels()[i].y() * 1000.0)}] = d;
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 7) {
    std::fprintf(stderr, "usage: %s <capture_dir> <cam_w> <cam_h> <ieee|swiftshader> <manuf> <model>\n",
                 argv[0]);
    return 2;
  }
  const std::string dir = argv[1];
  const int cam_w = std::atoi(argv[2]);
  const int cam_h = std::atoi(argv[3]);
  const bool ss = std::string(argv[4]) == "swiftshader";
  st::Json meta;
  if (!st::ReadJsonFile(dir + "/meta.json", &meta)) {
    std::fprintf(stderr, "cannot read %s/meta.json\n", dir.c_str());
    return 1;
  }
  const int rows = static_cast<int>(meta["rows"].number());
  const std::string dataset = meta["input"]["dataset"].str();
  const bool tum = meta["scenario"].str().rfind("tum", 0) == 0;
  st::Json index;
  if (!st::ReadJsonFile(dataset + (tum ? "/input.json" : "/record.json"), &index)) {
    std::fprintf(stderr, "cannot read dataset index\n");
    return 1;
  }
  const st::Json& frames = tum ? index["frames"] : index["recording"]["frames"];
  sf::PyramidSource source(cam_w, cam_h, sf::ShaderParams{},
                           ss ? slam_refact::ShaderNumericModel::kSwiftShaderApprox
                              : slam_refact::ShaderNumericModel::kIeee);
  const sf::IntrinsicsSet k = sf::ComputeIntrinsicsSet(argv[5], argv[6], cam_w, cam_h,
                                                       source.layout().levels[0].w,
                                                       source.layout().levels[0].h);
  sf::FeatureFrontend frontend;
  long n = 0, atlas_equal = 0;
  double r_min = 1.0, g_min = 1.0, j_min = 1.0, j_sum = 0.0;
  long kp_exact = 0;
  long px = 0;
  long eq[4] = {0, 0, 0, 0};
  const st::Json& caps = meta["captures"];
  for (size_t i = 0; i < caps.size(); ++i) {
    if (caps[i]["inputFrame"].is_null()) {
      continue;
    }
    const int cap = static_cast<int>(caps[i]["capture"].number());
    const int f = static_cast<int>(caps[i]["inputFrame"].number());
    const st::Json& q = caps[i]["stageArgs"]["qwxyz"];
    const c8::Quaternion device(static_cast<float>(q[0].number()), static_cast<float>(q[1].number()),
                                static_cast<float>(q[2].number()), static_cast<float>(q[3].number()));
    int w = 0, h = 0;
    std::vector<uint8_t> gray;
    const std::string pgm = dataset + "/mav0/cam0/data/" + frames[static_cast<size_t>(f)]["file"].str();
    if (!st::ReadPgm(pgm, &w, &h, &gray) || w != cam_w || h != cam_h) {
      std::fprintf(stderr, "bad frame %s\n", pgm.c_str());
      return 1;
    }
    char name[64];
    std::snprintf(name, sizeof(name), "/cap-%03d.rgba.gz", cap);
    sf::RgbaAtlas captured;
    if (!st::LoadCaptureAtlas(dir + name, rows, &captured)) {
      std::fprintf(stderr, "bad capture %s\n", name);
      return 1;
    }
    const c8::Gr8Pyramid mine = source.Get({}, {}, slam_refact::GrayImageView{gray.data(), w, h, w});
    const sf::RgbaAtlas& atlas = source.last_cpu_atlas();
    // Bytes that differ anywhere in the readback, per channel, and the first one.
    long diff[4] = {0, 0, 0, 0};
    long first = -1;
    for (size_t b = 0; b < atlas.rgba.size() && b < captured.rgba.size(); ++b) {
      if (atlas.rgba[b] != captured.rgba[b]) {
        ++diff[b % 4];
        first = first < 0 ? static_cast<long>(b) : first;
      }
    }
    const bool whole = atlas.size == captured.size && atlas.rows <= captured.rows && first < 0;
    const st::AtlasStats s = st::CompareLevels(atlas, captured, source.layout());
    const KeyMap km = DetectKeys(&frontend, mine, k.processing, device);
    const c8::Gr8Pyramid cap_pyr = sf::PyramidFromInjectedAtlas(
        slam_refact::PyramidAtlasView{captured.rgba.data(), captured.size, captured.rows}, source.layout());
    const KeyMap kc = DetectKeys(&frontend, cap_pyr, k.processing, device);
    long both = 0, same_desc = 0;
    for (const auto& [key, d] : km) {
      const auto it = kc.find(key);
      if (it != kc.end()) {
        ++both;
        same_desc += it->second == d ? 1 : 0;
      }
    }
    const long uni = static_cast<long>(km.size() + kc.size()) - both;
    const double jac = uni ? static_cast<double>(both) / static_cast<double>(uni) : 1.0;
    ++n;
    atlas_equal += whole ? 1 : 0;
    kp_exact += (km.size() == kc.size() && both == static_cast<long>(kc.size()) &&
                 same_desc == both) ? 1 : 0;
    r_min = std::min(r_min, s.identical_fraction(0));
    g_min = std::min(g_min, s.identical_fraction(1));
    j_min = std::min(j_min, jac);
    j_sum += jac;
    px += s.pixels;
    for (int c = 0; c < 4; ++c) {
      eq[c] += s.equal[c];
    }
    std::printf(
        "{\"capture\":%d,\"frame\":%d,\"atlas_equal\":%d,\"R\":%.6f,\"G\":%.6f,\"B\":%.6f,\"A\":%.6f,"
        "\"maxdiff\":[%ld,%ld,%ld,%ld],\"candJ\":%.6f,\"kp_cpu\":%zu,\"kp_cap\":%zu,\"kpJ\":%.6f,"
        "\"atlas_rows\":%d,\"atlas_diff_bytes\":[%ld,%ld,%ld,%ld],\"first_diff_xy\":[%ld,%ld]}\n",
        cap, f, whole ? 1 : 0, s.identical_fraction(0), s.identical_fraction(1),
        s.identical_fraction(2), s.identical_fraction(3), s.max_diff[0], s.max_diff[1],
        s.max_diff[2], s.max_diff[3], s.candidate_jaccard(), km.size(), kc.size(), jac, atlas.rows, diff[0],
        diff[1], diff[2], diff[3], first < 0 ? -1L : (first / 4) % atlas.size,
        first < 0 ? -1L : (first / 4) / atlas.size);
  }
  std::printf(
      "{\"summary\":1,\"capture_dir\":\"%s\",\"model\":\"%s\",\"frames\":%ld,\"atlas_bit_exact\":%ld,"
      "\"R_equal\":%.6f,\"G_equal\":%.6f,\"B_equal\":%.6f,\"A_equal\":%.6f,\"R_min\":%.6f,"
      "\"G_min\":%.6f,\"kpJ_min\":%.6f,\"kpJ_mean\":%.6f,\"kp_sets_identical\":%ld}\n",
      dir.c_str(), ss ? "swiftshader" : "ieee", n, atlas_equal,
      px ? static_cast<double>(eq[0]) / px : 1.0, px ? static_cast<double>(eq[1]) / px : 1.0,
      px ? static_cast<double>(eq[2]) / px : 1.0, px ? static_cast<double>(eq[3]) / px : 1.0, r_min,
      g_min, j_min, n ? j_sum / n : 1.0, kp_exact);
  return 0;
}
