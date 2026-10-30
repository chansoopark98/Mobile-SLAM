// Spec lane S1 (frontend) probe. Links the public Gr8Gl CPU half compiled by lane B
// (build/slam-refact-20261015/public-build/lib/cxx17) and the CPU shader emulation in
// gr8_gpu_ref.cc. No OpenCV.
//
//   frontend_probe selftest
//   frontend_probe layout <src_w> <src_h> [rotation]
//   frontend_probe pyramid <in.pgm> <out.rgba> [swiftshader_approx]  (writes tex_w x tex_h atlas)
//   frontend_probe compare <in.pgm> <captured.rgba> <captured_rows> <fx_level0> <qw> <qx> <qy> <qz>
//                          [linear_weight_bits] [swiftshader_approx] [binary_expression_forms]
//   frontend_probe isolate <captured.rgba> <rows> <src_w> <src_h> <rebuild_levels> [swiftshader_approx]
//   frontend_probe crop <atlas.rgba> <rows> <src_w> <src_h> [b0 b1 b2 b3]
//   frontend_probe detect <atlas.rgba> <rows> <src_w> <src_h> <fx_level0> <qw> <qx> <qy> <qz>
//
// compare/detect print one JSON object on stdout.
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "c8/geometry/device-pose.h"
#include "c8/hmatrix.h"
#include "c8/pixels/gr8-pyramid.h"
#include "c8/quaternion.h"
#include "gr8_gpu_ref.h"
#include "reality/engine/features/detection-config.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/gr8gl.h"

namespace c8 {
// Defined with external linkage in reality/engine/features/gr8gl.cc:600 (no header declaration).
void keypointsForCropJs(ConstRGBA8888PlanePixels s, uint8_t scale, int8_t roi, int edgeThreshold,
                        Vector<ImagePointLocation> &keypoints);
}  // namespace c8

namespace {

using spec_frontend::Gr8GpuParams;
using spec_frontend::Gr8GpuRef;
using spec_frontend::Layout;

bool ReadPgm(const char *path, int *w, int *h, std::vector<uint8_t> *gray) {
  FILE *f = std::fopen(path, "rb");
  if (!f) return false;
  int maxv = 0;
  if (std::fscanf(f, "P5 %d %d %d", w, h, &maxv) != 3 || maxv != 255) {
    std::fclose(f);
    return false;
  }
  std::fgetc(f);
  gray->resize(static_cast<size_t>(*w) * *h);
  const size_t n = std::fread(gray->data(), 1, gray->size(), f);
  std::fclose(f);
  return n == gray->size();
}

bool ReadRaw(const char *path, size_t bytes, std::vector<uint8_t> *out) {
  FILE *f = std::fopen(path, "rb");
  if (!f) return false;
  out->resize(bytes);
  const size_t n = std::fread(out->data(), 1, bytes, f);
  std::fclose(f);
  return n == bytes;
}

std::vector<uint8_t> GrayToRgba(const std::vector<uint8_t> &gray) {
  std::vector<uint8_t> rgba(gray.size() * 4);
  for (size_t i = 0; i < gray.size(); ++i) {
    rgba[4 * i] = rgba[4 * i + 1] = rgba[4 * i + 2] = gray[i];
    rgba[4 * i + 3] = 255;
  }
  return rgba;
}

// c8/geometry/intrinsics.cc:245-263 (capnp-free copy).
c8_PixelPinholeCameraModel CropAndScale(c8_PixelPinholeCameraModel k, int width,
                                             int height) {
  const float scale = std::max(static_cast<float>(width) / k.pixelsWidth,
                               static_cast<float>(height) / k.pixelsHeight);
  const float cx = (width - 1) / 2.0f + (k.centerPointX - (k.pixelsWidth - 1) / 2.0f) * scale;
  const float cy = (height - 1) / 2.0f + (k.centerPointY - (k.pixelsHeight - 1) / 2.0f) * scale;
  return {width, height, cx, cy, k.focalLengthHorizontal * scale, k.focalLengthVertical * scale};
}

c8::Gr8Pyramid MakePyramid(const std::vector<uint8_t> &atlas, int tex_w, int rows,
                           const std::vector<Layout> &levels) {
  c8::Gr8Pyramid p;
  p.data = c8::ConstRGBA8888PlanePixels(rows, tex_w, tex_w * 4, atlas.data());
  for (const auto &l : levels) {
    c8::LevelLayout ll;
    ll.r = l.r;
    ll.c = l.c;
    ll.w = l.w;
    ll.h = l.h;
    ll.rotated = l.rotated;
    p.levels.push_back(ll);
  }
  return p;
}

struct KeyRec {
  int scale;
  long x1000, y1000;
  int response;
  float angle, gravity_angle;
  bool has_orb, has_gorb;
  std::array<uint8_t, 32> orb{}, gorb{};
};

std::vector<KeyRec> Detect(const c8::Gr8Pyramid &pyr, const c8_PixelPinholeCameraModel &k,
                           const c8::Quaternion &device_pose) {
  // VioTracker::track (func784, dcmp 82457-82482): nKeypoints 500, gravityPortions[ORB] = 192
  // (SEVENTH|EIGHTH), allGorb = 1 (VioTracker.trackingDescriptorType = 2), gravityAngleOffset 0.
  c8::DetectionConfig cfg;
  cfg.nKeypoints = 500;
  cfg.allGorb = true;
  cfg.gravityPortions[c8::DescriptorType::ORB] =
    c8::GravityPortion(c8::GravityPortion::SEVENTH | c8::GravityPortion::EIGHTH);
  c8::FrameWithPoints frame(k, c8::HMatrixGen::i(), c8::xrRotationFromDeviceRotation(device_pose));
  static c8::Gr8Gl gr8 = c8::Gr8Gl::create();
  c8::ImagePoints pts = gr8.detectAndCompute(pyr, frame, cfg, 0.0f);
  std::vector<KeyRec> out;
  for (size_t i = 0; i < pts.size(); ++i) {
    const auto loc = pts[i].location();
    KeyRec r;
    r.scale = loc.scale;
    r.x1000 = std::lround(loc.pt.x * 1000.0);
    r.y1000 = std::lround(loc.pt.y * 1000.0);
    r.response = static_cast<int>(loc.response);
    r.angle = loc.angle;
    r.gravity_angle = loc.gravityAngle;
    const auto &fs = pts[i].features();
    r.has_orb = fs.orbFeature.has_value();
    r.has_gorb = fs.gorbFeature.has_value();
    if (r.has_orb) std::memcpy(r.orb.data(), fs.orbFeature->data(), 32);
    if (r.has_gorb) std::memcpy(r.gorb.data(), fs.gorbFeature->data(), 32);
    out.push_back(r);
  }
  return out;
}

int Popcount32(const std::array<uint8_t, 32> &a, const std::array<uint8_t, 32> &b) {
  int d = 0;
  for (int i = 0; i < 32; ++i) d += __builtin_popcount(a[i] ^ b[i]);
  return d;
}

void PrintKeyStats(const char *tag, const std::vector<KeyRec> &a) {
  int per_level[4] = {0, 0, 0, 0}, n_orb = 0, n_gorb = 0;
  for (const auto &r : a) {
    if (r.scale >= 0 && r.scale < 4) per_level[r.scale]++;
    n_orb += r.has_orb;
    n_gorb += r.has_gorb;
  }
  std::printf("\"%s\":{\"n\":%zu,\"per_level\":[%d,%d,%d,%d],\"orb\":%d,\"gorb\":%d}", tag, a.size(),
              per_level[0], per_level[1], per_level[2], per_level[3], n_orb, n_gorb);
}

// Unordered comparison keyed by (scale, x, y) at 1e-3 px.
void CompareKeys(const std::vector<KeyRec> &a, const std::vector<KeyRec> &b) {
  using Key = std::tuple<int, long, long>;
  std::map<Key, const KeyRec *> ma, mb;
  for (const auto &r : a) ma[{r.scale, r.x1000, r.y1000}] = &r;
  for (const auto &r : b) mb[{r.scale, r.x1000, r.y1000}] = &r;
  int both = 0, same_resp = 0, orb_eq = 0, gorb_eq = 0, orb_pairs = 0, gorb_pairs = 0;
  long orb_ham = 0, gorb_ham = 0;
  for (const auto &kv : ma) {
    auto it = mb.find(kv.first);
    if (it == mb.end()) continue;
    ++both;
    same_resp += kv.second->response == it->second->response;
    if (kv.second->has_orb && it->second->has_orb) {
      ++orb_pairs;
      const int d = Popcount32(kv.second->orb, it->second->orb);
      orb_ham += d;
      orb_eq += d == 0;
    }
    if (kv.second->has_gorb && it->second->has_gorb) {
      ++gorb_pairs;
      const int d = Popcount32(kv.second->gorb, it->second->gorb);
      gorb_ham += d;
      gorb_eq += d == 0;
    }
  }
  const size_t uni = ma.size() + mb.size() - both;
  std::printf(
    "\"keypoints\":{\"a\":%zu,\"b\":%zu,\"intersection\":%d,\"jaccard\":%.6f,\"same_response\":%d,"
    "\"orb_pairs\":%d,\"orb_identical\":%d,\"orb_mean_hamming\":%.4f,\"gorb_pairs\":%d,"
    "\"gorb_identical\":%d,\"gorb_mean_hamming\":%.4f}",
    ma.size(), mb.size(), both, uni ? static_cast<double>(both) / uni : 1.0, same_resp, orb_pairs,
    orb_eq, orb_pairs ? static_cast<double>(orb_ham) / orb_pairs : 0.0, gorb_pairs, gorb_eq,
    gorb_pairs ? static_cast<double>(gorb_ham) / gorb_pairs : 0.0);
}

// Channel statistics inside each level region (full region and the 21 px detection interior).
void CompareAtlas(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b, int tex_w, int rows,
                  const std::vector<Layout> &levels) {
  std::printf("\"atlas\":[");
  for (size_t li = 0; li < levels.size(); ++li) {
    const Layout &l = levels[li];
    long n = 0, eq[4] = {0, 0, 0, 0}, maxd[4] = {0, 0, 0, 0}, le1[4] = {0, 0, 0, 0};
    long cand_a = 0, cand_b = 0, cand_both = 0;
    for (int y = 0; y < l.h; ++y) {
      if (l.r + y >= rows) break;
      for (int x = 0; x < l.w; ++x) {
        const size_t o = (static_cast<size_t>(l.r + y) * tex_w + l.c + x) * 4;
        ++n;
        for (int ch = 0; ch < 4; ++ch) {
          const long d = std::labs(static_cast<long>(a[o + ch]) - b[o + ch]);
          eq[ch] += d == 0;
          le1[ch] += d <= 1;
          maxd[ch] = std::max(maxd[ch], d);
        }
        const bool interior = x >= 21 && y >= 21 && x < l.w - 21 && y < l.h - 21;
        if (interior) {
          cand_a += a[o + 2] != 0;
          cand_b += b[o + 2] != 0;
          cand_both += a[o + 2] != 0 && b[o + 2] != 0;
        }
      }
    }
    std::printf(
      "%s{\"level\":%zu,\"pixels\":%ld,\"eq\":[%ld,%ld,%ld,%ld],\"le1\":[%ld,%ld,%ld,%ld],"
      "\"maxdiff\":[%ld,%ld,%ld,%ld],\"cand_a\":%ld,\"cand_b\":%ld,\"cand_both\":%ld}",
      li ? "," : "", li, n, eq[0], eq[1], eq[2], eq[3], le1[0], le1[1], le1[2], le1[3], maxd[0],
      maxd[1], maxd[2], maxd[3], cand_a, cand_b, cand_both);
  }
  std::printf("]");
}

int SelfTest() {
  std::mt19937 rng(20261015u);
  std::uniform_int_distribution<int> byte(0, 255);
  long mismatches = 0;
  const long trials = 2000000;
  for (long t = 0; t < trials; ++t) {
    float d[16];
    const float c = byte(rng) / 255.0f;
    for (int i = 0; i < 16; ++i) d[i] = byte(rng) / 255.0f - c;
    if (t % 3 == 0) {  // structured corners: a bright or dark arc of random length
      const int start = byte(rng) % 16, len = 6 + byte(rng) % 8;
      const float s = (byte(rng) & 1) ? 1.0f : -1.0f;
      for (int i = 0; i < len; ++i) d[(start + i) % 16] = s * (byte(rng) + 1) / 255.0f;
    }
    if (spec_frontend::Max9FullScore(d) != spec_frontend::Fast9Textbook(d)) ++mismatches;
  }
  const auto q = Gr8GpuRef::SubpixelParams();
  std::printf("{\"fast9_trials\":%ld,\"fast9_mismatch\":%ld,\"subpixelQ\":[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]}\n",
              trials, mismatches, q[0], q[1], q[2], q[3], q[4], q[5]);
  return mismatches == 0 ? 0 : 1;
}

int PrintLayout(int w, int h, int rotation) {
  Gr8GpuRef ref;
  Gr8GpuParams p;
  p.rotation = rotation;
  ref.Initialize(w, h, p);
  std::printf("{\"src\":[%d,%d],\"input\":[%d,%d],\"filtered\":[%d,%d],\"tex\":[%d,%d],\"levels\":[", w, h,
              ref.input_w(), ref.input_h(), ref.filtered_w(), ref.filtered_h(), ref.atlas().w,
              ref.atlas().h);
  for (size_t i = 0; i < ref.levels().size(); ++i) {
    const auto &l = ref.levels()[i];
    std::printf("%s{\"r\":%d,\"c\":%d,\"w\":%d,\"h\":%d,\"rotated\":%d}", i ? "," : "", l.r, l.c, l.w,
                l.h, l.rotated);
  }
  std::printf("],\"rois\":[");
  for (size_t i = 0; i < ref.roi_layouts().size(); ++i) {
    const auto &l = ref.roi_layouts()[i];
    std::printf("%s{\"r\":%d,\"c\":%d,\"w\":%d,\"h\":%d}", i ? "," : "", l.r, l.c, l.w, l.h);
  }
  std::printf("]}\n");
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: see header\n");
    return 2;
  }
  const std::string mode = argv[1];
  if (mode == "selftest") return SelfTest();
  if (mode == "layout" && argc >= 4) {
    return PrintLayout(std::atoi(argv[2]), std::atoi(argv[3]), argc >= 5 ? std::atoi(argv[4]) : 0);
  }
  if ((mode == "pyramid" && argc >= 4) || (mode == "compare" && argc >= 10)) {
    int w = 0, h = 0;
    std::vector<uint8_t> gray;
    if (!ReadPgm(argv[2], &w, &h, &gray)) {
      std::fprintf(stderr, "cannot read pgm %s\n", argv[2]);
      return 2;
    }
    Gr8GpuParams p;
    // compare: optional argv[10] = linear_weight_bits, argv[11] = swiftshader_approx (0/1).
    if (mode == "compare" && argc >= 11) p.linear_weight_bits = std::atoi(argv[10]);
    if (mode == "compare" && argc >= 12) p.swiftshader_approx = std::atoi(argv[11]) != 0;
    if (mode == "compare" && argc >= 13) p.binary_expression_forms = std::atoi(argv[12]) != 0;
    if (mode == "pyramid" && argc >= 5) p.swiftshader_approx = std::atoi(argv[4]) != 0;
    Gr8GpuRef ref;
    ref.Initialize(w, h, p);
    const auto rgba = GrayToRgba(gray);
    ref.Draw(rgba.data(), w * 4);
    const auto &atlas = ref.atlas();
    if (mode == "pyramid") {
      FILE *f = std::fopen(argv[3], "wb");
      if (!f) return 2;
      std::fwrite(atlas.px.data(), 1, atlas.px.size(), f);
      std::fclose(f);
      std::printf("{\"tex\":[%d,%d],\"bytes\":%zu}\n", atlas.w, atlas.h, atlas.px.size());
      return 0;
    }
    const int rows = std::atoi(argv[4]);
    std::vector<uint8_t> cap;
    if (!ReadRaw(argv[3], static_cast<size_t>(rows) * atlas.w * 4, &cap)) {
      std::fprintf(stderr, "cannot read captured atlas %s\n", argv[3]);
      return 2;
    }
    std::vector<uint8_t> mine(atlas.px.begin(), atlas.px.begin() + static_cast<size_t>(rows) * atlas.w * 4);
    const float fx = static_cast<float>(std::atof(argv[5]));
    const auto &l0 = ref.levels()[0];
    const c8_PixelPinholeCameraModel k{l0.w, l0.h, (l0.w - 1) / 2.0f, (l0.h - 1) / 2.0f, fx, fx};
    const c8::Quaternion q(std::atof(argv[6]), std::atof(argv[7]), std::atof(argv[8]), std::atof(argv[9]));
    const auto ka = Detect(MakePyramid(mine, atlas.w, rows, ref.levels()), k, q);
    const auto kb = Detect(MakePyramid(cap, atlas.w, rows, ref.levels()), k, q);
    std::printf("{\"linear_weight_bits\":%d,\"swiftshader_approx\":%d,\"binary_expression_forms\":%d,",
                p.linear_weight_bits, p.swiftshader_approx ? 1 : 0, p.binary_expression_forms ? 1 : 0);
    CompareAtlas(mine, cap, atlas.w, rows, ref.levels());
    std::printf(",");
    PrintKeyStats("cpu_port", ka);
    std::printf(",");
    PrintKeyStats("captured", kb);
    std::printf(",");
    CompareKeys(ka, kb);
    std::printf("}\n");
    return 0;
  }
  if (mode == "isolate" && argc >= 7) {
    // isolate <captured.rgba> <rows> <src_w> <src_h> <rebuild_levels 0|1> [swiftshader_approx]
    const int rows = std::atoi(argv[3]);
    Gr8GpuParams p;
    if (argc >= 8) p.swiftshader_approx = std::atoi(argv[7]) != 0;
    Gr8GpuRef ref;
    ref.Initialize(std::atoi(argv[4]), std::atoi(argv[5]), p);
    std::vector<uint8_t> cap;
    if (!ReadRaw(argv[2], static_cast<size_t>(rows) * ref.atlas().w * 4, &cap)) return 2;
    ref.DrawFromCapturedGray(cap.data(), rows, std::atoi(argv[6]) != 0);
    std::vector<uint8_t> mine(ref.atlas().px.begin(),
                              ref.atlas().px.begin() + static_cast<size_t>(rows) * ref.atlas().w * 4);
    std::printf("{\"rebuild_levels\":%d,\"swiftshader_approx\":%d,", std::atoi(argv[6]), p.swiftshader_approx ? 1 : 0);
    CompareAtlas(mine, cap, ref.atlas().w, rows, ref.levels());
    std::printf("}\n");
    return 0;
  }
  if (mode == "crop" && argc >= 6) {
    // crop <atlas.rgba> <rows> <src_w> <src_h> [budgets 4 ints]: public keypointsForCropJs + retainBest
    // per level, printed in output order (same fields as binary_gr8_probe.mjs).
    const int rows = std::atoi(argv[3]);
    Gr8GpuRef ref;
    ref.Initialize(std::atoi(argv[4]), std::atoi(argv[5]), Gr8GpuParams{});
    std::vector<uint8_t> atlas;
    if (!ReadRaw(argv[2], static_cast<size_t>(rows) * ref.atlas().w * 4, &atlas)) return 2;
    int budgets[4] = {199, 138, 96, 67};
    for (int i = 0; i < 4 && 6 + i < argc; ++i) budgets[i] = std::atoi(argv[6 + i]);
    const auto pyr = MakePyramid(atlas, ref.atlas().w, rows, ref.levels());
    std::printf("{\"levels\":[");
    for (size_t li = 0; li < pyr.levels.size(); ++li) {
      c8::Vector<c8::ImagePointLocation> kps;
      c8::keypointsForCropJs(pyr.level(li), static_cast<uint8_t>(li), -1, 21, kps);
      const size_t candidates = kps.size();
      c8::Gr8Gl::retainBest(kps, budgets[li], pyr.levels[li].w, pyr.levels[li].h, 21, true);
      std::printf("%s{\"level\":%zu,\"candidates\":%zu,\"kept\":%zu,\"list\":[", li ? "," : "", li,
                  candidates, kps.size());
      for (size_t i = 0; i < kps.size(); ++i) {
        const auto &k = kps[i];
        std::printf("%s[%.9g,%.9g,%.9g,%u,%.9g,%.9g,%.9g,%d,%u]", i ? "," : "", k.pt.x, k.pt.y, k.size,
                    k.scale, k.angle, k.gravityAngle, k.response, k.roi, k.gravityPortion);
      }
      std::printf("]}");
    }
    std::printf("]}\n");
    return 0;
  }
  if (mode == "detect" && argc >= 11) {
    const int rows = std::atoi(argv[3]);
    Gr8GpuRef ref;
    ref.Initialize(std::atoi(argv[4]), std::atoi(argv[5]), Gr8GpuParams{});
    std::vector<uint8_t> atlas;
    if (!ReadRaw(argv[2], static_cast<size_t>(rows) * ref.atlas().w * 4, &atlas)) return 2;
    const auto &l0 = ref.levels()[0];
    const float fx = static_cast<float>(std::atof(argv[6]));
    const c8_PixelPinholeCameraModel k{l0.w, l0.h, (l0.w - 1) / 2.0f, (l0.h - 1) / 2.0f, fx, fx};
    const c8::Quaternion q(std::atof(argv[7]), std::atof(argv[8]), std::atof(argv[9]), std::atof(argv[10]));
    const auto keys = Detect(MakePyramid(atlas, ref.atlas().w, rows, ref.levels()), k, q);
    std::printf("{");
    PrintKeyStats("keys", keys);
    std::printf(",\"list\":[");
    for (size_t i = 0; i < keys.size(); ++i) {
      const auto &r = keys[i];
      std::printf("%s[%d,%.3f,%.3f,%d,%.3f,%.3f,%d,%d]", i ? "," : "", r.scale, r.x1000 / 1000.0,
                  r.y1000 / 1000.0, r.response, r.angle, r.gravity_angle, r.has_orb, r.has_gorb);
    }
    std::printf("]}\n");
    return 0;
  }
  std::fprintf(stderr, "bad arguments\n");
  return 2;
}
