// it-scale-v4 L0 diagnosis (REPORT tier, harness only): per-call record of c8::ImageTrackingState::track through a linker --wrap capture
// (no engine file is changed; the real function runs unchanged, results are bit-identical). Requires AGGRESSIVELY_FREE_MEMORY=false so the
// scratch (matches, inliers) is readable after the call. Output: one JSON line per track() call to $ITPROBE_OUT, tagged with the frame
// index set by itprobe_set_frame().
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "c8/random-numbers.h"
#include "frontend/pyramid_source.h"
#include "frontend/roi_draw.h"
#ifndef ITPROBE_NO_ROISS
#include "ss_roi_model.h"
#endif
#include "reality/engine/geometry/pose-pnp.h"
#include "reality/engine/imagedetection/image-tracking-state.h"

namespace {
int g_frame = -1;
std::FILE* g_out = nullptr;

size_t Nz(const c8::Vector<uint8_t>& v) {
  size_t n = 0;
  for (uint8_t b : v) n += b != 0;
  return n;
}
std::string Num(double v) {
  char b[48];
  std::snprintf(b, sizeof b, "%.9g", v);
  return b;
}
std::string Pose(const c8::HMatrix& m) {
  const auto t = c8::translation(m);
  return "[" + Num(t.x()) + "," + Num(t.y()) + "," + Num(t.z()) + "]";
}
c8::RandomNumbers* g_rng = nullptr;  // the shared XREngine stream (set at every track() call; changes when the engine is re-created at the start recenter)
bool g_in_track = false;  // solver calls from MapTracker / Relocalize are not logged and not perturbed
std::string g_stages;  // comma separated JSON objects of the solver calls since the last track() record

std::string Words(const c8::HMatrix* m) {  // the 32 floats (matrix, inverse) of an HMatrix as 8-digit hex words (little-endian float bits)
  std::string out;
  char b[12];
  const float* f = reinterpret_cast<const float*>(m);
  for (int k = 0; k < 32; ++k) {
    uint32_t u;
    std::memcpy(&u, &f[k], 4);
    std::snprintf(b, sizeof b, "%s%08x", k ? " " : "", u);
    out += b;
  }
  return out;
}
std::string WordsK(const void* p, int n) {
  std::string out;
  char b[12];
  for (int k = 0; k < n; ++k) {
    uint32_t u;
    std::memcpy(&u, static_cast<const char*>(p) + 4 * k, 4);
    std::snprintf(b, sizeof b, "%s%08x", k ? " " : "", u);
    out += b;
  }
  return out;
}

// RandomNumbers keeps a std::mt19937 as its first member (same assumption as tools/harness/replay/trkprobe.cc).
static_assert(std::is_trivially_copyable<std::mt19937>::value, "mt19937 bytewise");
uint64_t RngHash(c8::RandomNumbers* r) {
  std::mt19937 m;
  std::memcpy(&m, static_cast<void*>(r), sizeof m);
  uint64_t h = 1469598103934665603ull;
  for (int i = 0; i < 8; ++i) h = (h ^ m()) * 1099511628211ull;  // next 8 outputs of a COPY (the live stream is untouched)
  return h;
}
// Optional sensitivity experiment, OFF unless ITPROBE_RNG_SKIP="<frame>:<k>": discard k raw draws of the shared stream right before the
// global solver of that frame. The run is then NOT an engine-equivalent replay.
void MaybeSkip(c8::RandomNumbers* r) {
  static int sk_frame = -2, sk_k = 0;
  static bool init = false;
  if (!init) {
    init = true;
    const char* e = std::getenv("ITPROBE_RNG_SKIP");
    if (e != nullptr) std::sscanf(e, "%d:%d", &sk_frame, &sk_k);
  }
  if (g_in_track && g_frame == sk_frame && sk_k > 0) reinterpret_cast<std::mt19937*>(static_cast<void*>(r))->discard(static_cast<unsigned long long>(sk_k));
}
}  // namespace

std::string RngSig(c8::RandomNumbers* r) {  // next 8 raw outputs of a COPY of the stream (the live stream is untouched), hex, space separated
  std::mt19937 m;
  std::memcpy(&m, static_cast<void*>(r), sizeof m);
  std::string out;
  char b[16];
  for (int k = 0; k < 8; ++k) {
    std::snprintf(b, sizeof b, "%s%08x", k ? " " : "", static_cast<unsigned>(m()));
    out += b;
  }
  return out;
}

// ITPROBE_RNG_SET="<frame>:<file>": before processing <frame> the shared stream is replaced by the state in <file> (libstdc++ text form, written by
// mt_tools.py to_libstdcxx_text from the ORIGINAL's recorded state at the end of <frame>-1). The run is then NOT an engine-equivalent replay.
void itprobe_set_frame(int i) {
  g_frame = i;
  static int set_frame = -2;
  static std::string set_file;
  static bool init = false;
  if (!init) {
    init = true;
    const char* e = std::getenv("ITPROBE_RNG_SET");
    if (e != nullptr) {
      set_frame = std::atoi(e);
      const char* c = std::strchr(e, ':');
      if (c != nullptr) set_file = c + 1;
    }
  }
  if (i == set_frame && g_rng != nullptr) {
    std::ifstream f(set_file);
    std::stringstream ss;
    ss << f.rdbuf();
    std::mt19937* m = reinterpret_cast<std::mt19937*>(static_cast<void*>(g_rng));
    ss >> *m;
    if (ss.fail()) std::abort();
    std::fprintf(stderr, "itprobe: RNG state of frame %d set from %s\n", i, set_file.c_str());
  }
}

// ITPROBE_RNG_OUT=<new file>: one line per processed frame {"i","sig"} = next 8 outputs of the shared stream at the END of that frame
void itprobe_frame_end(int i) {
  static std::FILE* f = nullptr;
  static bool init = false;
  if (!init) {
    init = true;
    const char* p = std::getenv("ITPROBE_RNG_OUT");
    if (p != nullptr) {
      f = std::fopen(p, "wx");
      if (f == nullptr) std::abort();
    }
  }
  // ITPROBE_RNG_DUMP="<frame>:<file>": write the stream state at the end of <frame> in the libstdc++ text form (self test of ITPROBE_RNG_SET)
  const char* d = std::getenv("ITPROBE_RNG_DUMP");
  if (d != nullptr && g_rng != nullptr && std::atoi(d) == i && std::strchr(d, ':') != nullptr) {
    std::ofstream o(std::strchr(d, ':') + 1, std::ios::out | std::ios::trunc);
    o << *reinterpret_cast<std::mt19937*>(static_cast<void*>(g_rng));
  }
  if (f == nullptr || g_rng == nullptr) return;
  std::fprintf(f, "{\"i\":%d,\"sig\":\"%s\"}\n", i, RngSig(g_rng).c_str());
  std::fflush(f);
}

extern "C" {

bool __real__ZN2c89robustP2PERKSt6vectorINS_6HPointILm3EEESaIS2_EERKS0_INS1_ILm2EEESaIS7_EERKS7_SD_RKNS_7HMatrixERKNS_15RobustP2PParamsEPSE_PfPS0_IhSaIhEEPNS_13RandomNumbersEPNS_21RobustP2PScratchSpaceE(
    const std::vector<c8::HPoint3>&, const std::vector<c8::HPoint2>&, const c8::HPoint2&, const c8::HPoint2&, const c8::HMatrix&,
    const c8::RobustP2PParams&, c8::HMatrix*, float*, std::vector<uint8_t>*, c8::RandomNumbers*, c8::RobustP2PScratchSpace*);
bool __wrap__ZN2c89robustP2PERKSt6vectorINS_6HPointILm3EEESaIS2_EERKS0_INS1_ILm2EEESaIS7_EERKS7_SD_RKNS_7HMatrixERKNS_15RobustP2PParamsEPSE_PfPS0_IhSaIhEEPNS_13RandomNumbersEPNS_21RobustP2PScratchSpaceE(
    const std::vector<c8::HPoint3>& wp, const std::vector<c8::HPoint2>& rays, const c8::HPoint2& ll, const c8::HPoint2& ur, const c8::HMatrix& tp,
    const c8::RobustP2PParams& pr, c8::HMatrix* cm, float* conf, std::vector<uint8_t>* inl, c8::RandomNumbers* rnd, c8::RobustP2PScratchSpace* sc) {
  MaybeSkip(rnd);
  const uint64_t h0 = RngHash(rnd);
  const bool ok = __real__ZN2c89robustP2PERKSt6vectorINS_6HPointILm3EEESaIS2_EERKS0_INS1_ILm2EEESaIS7_EERKS7_SD_RKNS_7HMatrixERKNS_15RobustP2PParamsEPSE_PfPS0_IhSaIhEEPNS_13RandomNumbersEPNS_21RobustP2PScratchSpaceE(
      wp, rays, ll, ur, tp, pr, cm, conf, inl, rnd, sc);
  if (g_in_track) g_stages += std::string(g_stages.empty() ? "" : ",") + "{\"fn\":\"p2p\",\"ok\":" + (ok ? "1" : "0") + ",\"n\":" + std::to_string(wp.size()) +
              ",\"conf\":" + Num(*conf) + ",\"inl\":" + std::to_string(Nz(*inl)) + ",\"minInl\":" + std::to_string(pr.ransac.minInlierRequired) +
              ",\"rng\":\"" + std::to_string(h0) + "\"}";
  return ok;
}

bool __real__ZN2c89robustPnPERKSt6vectorINS_6HPointILm3EEESaIS2_EERKS0_INS1_ILm2EEESaIS7_EERKNS_7HMatrixENS_15RobustPnPParamsEPSC_PS0_IhSaIhEEPNS_13RandomNumbersEPNS_22RobustPoseScratchSpaceEPNS_17RobustPnPDebugOutE(
    const std::vector<c8::HPoint3>&, const std::vector<c8::HPoint2>&, const c8::HMatrix&, c8::RobustPnPParams, c8::HMatrix*, std::vector<uint8_t>*,
    c8::RandomNumbers*, c8::RobustPoseScratchSpace*, c8::RobustPnPDebugOut*);
bool __wrap__ZN2c89robustPnPERKSt6vectorINS_6HPointILm3EEESaIS2_EERKS0_INS1_ILm2EEESaIS7_EERKNS_7HMatrixENS_15RobustPnPParamsEPSC_PS0_IhSaIhEEPNS_13RandomNumbersEPNS_22RobustPoseScratchSpaceEPNS_17RobustPnPDebugOutE(
    const std::vector<c8::HPoint3>& wp, const std::vector<c8::HPoint2>& rays, const c8::HMatrix& tp, c8::RobustPnPParams pr, c8::HMatrix* cm,
    std::vector<uint8_t>* inl, c8::RandomNumbers* rnd, c8::RobustPoseScratchSpace* sc, c8::RobustPnPDebugOut* dbg) {
  MaybeSkip(rnd);
  const uint64_t h0 = RngHash(rnd);
  const bool ok = __real__ZN2c89robustPnPERKSt6vectorINS_6HPointILm3EEESaIS2_EERKS0_INS1_ILm2EEESaIS7_EERKNS_7HMatrixENS_15RobustPnPParamsEPSC_PS0_IhSaIhEEPNS_13RandomNumbersEPNS_22RobustPoseScratchSpaceEPNS_17RobustPnPDebugOutE(
      wp, rays, tp, pr, cm, inl, rnd, sc, dbg);
  if (g_in_track) g_stages += std::string(g_stages.empty() ? "" : ",") + "{\"fn\":\"pnp\",\"ok\":" + (ok ? "1" : "0") + ",\"n\":" + std::to_string(wp.size()) +
              ",\"inl\":" + std::to_string(Nz(*inl)) + ",\"minInl\":" + std::to_string(pr.minPoseInliers) + ",\"best\":" + Num(dbg != nullptr ? dbg->bestResidual : -1) +
              ",\"rng\":\"" + std::to_string(h0) + "\"}";
  return ok;
}

bool __real__ZN2c826solveImageTargetHomographyERKSt6vectorINS_6HPointILm2EEESaIS2_EES6_RKS0_IfSaIfEENS_14SolvePnPParamsEPNS_7HMatrixEPS0_IhSaIhEEPNS_22RobustPoseScratchSpaceE(
    const std::vector<c8::HPoint2>&, const std::vector<c8::HPoint2>&, const std::vector<float>&, c8::SolvePnPParams, c8::HMatrix*, std::vector<uint8_t>*,
    c8::RobustPoseScratchSpace*);
bool __wrap__ZN2c826solveImageTargetHomographyERKSt6vectorINS_6HPointILm2EEESaIS2_EES6_RKS0_IfSaIfEENS_14SolvePnPParamsEPNS_7HMatrixEPS0_IhSaIhEEPNS_22RobustPoseScratchSpaceE(
    const std::vector<c8::HPoint2>& a, const std::vector<c8::HPoint2>& b, const std::vector<float>& w, c8::SolvePnPParams pr, c8::HMatrix* m,
    std::vector<uint8_t>* inl, c8::RobustPoseScratchSpace* sc) {
  const std::string pre = g_in_track ? Words(m) : std::string();
  const bool ok = __real__ZN2c826solveImageTargetHomographyERKSt6vectorINS_6HPointILm2EEESaIS2_EES6_RKS0_IfSaIfEENS_14SolvePnPParamsEPNS_7HMatrixEPS0_IhSaIhEEPNS_22RobustPoseScratchSpaceE(
      a, b, w, pr, m, inl, sc);
  if (g_in_track) g_stages += std::string(g_stages.empty() ? "" : ",") + "{\"fn\":\"hom\",\"ok\":" + (ok ? "1" : "0") + ",\"n\":" + std::to_string(a.size()) + ",\"inl\":" + std::to_string(Nz(*inl)) + ",\"pre\":\"" + pre + "\",\"post\":\"" + Words(m) + "\"}";
  return ok;
}
void __real__ZN2c818ImageTrackingState5trackERKNS_15FrameWithPointsERKSt6vectorIS1_SaIS1_EERNS_14DetectionImageEPNS_13RandomNumbersERKNS_7HMatrixEPNS_17ImageWorldTrackerE(
    c8::ImageTrackingState*, const c8::FrameWithPoints&, const std::vector<c8::FrameWithPoints>&, c8::DetectionImage&, c8::RandomNumbers*,
    const c8::HMatrix&, c8::ImageWorldTracker*);
void __wrap__ZN2c818ImageTrackingState5trackERKNS_15FrameWithPointsERKSt6vectorIS1_SaIS1_EERNS_14DetectionImageEPNS_13RandomNumbersERKNS_7HMatrixEPNS_17ImageWorldTrackerE(
    c8::ImageTrackingState* self, const c8::FrameWithPoints& feats, const std::vector<c8::FrameWithPoints>& rois, c8::DetectionImage& im,
    c8::RandomNumbers* rnd, const c8::HMatrix& tam, c8::ImageWorldTracker* wt) {
  const c8::TrackedImage pre = self->trackedImage();
  g_in_track = true;
  g_rng = rnd;
  const bool pre_local = self->localMode();
  __real__ZN2c818ImageTrackingState5trackERKNS_15FrameWithPointsERKSt6vectorIS1_SaIS1_EERNS_14DetectionImageEPNS_13RandomNumbersERKNS_7HMatrixEPNS_17ImageWorldTrackerE(
      self, feats, rois, im, rnd, tam, wt);
  g_in_track = false;
  if (g_out == nullptr) {
    const char* p = std::getenv("ITPROBE_OUT");
    g_out = std::fopen(p != nullptr ? p : "/dev/null", "wx");
    if (g_out == nullptr) std::abort();
  }
  const c8::TrackedImage& post = self->trackedImage();
  size_t gm_pyr = 0, gm_roi = 0, gi_pyr = 0, gi_roi = 0;
  {
    const auto& gm = self->globalMatches();
    const auto& gi = self->globalInliers();
    for (size_t k = 0; k < gm.size(); ++k) {
      const bool pyr = gm[k].wordsIdx < feats.size();
      (pyr ? gm_pyr : gm_roi) += 1;
      if (k < gi.size() && gi[k] != 0) (pyr ? gi_pyr : gi_roi) += 1;
    }
  }
  std::string r = "{\"i\":" + std::to_string(g_frame) + ",\"name\":\"" + std::string(post.name.c_str()) + "\",\"preSt\":" +
                  std::to_string(static_cast<int>(pre.status)) + ",\"preLast\":" + std::to_string(pre.lastSeen) + ",\"preLocal\":" +
                  (pre_local ? "1" : "0") + ",\"feats\":" + std::to_string(feats.size()) + ",\"rois\":[";
  bool first = true;
  for (const c8::FrameWithPoints& f : rois) {
    r += std::string(first ? "" : ",") + "{\"src\":" + std::to_string(static_cast<int>(f.roi().source)) + ",\"n\":" + std::to_string(f.size()) +
         ",\"nm\":\"" + std::string(f.roi().name.c_str()) + "\"}";
    first = false;
  }
  r += "],\"locMatches\":" + std::to_string(self->localMatches().size()) + ",\"locIn\":" + std::to_string(Nz(self->inliers())) +
       ",\"gloMatches\":" + std::to_string(self->globalMatches().size()) + ",\"gloSplit\":[" + std::to_string(gm_pyr) + "," + std::to_string(gm_roi) + "," + std::to_string(gi_pyr) + "," + std::to_string(gi_roi) + "]" + ",\"gloIn\":" + std::to_string(Nz(self->globalInliers())) +
       ",\"postSt\":" + std::to_string(static_cast<int>(post.status)) + ",\"postLast\":" + std::to_string(post.lastSeen) + ",\"postFirst\":" +
       std::to_string(post.firstSeen) + ",\"angle\":" + Num(self->angleFromLastPose()) + ",\"poseWordsPost\":\"" + Words(&post.pose) + "\",\"tvPre\":" + Pose(pre.pose) + ",\"tvPost\":" +
       Pose(post.pose) + ",\"camPos\":" + Pose(tam) + ",\"stages\":[" + g_stages + "]}\n";
  g_stages.clear();
  std::fputs(r.c_str(), g_out);
  std::fflush(g_out);
}
}  // extern "C"

// ---- engine atlas dump (ITPROBE_ATLAS_DIR=<new dir>, frames <= ITPROBE_ATLAS_MAX, default 60): the CPU atlas after GetWithRois (levels + drawn ROI slots,
// RGBA size x rows) and the ROI list (source, name, warp) of that frame; skipped when the atlas was injected (ITPROBE_INJECT_DIR). ----
namespace {
void DumpAtlas(slam_refact::frontend::PyramidSource* self, const slam_refact::PyramidAtlasView& inj, const std::vector<c8::ImageRoi>& rois) {
  const char* dir = std::getenv("ITPROBE_ATLAS_DIR");
  if (dir == nullptr || inj.rgba != nullptr) return;
  const char* mx = std::getenv("ITPROBE_ATLAS_MAX");
  if (g_frame > (mx != nullptr ? std::atoi(mx) : 60)) return;
  const slam_refact::frontend::RgbaAtlas& a = self->last_cpu_atlas();
  char name[64];
  if (g_frame < 0) std::snprintf(name, sizeof name, "boot"); else std::snprintf(name, sizeof name, "f%04d", g_frame);
  const std::string base = std::string(dir) + "/" + name;
  std::ofstream(base + ".rgba", std::ios::binary).write(reinterpret_cast<const char*>(a.rgba.data()), static_cast<std::streamsize>(a.rgba.size()));
  const slam_refact::frontend::PyramidLayout& L = self->layout();
  std::ostringstream j;
  j << "{\"frame\":" << g_frame << ",\"size\":" << a.size << ",\"rows\":" << a.rows << ",\"levels\":[";
  for (size_t k = 0; k < L.levels.size(); ++k) j << (k ? "," : "") << "[" << L.levels[k].r << "," << L.levels[k].c << "," << L.levels[k].w << "," << L.levels[k].h << "," << L.levels[k].rotated << "]";
  j << "],\"roi_slots\":[";
  for (size_t k = 0; k < L.roi_slots.size(); ++k) j << (k ? "," : "") << "[" << L.roi_slots[k].r << "," << L.roi_slots[k].c << "," << L.roi_slots[k].w << "," << L.roi_slots[k].h << "," << L.roi_slots[k].rotated << "]";
  j << "],\"rois\":[";
  for (size_t k = 0; k < rois.size(); ++k) {
    j << (k ? "," : "") << "{\"source\":" << static_cast<int>(rois[k].source) << ",\"name\":\"" << rois[k].name.c_str() << "\",\"warp\":[";
    for (int m = 0; m < 16; ++m) { char b[40]; std::snprintf(b, sizeof b, "%s%.9g", m ? "," : "", rois[k].warp.data()[m]); j << b; }
    j << "]}";
  }
  j << "]}";
  std::ofstream(base + ".json") << j.str();
}
}  // namespace

extern "C" {
c8::Gr8Pyramid __real__ZN11slam_refact8frontend13PyramidSource11GetWithRoisERKNS_16PyramidAtlasViewERKNS_13RgbaImageViewERKNS_13GrayImageViewERKSt6vectorIN2c88ImageRoiESaISD_EE(
    slam_refact::frontend::PyramidSource*, const slam_refact::PyramidAtlasView&, const slam_refact::RgbaImageView&, const slam_refact::GrayImageView&,
    const std::vector<c8::ImageRoi>&);
c8::Gr8Pyramid __wrap__ZN11slam_refact8frontend13PyramidSource11GetWithRoisERKNS_16PyramidAtlasViewERKNS_13RgbaImageViewERKNS_13GrayImageViewERKSt6vectorIN2c88ImageRoiESaISD_EE(
    slam_refact::frontend::PyramidSource* self, const slam_refact::PyramidAtlasView& inj, const slam_refact::RgbaImageView& rgba,
    const slam_refact::GrayImageView& gray, const std::vector<c8::ImageRoi>& rois) {
  c8::Gr8Pyramid p = __real__ZN11slam_refact8frontend13PyramidSource11GetWithRoisERKNS_16PyramidAtlasViewERKNS_13RgbaImageViewERKNS_13GrayImageViewERKSt6vectorIN2c88ImageRoiESaISD_EE(
      self, inj, rgba, gray, rois);
  DumpAtlas(self, inj, rois);
  return p;
}
}  // extern "C"

extern "C" {
c8::Gr8Pyramid __real__ZN11slam_refact8frontend13PyramidSource3GetERKNS_16PyramidAtlasViewERKNS_13RgbaImageViewERKNS_13GrayImageViewE(
    slam_refact::frontend::PyramidSource*, const slam_refact::PyramidAtlasView&, const slam_refact::RgbaImageView&, const slam_refact::GrayImageView&);
c8::Gr8Pyramid __wrap__ZN11slam_refact8frontend13PyramidSource3GetERKNS_16PyramidAtlasViewERKNS_13RgbaImageViewERKNS_13GrayImageViewE(
    slam_refact::frontend::PyramidSource* self, const slam_refact::PyramidAtlasView& inj, const slam_refact::RgbaImageView& rgba,
    const slam_refact::GrayImageView& gray) {
  c8::Gr8Pyramid p = __real__ZN11slam_refact8frontend13PyramidSource3GetERKNS_16PyramidAtlasViewERKNS_13RgbaImageViewERKNS_13GrayImageViewE(self, inj, rgba, gray);
  DumpAtlas(self, inj, std::vector<c8::ImageRoi>());   // frames without ROI draw (engine.cc calls Get instead of GetWithRois)
  return p;
}
}  // extern "C"

#ifndef ITPROBE_NO_ROISS
// ---- ITPROBE_ROI_SS=1 (harness-only experiment, NOT an engine-equivalent replay): after the engine's own DrawRoisIntoAtlas, every planar ROI slot is redrawn with
// the SwiftShader quad model of ss_roi_model.h (validated byte-exact against the original's GPU readback) and stages 2 and 3 are rerun on the atlas. ----
extern "C" {
void __real__ZN11slam_refact8frontend17DrawRoisIntoAtlasERKNS0_13PyramidLayoutERKNS0_13CpuGr8PyramidENS_18ShaderNumericModelERKNS_13RgbaImageViewEiRKSt6vectorIN2c88ImageRoiESaISD_EEPNS0_9RgbaAtlasE(
    const slam_refact::frontend::PyramidLayout&, const slam_refact::frontend::CpuGr8Pyramid&, slam_refact::ShaderNumericModel, const slam_refact::RgbaImageView&, int,
    const std::vector<c8::ImageRoi>&, slam_refact::frontend::RgbaAtlas*);
void __wrap__ZN11slam_refact8frontend17DrawRoisIntoAtlasERKNS0_13PyramidLayoutERKNS0_13CpuGr8PyramidENS_18ShaderNumericModelERKNS_13RgbaImageViewEiRKSt6vectorIN2c88ImageRoiESaISD_EEPNS0_9RgbaAtlasE(
    const slam_refact::frontend::PyramidLayout& layout, const slam_refact::frontend::CpuGr8Pyramid& cpu, slam_refact::ShaderNumericModel model,
    const slam_refact::RgbaImageView& camera, int rotation_deg, const std::vector<c8::ImageRoi>& rois, slam_refact::frontend::RgbaAtlas* atlas) {
  __real__ZN11slam_refact8frontend17DrawRoisIntoAtlasERKNS0_13PyramidLayoutERKNS0_13CpuGr8PyramidENS_18ShaderNumericModelERKNS_13RgbaImageViewEiRKSt6vectorIN2c88ImageRoiESaISD_EEPNS0_9RgbaAtlasE(
      layout, cpu, model, camera, rotation_deg, rois, atlas);
  const char* on = std::getenv("ITPROBE_ROI_SS");
  if (on == nullptr || on[0] != '1') return;
  for (size_t i = 0; i < rois.size(); ++i) {
    if (rois[i].source == c8::ImageRoi::Source::CURVY_IMAGE_TARGET) continue;
    const slam_refact::frontend::LevelLayout& s = layout.roi_slots[i];
    for (int y = 0; y < s.h; ++y)
      for (int x = 0; x < s.w; ++x) atlas->rgba[(static_cast<size_t>(s.r + y) * atlas->size + (s.c + x)) * 4] = 0;
    ssroi::DrawSlot(s.c, s.r, s.w, s.h, rois[i].warp.data().data(), s.rotated ? rotation_deg - 90 : rotation_deg, camera, model, atlas->size, atlas->rgba.data());
  }
  cpu.RunStage2And3(atlas);
}
}  // extern "C"

#endif  // ITPROBE_NO_ROISS

// ---- c8::glImageTargetWarp (called from DetectionImageTracker::locate): inputs and result as raw float words, to ITPROBE_WARP_OUT (new file), tagged with the frame.
// Original counterpart: tag-5 slots of the instrumented wasm (func 670 = glImageTargetWarp inlined; instrument_it_wat.py). ----
#include "reality/engine/geometry/image-warp.h"
extern "C" {
c8::HMatrix __real__ZN2c817glImageTargetWarpE26c8_PixelPinholeCameraModelS0_RKNS_7HMatrixERKNS_19PlanarImageGeometryE(
    c8_PixelPinholeCameraModel, c8_PixelPinholeCameraModel, const c8::HMatrix&, const c8::PlanarImageGeometry&);
c8::HMatrix __wrap__ZN2c817glImageTargetWarpE26c8_PixelPinholeCameraModelS0_RKNS_7HMatrixERKNS_19PlanarImageGeometryE(
    c8_PixelPinholeCameraModel tk, c8_PixelPinholeCameraModel ck, const c8::HMatrix& pose, const c8::PlanarImageGeometry& geom) {
  c8::HMatrix r = __real__ZN2c817glImageTargetWarpE26c8_PixelPinholeCameraModelS0_RKNS_7HMatrixERKNS_19PlanarImageGeometryE(tk, ck, pose, geom);
  static std::FILE* f = nullptr;
  static bool init = false;
  if (!init) {
    init = true;
    const char* p = std::getenv("ITPROBE_WARP_OUT");
    if (p != nullptr) { f = std::fopen(p, "wx"); if (f == nullptr) std::abort(); }
  }
  if (f != nullptr) {
    std::fprintf(f, "{\"i\":%d,\"tk\":\"%s\",\"ck\":\"%s\",\"pose\":\"%s\",\"rot\":%d,\"out\":\"%s\"}\n", g_frame, WordsK(&tk, 6).c_str(), WordsK(&ck, 6).c_str(),
                 Words(&pose).c_str(), geom.isRotated ? 1 : 0, Words(&r).c_str());
    std::fflush(f);
  }
  return r;
}
}  // extern "C"

// ---- c8::scaleTranslation(float, const HMatrix&) while inside ImageTrackingState::track (the lost-frame pose extrapolation and the scale latch); logged into the stage list
// ({"fn":"scaleT","s","cam","out"} as raw float words). Original counterpart: tag-7 slots (func 661 call sites in func 938). ----
#include "c8/geometry/egomotion.h"
extern "C" {
c8::HMatrix __real__ZN2c816scaleTranslationEfRKNS_7HMatrixE(float, const c8::HMatrix&);
c8::HMatrix __wrap__ZN2c816scaleTranslationEfRKNS_7HMatrixE(float s, const c8::HMatrix& cam) {
  c8::HMatrix r = __real__ZN2c816scaleTranslationEfRKNS_7HMatrixE(s, cam);
  if (g_in_track) {
    g_stages += std::string(g_stages.empty() ? "" : ",") + "{\"fn\":\"scaleT\",\"s\":\"" + WordsK(&s, 1) + "\",\"cam\":\"" + Words(&cam) + "\",\"out\":\"" + Words(&r) + "\"}";
  }
  return r;
}
}  // extern "C"
