// Loop close of MapBuilder::Update (US-025 LC-4; docs/spec/12-loop-close.md, design/us025-lc3-impl.md,
// design/us025-lc4-tests.md). Compiled only with SLAM_REFACT_LOOP_CLOSE=ON.
//
// Expected values come from the original xr-slam.wasm (lc-g dump, design/us025-lcg-golden.md), extracted by
// tools/re/loop_close/gen_loop_close_test_data.py into tests/data/loop_close/; none of them is produced by the
// engine. Tiers:
//   Golden      candidate selection (G-C), local match (G-D), PnP (G-E), correction matrix (G-F) against the
//               original's recorded inputs and outputs.
//   Synthetic   boundaries (Hamming 79/80, matches 74/75, correspondences 34/35, slots 4/5, shared points 9/10), the
//               ring, the merge/associate/unlink branches. The branches after a successful pose (spec S12-c/d) are
//               the engine's defined behaviour (deviation N2): the original crashes in that loop (U3), so they
//               cannot be compared with it.
//   OFF         loop close disabled: no descriptor, no correction, no id consumed.
//
// Pre-declared tolerances (spec 12 §8, fixed before the results were seen): selection, local match, counts and
// the correction matrix are exact (integers / float bits); the PnP output pose is compared with 3 ulp per float.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "config/params.h"
#include "config/settings.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map_builder/agate_descriptor.h"
#include "map_builder/loop_close.h"
#include "map_builder/map_builder.h"
#include "map_builder_test_util.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/image-descriptor.h"
#include "reality/engine/features/local-matcher.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/observed-point.h"

namespace slam_refact::map_builder {
namespace {

using testing::BitsToFloat;
using testing::Pose;
using testing::Project;
using testing::ReadFile;
using testing::Split;
using testing::TestIntrinsics;

using Desc = std::array<uint8_t, 32>;

const std::string kDataDir = std::string(SR_TEST_DATA_DIR) + "/loop_close/";

// ---------------------------------------------------------------------------------------------
// small numeric helpers
// ---------------------------------------------------------------------------------------------

uint32_t BitsOf(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

int64_t OrderedBits(float f) {
  int32_t i;
  std::memcpy(&i, &f, sizeof(i));
  return i < 0 ? static_cast<int64_t>(INT32_MIN) - i : i;
}

int64_t UlpDistance(float a, float b) { return std::llabs(OrderedBits(a) - OrderedBits(b)); }

uint32_t ReadU32(const uint8_t* p) {
  uint32_t v;
  std::memcpy(&v, p, 4);
  return v;
}

float ReadF32(const uint8_t* p) {
  float v;
  std::memcpy(&v, p, 4);
  return v;
}

Desc HexDesc(const std::string& hex) {
  Desc d{};
  for (size_t i = 0; i < d.size(); ++i) {
    d[i] = static_cast<uint8_t>(std::stoul(hex.substr(2 * i, 2), nullptr, 16));
  }
  return d;
}

Desc FilledDesc(uint8_t v) {
  Desc d;
  d.fill(v);
  return d;
}

// A descriptor with exactly `bits` low bits set (Hamming distance `bits` from the all-zero descriptor).
Desc DescWithBits(int bits) {
  Desc d{};
  for (int i = 0; i < bits; ++i) {
    d[static_cast<size_t>(i / 8)] = static_cast<uint8_t>(d[static_cast<size_t>(i / 8)] | (1u << (i % 8)));
  }
  return d;
}

// 0x84 B original HMatrix record (forward float[16] column major, inverse float[16], two flag bytes).
c8::HMatrix HMatrixFromRecord(const uint8_t* p) {
  float fwd[16];
  float inv[16];
  std::memcpy(fwd, p, 64);
  std::memcpy(inv, p + 64, 64);
  EXPECT_EQ(p[128], 0);  // noInvert
  EXPECT_EQ(p[129], 0);  // invertFailed
  return c8::HMatrix(fwd, inv);
}

// The original's 4x4 product as recorded by lc-g for func 660 (7/7 calls bit-identical, us025-lcg-golden.md §7 F4/F5):
// column major, out(r,c) = ((a(r,0)*b(0,c) + a(r,1)*b(1,c)) + a(r,2)*b(2,c)) + a(r,3)*b(3,c). Restated here
// independently of the engine's HMatrix.
std::array<float, 16> MulUnrolled(const std::array<float, 16>& a, const std::array<float, 16>& b) {
  std::array<float, 16> o{};
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      volatile float t0 = a[static_cast<size_t>(r)] * b[static_cast<size_t>(4 * c)];
      volatile float t1 = a[static_cast<size_t>(4 + r)] * b[static_cast<size_t>(4 * c + 1)];
      volatile float s = t0 + t1;
      volatile float t2 = a[static_cast<size_t>(8 + r)] * b[static_cast<size_t>(4 * c + 2)];
      s = s + t2;
      volatile float t3 = a[static_cast<size_t>(12 + r)] * b[static_cast<size_t>(4 * c + 3)];
      s = s + t3;
      o[static_cast<size_t>(4 * c + r)] = s;
    }
  }
  return o;
}

// The summation order spec 12 §5.1 originally read from func 789 (refuted by the same 7 recordings, 0/7).
std::array<float, 16> Mul789(const std::array<float, 16>& a, const std::array<float, 16>& b) {
  std::array<float, 16> o{};
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      volatile float t3 = a[static_cast<size_t>(12 + r)] * b[static_cast<size_t>(4 * c + 3)];
      volatile float t2 = a[static_cast<size_t>(8 + r)] * b[static_cast<size_t>(4 * c + 2)];
      volatile float s = t3 + t2;
      volatile float t0 = a[static_cast<size_t>(r)] * b[static_cast<size_t>(4 * c)];
      s = s + t0;
      volatile float t1 = a[static_cast<size_t>(4 + r)] * b[static_cast<size_t>(4 * c + 1)];
      s = s + t1;
      o[static_cast<size_t>(4 * c + r)] = s;
    }
  }
  return o;
}

int DifferingWords(const std::array<float, 16>& a, const std::array<float, 16>& b) {
  int n = 0;
  for (size_t i = 0; i < 16; ++i) {
    n += BitsOf(a[i]) != BitsOf(b[i]) ? 1 : 0;
  }
  return n;
}

// ---------------------------------------------------------------------------------------------
// frames without features, features from golden data
// ---------------------------------------------------------------------------------------------

c8::FrameWithPoints PlainFrame(size_t rays) {
  c8::FrameWithPoints f(TestIntrinsics());
  for (size_t i = 0; i < rays; ++i) {
    f.addImageRayPoint(c8::HPoint2(-0.5f + 0.002f * static_cast<float>(i), 0.1f), 1, 0.25f, 0.5f,
                       c8::FeatureSet());
  }
  return f;
}

c8::FeatureSet OrbSet(const Desc& d) {
  c8::FeatureSet set;
  set.orbFeature = c8::OrbFeature(d);
  return set;
}

// ---------------------------------------------------------------------------------------------
// G-C data: select-attempts.tsv
// ---------------------------------------------------------------------------------------------

struct Edge {
  size_t a;
  size_t b;
  int shared;
};

struct TailEntry {
  size_t m;
  uint32_t bits;
  uint32_t index;
  uint32_t id;
};

struct Attempt {
  std::string run;
  uint32_t seq = 0;
  bool hook = false;
  size_t n_slots = 0;
  size_t newest = 0;
  std::vector<uint32_t> ids;  // 0 = free slot
  uint32_t pick = 0;          // the original's ring entry after the call (0 = no candidate)
  std::vector<Edge> edges;    // covisibility with shared points 9 or 10
  uint32_t best_bits = 0;     // hook runs: u2-result
  uint32_t cand_idx = 0;
  uint32_t cand_id = 0;
  int found = 0;                // u2-result.found (set by the pose stage, not by the selection)
  std::vector<TailEntry> tail;  // hook runs: the uninitialised tail of the original's distance list
  size_t live = 0;
};

struct SelectData {
  std::map<std::pair<std::string, uint32_t>, Desc> desc;
  std::vector<Attempt> attempts;
};

SelectData LoadSelect() {
  SelectData d;
  const std::string text = ReadFile(kDataDir + "select-attempts.tsv");
  for (const std::string& line : Split(text, '\n')) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const std::vector<std::string> f = Split(line, '\t');
    if (f[0] == "D") {
      d.desc[{f[1], static_cast<uint32_t>(std::stoul(f[2]))}] = HexDesc(f[3]);
      continue;
    }
    Attempt a;
    a.run = f[1];
    a.seq = static_cast<uint32_t>(std::stoul(f[2]));
    a.hook = f[3] == "1";
    a.n_slots = std::stoul(f[4]);
    a.newest = std::stoul(f[5]);
    for (const std::string& t : Split(f[6], ',')) {
      a.ids.push_back(static_cast<uint32_t>(std::stoul(t)));
    }
    a.pick = static_cast<uint32_t>(std::stoul(f[7]));
    if (f[8] != "-") {
      for (const std::string& t : Split(f[8], ';')) {
        const std::vector<std::string> e = Split(t, '-');
        a.edges.push_back({std::stoul(e[0]), std::stoul(e[1]), std::stoi(e[2])});
      }
    }
    if (f[9] != "-") {
      const std::vector<std::string> r = Split(f[9], ':');
      a.best_bits = static_cast<uint32_t>(std::stoul(r[0], nullptr, 16));
      a.cand_idx = static_cast<uint32_t>(std::stoul(r[1]));
      a.cand_id = static_cast<uint32_t>(std::stoul(r[2]));
      a.found = std::stoi(r[3]);
    }
    if (f[10] != "-") {
      for (const std::string& t : Split(f[10], ';')) {
        const std::vector<std::string> e = Split(t, ':');
        a.tail.push_back({std::stoul(e[0]), static_cast<uint32_t>(std::stoul(e[1], nullptr, 16)),
                          static_cast<uint32_t>(std::stoul(e[2])), static_cast<uint32_t>(std::stoul(e[3]))});
      }
    }
    for (uint32_t id : a.ids) {
      a.live += id != 0 ? 1 : 0;
    }
    d.attempts.push_back(std::move(a));
  }
  return d;
}

const SelectData& Select() {
  static const SelectData data = LoadSelect();
  return data;
}

// A WorldMap shaped like one original attempt: same slot count, free slots, newest slot, ids (the newest
// keyframe keeps the id the engine's generator gave it; it is mapped back with ToGolden), descriptors and
// covisibility (shared >= 10 or exactly 9 points). The frames carry no features, so the pose stage ends with
// too few matches whichever candidate is chosen.
struct BuiltAttempt {
  std::unique_ptr<map::WorldMaps> maps;
  uint32_t engine_newest_id = 0;
  uint32_t golden_newest_id = 0;
  uint32_t ToGolden(uint32_t id) const { return id == engine_newest_id ? golden_newest_id : id; }
  uint32_t FromGolden(uint32_t id) const { return id == golden_newest_id ? engine_newest_id : id; }
};

BuiltAttempt Build(const Attempt& a) {
  BuiltAttempt b;
  b.maps = std::make_unique<map::WorldMaps>();
  map::WorldMap& wm = b.maps->active();
  const size_t n = a.n_slots;
  std::vector<size_t> need(n, 0);
  std::vector<size_t> next_obs(n, 0);
  for (const Edge& e : a.edges) {
    need[e.a] += static_cast<size_t>(e.shared);
    need[e.b] += static_cast<size_t>(e.shared);
  }
  std::vector<map::KeyframeId> h(n);
  for (size_t s = 0; s < n; ++s) {
    h[s] = wm.AddKeyframe(PlainFrame(need[s]), static_cast<int64_t>(5 * s));
  }
  const size_t k = a.newest;
  for (size_t s = 0; s < n; ++s) {
    if (a.ids[s] == 0) {
      wm.RemoveKeyframe(h[s]);
    }
  }
  if (k != n - 1) {
    wm.RemoveKeyframe(h[k]);  // pushed last on the free list, so the next AddKeyframe reuses slot k
    h[k] = wm.AddKeyframe(PlainFrame(need[k]), static_cast<int64_t>(5 * k));
  }
  EXPECT_EQ(h[k].index(), k);
  EXPECT_TRUE(wm.newest() == h[k]);
  b.engine_newest_id = h[k].id();
  b.golden_newest_id = a.ids[k];
  for (size_t s = 0; s < n; ++s) {
    if (a.ids[s] == 0 || s == k) {
      continue;
    }
    wm.MutableKeyframeAtSlot(s).id = a.ids[s];
    h[s] = map::KeyframeId(s, a.ids[s]);
  }
  for (size_t s = 0; s < n; ++s) {
    if (a.ids[s] != 0) {
      wm.MutableKeyframeAtSlot(s).global_descriptor = Select().desc.at({a.run, a.ids[s]});
    }
  }
  for (const Edge& e : a.edges) {
    for (int i = 0; i < e.shared; ++i) {
      const map::PointId p =
          wm.AddPoint(wm.GetKeyframe(h[e.a]).seq, c8::HPoint3(0.f, 0.f, 2.f), map::PointSource::kGround);
      wm.AddObservation(p, static_cast<uint32_t>(next_obs[e.a]++), h[e.a]);
      wm.AddObservation(p, static_cast<uint32_t>(next_obs[e.b]++), h[e.b]);
    }
  }
  return b;
}

struct PickResult {
  LoopCloseOutcome outcome = LoopCloseOutcome::kTooFewSlots;
  bool picked = false;
  uint32_t id = 0;     // golden id space
  uint32_t index = 0;
  float distance = -1.0f;
};

PickResult RunSelection(BuiltAttempt* b, LoopCloseState* state) {
  // Every attempt of a real run has a new newest keyframe, so the ring is cleared first (spec 12 S4). The rebuilt
  // maps repeat the engine's id sequence, so make that explicit; the distance storage of `state` is kept.
  state->last_newest = map::KeyframeId(~static_cast<size_t>(0) >> 1, 0);
  MapBuilderOutput out;
  LoopCloseReport rep;
  LoopClose(b->maps->active(), state, &out, &rep);
  PickResult r;
  r.outcome = rep.outcome;
  r.picked = rep.outcome != LoopCloseOutcome::kTooFewSlots && rep.outcome != LoopCloseOutcome::kNoCandidate;
  if (r.picked) {
    r.id = b->ToGolden(rep.candidate.id());
    r.index = static_cast<uint32_t>(rep.candidate.index());
    r.distance = rep.candidate_distance;
  }
  return r;
}

struct SelectRow {
  const Attempt* attempt = nullptr;
  PickResult id0;   // default model, one state per run (the engine keeps one state for a whole run)
  PickResult tail;  // observed original tail seeded into the distance storage (hook runs only)
  bool has_tail = false;
};

const std::vector<SelectRow>& SelectRows() {
  static const std::vector<SelectRow> rows = [] {
    std::vector<SelectRow> out;
    std::map<std::string, LoopCloseState> run_states;
    for (const Attempt& a : Select().attempts) {
      SelectRow row;
      row.attempt = &a;
      BuiltAttempt b = Build(a);
      row.id0 = RunSelection(&b, &run_states[a.run]);
      if (a.hook) {
        LoopCloseState seeded;
        seeded.model_stale_dist_tail = true;  // keep the storage, so the seeded tail is what the loop reads
        seeded.dist_storage.assign(a.n_slots, LoopCloseDistEntry{0.0f, 0, 0});
        for (const TailEntry& t : a.tail) {
          LoopCloseDistEntry& e = seeded.dist_storage[t.m];
          std::memcpy(&e.distance, &t.bits, 4);
          e.index = t.index;
          e.id = b.FromGolden(t.id);
        }
        row.tail = RunSelection(&b, &seeded);
        row.has_tail = true;
      }
      out.push_back(row);
    }
    return out;
  }();
  return rows;
}

bool IsLive(const Attempt& a, uint32_t id) { return std::find(a.ids.begin(), a.ids.end(), id) != a.ids.end() && id != 0; }

TEST(LoopCloseSelectGolden, InventoryOfTheRecordedAttempts) {
  const SelectData& d = Select();
  size_t hook = 0;
  size_t free_slots = 0;
  size_t edges9 = 0;
  size_t edges10 = 0;
  for (const Attempt& a : d.attempts) {
    hook += a.hook ? 1 : 0;
    free_slots += a.live < a.n_slots ? 1 : 0;
    for (const Edge& e : a.edges) {
      (e.shared == 9 ? edges9 : edges10)++;
    }
    EXPECT_GE(a.n_slots, 5u);
    EXPECT_EQ(a.ids.size(), a.n_slots);
    EXPECT_NE(a.ids[a.newest], 0u);
  }
  EXPECT_EQ(d.attempts.size(), 615u);
  EXPECT_EQ(hook, 433u);
  EXPECT_EQ(free_slots, 327u);  // attempts in which the compressed list differs from the slot list
  EXPECT_GT(edges9, 0u);        // shared-point counts on both sides of requiredPointsForCovisibility (10)
  EXPECT_GT(edges10, 0u);
}

// The original's pick reproduced with the original's own uninitialised tail: compressed live prefix, strict <,
// newest-index skip and 2-hop covisible exclusion applied at the compressed position (spec 12 S3-S5).
TEST(LoopCloseSelectGolden, ObservedTailReproducesEveryOriginalPick) {
  int agree = 0;
  int total = 0;
  int picks_at_79 = 0;
  for (const SelectRow& r : SelectRows()) {
    if (!r.has_tail) {
      continue;
    }
    const Attempt& a = *r.attempt;
    ++total;
    picks_at_79 += a.pick != 0 && BitsOf(79.0f) == a.best_bits ? 1 : 0;
    const bool ok = r.tail.id == a.pick && (a.pick != 0) == r.tail.picked;
    agree += ok ? 1 : 0;
    EXPECT_TRUE(ok) << a.run << " seq " << a.seq << ": original pick " << a.pick << ", engine " << r.tail.id;
    if (a.pick != 0 && ok) {  // u2-result.found is the pose stage flag, not "a candidate exists"
      EXPECT_EQ(r.tail.index, a.cand_idx) << a.run << " seq " << a.seq;
      EXPECT_EQ(r.tail.id, a.cand_id) << a.run << " seq " << a.seq;
      EXPECT_EQ(BitsOf(r.tail.distance), a.best_bits) << a.run << " seq " << a.seq;
    }
  }
  EXPECT_EQ(total, 433);
  EXPECT_EQ(agree, 433);
  EXPECT_EQ(picks_at_79, 3);  // the recorded picks include the last distance below the threshold 80
  // A pick of the unreproducible heap tail is not a live keyframe: the pose stage stops silently at S7.
  int original_not_live = 0;
  int engine_invalid_handle = 0;
  for (const SelectRow& r : SelectRows()) {
    if (!r.has_tail) {
      continue;
    }
    original_not_live += r.attempt->pick != 0 && !IsLive(*r.attempt, r.attempt->pick) ? 1 : 0;
    engine_invalid_handle += r.tail.outcome == LoopCloseOutcome::kInvalidHandle ? 1 : 0;
  }
  std::printf("[loop_close] G-C picks of a non-live id (heap tail): %d\n", original_not_live);
  EXPECT_EQ(engine_invalid_handle, original_not_live);
  EXPECT_EQ(original_not_live, 33);  // lc-g: 33 of 433 attempts (7.6%)
}

// With the tail modelled as id 0 entries (the engine default) the picks agree with the original on a fixed count;
// every disagreement is a pick the original took from the unreproducible heap tail (a non-live id found in the
// tail), never a selection-rule difference.
TEST(LoopCloseSelectGolden, IdZeroTailModelAgreementIsPinned) {
  int agree_hook = 0;
  int agree_plain = 0;
  int total_hook = 0;
  int total_plain = 0;
  for (const SelectRow& r : SelectRows()) {
    const Attempt& a = *r.attempt;
    const bool ok = r.id0.id == a.pick && r.id0.picked == (a.pick != 0);
    (a.hook ? total_hook : total_plain)++;
    if (ok) {
      (a.hook ? agree_hook : agree_plain)++;
      continue;
    }
    if (a.hook) {
      const bool explained = a.pick != 0 && !IsLive(a, a.pick) &&
                             std::any_of(a.tail.begin(), a.tail.end(),
                                         [&](const TailEntry& t) { return t.id == a.pick; });
      EXPECT_TRUE(explained) << a.run << " seq " << a.seq << " pick " << a.pick << " engine " << r.id0.id;
    }
  }
  EXPECT_EQ(total_hook, 433);
  EXPECT_EQ(total_plain, 182);
  EXPECT_EQ(agree_hook, 400);   // lc-g G-C: 400/433 (92%)
  EXPECT_EQ(agree_plain, 174);  // lc-g G-C: 174/182 (96%)
}

// N1: a free slot before the newest keyframe moves its compressed position, so the newest keyframe is not skipped
// and (distance 0) becomes its own candidate. The original does the same (77 of its 449 picks).
TEST(LoopCloseSelectGolden, NewestKeyframeIsItsOwnCandidateWhenAFreeSlotPrecedesIt) {
  int self_orig = 0;
  int self_engine = 0;
  for (const SelectRow& r : SelectRows()) {
    const Attempt& a = *r.attempt;
    if (!a.hook) {
      continue;
    }
    const uint32_t newest_id = a.ids[a.newest];
    const bool orig_self = a.pick == newest_id;
    const bool eng_self = r.tail.picked && r.tail.id == newest_id;
    self_orig += orig_self ? 1 : 0;
    self_engine += eng_self ? 1 : 0;
    EXPECT_EQ(orig_self, eng_self) << a.run << " seq " << a.seq;
    if (orig_self) {
      bool free_before = false;
      for (size_t s = 0; s < a.newest; ++s) {
        free_before = free_before || a.ids[s] == 0;
      }
      EXPECT_TRUE(free_before) << a.run << " seq " << a.seq;
    }
  }
  EXPECT_EQ(self_engine, self_orig);
  EXPECT_EQ(self_orig, 50);
  std::printf("[loop_close] G-C self-candidate picks in hook runs: original %d, engine %d\n", self_orig, self_engine);
}

// ---------------------------------------------------------------------------------------------
// synthetic keyframe maps (selection)
// ---------------------------------------------------------------------------------------------

class SynthMap {
 public:
  explicit SynthMap(size_t slots, size_t rays_per_frame = 120) : rays_(rays_per_frame) {
    wm_ = &maps_.active();
    for (size_t s = 0; s < slots; ++s) {
      h_.push_back(wm_->AddKeyframe(PlainFrame(rays_), static_cast<int64_t>(5 * s)));
      next_obs_.push_back(0);
      wm_->MutableKeyframeAtSlot(s).global_descriptor = FilledDesc(0xFF);  // 256 bits from the newest
    }
    wm_->MutableKeyframeAtSlot(slots - 1).global_descriptor = Desc{};  // the newest: all zero
  }
  map::WorldMap& wm() { return *wm_; }
  map::KeyframeId h(size_t s) const { return h_[s]; }
  void SetDistance(size_t s, int bits) { wm_->MutableKeyframeAtSlot(s).global_descriptor = DescWithBits(bits); }
  void Free(size_t s) { wm_->RemoveKeyframe(h_[s]); }
  // Makes slot k the newest keyframe (remove and re-add: the freed slot is reused).
  void MakeNewest(size_t k) {
    wm_->RemoveKeyframe(h_[k]);
    h_[k] = wm_->AddKeyframe(PlainFrame(rays_), 5 * static_cast<int64_t>(k));
    ASSERT_EQ(h_[k].index(), k);
    wm_->MutableKeyframeAtSlot(k).global_descriptor = Desc{};
  }
  void Covisible(size_t a, size_t b, int shared) {
    for (int i = 0; i < shared; ++i) {
      const map::PointId p =
          wm_->AddPoint(wm_->GetKeyframe(h_[a]).seq, c8::HPoint3(0.f, 0.f, 2.f), map::PointSource::kGround);
      wm_->AddObservation(p, static_cast<uint32_t>(next_obs_[a]++), h_[a]);
      wm_->AddObservation(p, static_cast<uint32_t>(next_obs_[b]++), h_[b]);
    }
  }

 private:
  map::WorldMaps maps_;
  map::WorldMap* wm_ = nullptr;
  size_t rays_;
  std::vector<map::KeyframeId> h_;
  std::vector<size_t> next_obs_;
};

LoopCloseReport Attempt1(SynthMap* m, LoopCloseState* st) {
  MapBuilderOutput out;
  LoopCloseReport rep;
  LoopClose(m->wm(), st, &out, &rep);
  return rep;
}

bool Picked(const LoopCloseReport& r) {
  return r.outcome != LoopCloseOutcome::kTooFewSlots && r.outcome != LoopCloseOutcome::kNoCandidate;
}

// [I] overwrites the success flag on every run (out+0x1c, spec 12 s6); the correction is left alone on failure.
TEST(LoopCloseSelect, FailedAttemptClearsTheTransformFlagAndKeepsTheCorrection) {
  SynthMap m(4);
  LoopCloseState st;
  MapBuilderOutput out;
  out.world_transform_updated = true;
  const c8::HMatrix marker = Pose(0.3f, 1.f, 2.f, 3.f);
  out.world_correction = marker;
  LoopCloseReport rep;
  EXPECT_FALSE(LoopClose(m.wm(), &st, &out, &rep));
  EXPECT_FALSE(out.world_transform_updated);
  EXPECT_EQ(DifferingWords(out.world_correction.data(), marker.data()), 0);
}

TEST(LoopCloseSelect, FewerThanFiveSlotsDoNotRunAndLeaveTheStateUntouched) {
  SynthMap m(4);
  m.SetDistance(0, 3);
  LoopCloseState st;
  st.ring = {7, 8, 9};
  st.cursor = 2;
  st.last_newest = map::KeyframeId(1, 77);
  const LoopCloseReport rep = Attempt1(&m, &st);
  EXPECT_EQ(rep.outcome, LoopCloseOutcome::kTooFewSlots);
  EXPECT_EQ(st.ring, (std::array<uint32_t, 3>{7, 8, 9}));
  EXPECT_EQ(st.cursor, 2u);
  EXPECT_TRUE(st.last_newest == map::KeyframeId(1, 77));
}

// S1 counts slots, free slots included: 5 slots with two free ones (3 live) still run the search.
TEST(LoopCloseSelect, SlotCountNotLiveCountGatesTheSearch) {
  SynthMap m(5);
  m.MakeNewest(0);
  m.SetDistance(4, 200);
  m.SetDistance(1, 10);
  m.Free(3);
  m.Free(4);
  ASSERT_EQ(m.wm().NumKeyframeSlots(), 5u);
  ASSERT_EQ(m.wm().NumKeyframes(), 3);
  LoopCloseState st;
  const LoopCloseReport rep = Attempt1(&m, &st);
  EXPECT_NE(rep.outcome, LoopCloseOutcome::kTooFewSlots);
  EXPECT_EQ(rep.num_slots, 5u);
  EXPECT_EQ(rep.num_live, 3u);
  EXPECT_EQ(rep.candidate.id(), m.h(1).id());
}

// S2 (G936:4247-4256): a newest handle that does not match its slot aborts, it is not a silent not-found.
TEST(LoopCloseSelect, NewestHandleMismatchAborts) {
  EXPECT_DEATH(
      {
        SynthMap m(6);
        m.wm().MutableKeyframeAtSlot(5).id = 12345;  // the slot of the newest keyframe now holds another id
        LoopCloseState st;
        Attempt1(&m, &st);
      },
      "Keyframe with this id doesn't exist");
}

TEST(LoopCloseSelect, HammingThresholdIsStrictAtEighty) {
  {
    SynthMap m(6);
    m.SetDistance(0, 79);
    LoopCloseState st;
    const LoopCloseReport rep = Attempt1(&m, &st);
    ASSERT_TRUE(Picked(rep));
    EXPECT_EQ(rep.candidate.id(), m.h(0).id());
    EXPECT_EQ(rep.candidate_distance, 79.0f);
  }
  {
    SynthMap m(6);
    m.SetDistance(0, 80);
    LoopCloseState st;
    const LoopCloseReport rep = Attempt1(&m, &st);
    EXPECT_EQ(rep.outcome, LoopCloseOutcome::kNoCandidate);
    EXPECT_TRUE(st.ring == (std::array<uint32_t, 3>{0, 0, 0}));
    EXPECT_EQ(st.cursor, 0u);
  }
}

TEST(LoopCloseSelect, ThresholdComesFromTheConfigKey) {
  {
    // raised above the default: a candidate at 90 bits is found
    const testing::ScopedParamOverrides raised({{"WorldMaps.loopClosingDistanceThreshold", ParamValue(100.0f)}});
    SynthMap m(6);
    m.SetDistance(0, 90);
    LoopCloseState st;
    const LoopCloseReport rep = Attempt1(&m, &st);
    ASSERT_TRUE(Picked(rep));
    EXPECT_EQ(rep.candidate.id(), m.h(0).id());
  }
  const testing::ScopedParamOverrides overrides({{"WorldMaps.loopClosingDistanceThreshold", ParamValue(50.0f)}});
  {
    SynthMap m(6);
    m.SetDistance(0, 49);
    LoopCloseState st;
    EXPECT_TRUE(Picked(Attempt1(&m, &st)));
  }
  {
    SynthMap m(6);
    m.SetDistance(0, 50);
    LoopCloseState st;
    EXPECT_EQ(Attempt1(&m, &st).outcome, LoopCloseOutcome::kNoCandidate);
  }
}

TEST(LoopCloseSelect, EqualDistanceKeepsTheFirstSlotAndASmallerOneReplacesIt) {
  {
    SynthMap m(7);
    m.SetDistance(1, 40);
    m.SetDistance(3, 40);
    LoopCloseState st;
    const LoopCloseReport rep = Attempt1(&m, &st);
    EXPECT_EQ(rep.candidate.id(), m.h(1).id());  // strict <: the later equal entry does not replace
  }
  {
    SynthMap m(7);
    m.SetDistance(1, 40);
    m.SetDistance(3, 39);
    LoopCloseState st;
    const LoopCloseReport rep = Attempt1(&m, &st);
    EXPECT_EQ(rep.candidate.id(), m.h(3).id());
    EXPECT_EQ(rep.candidate_distance, 39.0f);
  }
}

// 1143 + the 2-hop set: the covisible keyframes of the newest and their covisible keyframes are excluded, by slot
// index. A neighbour with a smaller distance than the one that is picked is skipped.
TEST(LoopCloseSelect, TwoHopCovisibleKeyframesAreExcluded) {
  SynthMap m(8);
  m.SetDistance(0, 5);   // covisible with the newest
  m.SetDistance(1, 6);   // covisible with slot 0 (second hop)
  m.SetDistance(2, 7);   // not covisible with anything
  m.Covisible(7, 0, 10);
  m.Covisible(0, 1, 10);
  LoopCloseState st;
  const LoopCloseReport rep = Attempt1(&m, &st);
  ASSERT_TRUE(Picked(rep));
  EXPECT_EQ(rep.candidate.id(), m.h(2).id());
}

TEST(LoopCloseSelect, NineSharedPointsAreNotCovisibleAndTenAre) {
  {
    SynthMap m(7);
    m.SetDistance(0, 5);
    m.Covisible(6, 0, 9);
    LoopCloseState st;
    EXPECT_EQ(Attempt1(&m, &st).candidate.id(), m.h(0).id());
  }
  {
    SynthMap m(7);
    m.SetDistance(0, 5);
    m.SetDistance(1, 60);
    m.Covisible(6, 0, 10);
    LoopCloseState st;
    EXPECT_EQ(Attempt1(&m, &st).candidate.id(), m.h(1).id());
  }
  {
    // requiredPointsForCovisibility is read from the configuration.
    const testing::ScopedParamOverrides overrides({{"WorldMap.requiredPointsForCovisibility", ParamValue(9)}});
    SynthMap m(7);
    m.SetDistance(0, 5);
    m.SetDistance(1, 60);
    m.Covisible(6, 0, 9);
    LoopCloseState st;
    EXPECT_EQ(Attempt1(&m, &st).candidate.id(), m.h(1).id());
  }
}

// S4/S6: a recent pick is not chosen again (ring of three), the cursor wraps and the oldest entry is overwritten.
TEST(LoopCloseSelect, RingExcludesRecentPicksAndWrapsAfterThree) {
  SynthMap m(7);
  m.SetDistance(0, 10);
  m.SetDistance(1, 20);
  m.SetDistance(2, 30);
  m.SetDistance(3, 40);
  LoopCloseState st;
  LoopCloseReport r1 = Attempt1(&m, &st);
  EXPECT_EQ(r1.candidate.id(), m.h(0).id());
  EXPECT_EQ(st.cursor, 1u);
  LoopCloseReport r2 = Attempt1(&m, &st);
  EXPECT_EQ(r2.candidate.id(), m.h(1).id());
  EXPECT_EQ(st.cursor, 2u);
  LoopCloseReport r3 = Attempt1(&m, &st);
  EXPECT_EQ(r3.candidate.id(), m.h(2).id());
  EXPECT_EQ(st.cursor, 0u);  // wrapped
  EXPECT_EQ(st.ring[0], m.h(0).id());
  EXPECT_EQ(st.ring[1], m.h(1).id());
  EXPECT_EQ(st.ring[2], m.h(2).id());
  LoopCloseReport r4 = Attempt1(&m, &st);
  EXPECT_EQ(r4.candidate.id(), m.h(3).id());
  EXPECT_EQ(st.ring[0], m.h(3).id());  // the oldest entry is overwritten
  EXPECT_EQ(st.cursor, 1u);
  LoopCloseReport r5 = Attempt1(&m, &st);
  EXPECT_EQ(r5.candidate.id(), m.h(0).id());  // slot 0 is no longer in the ring
  EXPECT_EQ(st.ring[1], m.h(0).id());
}

TEST(LoopCloseSelect, RingIsClearedWhenTheNewestKeyframeChanges) {
  SynthMap m(7);
  m.SetDistance(0, 10);
  LoopCloseState st;
  EXPECT_EQ(Attempt1(&m, &st).candidate.id(), m.h(0).id());
  EXPECT_EQ(st.ring[0], m.h(0).id());
  // Same newest keyframe: slot 0 is excluded, nothing else qualifies.
  EXPECT_EQ(Attempt1(&m, &st).outcome, LoopCloseOutcome::kNoCandidate);
  EXPECT_EQ(st.ring[0], m.h(0).id());
  // A new newest keyframe: the ring is cleared (the pick of the new attempt lands in entry 0 again).
  m.MakeNewest(5);
  m.SetDistance(6, 255);
  LoopCloseReport rep = Attempt1(&m, &st);
  EXPECT_TRUE(st.last_newest == m.h(5));
  EXPECT_EQ(rep.candidate.id(), m.h(0).id());
  EXPECT_EQ(st.ring[0], m.h(0).id());
  EXPECT_EQ(st.ring[1], 0u);
  EXPECT_EQ(st.ring[2], 0u);
  EXPECT_EQ(st.cursor, 1u);
}

// U2 compression (spec 12 S3/S5): live entries are packed, but the loop reads them at slot indices. With free slot 0
// before the newest keyframe (slot 1), the newest is at compressed position 0 (not skipped, distance 0) and is its
// own candidate (N1).
TEST(LoopCloseSelect, NewestIsItsOwnCandidateWhenAFreeSlotPrecedesIt) {
  SynthMap m(5);
  m.SetDistance(2, 5);
  m.SetDistance(3, 6);
  m.SetDistance(4, 7);
  m.MakeNewest(1);
  m.Free(0);
  LoopCloseState st;
  const LoopCloseReport rep = Attempt1(&m, &st);
  EXPECT_EQ(rep.num_live, 4u);
  ASSERT_TRUE(Picked(rep));
  EXPECT_EQ(rep.candidate.id(), m.h(1).id());  // the newest keyframe itself
  EXPECT_EQ(rep.candidate_distance, 0.0f);
}

// The skip `m == newest.index` and the covisible exclusion act on the compressed position. Same map as above with the
// newest keyframe in the ring (so it cannot be chosen): X (slot 2, distance 5) sits at compressed position 1 ==
// newest.index and is skipped; the keyframe at compressed position 2 (slot 3, distance 6) wins. An uncompressed list
// would have chosen X.
TEST(LoopCloseSelect, KeyframeAtTheCompressedPositionOfTheNewestIndexIsSkipped) {
  SynthMap m(5);
  m.SetDistance(2, 5);
  m.SetDistance(3, 6);
  m.SetDistance(4, 7);
  m.MakeNewest(1);
  m.Free(0);
  LoopCloseState st;
  st.last_newest = m.h(1);          // same newest keyframe: the ring is not cleared
  st.ring = {m.h(1).id(), 0, 0};
  st.cursor = 1;
  const LoopCloseReport rep = Attempt1(&m, &st);
  ASSERT_TRUE(Picked(rep));
  EXPECT_EQ(rep.candidate.id(), m.h(3).id());
  EXPECT_EQ(rep.candidate_distance, 6.0f);
}

// Without a free slot before it, the newest keyframe sits at its own compressed position and is skipped.
TEST(LoopCloseSelect, FreeSlotAfterTheNewestDoesNotShiftIt) {
  SynthMap m(6);
  m.SetDistance(2, 5);
  m.MakeNewest(1);
  m.SetDistance(5, 200);
  m.Free(4);
  LoopCloseState st;
  const LoopCloseReport rep = Attempt1(&m, &st);
  ASSERT_TRUE(Picked(rep));
  EXPECT_EQ(rep.candidate.id(), m.h(2).id());
}

// The default tail model is "id 0" (skipped): the previous attempt's entries are not left behind in the tail.
TEST(LoopCloseSelect, TailOfTheDistanceListIsNotCarriedBetweenAttempts) {
  LoopCloseState st;
  EXPECT_FALSE(st.model_stale_dist_tail);
  {
    SynthMap m1(6);
    m1.SetDistance(5, 1);  // a candidate sitting in the last slot
    m1.MakeNewest(0);
    m1.SetDistance(0, 0);
    for (size_t s = 1; s < 5; ++s) {
      m1.SetDistance(s, 200);
    }
    const LoopCloseReport rep = Attempt1(&m1, &st);
    ASSERT_TRUE(Picked(rep));
    EXPECT_EQ(rep.candidate.id(), m1.h(5).id());
  }
  st.ring.fill(0);  // only the distance storage of the first attempt is carried over, not its ring
  st.cursor = 0;
  {
    SynthMap m2(6);
    m2.MakeNewest(0);
    m2.Free(5);  // one live slot fewer: the list has an unwritten tail entry at position 5
    for (size_t s = 1; s < 5; ++s) {
      m2.SetDistance(s, 200);
    }
    const LoopCloseReport rep = Attempt1(&m2, &st);
    EXPECT_EQ(rep.outcome, LoopCloseOutcome::kNoCandidate);
  }
}

// ---------------------------------------------------------------------------------------------
// G-D / G-E: pairs.tsv, pairs.bin
// ---------------------------------------------------------------------------------------------

struct FrameRec {
  size_t off = 0;
  size_t n_pts = 0;
  size_t n_orb = 0;
  uint32_t words[6] = {};
  int excluded = 0;
};

struct CaseRec {
  std::string id;
  bool self = false;
  std::string new_key;
  std::string cand_key;
  size_t n_matches = 0;
  size_t matches_off = 0;
  bool has_wpi = false;
  std::vector<size_t> wpi;
  int corr = -1;       // expected correspondence count (cases with >= 75 matches), -1 otherwise
  long pnp_off = -1;
};

struct PairData {
  std::vector<uint8_t> bin;
  std::map<std::string, FrameRec> frames;
  std::vector<CaseRec> cases;
};

PairData LoadPairs() {
  PairData d;
  const std::string raw = ReadFile(kDataDir + "pairs.bin");
  d.bin.assign(raw.begin(), raw.end());
  for (const std::string& line : Split(ReadFile(kDataDir + "pairs.tsv"), '\n')) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const std::vector<std::string> f = Split(line, '\t');
    if (f[0] == "F") {
      FrameRec fr;
      fr.off = std::stoul(f[2]);
      fr.n_pts = std::stoul(f[4]);
      fr.n_orb = std::stoul(f[5]);
      const std::vector<std::string> w = Split(f[6], ',');
      for (size_t i = 0; i < 6; ++i) {
        fr.words[i] = i < 2 ? static_cast<uint32_t>(std::stoul(w[i])) : static_cast<uint32_t>(std::stoul(w[i], nullptr, 16));
      }
      fr.excluded = std::stoi(f[7]);
      d.frames[f[1]] = fr;
    } else {
      CaseRec c;
      c.id = f[1];
      c.self = f[2] == "self";
      c.new_key = f[3];
      c.cand_key = f[4];
      c.n_matches = std::stoul(f[5]);
      c.matches_off = std::stoul(f[6]);
      if (f[7] != "-") {
        c.has_wpi = true;
        for (const std::string& t : Split(f[7], ',')) {
          c.wpi.push_back(std::stoul(t));
        }
      }
      c.corr = f[8] == "-" ? -1 : std::stoi(f[8]);
      c.pnp_off = f[9] == "-" ? -1 : std::stol(f[9]);
      d.cases.push_back(std::move(c));
    }
  }
  return d;
}

const PairData& Pairs() {
  static const PairData data = LoadPairs();
  return data;
}

// keyframe frame of the original (500 points, ORB on the recorded ones) with its intrinsics and edge exclusion.
c8::FrameWithPoints BuildGoldenFrame(const FrameRec& fr) {
  c8_PixelPinholeCameraModel k;
  k.pixelsWidth = static_cast<int>(fr.words[0]);
  k.pixelsHeight = static_cast<int>(fr.words[1]);
  k.centerPointX = BitsToFloat(fr.words[2]);
  k.centerPointY = BitsToFloat(fr.words[3]);
  k.focalLengthHorizontal = BitsToFloat(fr.words[4]);
  k.focalLengthVertical = BitsToFloat(fr.words[5]);
  c8::FrameWithPoints f(k);
  const uint8_t* base = Pairs().bin.data() + fr.off;
  size_t next = 0;
  for (size_t p = 0; p < fr.n_pts; ++p) {
    float x = 0.0f;
    float y = 0.0f;
    uint8_t scale = 0;
    c8::FeatureSet set;
    if (next < fr.n_orb && ReadU32(base + 45 * next) == p) {
      x = ReadF32(base + 45 * next + 4);
      y = ReadF32(base + 45 * next + 8);
      scale = base[45 * next + 12];
      Desc d;
      std::memcpy(d.data(), base + 45 * next + 13, 32);
      set = OrbSet(d);
      ++next;
    }
    f.addImageRayPoint(c8::HPoint2(x, y), scale, 0.25f, 0.5f, std::move(set));
  }
  EXPECT_EQ(next, fr.n_orb);
  f.setExcludedEdgePixels(fr.excluded);
  return f;
}

struct PnpBlock {
  uint32_t n = 0;
  uint32_t ret = 0;
  std::vector<c8::HPoint3> wp;
  const uint8_t* pose_in = nullptr;
  const uint8_t* pose_out = nullptr;
  const uint8_t* a = nullptr;
  const uint8_t* b = nullptr;
  const uint8_t* c = nullptr;
};

PnpBlock ReadPnp(const CaseRec& c) {
  PnpBlock p;
  const uint8_t* base = Pairs().bin.data() + c.pnp_off;
  p.n = ReadU32(base);
  p.ret = ReadU32(base + 4);
  for (uint32_t i = 0; i < p.n; ++i) {
    const uint8_t* q = base + 8 + 16 * i;
    p.wp.emplace_back(ReadF32(q), ReadF32(q + 4), ReadF32(q + 8));
  }
  const uint8_t* poses = base + 8 + 16 * p.n;
  p.pose_in = poses;
  p.pose_out = poses + 0x84;
  p.a = poses + 2 * 0x84;
  p.b = poses + 3 * 0x84;
  p.c = poses + 4 * 0x84;
  return p;
}

// Local matcher with the constants of spec 12 S8, restated here (15x20 bins, radius 1.0, Hamming <= 32, ratio test
// on, scale filter off). This tier checks the golden data and the vendored matcher; the engine's own constants are
// checked through LoopClose (num_matches) below.
TEST(LoopClosePairGolden, VendoredMatcherReproducesTheOriginalMatchListsBitForBit) {
  const PairData& d = Pairs();
  ASSERT_EQ(d.cases.size(), 69u);
  size_t compared = 0;
  size_t listed = 0;
  size_t max_dist_32 = 0;
  for (const CaseRec& c : d.cases) {
    const c8::FrameWithPoints cand = BuildGoldenFrame(d.frames.at(c.cand_key));
    const c8::FrameWithPoints newest = BuildGoldenFrame(d.frames.at(c.new_key));
    c8::LocalMatcher<c8::OrbFeature> matcher(15, 20, 1.0f, 32.0f);
    matcher.useRatioTest(true);
    matcher.useScaleFilter(false);
    matcher.setQueryPointsPointer(cand);
    c8::Vector<c8::PointMatch> matches;
    c8::MatchResultFrequency freq;
    matcher.findMatches(newest.points(), newest.store(), &matches, &freq);
    ASSERT_EQ(matches.size(), c.n_matches) << c.id;
    const uint8_t* g = d.bin.data() + c.matches_off;
    for (size_t i = 0; i < matches.size(); ++i) {
      const uint8_t* e = g + 20 * i;
      EXPECT_EQ(matches[i].wordsIdx, ReadU32(e)) << c.id << " match " << i;
      EXPECT_EQ(matches[i].dictionaryIdx, ReadU32(e + 4)) << c.id << " match " << i;
      EXPECT_EQ(BitsOf(matches[i].rotation), ReadU32(e + 8)) << c.id << " match " << i;
      EXPECT_EQ(BitsOf(matches[i].descriptorDist), ReadU32(e + 12)) << c.id << " match " << i;
      EXPECT_EQ(matches[i].dictionaryScale, e[16]) << c.id << " match " << i;
      max_dist_32 += matches[i].descriptorDist == 32.0f ? 1 : 0;
      ++listed;
    }
    ++compared;
  }
  EXPECT_EQ(compared, 69u);
  EXPECT_GT(listed, 1000u);
  EXPECT_GT(max_dist_32, 0u);  // the Hamming limit 32 itself is exercised by the recorded matches
}

// A pair world: slot 0 (or the freed slot 0 for a self case) ... newest in slot 4, descriptors such that the search
// picks the case's candidate (distance 0); the other slots are 256 bits away.
struct PairWorld {
  std::unique_ptr<map::WorldMaps> maps;
  map::KeyframeId newest;
  map::KeyframeId cand;
  size_t unlinked_world_points = 0;
};

PairWorld BuildPairWorld(const CaseRec& c, bool with_world_points) {
  const PairData& d = Pairs();
  PairWorld w;
  w.maps = std::make_unique<map::WorldMaps>();
  map::WorldMap& wm = w.maps->active();
  const c8::FrameWithPoints newest = BuildGoldenFrame(d.frames.at(c.new_key));
  std::vector<map::KeyframeId> h;
  if (c.self) {
    h.push_back(wm.AddKeyframe(PlainFrame(0), 0));  // freed below: shifts the compressed positions
  } else {
    const c8::FrameWithPoints cand = BuildGoldenFrame(d.frames.at(c.cand_key));
    h.push_back(wm.AddKeyframe(cand, 0));
  }
  for (int i = 1; i <= 3; ++i) {
    h.push_back(wm.AddKeyframe(PlainFrame(0), 5 * i));
  }
  h.push_back(wm.AddKeyframe(newest, 20));
  for (size_t s = 0; s < 5; ++s) {
    wm.MutableKeyframeAtSlot(s).global_descriptor = (s == 0 && !c.self) || s == 4 ? Desc{} : FilledDesc(0xFF);
  }
  if (c.self) {
    wm.RemoveKeyframe(h[0]);
  }
  w.newest = h[4];
  w.cand = c.self ? h[4] : h[0];
  if (with_world_points && c.has_wpi) {
    std::map<size_t, c8::HPoint3> pos;
    if (c.pnp_off >= 0) {
      const PnpBlock p = ReadPnp(c);
      wm.SetKeyframePose(h[4], HMatrixFromRecord(p.pose_in));
      const uint8_t* g = d.bin.data() + c.matches_off;
      const std::set<size_t> in_wpi(c.wpi.begin(), c.wpi.end());
      size_t j = 0;
      for (size_t i = 0; i < c.n_matches; ++i) {
        const size_t q = ReadU32(g + 20 * i + 4);
        if (in_wpi.count(q) != 0) {
          pos[q] = p.wp[j++];
        }
      }
      EXPECT_EQ(j, p.n);
    }
    for (size_t q : c.wpi) {
      const c8::HPoint3 position = pos.count(q) != 0 ? pos[q] : c8::HPoint3(0.f, 0.f, 3.f);
      const map::PointId pid = wm.AddPoint(wm.GetKeyframe(h[4]).seq, position, map::PointSource::kTriangulated);
      wm.AddObservation(pid, static_cast<uint32_t>(q), h[4]);
    }
  }
  return w;
}

LoopCloseOutcome ExpectedOutcome(const CaseRec& c, const PnpBlock* pnp) {
  if (c.n_matches < 75) {
    return LoopCloseOutcome::kTooFewMatches;
  }
  if (c.corr < 35) {
    return LoopCloseOutcome::kTooFewCorrespondences;
  }
  return pnp != nullptr && pnp->ret == 0 ? LoopCloseOutcome::kPoseEstimationFailed : LoopCloseOutcome::kApplied;
}

// Through LoopClose (candidate search + local match + thresholds) every recorded call ends where the original's did:
// the same match count, the same correspondence count, and the pose stage only when the original ran it.
TEST(LoopClosePairGolden, EngineLocalMatchCountsAndStageOutcomesFollowTheOriginal) {
  const PairData& d = Pairs();
  size_t below_75 = 0;
  size_t s11_stop = 0;
  size_t pose_stage = 0;
  for (const CaseRec& c : d.cases) {
    PairWorld w = BuildPairWorld(c, true);
    LoopCloseState st;
    MapBuilderOutput out;
    LoopCloseReport rep;
    LoopClose(w.maps->active(), &st, &out, &rep);
    ASSERT_NE(rep.outcome, LoopCloseOutcome::kNoCandidate) << c.id;
    EXPECT_TRUE(rep.candidate == w.cand) << c.id;
    EXPECT_EQ(rep.num_matches, c.n_matches) << c.id;
    PnpBlock pnp;
    const bool has_pnp = c.pnp_off >= 0;
    if (has_pnp) {
      pnp = ReadPnp(c);
    }
    const LoopCloseOutcome want = ExpectedOutcome(c, has_pnp ? &pnp : nullptr);
    if (c.n_matches < 75) {
      ++below_75;
    } else if (c.corr < 35) {
      ++s11_stop;
      EXPECT_EQ(rep.num_correspondences, static_cast<size_t>(c.corr)) << c.id;
    } else {
      ++pose_stage;
      EXPECT_EQ(rep.num_correspondences, static_cast<size_t>(c.corr)) << c.id;
    }
    EXPECT_EQ(rep.outcome, want) << c.id << " matches " << c.n_matches << " corr " << c.corr;
  }
  EXPECT_EQ(below_75, 53u);
  EXPECT_EQ(s11_stop, 8u);   // >= 75 matches but < 35 correspondences: the original never called 1146
  EXPECT_EQ(pose_stage, 8u);
}

// 74 matches is the original's own near miss (hkd-5 seq 134, a self call); it stops at the match threshold.
TEST(LoopClosePairGolden, SeventyFourMatchesStopAtTheMatchThreshold) {
  bool found = false;
  for (const CaseRec& c : Pairs().cases) {
    if (c.id != "hkd-5:134") {
      continue;
    }
    found = true;
    EXPECT_EQ(c.n_matches, 74u);
    PairWorld w = BuildPairWorld(c, true);
    LoopCloseState st;
    MapBuilderOutput out;
    LoopCloseReport rep;
    EXPECT_FALSE(LoopClose(w.maps->active(), &st, &out, &rep));
    EXPECT_EQ(rep.outcome, LoopCloseOutcome::kTooFewMatches);
    EXPECT_EQ(rep.num_matches, 74u);
  }
  EXPECT_TRUE(found);
}

// G-E: the PnP of the pose stage on the original's recorded input. Pre-declared tolerance: 3 ulp per float (spec 12 §8).
TEST(LoopClosePairGolden, PoseEstimationStaysWithinThreeUlpOfTheOriginal) {
  const PairData& d = Pairs();
  size_t applied = 0;
  size_t failed_like_original = 0;
  int64_t worst = 0;
  for (const CaseRec& c : d.cases) {
    if (c.pnp_off < 0) {
      continue;
    }
    const PnpBlock p = ReadPnp(c);
    PairWorld w = BuildPairWorld(c, true);
    LoopCloseState st;
    MapBuilderOutput out;
    LoopCloseReport rep;
    const bool ok = LoopClose(w.maps->active(), &st, &out, &rep);
    EXPECT_EQ(rep.num_correspondences, p.n) << c.id;
    if (p.ret == 0) {
      EXPECT_FALSE(ok) << c.id << ": the original's 1146 returned false";
      EXPECT_EQ(rep.outcome, LoopCloseOutcome::kPoseEstimationFailed) << c.id;
      failed_like_original += rep.outcome == LoopCloseOutcome::kPoseEstimationFailed ? 1 : 0;
      continue;
    }
    ASSERT_TRUE(ok) << c.id;
    ++applied;
    const c8::HMatrix want = HMatrixFromRecord(p.pose_out);
    const c8::HMatrix& got = w.maps->active().GetKeyframe(w.newest).frame.extrinsic();
    int64_t case_worst = 0;
    for (size_t i = 0; i < 16; ++i) {
      case_worst = std::max(case_worst, UlpDistance(got.data()[i], want.data()[i]));
      case_worst = std::max(case_worst, UlpDistance(got.inverseData()[i], want.inverseData()[i]));
    }
    worst = std::max(worst, case_worst);
    EXPECT_LE(case_worst, 3) << c.id << ": engine pose is " << case_worst << " ulp from the original's 1146 output";
  }
  EXPECT_EQ(applied, 7u);
  EXPECT_EQ(failed_like_original, 1u);
  std::printf("[loop_close] G-E worst pose distance %lld ulp over %zu PnP results\n", static_cast<long long>(worst),
              applied);
}

// ---------------------------------------------------------------------------------------------
// G-F: correction matrix
// ---------------------------------------------------------------------------------------------

TEST(LoopCloseCorrectionGolden, VendoredProductReproducesTheOriginalsCorrectionBitForBit) {
  size_t records = 0;
  for (const CaseRec& c : Pairs().cases) {
    if (c.pnp_off < 0) {
      continue;
    }
    const PnpBlock p = ReadPnp(c);
    if (p.ret == 0) {
      continue;
    }
    ++records;
    const c8::HMatrix a = HMatrixFromRecord(p.a);
    const c8::HMatrix b = HMatrixFromRecord(p.b);
    const c8::HMatrix c660 = a.inv() * b;  // 660: out = A.inv * B
    // forward and inverse blocks of the recorded C (the two flag bytes after them are not compared)
    for (size_t i = 0; i < 16; ++i) {
      EXPECT_EQ(BitsOf(c660.data()[i]), ReadU32(p.c + 4 * i)) << c.id << " forward " << i;
      EXPECT_EQ(BitsOf(c660.inverseData()[i]), ReadU32(p.c + 64 + 4 * i)) << c.id << " inverse " << i;
    }
    // the same, restated from the summation order
    const std::array<float, 16> fwd = MulUnrolled(a.inverseData(), b.data());
    const std::array<float, 16> inv = MulUnrolled(b.inverseData(), a.data());
    for (size_t i = 0; i < 16; ++i) {
      EXPECT_EQ(BitsOf(fwd[i]), ReadU32(p.c + 4 * i)) << c.id << " forward " << i;
      EXPECT_EQ(BitsOf(inv[i]), ReadU32(p.c + 64 + 4 * i)) << c.id << " inverse " << i;
    }
    // the order spec 12 first read from func 789 does not give the recorded matrix (0/7 in the dump)
    const std::array<float, 16> f789 = Mul789(a.inverseData(), b.data());
    const std::array<float, 16> i789 = Mul789(b.inverseData(), a.data());
    int differs = 0;
    for (size_t i = 0; i < 16; ++i) {
      differs += BitsOf(f789[i]) != ReadU32(p.c + 4 * i) ? 1 : 0;
      differs += BitsOf(i789[i]) != ReadU32(p.c + 64 + 4 * i) ? 1 : 0;
    }
    EXPECT_GT(differs, 0) << c.id << ": the 789 order would also match; this record cannot tell them apart";
  }
  EXPECT_EQ(records, 7u);
}

// Scene of the live-run tests: the candidate keyframe sees `points` world points, the newest keyframe sees the same
// appearances (identical descriptors) from a nearby pose, and its stored pose is a general rotation (so the
// summation order of the correction product matters).
struct Revisit {
  map::WorldMaps maps;
  map::KeyframeId first;
  map::KeyframeId newest;
  std::vector<c8::HPoint3> truth;
  std::vector<map::PointId> points;
  c8::HMatrix pose_first = Pose(0.0f, 0.0f, 0.0f, 0.0f);
  c8::HMatrix pose_true = Pose(0.01f, 0.03f, 0.005f, 0.02f);
  c8::HMatrix pose_stored = c8::HMatrixGen::translation(0.04f, -0.01f, 0.03f) * c8::HMatrixGen::xRadians(0.31f) *
                            c8::HMatrixGen::yRadians(-0.52f) * c8::HMatrixGen::zRadians(0.17f);
  int n_points = 0;

  // matching: how many appearances carry equal descriptors in both keyframes; flip_bits: Hamming distance between
  // the two descriptors of a matching appearance; linked: candidate observations 0..linked-1 own a world point
  // (sources: `sources` cycles over them; empty = all triangulated).
  Revisit(int n, int matching, int linked, int flip_bits = 0,
          std::vector<map::PointSource> sources = {}) {
    n_points = n;
    map::WorldMap& wm = maps.active();
    c8::FrameWithPoints fa(TestIntrinsics());
    c8::FrameWithPoints fb(TestIntrinsics());
    for (int i = 0; i < n; ++i) {
      const c8::HPoint3 x(-1.2f + 0.03f * static_cast<float>(i), -0.5f + 0.03f * static_cast<float>((i * 7) % 30),
                          2.0f + 0.06f * static_cast<float>((i * 11) % 30));
      truth.push_back(x);
      const c8::HPoint2 ra = Project(pose_first, x);
      const c8::HPoint2 rb = Project(pose_true, x);
      Desc da = testing::Descriptor(static_cast<uint32_t>(100 + i));
      Desc db = da;
      if (i < matching) {
        for (int b = 0; b < flip_bits; ++b) {
          db[static_cast<size_t>(b / 8)] = static_cast<uint8_t>(db[static_cast<size_t>(b / 8)] ^ (1u << (b % 8)));
        }
      } else {
        db = testing::Descriptor(static_cast<uint32_t>(9000 + i));
      }
      fa.addImageRayPoint(ra, 1, 0.25f, 0.5f, OrbSet(da));
      fb.addImageRayPoint(rb, 1, 0.25f, 0.5f, OrbSet(db));
    }
    first = wm.AddKeyframe(fa, 0);
    wm.SetKeyframePose(first, pose_first);
    for (int i = 0; i < linked; ++i) {
      const map::PointSource src = sources.empty() ? map::PointSource::kTriangulated
                                                   : sources[static_cast<size_t>(i) % sources.size()];
      const map::PointId p = wm.AddPoint(wm.GetKeyframe(first).seq, truth[static_cast<size_t>(i)], src);
      wm.AddObservation(p, static_cast<uint32_t>(i), first);
      points.push_back(p);
    }
    for (int k = 1; k <= 4; ++k) {
      c8::FrameWithPoints ff(TestIntrinsics());
      for (int i = 0; i < 40; ++i) {
        ff.addImageRayPoint(c8::HPoint2(-0.5f + 0.025f * static_cast<float>(i), 0.1f * static_cast<float>(k) - 0.2f), 1,
                            0.25f, 0.5f, OrbSet(testing::Descriptor(static_cast<uint32_t>(1000 * k + i))));
      }
      const map::KeyframeId kf = wm.AddKeyframe(ff, 5 * k);
      wm.SetKeyframePose(kf, Pose(0.0f, 0.1f * static_cast<float>(k), 0.f, 0.f));
      wm.MutableKeyframe(kf).global_descriptor.fill(0xFF);
    }
    newest = wm.AddKeyframe(fb, 30);
    wm.SetKeyframePose(newest, pose_stored);
  }

  map::WorldMap& wm() { return maps.active(); }
};

struct RevisitRun {
  bool applied = false;
  LoopCloseReport report;
  MapBuilderOutput out;
  c8::HMatrix old_pose = c8::HMatrixGen::i();
};

RevisitRun RunRevisit(Revisit* r) {
  RevisitRun run;
  run.old_pose = r->wm().GetKeyframe(r->newest).frame.extrinsic();
  LoopCloseState st;
  run.applied = LoopClose(r->wm(), &st, &run.out, &run.report);
  return run;
}

// Live run: C is bit-identical to the original's product A.inv * B of the engine's own old and new pose.
TEST(LoopCloseCorrectionGolden, LiveCorrectionIsTheOriginalProductOfTheOldInverseAndTheNewPose) {
  Revisit r(80, 80, 80);
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  const c8::HMatrix& moved = r.wm().GetKeyframe(r.newest).frame.extrinsic();
  const std::array<float, 16> fwd = MulUnrolled(run.old_pose.inverseData(), moved.data());
  const std::array<float, 16> inv = MulUnrolled(moved.inverseData(), run.old_pose.data());
  EXPECT_EQ(DifferingWords(fwd, run.out.world_correction.data()), 0);
  EXPECT_EQ(DifferingWords(inv, run.out.world_correction.inverseData()), 0);
  // The scene separates the two summation orders (otherwise this test could not detect the 789 order).
  EXPECT_GT(DifferingWords(Mul789(run.old_pose.inverseData(), moved.data()), fwd) +
                DifferingWords(Mul789(moved.inverseData(), run.old_pose.data()), inv),
            0);
}

// S11: the pose stage solves poseEstimationAnalytical from the CANDIDATE keyframe's pose with the newest keyframe's
// rays and the candidate's world points, parameters {suppressOutliers true, scale 1.0, image target false, camera
// motion weight 0.0, log false} (spec 12 s4), and adopts the result unchecked. Recomputed here independently.
TEST(LoopCloseCorrectionGolden, PoseIsSolvedFromTheCandidatePoseWithTheSpecParameters) {
  Revisit r(80, 80, 80);
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  c8::Vector<c8::ObservedPoint> rays;
  c8::Vector<c8::HPoint3> world_points;
  for (int i = 0; i < 80; ++i) {
    rays.push_back(c8::ObservedPoint{Project(r.pose_true, r.truth[static_cast<size_t>(i)]), 1, 0.0f, 1.0f});
    world_points.push_back(r.truth[static_cast<size_t>(i)]);
  }
  c8::PoseEstimationAnalyticalParams params;
  params.suppressOutliers = true;
  params.scale = 1.0;
  params.useImageTargetCostFunction = false;
  params.cameraMotionWeight = 0.0;
  params.logReport = false;
  c8::HMatrix expected = r.pose_first;  // the candidate keyframe's pose
  const std::deque<c8::HMatrix> last_two;
  ASSERT_TRUE(c8::poseEstimationAnalytical(rays, world_points, last_two, &expected, params, nullptr));
  const c8::HMatrix& moved = r.wm().GetKeyframe(r.newest).frame.extrinsic();
  for (size_t i = 0; i < 16; ++i) {
    EXPECT_EQ(BitsOf(moved.data()[i]), BitsOf(expected.data()[i])) << i;
    EXPECT_EQ(BitsOf(moved.inverseData()[i]), BitsOf(expected.inverseData()[i])) << i;
  }
  // the start pose matters: solving from the newest keyframe's stored pose gives a different answer here
  c8::HMatrix other = r.pose_stored;
  const bool other_ok = c8::poseEstimationAnalytical(rays, world_points, last_two, &other, params, nullptr);
  bool differs = !other_ok;
  for (size_t i = 0; i < 16 && other_ok; ++i) {
    differs = differs || BitsOf(other.data()[i]) != BitsOf(expected.data()[i]);
  }
  EXPECT_TRUE(differs);
}

// C = P_old^-1 * P_new: P_old * C is the new pose, and the reversed product is not (the poses do not commute).
TEST(LoopCloseCorrectionGolden, CorrectionMapsTheOldPoseOntoTheNewOneByRightMultiplication) {
  Revisit r(80, 80, 80);
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  const c8::HMatrix& moved = r.wm().GetKeyframe(r.newest).frame.extrinsic();
  const c8::HMatrix right = run.old_pose * run.out.world_correction;
  const c8::HMatrix left = run.out.world_correction * run.old_pose;
  float max_right = 0.0f;
  float max_left = 0.0f;
  for (size_t i = 0; i < 16; ++i) {
    max_right = std::max(max_right, std::fabs(right.data()[i] - moved.data()[i]));
    max_left = std::max(max_left, std::fabs(left.data()[i] - moved.data()[i]));
  }
  EXPECT_LT(max_right, 1e-5f);
  EXPECT_GT(max_left, 1e-2f);
  EXPECT_TRUE(run.out.world_transform_updated);
}

// The correction uses the pose the keyframe had BEFORE the update (660 runs before 655).
TEST(LoopCloseCorrectionGolden, CorrectionIsComputedFromThePoseBeforeTheKeyframeIsMoved) {
  Revisit r(80, 80, 80);
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  const c8::HMatrix& moved = r.wm().GetKeyframe(r.newest).frame.extrinsic();
  const c8::HMatrix wrong = moved.inv() * moved;  // what a correction from the already moved pose would be: identity
  float diff_to_identity = 0.0f;
  for (size_t i = 0; i < 16; ++i) {
    diff_to_identity = std::max(diff_to_identity, std::fabs(run.out.world_correction.data()[i] - wrong.data()[i]));
  }
  EXPECT_GT(diff_to_identity, 1e-2f);
}

// G-G (the part of the original that can be compared: the self-candidate calls up to the unlink loop): the recorded
// successful calls merge and relink nothing (eb == db for every match) and move no point; engine-defined from there.
TEST(LoopCloseCorrectionGolden, SelfCandidateSuccessLeavesTheTriangulatedPointsUntouched) {
  size_t runs = 0;
  for (const CaseRec& c : Pairs().cases) {
    if (c.pnp_off < 0 || ReadPnp(c).ret == 0) {
      continue;
    }
    PairWorld w = BuildPairWorld(c, true);
    map::WorldMap& wm = w.maps->active();
    const std::array<int, 5> before = wm.NumPointsBySource();
    std::vector<map::PointId> ids_before;
    for (const c8::FramePoint& fp : wm.GetKeyframe(w.newest).frame.points()) {
      ids_before.push_back(fp.id());
    }
    LoopCloseState st;
    MapBuilderOutput out;
    LoopCloseReport rep;
    ASSERT_TRUE(LoopClose(wm, &st, &out, &rep)) << c.id;
    EXPECT_EQ(wm.NumPointsBySource(), before) << c.id;
    EXPECT_EQ(rep.num_unlinked, 0u) << c.id;  // every linked point is TRIANGULATED here
    size_t i = 0;
    for (const c8::FramePoint& fp : wm.GetKeyframe(w.newest).frame.points()) {
      EXPECT_TRUE(fp.id() == ids_before[i++]) << c.id;
    }
    ++runs;
  }
  EXPECT_EQ(runs, 7u);
}

// ---------------------------------------------------------------------------------------------
// boundaries of the pose stage (spec 12 S9, S11) and the matcher constants
// ---------------------------------------------------------------------------------------------

TEST(LoopCloseThresholds, SeventyFourMatchesFailAndSeventyFiveContinue) {
  {
    Revisit r(80, 74, 80);
    const RevisitRun run = RunRevisit(&r);
    EXPECT_FALSE(run.applied);
    EXPECT_EQ(run.report.outcome, LoopCloseOutcome::kTooFewMatches);
    EXPECT_EQ(run.report.num_matches, 74u);
  }
  {
    Revisit r(80, 75, 80);
    const RevisitRun run = RunRevisit(&r);
    EXPECT_EQ(run.report.num_matches, 75u);
    EXPECT_EQ(run.report.num_correspondences, 75u);
    EXPECT_TRUE(run.applied);
  }
}

TEST(LoopCloseThresholds, ThirtyFourCorrespondencesFailAndThirtyFiveRunThePnP) {
  {
    Revisit r(80, 80, 34);
    const RevisitRun run = RunRevisit(&r);
    EXPECT_FALSE(run.applied);
    EXPECT_EQ(run.report.outcome, LoopCloseOutcome::kTooFewCorrespondences);
    EXPECT_EQ(run.report.num_matches, 80u);
    EXPECT_EQ(run.report.num_correspondences, 34u);
  }
  {
    Revisit r(80, 80, 35);
    const RevisitRun run = RunRevisit(&r);
    EXPECT_EQ(run.report.num_correspondences, 35u);
    EXPECT_TRUE(run.applied);
  }
}

TEST(LoopCloseThresholds, MatchAndCorrespondenceLimitsComeFromTheConfigKeys) {
  {
    const testing::ScopedParamOverrides overrides({{"WorldMaps.loopClosingMatchesRequired", ParamValue(76)}});
    Revisit r(80, 75, 80);
    EXPECT_EQ(RunRevisit(&r).report.outcome, LoopCloseOutcome::kTooFewMatches);
  }
  {
    const testing::ScopedParamOverrides overrides(
        {{"WorldMaps.loopClosingPoseEstimationPointsRequired", ParamValue(40)}});
    Revisit r(80, 80, 39);
    EXPECT_EQ(RunRevisit(&r).report.outcome, LoopCloseOutcome::kTooFewCorrespondences);
    Revisit r2(80, 80, 40);
    EXPECT_TRUE(RunRevisit(&r2).applied);
  }
}

// Local match Hamming limit: 32 bits apart still match, 33 do not (spec 12 S8).
TEST(LoopCloseThresholds, LocalMatchAcceptsThirtyTwoBitsAndRejectsThirtyThree) {
  {
    Revisit r(80, 80, 80, 32);
    const RevisitRun run = RunRevisit(&r);
    EXPECT_EQ(run.report.num_matches, 80u);
    EXPECT_TRUE(run.applied);
  }
  {
    Revisit r(80, 80, 80, 33);
    const RevisitRun run = RunRevisit(&r);
    EXPECT_EQ(run.report.num_matches, 0u);
    EXPECT_EQ(run.report.outcome, LoopCloseOutcome::kTooFewMatches);
  }
}

// ---------------------------------------------------------------------------------------------
// post-processing (S12-c, S12-d): engine-defined behaviour, deviation N2
// ---------------------------------------------------------------------------------------------

// A candidate point and a different newest point on the same appearance: the newest point is merged into the
// candidate's (1124 eb -> db), so the newest observation ends up on the candidate's point and eb is retired.
TEST(LoopClosePostProcessing, DifferentPointsOnAMatchAreMergedIntoTheCandidatesPoint) {
  Revisit r(80, 80, 80);
  map::WorldMap& wm = r.wm();
  std::vector<map::PointId> extra;
  for (int i = 0; i < 10; ++i) {
    const map::PointId e = wm.AddPoint(wm.GetKeyframe(r.newest).seq, r.truth[static_cast<size_t>(i)],
                                       map::PointSource::kTriangulated);
    wm.AddObservation(e, static_cast<uint32_t>(i), r.newest);
    extra.push_back(e);
  }
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  for (int i = 0; i < 10; ++i) {
    const size_t k = static_cast<size_t>(i);
    EXPECT_EQ(wm.Point(extra[k]).state, map::PointState::kDead) << i;
    EXPECT_TRUE(wm.GetKeyframe(r.newest).frame.points()[k].id() == r.points[k]) << i;
  }
  EXPECT_EQ(run.report.num_unlinked, 0u);
}

// Candidate observation linked, newest observation not: the newest observation is attached to the candidate's point.
TEST(LoopClosePostProcessing, UnlinkedNewestObservationIsAttachedToTheCandidatesPoint) {
  Revisit r(80, 80, 80);
  ASSERT_EQ(r.wm().GetKeyframe(r.newest).frame.worldPointIndices().size(), 0u);
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  const map::Keyframe& kn = r.wm().GetKeyframe(r.newest);
  EXPECT_EQ(kn.frame.worldPointIndices().size(), 80u);
  for (size_t i = 0; i < 80; ++i) {
    EXPECT_TRUE(kn.frame.points()[i].id() == r.points[i]) << i;
  }
}

// Candidate observation not linked, newest observation linked: the newest keyframe's point is attached to the
// candidate's observation (the else-if branch of the match loop).
TEST(LoopClosePostProcessing, UnlinkedCandidateObservationReceivesTheNewestPoint) {
  Revisit r(80, 80, 50);  // candidate observations 50..79 have no world point
  map::WorldMap& wm = r.wm();
  std::vector<map::PointId> own;
  for (int i = 50; i < 60; ++i) {
    const map::PointId e = wm.AddPoint(wm.GetKeyframe(r.newest).seq, r.truth[static_cast<size_t>(i)],
                                       map::PointSource::kTriangulated);
    wm.AddObservation(e, static_cast<uint32_t>(i), r.newest);
    own.push_back(e);
  }
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  EXPECT_EQ(run.report.num_correspondences, 50u);
  const map::Keyframe& kc = wm.GetKeyframe(r.first);
  for (int i = 50; i < 60; ++i) {
    EXPECT_TRUE(kc.frame.points()[static_cast<size_t>(i)].id() == own[static_cast<size_t>(i - 50)]) << i;
  }
  for (int i = 60; i < 80; ++i) {
    EXPECT_TRUE(kc.frame.points()[static_cast<size_t>(i)].id() == map::PointId::nullId()) << i;
  }
}

// The same point on both sides of a match is left alone. (The candidate must not be covisible with the newest
// keyframe, so only 9 points are shared: one fewer than requiredPointsForCovisibility.)
TEST(LoopClosePostProcessing, SamePointOnBothSidesChangesNothing) {
  Revisit r(80, 80, 80);
  map::WorldMap& wm = r.wm();
  for (int i = 0; i < 9; ++i) {
    wm.AddObservation(r.points[static_cast<size_t>(i)], static_cast<uint32_t>(i), r.newest);
  }
  const std::array<int, 5> before = wm.NumPointsBySource();
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied) << static_cast<int>(run.report.outcome) << " matches " << run.report.num_matches
                           << " corr " << run.report.num_correspondences;
  EXPECT_EQ(wm.NumPointsBySource(), before);
  for (int i = 0; i < 80; ++i) {
    const size_t k = static_cast<size_t>(i);
    EXPECT_NE(wm.Point(r.points[k]).state, map::PointState::kDead) << i;
    EXPECT_TRUE(wm.GetKeyframe(r.newest).frame.points()[k].id() == r.points[k]) << i;
  }
  EXPECT_EQ(run.report.num_unlinked, 0u);
}

// S12-d: after the matches are processed, every observation of the newest keyframe that is not on a TRIANGULATED
// point is unlinked (once each); TRIANGULATED links stay.
TEST(LoopClosePostProcessing, NonTriangulatedLinksOfTheNewestKeyframeAreRemoved) {
  const std::vector<map::PointSource> sources = {map::PointSource::kGround, map::PointSource::kTriangulated,
                                                 map::PointSource::kTriangulated, map::PointSource::kSky};
  Revisit r(80, 80, 80, 0, sources);
  const RevisitRun run = RunRevisit(&r);
  ASSERT_TRUE(run.applied);
  map::WorldMap& wm = r.wm();
  const map::Keyframe& kn = wm.GetKeyframe(r.newest);
  size_t linked = 0;
  for (size_t i = 0; i < 80; ++i) {
    const bool triangulated = sources[i % 4] == map::PointSource::kTriangulated;
    EXPECT_EQ(kn.frame.points()[i].id() == r.points[i], triangulated) << i;
    EXPECT_EQ(kn.frame.points()[i].id() == map::PointId::nullId(), !triangulated) << i;
    linked += triangulated ? 1 : 0;
  }
  EXPECT_EQ(kn.frame.worldPointIndices().size(), linked);
  EXPECT_EQ(run.report.num_unlinked, 40u);
  // the points themselves and the candidate's links survive
  const map::Keyframe& kc = wm.GetKeyframe(r.first);
  EXPECT_EQ(kc.frame.worldPointIndices().size(), 80u);
  for (size_t i = 0; i < 80; ++i) {
    EXPECT_NE(wm.Point(r.points[i]).state, map::PointState::kDead) << i;
  }
}

// ---------------------------------------------------------------------------------------------
// OFF: loop close disabled
// ---------------------------------------------------------------------------------------------

constexpr int kGroundRays = 24;
constexpr int kSkyRays = 40;

std::unique_ptr<TrackerInput> MakeInput(int frame) {
  auto in = std::make_unique<TrackerInput>(TestIntrinsics(), c8::Quaternion(1.f, 0.f, 0.f, 0.f),
                                           static_cast<int64_t>(frame) * 33333333);
  std::vector<testing::RayFeature> rays;
  for (int i = 0; i < kGroundRays; ++i) {
    rays.push_back({-0.4f + 0.035f * static_cast<float>(i), -0.1f - 0.01f * static_cast<float>(i % 7), 1,
                    static_cast<uint32_t>(frame * 1000 + i)});
  }
  for (int i = 0; i < kSkyRays; ++i) {
    rays.push_back({-0.4f + 0.02f * static_cast<float>(i), 0.2f + 0.003f * static_cast<float>(i), 1,
                    static_cast<uint32_t>(frame * 1000 + 500 + i)});
  }
  testing::AddRays(&in->frame_points, rays);
  return in;
}

MapTrackerOutput MakeTrk(int32_t state, int64_t consecutive, const c8::HMatrix& pose) {
  MapTrackerOutput trk;
  trk.pose = pose;
  trk.state = state;
  trk.frame_tracker_ok = true;
  trk.consecutive_frame_tracker_ok = consecutive;
  for (uint32_t i = 0; i < kGroundRays; ++i) {
    trk.ground_idx.push_back(i);
  }
  for (uint32_t i = 0; i < kSkyRays; ++i) {
    trk.sky_idx.push_back(kGroundRays + i);
  }
  return trk;
}

struct SequenceResult {
  std::vector<uint32_t> keyframe_ids;  // all slots, free ones as 0
  std::vector<bool> keyframe_created;
  size_t nonzero_descriptors = 0;
  size_t corrections = 0;
  std::vector<bool> world_transform_updated;
};

SequenceResult RunSequence(int frames, bool loop_close) {
  map::WorldMaps maps;
  MapBuilder mb;
  SequenceResult res;
  for (int f = 0; f < frames; ++f) {
    const c8::HMatrix pose = Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f);
    MapBuilderOutput out;
    if (loop_close) {
      const testing::ScopedParamOverrides overrides({{"MapBuilder.loopCloseEnabled", ParamValue(true)}});
      mb.Update(&maps, *MakeInput(f), MakeTrk(f == 0 ? 0 : 1, 7 + f, pose), f * 33, &out);
    } else {
      mb.Update(&maps, *MakeInput(f), MakeTrk(f == 0 ? 0 : 1, 7 + f, pose), f * 33, &out);
    }
    res.keyframe_created.push_back(out.keyframe_created);
    res.world_transform_updated.push_back(out.world_transform_updated);
    res.corrections += out.world_transform_updated ? 1 : 0;
  }
  const std::array<uint8_t, 32> zero{};
  for (size_t s = 0; s < maps.active().NumKeyframeSlots(); ++s) {
    const map::Keyframe& kf = maps.active().KeyframeAtSlot(s);
    res.keyframe_ids.push_back(kf.id);
    res.nonzero_descriptors += kf.global_descriptor != zero ? 1 : 0;
  }
  return res;
}

// OFF (the default): no descriptor is computed, no correction is reported.
TEST(LoopCloseOff, DisabledRunComputesNoDescriptorAndReportsNoCorrection) {
  const SequenceResult off = RunSequence(80, false);
  EXPECT_GE(off.keyframe_ids.size(), 5u);
  EXPECT_EQ(off.nonzero_descriptors, 0u);
  EXPECT_EQ(off.corrections, 0u);
}

// ON consumes no keyframe/point ids and changes no keyframe decision: the id generator draws stay aligned with the
// OFF run (the descriptor uses a local mt19937(0) only).
TEST(LoopCloseOff, EnabledRunConsumesNoIdsAndKeepsTheKeyframeDecisions) {
  const SequenceResult off = RunSequence(80, false);
  const SequenceResult on = RunSequence(80, true);
  EXPECT_EQ(off.keyframe_ids, on.keyframe_ids);
  EXPECT_EQ(off.keyframe_created, on.keyframe_created);
  EXPECT_GT(on.nonzero_descriptors, 0u);  // the ON run did compute descriptors, the OFF run did not
}

// D1: with loop close on, every keyframe stores the agate descriptor of its own ORB descriptors (descriptor order),
// computed right after it is created (func 654 -> 1263).
TEST(LoopCloseOff, EnabledRunStoresTheAgateDescriptorOfEveryKeyframesOrbs) {
  map::WorldMaps maps;
  MapBuilder mb;
  const AgateDescriptor agate;
  const testing::ScopedParamOverrides overrides({{"MapBuilder.loopCloseEnabled", ParamValue(true)}});
  for (int f = 0; f < 60; ++f) {
    MapBuilderOutput out;
    mb.Update(&maps, *MakeInput(f), MakeTrk(f == 0 ? 0 : 1, 7 + f, Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f)),
              f * 33, &out);
  }
  size_t checked = 0;
  for (size_t s = 0; s < maps.active().NumKeyframeSlots(); ++s) {
    const map::Keyframe& kf = maps.active().KeyframeAtSlot(s);
    if (kf.id == 0) {
      continue;
    }
    const auto& orbs = kf.frame.store().getFeatures<c8::OrbFeature>();
    ASSERT_GT(orbs.size(), 0u);
    std::vector<uint8_t> bytes(orbs.size() * 32);
    for (size_t i = 0; i < orbs.size(); ++i) {
      std::memcpy(bytes.data() + 32 * i, orbs[i].data(), 32);
    }
    EXPECT_EQ(agate.Extract(bytes.data(), orbs.size()), kf.global_descriptor) << "slot " << s;
    ++checked;
  }
  EXPECT_GE(checked, 5u);
}

}  // namespace
}  // namespace slam_refact::map_builder
