// MB-TRI and MB-EPI (architecture.md §6.6.3, S3 §8.1, §8.5): the engine against outputs of the
// original xr-slam.wasm functions run in isolation. Tolerances are the pre-registered ones and are
// written in the generators' headers before their first run:
//   MB-TRI: |ours - wasm| <= 1e-12 * max(1, |wasm|) (triangulate_probe.mjs, S3 §8.1)
//   MB-EPI: |ours - wasm| <= 1e-6 * max(1, |wasm|), Status/bool exact, sort order exact
//           (gen_epipolar_golden.mjs)

#include <array>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/hvector.h"
#include "gtest/gtest.h"
#include "map_builder/map_builder_internal.h"
#include "map_builder_test_util.h"
#include "reality/engine/geometry/bundle-residual-analytic.h"
#include "reality/engine/geometry/epipolar.h"

namespace slam_refact::map_builder {
namespace {

using testing::BitsToDouble;
using testing::BitsToFloat;
using testing::HexWords;
using testing::Near;
using testing::ReadFile;
using testing::Split;

const std::string kDataDir = SR_TEST_DATA_DIR "/map-builder/probes";

// ---------------------------------------------------------------------------------------------
// MB-TRI

struct TriCase {
  std::array<double, 2> ray;
  std::array<double, 12> projection;
  std::array<double, 3> point;
};

std::vector<TriCase> ReadTriCases() {
  std::vector<TriCase> cases;
  for (const std::string& line : Split(ReadFile(kDataDir + "/triangulate-r1/cases.tsv"), '\n')) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const std::vector<std::string> cols = Split(line, '\t');
    if (cols.size() != 4) {
      continue;
    }
    auto doubles = [](const std::string& csv) {
      std::vector<double> v;
      for (const std::string& t : Split(csv, ',')) {
        v.push_back(BitsToDouble(std::strtoull(t.c_str(), nullptr, 16)));
      }
      return v;
    };
    const std::vector<double> ray = doubles(cols[1]);
    const std::vector<double> proj = doubles(cols[2]);
    const std::vector<double> pt = doubles(cols[3]);
    TriCase c;
    std::copy(ray.begin(), ray.end(), c.ray.begin());
    std::copy(proj.begin(), proj.end(), c.projection.begin());
    std::copy(pt.begin(), pt.end(), c.point.begin());
    cases.push_back(c);
  }
  return cases;
}

struct TriResult {
  int k = -1;
  bool with_jac = false;
  int ret = 0;
  std::vector<double> r;
  std::vector<double> j;
};

// Minimal reader for result.json (JSON.stringify(..., null, 1)): the "k", "withJac", "ret", "r"
// and "j" members of every element of "results", in file order.
std::vector<TriResult> ReadTriResults() {
  const std::string s = ReadFile(kDataDir + "/triangulate-r1/result.json");
  std::vector<TriResult> out;
  size_t pos = s.find("\"results\"");
  auto number_after = [&](const std::string& key, size_t from, size_t* end) {
    const size_t at = s.find(key, from);
    if (at == std::string::npos) {
      *end = std::string::npos;
      return 0.0;
    }
    const char* begin = s.c_str() + at + key.size();
    char* stop = nullptr;
    const double v = std::strtod(begin, &stop);
    *end = static_cast<size_t>(stop - s.c_str());
    return v;
  };
  auto array_after = [&](const std::string& key, size_t from, size_t* end) {
    std::vector<double> v;
    size_t at = s.find(key, from);
    at += key.size();
    while (s[at] == ' ') {
      ++at;
    }
    if (s.compare(at, 4, "null") == 0) {
      *end = at + 4;
      return v;
    }
    ++at;  // '['
    while (true) {
      while (s[at] == ' ' || s[at] == '\n' || s[at] == ',') {
        ++at;
      }
      if (s[at] == ']') {
        break;
      }
      char* stop = nullptr;
      v.push_back(std::strtod(s.c_str() + at, &stop));
      at = static_cast<size_t>(stop - s.c_str());
    }
    *end = at + 1;
    return v;
  };
  while (true) {
    size_t end = 0;
    const size_t at = s.find("\"k\":", pos);
    if (at == std::string::npos) {
      break;
    }
    TriResult r;
    r.k = static_cast<int>(number_after("\"k\":", at, &end));
    const size_t wj = s.find("\"withJac\":", end);
    r.with_jac = s.compare(wj + 10, 5, " true") == 0;
    r.ret = static_cast<int>(number_after("\"ret\":", wj, &end));
    r.r = array_after("\"r\":", end, &end);
    r.j = array_after("\"j\":", s.find("\"refR\":", end), &end);
    out.push_back(r);
    pos = end;
  }
  return out;
}

TEST(MapBuilderParity, MbTriTriangulatePointMultiviewAnalytic128Cases) {
  const std::vector<TriCase> cases = ReadTriCases();
  const std::vector<TriResult> results = ReadTriResults();
  ASSERT_EQ(cases.size(), 64u);
  ASSERT_EQ(results.size(), 128u);
  int passed = 0;
  int bit_exact = 0;
  double max_abs = 0.0;
  for (const TriResult& res : results) {
    ASSERT_GE(res.k, 0);
    ASSERT_LT(res.k, 64);
    const TriCase& c = cases[static_cast<size_t>(res.k)];
    const c8::TriangulatePointMultiviewAnalytic f(c.ray, c.projection);
    const double* params[1] = {c.point.data()};
    double residual[2] = {0.0, 0.0};
    double jacobian[6] = {0.0};
    double* jacobians[1] = {jacobian};
    const bool ok = f.Evaluate(params, residual, res.with_jac ? jacobians : nullptr);
    bool pass = ok && res.ret == 1 && res.r.size() == 2;
    bool exact = pass;
    for (size_t i = 0; pass && i < 2; ++i) {
      pass = Near(residual[i], res.r[i], 1e-12);
      exact = exact && residual[i] == res.r[i];
      max_abs = std::max(max_abs, std::fabs(residual[i] - res.r[i]));
    }
    if (res.with_jac) {
      pass = pass && res.j.size() == 6;
      for (size_t i = 0; pass && i < 6; ++i) {
        pass = Near(jacobian[i], res.j[i], 1e-12);
        exact = exact && jacobian[i] == res.j[i];
        max_abs = std::max(max_abs, std::fabs(jacobian[i] - res.j[i]));
      }
    }
    EXPECT_TRUE(pass) << "case k=" << res.k << " withJac=" << res.with_jac;
    passed += pass ? 1 : 0;
    bit_exact += (pass && exact) ? 1 : 0;
  }
  EXPECT_EQ(passed, 128);
  std::printf("[MB-TRI] passed %d/128, bit-exact %d, max |diff| %.3g\n", passed, bit_exact, max_abs);
}

// ---------------------------------------------------------------------------------------------
// MB-EPI

struct GoldenRow {
  std::string kind;
  int index = 0;
  std::vector<uint32_t> in;
  std::vector<uint32_t> out;
};

std::vector<GoldenRow> ReadEpipolarGolden() {
  std::vector<GoldenRow> rows;
  for (const std::string& line : Split(ReadFile(kDataDir + "/epipolar-r1/golden.tsv"), '\n')) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const std::vector<std::string> cols = Split(line, '\t');
    if (cols.size() != 4 || cols[2].rfind("IN:", 0) != 0 || cols[3].rfind("OUT:", 0) != 0) {
      continue;
    }
    GoldenRow r;
    r.kind = cols[0];
    r.index = std::atoi(cols[1].c_str());
    r.in = HexWords(cols[2].substr(3));
    r.out = HexWords(cols[3].substr(4));
    rows.push_back(std::move(r));
  }
  return rows;
}

c8::HMatrix MatrixFromWords(const std::vector<uint32_t>& w, size_t at) {
  float m[16];
  float inv[16];
  for (size_t i = 0; i < 16; ++i) {
    m[i] = BitsToFloat(w[at + i]);
    inv[i] = BitsToFloat(w[at + 16 + i]);
  }
  return c8::HMatrix(m, inv);
}

template <size_t N>
void SetRaw(std::array<float, N>* raw, const std::vector<uint32_t>& w, size_t at) {
  for (size_t i = 0; i < N; ++i) {
    (*raw)[i] = BitsToFloat(w[at + i]);
  }
}

// EpipolarDepthImagePrework from its 292-byte wasm32 image (offsets in the generator header).
c8::EpipolarDepthImagePrework ImagePreworkFromWords(const std::vector<uint32_t>& w) {
  c8::EpipolarDepthImagePrework iw;
  iw.nearDist = BitsToFloat(w[0]);
  iw.viewOverlap = BitsToFloat(w[1]);
  iw.projectToDictionary = MatrixFromWords(w, 2);
  iw.rotateToDictionary = MatrixFromWords(w, 35);
  SetRaw(&iw.dictionaryClipPlaneNormal.raw(), w, 68);
  iw.isPureRotation = (w[72] & 0xff) != 0;
  return iw;
}

c8::EpipolarDepthPointPrework PointPreworkFromWords(const std::vector<uint32_t>& w) {
  c8::EpipolarDepthPointPrework pw;
  SetRaw(&pw.lineSegment.first.raw(), w, 0);
  SetRaw(&pw.lineSegment.second.raw(), w, 3);
  SetRaw(&pw.segmentVector.raw(), w, 6);
  pw.isPoint = (w[9] & 0xff) != 0;
  SetRaw(&pw.normSegmentVector.raw(), w, 10);
  SetRaw(&pw.rotatedWordsPoint.raw(), w, 13);
  SetRaw(&pw.dzScale.raw(), w, 17);
  return pw;
}

// Field-wise compare helper; records the first failure.
struct Checker {
  int fields = 0;
  int failures = 0;
  std::string first;
  void F(float ours, uint32_t ref_bits, const std::string& what) {
    ++fields;
    if (!Near(ours, BitsToFloat(ref_bits), 1e-6)) {
      Fail(what + " ours=" + std::to_string(ours) + " wasm=" + std::to_string(BitsToFloat(ref_bits)));
    }
  }
  void D(double ours, double ref, const std::string& what) {
    ++fields;
    if (!Near(ours, ref, 1e-6)) {
      Fail(what + " ours=" + std::to_string(ours) + " wasm=" + std::to_string(ref));
    }
  }
  void Exact(int64_t ours, int64_t ref, const std::string& what) {
    ++fields;
    if (ours != ref) {
      Fail(what + " ours=" + std::to_string(ours) + " wasm=" + std::to_string(ref));
    }
  }
  void Fail(const std::string& s) {
    if (failures++ == 0) {
      first = s;
    }
  }
};

template <size_t N>
void CompareRaw(Checker* c, const std::array<float, N>& raw, const std::vector<uint32_t>& w,
                size_t at, const std::string& what) {
  for (size_t i = 0; i < N; ++i) {
    c->F(raw[i], w[at + i], what + "[" + std::to_string(i) + "]");
  }
}

void CompareMatrix(Checker* c, const c8::HMatrix& m, const std::vector<uint32_t>& w, size_t at,
                   const std::string& what) {
  const auto& fwd = m.data();
  const c8::HMatrix m_inv = m.inv();
  const auto& inv = m_inv.data();
  for (size_t i = 0; i < 16; ++i) {
    c->F(fwd[i], w[at + i], what + ".m[" + std::to_string(i) + "]");
    c->F(inv[i], w[at + 16 + i], what + ".inv[" + std::to_string(i) + "]");
  }
  // noInvert / invertFailed bytes: the engine prework matrices are invertible.
  c->Exact(0, w[at + 32] & 0xffff, what + ".flags");
}

class MapBuilderEpipolarParity : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rows_ = new std::vector<GoldenRow>(ReadEpipolarGolden()); }
  static void TearDownTestSuite() {
    delete rows_;
    rows_ = nullptr;
  }
  static std::vector<const GoldenRow*> Rows(const std::string& kind) {
    std::vector<const GoldenRow*> out;
    for (const GoldenRow& r : *rows_) {
      if (r.kind == kind) {
        out.push_back(&r);
      }
    }
    return out;
  }
  static std::vector<GoldenRow>* rows_;
};
std::vector<GoldenRow>* MapBuilderEpipolarParity::rows_ = nullptr;

TEST_F(MapBuilderEpipolarParity, MbEpi1115ImagePrework) {
  const auto rows = Rows("E1115");
  ASSERT_EQ(rows.size(), 64u);
  Checker c;
  for (const GoldenRow* r : rows) {
    const c8::HMatrix words = MatrixFromWords(r->in, 0);
    const c8::HMatrix dict = MatrixFromWords(r->in, 32);
    const c8::EpipolarDepthImagePrework iw = c8::epipolarDepthImagePrework(words, dict);
    const std::string tag = "E1115#" + std::to_string(r->index);
    c.F(iw.nearDist, r->out[0], tag + ".nearDist");
    c.F(iw.viewOverlap, r->out[1], tag + ".viewOverlap");
    CompareMatrix(&c, iw.projectToDictionary, r->out, 2, tag + ".projectToDictionary");
    CompareMatrix(&c, iw.rotateToDictionary, r->out, 35, tag + ".rotateToDictionary");
    CompareRaw(&c, iw.dictionaryClipPlaneNormal.raw(), r->out, 68, tag + ".normal");
    c.Exact(iw.isPureRotation ? 1 : 0, r->out[72] & 0xff, tag + ".isPureRotation");
  }
  EXPECT_EQ(c.failures, 0) << c.first;
  std::printf("[MB-EPI 1115] %zu cases, %d fields, %d failures\n", rows.size(), c.fields, c.failures);
}

TEST_F(MapBuilderEpipolarParity, MbEpi1116PointPrework) {
  const auto image_rows = Rows("E1115");
  const auto rows = Rows("E1116");
  ASSERT_EQ(rows.size(), 256u);
  Checker c;
  for (const GoldenRow* r : rows) {
    const c8::EpipolarDepthImagePrework iw = ImagePreworkFromWords(image_rows.at(r->in[0])->out);
    const c8::EpipolarDepthPointPrework pw = c8::epipolarDepthPointPrework(
        iw, c8::HPoint2(BitsToFloat(r->in[1]), BitsToFloat(r->in[2])));
    const std::string tag = "E1116#" + std::to_string(r->index);
    CompareRaw(&c, pw.lineSegment.first.raw(), r->out, 0, tag + ".first");
    CompareRaw(&c, pw.lineSegment.second.raw(), r->out, 3, tag + ".second");
    CompareRaw(&c, pw.segmentVector.raw(), r->out, 6, tag + ".segmentVector");
    c.Exact(pw.isPoint ? 1 : 0, r->out[9] & 0xff, tag + ".isPoint");
    CompareRaw(&c, pw.normSegmentVector.raw(), r->out, 10, tag + ".normSegmentVector");
    CompareRaw(&c, pw.rotatedWordsPoint.raw(), r->out, 13, tag + ".rotatedWordsPoint");
    CompareRaw(&c, pw.dzScale.raw(), r->out, 17, tag + ".dzScale");
  }
  EXPECT_EQ(c.failures, 0) << c.first;
  std::printf("[MB-EPI 1116] %zu cases, %d fields, %d failures\n", rows.size(), c.fields, c.failures);
}

TEST_F(MapBuilderEpipolarParity, MbEpi1118DepthOfEpipolarPoint) {
  const auto image_rows = Rows("E1115");
  const auto point_rows = Rows("E1116");
  const auto rows = Rows("E1118");
  ASSERT_EQ(rows.size(), 512u);
  Checker c;
  std::map<int, int> statuses;
  for (const GoldenRow* r : rows) {
    const GoldenRow* pr = point_rows.at(r->in[0]);
    const c8::EpipolarDepthImagePrework iw = ImagePreworkFromWords(image_rows.at(pr->in[0])->out);
    const c8::EpipolarDepthPointPrework pw = PointPreworkFromWords(pr->out);
    const c8::EpipolarDepthResult res = c8::depthOfEpipolarPoint(
        iw, pw, c8::HVector2(BitsToFloat(r->in[1]), BitsToFloat(r->in[2])),
        static_cast<uint8_t>(r->in[3]));
    const std::string tag = "E1118#" + std::to_string(r->index);
    const double depth = BitsToDouble(static_cast<uint64_t>(r->out[1]) << 32 | r->out[0]);
    const double certainty = BitsToDouble(static_cast<uint64_t>(r->out[3]) << 32 | r->out[2]);
    c.D(res.depthInWords, depth, tag + ".depthInWords");
    c.D(res.certainty, certainty, tag + ".certainty");
    c.Exact(static_cast<int>(res.status), static_cast<int32_t>(r->out[4]), tag + ".status");
    ++statuses[static_cast<int32_t>(r->out[4])];
  }
  EXPECT_EQ(c.failures, 0) << c.first;
  std::printf("[MB-EPI 1118] %zu cases, %d fields, %d failures; statuses OK=%d UNCERTAIN=%d "
              "PURE_ROTATION=%d FOE=%d\n",
              rows.size(), c.fields, c.failures, statuses[0], statuses[1], statuses[2],
              statuses[3]);
}

TEST_F(MapBuilderEpipolarParity, MbEpi1122And1123RayPairPoint) {
  const auto rows = Rows("E1123");
  ASSERT_EQ(rows.size(), 128u);
  Checker c;
  for (const GoldenRow* r : rows) {
    const c8::HMatrix first = MatrixFromWords(r->in, 0);
    const c8::HMatrix second = MatrixFromWords(r->in, 32);
    const RayPairPrework w = MakeRayPairPrework(first, second);
    const std::string tag = "E1123#" + std::to_string(r->index);
    CompareRaw(&c, w.essential, r->out, 0, tag + ".essential");
    CompareRaw(&c, w.essential_t, r->out, 33, tag + ".essential_t");
    CompareRaw(&c, w.rotation_first, r->out, 66, tag + ".rotation_first");
    CompareRaw(&c, w.center_first, r->out, 99, tag + ".center_first");
    CompareRaw(&c, w.second_block, r->out, 103, tag + ".second_block");
    const c8::HPoint3 ray1(BitsToFloat(r->in[64]), BitsToFloat(r->in[65]), 1.0f);
    const c8::HPoint3 ray2(BitsToFloat(r->in[66]), BitsToFloat(r->in[67]), 1.0f);
    const c8::HPoint3 p = RayPairPoint(ray1, ray2, w);
    CompareRaw(&c, p.raw(), r->out, 136, tag + ".point");
  }
  EXPECT_EQ(c.failures, 0) << c.first;
  std::printf("[MB-EPI 1122/1123] %zu cases, %d fields, %d failures\n", rows.size(), c.fields,
              c.failures);
}

void CheckSortRows(const std::vector<const GoldenRow*>& rows, const std::string& kind) {
  ASSERT_EQ(rows.size(), 19u);
  int exact = 0;
  for (const GoldenRow* r : rows) {
    const size_t n = r->in.size() / 5;
    std::vector<c8::PointMatch> v(n);
    std::map<std::array<uint32_t, 4>, int> keys;
    for (size_t i = 0; i < n; ++i) {
      v[i].wordsIdx = r->in[5 * i];
      v[i].dictionaryIdx = r->in[5 * i + 1];
      v[i].rotation = BitsToFloat(r->in[5 * i + 2]);
      v[i].descriptorDist = BitsToFloat(r->in[5 * i + 3]);
      v[i].dictionaryScale = static_cast<uint8_t>(r->in[5 * i + 4]);
      ++keys[{r->in[5 * i + 3], r->in[5 * i + 4], r->in[5 * i], r->in[5 * i + 1]}];
    }
    ASSERT_EQ(keys.size(), n) << "input keys must be unique (no tie-order claim)";
    SortEpipolarMatches(&v);
    bool same = true;
    for (size_t i = 0; i < n; ++i) {
      same = same && static_cast<uint32_t>(v[i].wordsIdx) == r->out[5 * i] &&
             static_cast<uint32_t>(v[i].dictionaryIdx) == r->out[5 * i + 1] &&
             v[i].rotation == BitsToFloat(r->out[5 * i + 2]) &&
             v[i].descriptorDist == BitsToFloat(r->out[5 * i + 3]) &&
             v[i].dictionaryScale == (r->out[5 * i + 4] & 0xff);
    }
    EXPECT_TRUE(same) << kind << " case " << r->index << " (n=" << n << ")";
    exact += same ? 1 : 0;
  }
  std::printf("[MB-EPI %s] order identical %d/%zu\n", kind.c_str(), exact, rows.size());
}

TEST_F(MapBuilderEpipolarParity, MbEpi1121SortOrderOrb) { CheckSortRows(Rows("S1121"), "1121"); }

TEST_F(MapBuilderEpipolarParity, MbEpi1120SortOrderGorb) { CheckSortRows(Rows("S1120"), "1120"); }

}  // namespace
}  // namespace slam_refact::map_builder
