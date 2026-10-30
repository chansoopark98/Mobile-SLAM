// mlmodel tests (US-016/US-017/RM-1; docs/design/ml-runtime.md §5.2, §5.3, §6): the parser reads the
// seven models and the engine runs them end to end against the numpy reference outputs written by
// the sr_ml_ref_generate fixture (tools/ml_ref/ml_ref.py run-all). The models are read from
// assets/references (gitignored, not copied); a missing file fails the test, it is not skipped.
#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <ostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "ml/compare.h"
#include "ml/interpreter.h"
#include "ml/tensor_bundle.h"
#include "ml/tflite_model.h"
#include "regression_gate.h"

namespace slam_refact {
namespace ml {
namespace {

// SLAM_REFACT_ML_REPO_ROOT overrides the repository root (used to check that a missing model
// fails the tests instead of skipping them).
std::string RepoRoot() {
  const char* env = std::getenv("SLAM_REFACT_ML_REPO_ROOT");
  return env != nullptr && *env != '\0' ? std::string(env) : std::string(SR_REPO_ROOT);
}

const std::string kRepo = RepoRoot();
// The numpy reference bundles. SR_ML_REF_DIR in the environment overrides the build-tree directory
// (the registered gate run uses a pinned copy: ml_ref.py run-all is not reproducible to the last bit,
// its BLAS threading changes the float64 sums by up to 7e-15 relative).
std::string RefDir() {
  const char* env = std::getenv("SR_ML_REF_DIR");
  return env != nullptr && *env != '\0' ? std::string(env) : std::string(SR_ML_REF_DIR);
}

const std::string kRefDir = RefDir();
const std::string kGateTsv = SR_ML_GATE_TSV;
const std::string kGateAggregateTsv = SR_ML_GATE_AGGREGATE_TSV;

// Where report-native-<model>.json is written: SR_ML_REPORT_DIR if set (the directory must exist;
// a gate run uses a fresh one so that no earlier report is overwritten), else the reference dir.
std::string ReportDir() {
  const char* env = std::getenv("SR_ML_REPORT_DIR");
  return env != nullptr && *env != '\0' ? std::string(env) : kRefDir;
}

struct ModelSpec {
  const char* name;
  const char* path;
  const char* sha256;
  // Anchors from the feasibility study's independent inspector (scratchpad tflite_inspect.py).
  size_t tensors;
  size_t ops;
};

const ModelSpec kModels[] = {
    {"face-model", "assets/references/xr_reference/engine/resources/face-model.tflite",
     "0d800dd37230bb0503e2bc84ce547a1328dc0e2d8c54a5a92df953710decc4ed", 176, 90},
    {"face-mesh-model", "assets/references/xr_reference/engine/resources/face-mesh-model.tflite",
     "97cd5b1d16d1d205445134d0d8882a619068c1a49277652c468049d0fc7f2756", 1096, 712},
    {"face-ear-model", "assets/references/xr_reference/engine/resources/face-ear-model.tflite",
     "4cd07c3ecc4ac3a19097ad5d8e2920113e8786d29302e48064b5e21b9f828552", 267, 167},
    {"semantics-model", "assets/references/xr_reference/engine/resources/semantics-model.tflite",
     "b25bebe3ec1b7390feec937d1e4eafb3d6f2104d584a738c0bdf6486030b70a0", 419, 276},
    {"palm_detection",
     "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/palm_detection.tflite",
     "ab2828eeeb871a1229699ac59f870f5d915b5de8582c5dd17330b83068c458ff", 379, 193},
    {"hand_landmark",
     "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/hand_landmark.tflite",
     "06d03d84801cabd033ea56a366ff3dcd8cd40812a9e37c90df8dc1435d704018", 895, 388},
    {"hand_landmark_3d",
     "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/"
     "hand_landmark_3d.tflite",
     "12f500618bccaf7b208b1375e7fc60c66209fe570b5897defaeb5e3c592f33fa", 387, 196},
};

// ---- SHA-256 (FIPS 180-4) ------------------------------------------------------------------------
std::string Sha256Hex(const std::vector<uint8_t>& data) {
  static const uint32_t k[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
      0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
      0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
      0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
      0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
      0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
      0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
      0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
      0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
      0xc67178f2};
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::vector<uint8_t> msg = data;
  const uint64_t bits = static_cast<uint64_t>(data.size()) * 8;
  msg.push_back(0x80);
  while (msg.size() % 64 != 56) msg.push_back(0);
  for (int i = 7; i >= 0; --i) msg.push_back(static_cast<uint8_t>(bits >> (8 * i)));
  auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
  for (size_t off = 0; off < msg.size(); off += 64) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<uint32_t>(msg[off + 4 * i]) << 24) |
             (static_cast<uint32_t>(msg[off + 4 * i + 1]) << 16) |
             (static_cast<uint32_t>(msg[off + 4 * i + 2]) << 8) | msg[off + 4 * i + 3];
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = hh + s1 + ch + k[i] + w[i];
      const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = s0 + maj;
      hh = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
  }
  char out[65];
  for (int i = 0; i < 8; ++i) std::snprintf(out + 8 * i, 9, "%08x", h[i]);
  return std::string(out, 64);
}

std::vector<uint8_t> ReadFile(const std::string& path, bool* ok) {
  std::ifstream in(path, std::ios::binary);
  *ok = static_cast<bool>(in);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
}

std::string ReadText(const std::string& path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string FirstDifference(const std::string& a, const std::string& b) {
  std::istringstream sa(a);
  std::istringstream sb(b);
  std::string la;
  std::string lb;
  int line = 0;
  while (true) {
    const bool ga = static_cast<bool>(std::getline(sa, la));
    const bool gb = static_cast<bool>(std::getline(sb, lb));
    ++line;
    if (!ga && !gb) return "identical";
    if (!ga || !gb || la != lb) {
      return "line " + std::to_string(line) + "\n  C++  : " + (ga ? la : "<end>") +
             "\n  numpy: " + (gb ? lb : "<end>");
    }
  }
}

TEST(MlModels, ParserReadsTheSevenModels) {
  std::set<std::string> op_names;
  for (const ModelSpec& m : kModels) {
    const std::string path = kRepo + "/" + m.path;
    bool ok = false;
    std::vector<uint8_t> bytes = ReadFile(path, &ok);
    ASSERT_TRUE(ok) << "model file missing: " << path
                    << " (assets/references is required for ml-evidence; it is never copied "
                       "into the repo)";
    EXPECT_EQ(Sha256Hex(bytes), m.sha256) << m.name;
    std::string error;
    std::unique_ptr<TfliteModel> model = TfliteModel::FromBytes(std::move(bytes), &error);
    ASSERT_NE(model, nullptr) << m.name << ": " << error;
    ASSERT_EQ(model->subgraphs().size(), 1u) << m.name;
    const SubgraphDef& sg = model->subgraphs()[0];
    EXPECT_EQ(sg.tensors.size(), m.tensors) << m.name;
    EXPECT_EQ(sg.ops.size(), m.ops) << m.name;
    size_t quantized = 0;
    for (const TensorDef& t : sg.tensors) quantized += t.quantization.scale.empty() ? 0 : 1;
    EXPECT_EQ(quantized, 0u) << m.name << " (no model uses quantization parameters)";
    for (const OperatorDef& op : sg.ops) op_names.insert(TfliteModel::OpName(op));
    // Every tensor, op (with options and custom option bytes), input and output equals the
    // numpy parser's reading.
    const std::string want = ReadText(kRefDir + "/" + m.name + "/graph.txt");
    ASSERT_FALSE(want.empty()) << "numpy reference graph missing for " << m.name
                               << " (run the sr_ml_ref_generate fixture)";
    const std::string got = CanonicalGraphText(*model);
    EXPECT_EQ(got, want) << m.name << ": " << FirstDifference(got, want);
  }
  // Union of the seven models: 24 builtin + 4 custom ops.
  EXPECT_EQ(op_names.size(), 28u);
  const std::set<std::string> custom = {"TransformLandmarks", "Landmarks2TransformMatrix",
                                        "TransformTensorBilinear",
                                        "Convolution2DTransposeBias"};
  size_t n_custom = 0;
  for (const std::string& n : op_names) n_custom += custom.count(n);
  EXPECT_EQ(n_custom, 4u);
}

void PrintTo(const ModelSpec& m, std::ostream* os) { *os << m.name; }

class MlModelRun : public ::testing::TestWithParam<ModelSpec> {};

// Rule 3 of §5.2 for face-ear-model's arg-max derived landmark output.
struct ArgmaxCheck {
  size_t rows = 0;
  size_t exact_rows = 0;
  size_t accepted_ties = 0;
  size_t failures = 0;
};

ArgmaxCheck CheckEarArgmax(const Interpreter& interp, const TfliteModel& model,
                           const BundleTensor& ref_heat, const BundleTensor& ref_lm,
                           const TensorView& got_lm) {
  ArgmaxCheck r;
  int argmax_out = -1;
  for (const OperatorDef& op : model.subgraphs()[0].ops) {
    if (op.builtin_code == kOpArgMax) argmax_out = op.outputs[0];
  }
  if (argmax_out < 0) {
    r.failures = 1;
    return r;
  }
  const TensorView am = interp.tensor(argmax_out);  // [B, J] int64
  const int b_n = ref_heat.shape[0];
  const int h = ref_heat.shape[1];
  const int w = ref_heat.shape[2];
  const int j_n = ref_heat.shape[3];
  double heat_max = 0.0;
  for (double v : ref_heat.values) heat_max = std::max(heat_max, std::fabs(v));
  const double gap_tol = 2.0 * kModelMaxAbsTolerance * ScaleFloor(heat_max);
  for (int b = 0; b < b_n; ++b) {
    for (int j = 0; j < j_n; ++j) {
      ++r.rows;
      auto heat = [&](int pos) {
        const int y = pos / w;
        const int x = pos % w;
        return ref_heat.values[((static_cast<size_t>(b) * h + y) * w + x) * j_n + j];
      };
      int best = 0;
      for (int pos = 1; pos < h * w; ++pos) {
        if (heat(pos) > heat(best)) best = pos;
      }
      const int64_t got = am.i64[b * j_n + j];
      const size_t row = static_cast<size_t>(b * j_n + j) * 2;
      if (got == best) {
        const bool same = got_lm.f32[row] == static_cast<float>(ref_lm.values[row]) &&
                          got_lm.f32[row + 1] == static_cast<float>(ref_lm.values[row + 1]);
        if (same) {
          ++r.exact_rows;
        } else {
          ++r.failures;
        }
      } else if (got >= 0 && got < h * w && heat(best) - heat(static_cast<int>(got)) <= gap_tol) {
        ++r.accepted_ties;
      } else {
        ++r.failures;
      }
    }
  }
  return r;
}

TEST_P(MlModelRun, MatchesNumpyReferenceWithinPreRegisteredTolerance) {
  const ModelSpec& m = GetParam();
  const std::string path = kRepo + "/" + m.path;
  std::string error;
  std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(path, &error);
  ASSERT_NE(model, nullptr) << "model file missing or unreadable: " << path << ": " << error;
  std::unique_ptr<Interpreter> interp = Interpreter::Create(*model, &error);
  ASSERT_NE(interp, nullptr) << error;
  std::vector<GateRow> gate;
  ASSERT_TRUE(LoadRegressionGate(kGateTsv, &gate, &error)) << error;
  std::ostringstream report;
  report << "{\"model\": \"" << m.name << "\", \"tolerance\": {\"max_abs\": "
         << kModelMaxAbsTolerance << ", \"rms\": " << kModelRmsTolerance << "}, \"inputs\": [";
  for (int k = 0; k < 2; ++k) {
    const std::string base = kRefDir + "/" + m.name + "/";
    std::vector<BundleTensor> in;
    std::vector<BundleTensor> ref;
    ASSERT_TRUE(ReadTensorBundle(base + "inputs-" + std::to_string(k) + ".srt", &in, &error))
        << error << " (run the sr_ml_ref_generate fixture)";
    ASSERT_TRUE(ReadTensorBundle(base + "ref-" + std::to_string(k) + ".srt", &ref, &error))
        << error;
    ASSERT_EQ(in.size(), 1u);
    ASSERT_EQ(in[0].shape, interp->input_shape(0));
    float* dst = interp->input_data(0);
    for (size_t i = 0; i < in[0].values.size(); ++i) dst[i] = static_cast<float>(in[0].values[i]);
    ASSERT_TRUE(interp->Invoke(&error)) << error;
    ASSERT_EQ(ref.size(), interp->num_outputs());
    report << (k ? ", " : "") << "{\"input\": " << k << ", \"outputs\": [";
    std::vector<std::vector<float>> first;
    for (size_t o = 0; o < ref.size(); ++o) {
      const TensorView v = interp->output(o);
      const OutputError e = CompareOutput(v, ref[o]);
      EXPECT_EQ(v.name, ref[o].name);
      ASSERT_TRUE(e.shape_ok && e.type_ok) << m.name << " " << ref[o].name;
      const bool argmax_output =
          std::string(m.name) == "face-ear-model" && ref[o].name == "PartitionedCall:1";
      bool pass = false;
      std::string extra;
      if (argmax_output) {
        const BundleTensor* heat = nullptr;
        for (const BundleTensor& t : ref) {
          if (t.name == "PartitionedCall:0") heat = &t;
        }
        ASSERT_NE(heat, nullptr);
        const ArgmaxCheck a = CheckEarArgmax(*interp, *model, *heat, ref[o], v);
        pass = a.failures == 0;
        extra = ", \"argmax_rows\": " + std::to_string(a.rows) +
                ", \"argmax_exact_rows\": " + std::to_string(a.exact_rows) +
                ", \"argmax_accepted_ties\": " + std::to_string(a.accepted_ties) +
                ", \"argmax_failures\": " + std::to_string(a.failures);
        EXPECT_TRUE(pass) << m.name << " input " << k << " " << ref[o].name << extra;
      } else {
        pass = PassesModelTolerance(e);
        EXPECT_TRUE(pass) << m.name << " input " << k << " " << ref[o].name << ": "
                          << DescribeError(e);
      }
      const double abs_ratio = e.max_abs / (kModelMaxAbsTolerance * ScaleFloor(e.ref_max_abs));
      const double rms_ratio = e.rms / (kModelRmsTolerance * ScaleFloor(e.ref_rms));
      // Relative regression gate (section 5.3): a missing row fails, it is never skipped.
      const GateRow* gate_row = FindGateRow(gate, m.name, k, ref[o].name);
      EXPECT_NE(gate_row, nullptr) << m.name << " input " << k << " " << ref[o].name
                                   << ": no row in " << kGateTsv;
      GateVerdict verdict;
      if (gate_row != nullptr) {
        verdict = EvaluateGate(*gate_row, e);
        EXPECT_TRUE(verdict.reference_ok)
            << m.name << " input " << k << " " << ref[o].name
            << ": the reference scales differ from the registered table (max|r| " << e.ref_max_abs
            << " vs " << gate_row->ref_max_abs << ", RMS(r) " << e.ref_rms << " vs "
            << gate_row->ref_rms << "): the numpy reference or the inputs changed";
        EXPECT_TRUE(verdict.pass)
            << m.name << " input " << k << " " << ref[o].name
            << " is beyond baseline + 3 x resolution of its registered gate row: max_abs " << e.max_abs
            << " (limit " << verdict.max_abs_limit << "), rms " << e.rms << " (limit "
            << verdict.rms_limit << ")";
      }
      char buf[900];
      std::snprintf(buf, sizeof(buf),
                    "{\"name\": \"%s\", \"count\": %zu, \"max_abs\": %.17g, \"ref_max_abs\": "
                    "%.17g, \"rms\": %.17g, \"ref_rms\": %.17g, \"max_abs_over_tol\": %.4f, "
                    "\"rms_over_tol\": %.4f, \"gate_max_abs_limit\": %.17g, "
                    "\"gate_rms_limit\": %.17g, \"gate_max_abs_ratio\": %.4f, "
                    "\"gate_rms_ratio\": %.4f, \"bit_mismatches\": %zu, \"pass\": %s",
                    ref[o].name.c_str(), e.count, e.max_abs, e.ref_max_abs, e.rms, e.ref_rms,
                    abs_ratio, rms_ratio, verdict.max_abs_limit, verdict.rms_limit,
                    verdict.max_abs_ratio, verdict.rms_ratio, e.bit_mismatches,
                    pass ? "true" : "false");
      report << (o ? ", " : "") << buf << extra << "}";
      if (v.f32) {
        first.emplace_back(v.f32, v.f32 + v.count);
      } else {
        first.emplace_back();
      }
    }
    report << "]}";
    // Determinism: the same input gives bit-identical outputs.
    ASSERT_TRUE(interp->Invoke(&error)) << error;
    for (size_t o = 0; o < ref.size(); ++o) {
      const TensorView v = interp->output(o);
      if (v.f32) {
        size_t diff = 0;
        size_t first_diff = 0;
        for (size_t i = 0; i < v.count; ++i) {
          if (std::memcmp(&v.f32[i], &first[o][i], sizeof(float)) != 0) {
            if (diff == 0) first_diff = i;
            ++diff;
          }
        }
        EXPECT_EQ(diff, 0u) << m.name << " " << v.name << ": first differing element "
                            << first_diff << " (" << first[o][first_diff] << " then "
                            << v.f32[first_diff] << ")";
      }
    }
  }
  report << "], \"planned_ops\": " << interp->num_planned_ops()
         << ", \"folded_ops\": " << interp->num_folded_ops()
         << ", \"arena_bytes\": " << interp->arena_bytes() << "}\n";
  std::ofstream f(ReportDir() + "/report-native-" + m.name + ".json");
  f << report.str();
  EXPECT_TRUE(static_cast<bool>(f));
}

std::string ModelName(const ::testing::TestParamInfo<ModelSpec>& info) {
  std::string n = info.param.name;
  for (char& c : n) {
    if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
  }
  return n;
}

INSTANTIATE_TEST_SUITE_P(Models, MlModelRun, ::testing::ValuesIn(kModels), ModelName);

// Mutation check of the regression gate: an input scaled by 1 + 1e-4 changes every output by about
// 1e-4 relative, which the absolute tolerances of section 5.2 accept (1e-3 * max|r|) but the gate
// (baseline + 3 x resolution, about 1e-6 relative at most for these outputs) must reject. The
// smallest perturbation the gate sees is measured by the registration's mutation run
// (preregistration/rm1-regression-gate.v3.md), not by this test.
TEST(MlModels, RegressionGateRejectsAPerturbedInput) {
  const ModelSpec& m = kModels[0];  // face-model, the cheapest
  std::string error;
  std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(kRepo + "/" + m.path, &error);
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<Interpreter> interp = Interpreter::Create(*model, &error);
  ASSERT_NE(interp, nullptr) << error;
  std::vector<GateRow> gate;
  ASSERT_TRUE(LoadRegressionGate(kGateTsv, &gate, &error)) << error;
  std::vector<BundleTensor> in;
  std::vector<BundleTensor> ref;
  ASSERT_TRUE(ReadTensorBundle(kRefDir + "/" + m.name + "/inputs-0.srt", &in, &error)) << error;
  ASSERT_TRUE(ReadTensorBundle(kRefDir + "/" + m.name + "/ref-0.srt", &ref, &error)) << error;
  float* dst = interp->input_data(0);
  for (size_t i = 0; i < in[0].values.size(); ++i) {
    dst[i] = static_cast<float>(in[0].values[i] * (1.0 + 1e-4));
  }
  ASSERT_TRUE(interp->Invoke(&error)) << error;
  size_t gate_failures = 0;
  size_t absolute_passes = 0;
  for (size_t o = 0; o < ref.size(); ++o) {
    const OutputError e = CompareOutput(interp->output(o), ref[o]);
    const GateRow* row = FindGateRow(gate, m.name, 0, ref[o].name);
    ASSERT_NE(row, nullptr) << ref[o].name;
    if (!EvaluateGate(*row, e).pass) ++gate_failures;
    if (PassesModelTolerance(e)) ++absolute_passes;
  }
  EXPECT_EQ(gate_failures, ref.size()) << "the gate missed a perturbed output";
  RecordProperty("outputs_failing_gate", static_cast<int>(gate_failures));
  RecordProperty("outputs_passing_absolute_tolerance", static_cast<int>(absolute_passes));
}

// Aggregate rule of the gate (regression_gate.h): the geometric mean of observed / baseline over the
// 32 non-trivial rows. It runs all seven models itself, because ctest starts every gtest test in its
// own process and the per-model tests cannot share their results.
TEST(MlModels, RegressionGateAggregateStaysWithinItsLimit) {
  std::vector<GateRow> gate;
  GateAggregateLimits limits;
  std::string error;
  ASSERT_TRUE(LoadRegressionGate(kGateTsv, &gate, &error)) << error;
  ASSERT_TRUE(LoadAggregateLimits(kGateAggregateTsv, &limits, &error)) << error;
  AggregateAccumulator acc;
  for (const ModelSpec& m : kModels) {
    std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(kRepo + "/" + m.path, &error);
    ASSERT_NE(model, nullptr) << m.name << ": " << error;
    std::unique_ptr<Interpreter> interp = Interpreter::Create(*model, &error);
    ASSERT_NE(interp, nullptr) << m.name << ": " << error;
    for (int k = 0; k < 2; ++k) {
      const std::string base = kRefDir + "/" + m.name + "/";
      std::vector<BundleTensor> in;
      std::vector<BundleTensor> ref;
      ASSERT_TRUE(ReadTensorBundle(base + "inputs-" + std::to_string(k) + ".srt", &in, &error)) << error;
      ASSERT_TRUE(ReadTensorBundle(base + "ref-" + std::to_string(k) + ".srt", &ref, &error)) << error;
      float* dst = interp->input_data(0);
      for (size_t i = 0; i < in[0].values.size(); ++i) dst[i] = static_cast<float>(in[0].values[i]);
      ASSERT_TRUE(interp->Invoke(&error)) << error;
      for (size_t o = 0; o < ref.size(); ++o) {
        const GateRow* row = FindGateRow(gate, m.name, k, ref[o].name);
        ASSERT_NE(row, nullptr) << m.name << " " << ref[o].name;
        acc.Add(*row, CompareOutput(interp->output(o), ref[o]));
      }
    }
  }
  EXPECT_EQ(acc.rows, limits.rows) << "the aggregate rows differ from the registered count";
  EXPECT_LE(acc.MaxAbsRatio(), limits.max_abs_limit);
  EXPECT_LE(acc.RmsRatio(), limits.rms_limit);
  RecordProperty("aggregate_rows", static_cast<int>(acc.rows));
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", acc.MaxAbsRatio());
  RecordProperty("aggregate_max_abs_ratio", buf);
  std::snprintf(buf, sizeof(buf), "%.17g", acc.RmsRatio());
  RecordProperty("aggregate_rms_ratio", buf);
  std::ofstream f(ReportDir() + "/report-native-aggregate.json");
  f.precision(17);
  f << "{\"rows\": " << acc.rows << ", \"max_abs_ratio\": " << acc.MaxAbsRatio()
    << ", \"max_abs_limit\": " << limits.max_abs_limit << ", \"rms_ratio\": " << acc.RmsRatio()
    << ", \"rms_limit\": " << limits.rms_limit << "}\n";
}

// Parser and Create robustness on the real models (sanitizer runs raise the counts with
// SR_ML_FUZZ_FLIPS / SR_ML_FUZZ_INVOKES). Positions and values come from a fixed LCG, so a run is
// reproducible; a flip that leaves a valid model is run only if its activation memory is small.
TEST_P(MlModelRun, SurvivesRandomByteFlipsAndTruncation) {
  const ModelSpec& m = GetParam();
  const char* flips_env = std::getenv("SR_ML_FUZZ_FLIPS");
  const char* invokes_env = std::getenv("SR_ML_FUZZ_INVOKES");
  const int flips = flips_env != nullptr ? std::atoi(flips_env) : 200;
  const int max_invokes = invokes_env != nullptr ? std::atoi(invokes_env) : 3;
  bool ok = false;
  const std::vector<uint8_t> bytes = ReadFile(kRepo + "/" + m.path, &ok);
  ASSERT_TRUE(ok) << "model file missing: " << kRepo << "/" << m.path;
  uint32_t state = 0x9e3779b9u ^ static_cast<uint32_t>(m.tensors);
  auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return state;
  };
  int created = 0;
  int invoked = 0;
  int rejected = 0;
  for (int i = 0; i < flips; ++i) {
    std::vector<uint8_t> mutated = bytes;
    const size_t pos = (static_cast<uint64_t>(next()) * mutated.size()) >> 32;
    // A third of the flips go to the first and a third to the last 4 KiB of the file (root table,
    // vtables, tensor / operator tables live there); the rest are uniform (mostly weights).
    const size_t window = std::min<size_t>(mutated.size(), 4096);
    const size_t where = i % 3 == 0   ? pos % window
                         : i % 3 == 1 ? mutated.size() - 1 - pos % window
                                      : pos;
    const uint8_t xor_value = static_cast<uint8_t>(1u << (next() >> 29));  // one bit flip
    mutated[where] ^= xor_value;
    std::string error;
    std::unique_ptr<TfliteModel> mm = TfliteModel::FromBytes(std::move(mutated), &error);
    if (!mm) {
      ASSERT_FALSE(error.empty());
      ++rejected;
      continue;
    }
    std::unique_ptr<Interpreter> it = Interpreter::Create(*mm, &error);
    if (!it) {
      ASSERT_FALSE(error.empty());
      ++rejected;
      continue;
    }
    ++created;
    if (invoked < max_invokes && it->arena_bytes() < (size_t{64} << 20)) {
      for (size_t k = 0; k < it->num_inputs(); ++k) {
        float* in = it->input_data(k);
        for (size_t j = 0; j < it->input_count(k); ++j) in[j] = 0.25f;
      }
      EXPECT_TRUE(it->Invoke(&error)) << error;
      ++invoked;
    }
  }
  // Truncations at 64 positions spread over the file.
  for (int i = 0; i < 64; ++i) {
    const size_t n = bytes.size() * static_cast<size_t>(i) / 64;
    std::string error;
    std::unique_ptr<TfliteModel> mm =
        TfliteModel::FromBytes(std::vector<uint8_t>(bytes.begin(), bytes.begin() + n), &error);
    if (!mm) {
      ASSERT_FALSE(error.empty());
    } else {
      Interpreter::Create(*mm, &error);
    }
  }
  RecordProperty("flips", flips);
  RecordProperty("rejected", rejected);
  RecordProperty("created", created);
  RecordProperty("invoked", invoked);
}

}  // namespace
}  // namespace ml
}  // namespace slam_refact
