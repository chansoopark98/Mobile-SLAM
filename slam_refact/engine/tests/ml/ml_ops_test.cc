// Unit tests of the ML engine (US-016/US-017; docs/design/ml-runtime.md §5.1, §6): FP16
// conversion, FlexBuffers, parser robustness, and every single-op case against the numpy
// reference with the pre-registered tolerance classes.
#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <ostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ml/compare.h"
#include "ml/flexbuffers.h"
#include "ml/gemm.h"
#include "ml/interpreter.h"
#include "ml/kernels.h"
#include "ml/tensor_bundle.h"
#include "ml/tflite_model.h"
#include "regression_gate.h"

namespace slam_refact {
namespace ml {
namespace {

const std::string kCases = SR_ML_OP_CASES_DIR;
const std::string kGateTsv = SR_ML_GATE_TSV;
const std::string kGateAggregateTsv = SR_ML_GATE_AGGREGATE_TSV;

std::vector<uint8_t> ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
}

std::string ReadText(const std::string& path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

struct CaseRow {
  std::string name;
  std::string cls;
  std::string op;
};

std::vector<CaseRow> ReadCases() {
  std::vector<CaseRow> rows;
  std::ifstream in(kCases + "/cases.tsv");
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    CaseRow r;
    std::getline(ls, r.name, '\t');
    std::getline(ls, r.cls, '\t');
    std::getline(ls, r.op, '\t');
    rows.push_back(r);
  }
  return rows;
}

// Independent binary16 decoding with ldexp.
double HalfReference(uint16_t h) {
  const int sign = (h >> 15) & 1;
  const int exponent = (h >> 10) & 31;
  const int mantissa = h & 1023;
  double v;
  if (exponent == 0) {
    v = std::ldexp(static_cast<double>(mantissa), -24);
  } else if (exponent == 31) {
    v = mantissa ? NAN : INFINITY;
  } else {
    v = std::ldexp(1.0 + mantissa / 1024.0, exponent - 15);
  }
  return sign ? -v : v;
}

TEST(MlFp16, AllValuesMatchLdexpDecoding) {
  for (uint32_t h = 0; h < 65536; ++h) {
    const float got = kernels::HalfToFloat(static_cast<uint16_t>(h));
    const double want = HalfReference(static_cast<uint16_t>(h));
    if (std::isnan(want)) {
      EXPECT_TRUE(std::isnan(got)) << h;
    } else {
      EXPECT_EQ(static_cast<double>(got), want) << h;
      EXPECT_EQ(std::signbit(got), std::signbit(want)) << h;
    }
  }
}

TEST(MlFlexBuffers, ReadsTheFaceMeshTransformTensorBilinearOptions) {
  // custom_options of op 195 of face-mesh-model.tflite (written by the FlexBuffers library;
  // 1-byte widths): {output_height: 16, output_width: 16}.
  const std::vector<uint8_t> blob = {
      0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x5f, 0x68, 0x65, 0x69, 0x67, 0x68, 0x74, 0x00,
      0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x5f, 0x77, 0x69, 0x64, 0x74, 0x68, 0x00, 0x02,
      0x1c, 0x0f, 0x02, 0x01, 0x02, 0x10, 0x10, 0x04, 0x04, 0x04, 0x24, 0x01};
  FlexMap map;
  std::string error;
  ASSERT_TRUE(ParseFlexMap(blob.data(), blob.size(), &map, &error)) << error;
  ASSERT_EQ(map.size(), 2u);
  int64_t h = 0;
  int64_t w = 0;
  EXPECT_TRUE(FlexGetInt(map, "output_height", &h));
  EXPECT_TRUE(FlexGetInt(map, "output_width", &w));
  EXPECT_EQ(h, 16);
  EXPECT_EQ(w, 16);
}

TEST(MlFlexBuffers, ReadsScalarsAndIntVectorsWithFourByteWidths) {
  // tools/ml_ref/tflite_writer.py flex_map({'alpha': 3, 'beta': 1.5, 'gamma': [1, -2, 300000],
  // 'delta': -7}).
  const std::vector<uint8_t> blob = {
      0x61, 0x6c, 0x70, 0x68, 0x61, 0x00, 0x62, 0x65, 0x74, 0x61, 0x00, 0x64, 0x65, 0x6c, 0x74,
      0x61, 0x00, 0x67, 0x61, 0x6d, 0x6d, 0x61, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x93, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x2c,
      0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00, 0x27, 0x00, 0x00, 0x00,
      0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
      0x00, 0x00, 0x00, 0xc0, 0x3f, 0xf9, 0xff, 0xff, 0xff, 0x38, 0x00, 0x00, 0x00, 0x06, 0x0e,
      0x06, 0x2e, 0x14, 0x00, 0x00, 0x00, 0x26, 0x04};
  FlexMap map;
  std::string error;
  ASSERT_TRUE(ParseFlexMap(blob.data(), blob.size(), &map, &error)) << error;
  int64_t a = 0;
  int64_t d = 0;
  double b = 0.0;
  std::vector<int64_t> g;
  EXPECT_TRUE(FlexGetInt(map, "alpha", &a));
  EXPECT_TRUE(FlexGetFloat(map, "beta", &b));
  EXPECT_TRUE(FlexGetInt(map, "delta", &d));
  EXPECT_TRUE(FlexGetIntVector(map, "gamma", &g));
  EXPECT_EQ(a, 3);
  EXPECT_EQ(b, 1.5);
  EXPECT_EQ(d, -7);
  EXPECT_EQ(g, (std::vector<int64_t>{1, -2, 300000}));
  EXPECT_FALSE(FlexGetInt(map, "beta", &a));  // float is not an int
  EXPECT_FALSE(FlexGetInt(map, "missing", &a));
  // Every truncation is rejected or parsed without reading outside the blob.
  for (size_t n = 0; n < blob.size(); ++n) {
    FlexMap m;
    std::string e;
    const bool ok = ParseFlexMap(blob.data(), n, &m, &e);
    if (!ok) {
      EXPECT_FALSE(e.empty()) << n;
    }
  }
}

TEST(MlParser, RejectsMalformedHeaders) {
  std::string error;
  EXPECT_EQ(TfliteModel::FromBytes({}, &error), nullptr);
  EXPECT_EQ(TfliteModel::FromBytes({1, 2, 3, 4, 'T', 'F', 'L', '2'}, &error), nullptr);
  EXPECT_EQ(TfliteModel::FromBytes({200, 0, 0, 0, 'T', 'F', 'L', '3'}, &error), nullptr);
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(TfliteModel::FromFile(kCases + "/does-not-exist.tflite", &error), nullptr);
}

TEST(MlParser, RejectsUnsupportedModelsWithAReason) {
  struct Expect {
    const char* file;
    const char* reason;
  };
  const Expect cases[] = {{"unsupported_builtin.tflite", "unsupported builtin op"},
                          {"unsupported_custom.tflite", "unsupported custom op"},
                          {"bad_shape.tflite", "computed output shape [1,6,6,2] != declared"}};
  for (const Expect& c : cases) {
    std::string error;
    std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(kCases + "/" + c.file, &error);
    ASSERT_NE(model, nullptr) << c.file << ": " << error;
    EXPECT_EQ(Interpreter::Create(*model, &error), nullptr) << c.file;
    EXPECT_NE(error.find(c.reason), std::string::npos) << c.file << ": " << error;
  }
}

// Truncated and byte-flipped models: parsing and Create either fail with a message or yield a
// model that runs; nothing reads outside the file (the run would crash otherwise).
TEST(MlParser, SurvivesTruncationAndByteFlips) {
  for (const char* name : {"conv_same_s1_k3_relu", "transform_tensor_bilinear", "ear_argmax_chain",
                           "landmarks_to_matrix_odd_subset"}) {
    const std::vector<uint8_t> bytes = ReadFile(kCases + "/" + name + ".tflite");
    ASSERT_FALSE(bytes.empty()) << name;
    size_t parsed = 0;
    size_t ran = 0;
    for (size_t n = 0; n < bytes.size(); ++n) {
      std::string error;
      std::vector<uint8_t> cut(bytes.begin(), bytes.begin() + n);
      std::unique_ptr<TfliteModel> m = TfliteModel::FromBytes(cut, &error);
      if (m) ++parsed;
      if (!m) {
        EXPECT_FALSE(error.empty());
      }
    }
    for (size_t i = 0; i < bytes.size(); ++i) {
      std::vector<uint8_t> flipped = bytes;
      flipped[i] ^= 0xff;
      std::string error;
      std::unique_ptr<TfliteModel> m = TfliteModel::FromBytes(flipped, &error);
      if (!m) continue;
      std::unique_ptr<Interpreter> it = Interpreter::Create(*m, &error);
      if (!it) continue;
      for (size_t k = 0; k < it->num_inputs(); ++k) {
        float* in = it->input_data(k);
        for (size_t j = 0; j < it->input_count(k); ++j) in[j] = 0.5f;
      }
      ASSERT_TRUE(it->Invoke(&error)) << error;
      ++ran;
    }
    RecordProperty(std::string(name) + "_truncations_parsed", static_cast<int>(parsed));
    RecordProperty(std::string(name) + "_flips_ran", static_cast<int>(ran));
  }
}

void PrintTo(const CaseRow& r, std::ostream* os) { *os << r.name; }

class MlOpCase : public ::testing::TestWithParam<CaseRow> {};

TEST_P(MlOpCase, MatchesNumpyReference) {
  const CaseRow& row = GetParam();
  const std::string base = kCases + "/" + row.name;
  std::string error;
  std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(base + ".tflite", &error);
  ASSERT_NE(model, nullptr) << error;
  // Parser: canonical graph text equals the numpy parser's.
  EXPECT_EQ(CanonicalGraphText(*model), ReadText(base + ".graph.txt"));

  std::unique_ptr<Interpreter> interp = Interpreter::Create(*model, &error);
  ASSERT_NE(interp, nullptr) << error;
  std::vector<BundleTensor> bundle;
  ASSERT_TRUE(ReadTensorBundle(base + ".srt", &bundle, &error)) << error;
  std::vector<const BundleTensor*> ins;
  std::vector<const BundleTensor*> outs;
  for (const BundleTensor& t : bundle) {
    if (t.name.rfind("in:", 0) == 0) ins.push_back(&t);
    if (t.name.rfind("out:", 0) == 0) outs.push_back(&t);
  }
  ASSERT_EQ(ins.size(), interp->num_inputs());
  ASSERT_EQ(outs.size(), interp->num_outputs());
  for (size_t k = 0; k < ins.size(); ++k) {
    ASSERT_EQ(ins[k]->shape, interp->input_shape(k));
    float* dst = interp->input_data(k);
    for (size_t i = 0; i < ins[k]->values.size(); ++i) {
      dst[i] = static_cast<float>(ins[k]->values[i]);
    }
  }
  ASSERT_TRUE(interp->Invoke(&error)) << error;
  std::vector<std::vector<uint8_t>> first;
  for (size_t o = 0; o < outs.size(); ++o) {
    const TensorView v = interp->output(o);
    const OutputError e = CompareOutput(v, *outs[o]);
    char ratio[64];
    std::snprintf(ratio, sizeof(ratio), "%.3e",
                  e.max_abs / (kOpTolerance * ScaleFloor(e.ref_max_abs)));
    RecordProperty("out" + std::to_string(o) + "_max_abs_over_tol", ratio);
    RecordProperty("out" + std::to_string(o) + "_bit_mismatches", static_cast<int>(e.bit_mismatches));
    if (row.cls == "E") {
      // Class E is bit identity, +0 and -0 included (RM-1b). The numpy RELU is the TFLite
      // expression `val < lower ? lower : val` since RM-1c, so -0 stays -0 like in the engine; there
      // is no case-specific exception any more.
      EXPECT_TRUE(PassesExact(e)) << row.name << " out " << o << ": " << DescribeError(e);
    } else {
      ASSERT_EQ(row.cls, "F");
      EXPECT_TRUE(PassesOpTolerance(e)) << row.name << " out " << o << ": " << DescribeError(e);
    }
    const uint8_t* p = v.f32 ? reinterpret_cast<const uint8_t*>(v.f32)
                     : v.i32 ? reinterpret_cast<const uint8_t*>(v.i32)
                             : reinterpret_cast<const uint8_t*>(v.i64);
    const size_t bytes = v.count * (v.type == TensorType::kInt64 ? 8 : 4);
    first.emplace_back(p, p + (bytes == 0 ? 0 : bytes));
  }
  // Determinism: a second run gives bit-identical outputs.
  ASSERT_TRUE(interp->Invoke(&error)) << error;
  for (size_t o = 0; o < outs.size(); ++o) {
    const TensorView v = interp->output(o);
    const uint8_t* p = v.f32 ? reinterpret_cast<const uint8_t*>(v.f32)
                     : v.i32 ? reinterpret_cast<const uint8_t*>(v.i32)
                             : reinterpret_cast<const uint8_t*>(v.i64);
    if (!first[o].empty()) {  // zero-size outputs have no bytes to compare
      EXPECT_EQ(std::memcmp(p, first[o].data(), first[o].size()), 0) << row.name << " out " << o;
    }
  }
}

std::string CaseName(const ::testing::TestParamInfo<CaseRow>& info) {
  std::string n = info.param.name;
  for (char& c : n) {
    if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
  }
  return n;
}

INSTANTIATE_TEST_SUITE_P(MlOps, MlOpCase, ::testing::ValuesIn(ReadCases()), CaseName);

TEST(MlOpCoverage, EveryOpOfTheSevenModelsHasACase) {
  const char* ops[] = {"CONV_2D",       "DEPTHWISE_CONV_2D", "ADD",          "SUB",
                       "MUL",           "RELU",              "PRELU",        "LEAKY_RELU",
                       "HARD_SWISH",    "LOGISTIC",          "PAD",          "RESHAPE",
                       "MAX_POOL_2D",   "CONCATENATION",     "DEQUANTIZE",   "STRIDED_SLICE",
                       "RESIZE_BILINEAR", "MEAN",            "MINIMUM",      "REDUCE_MAX",
                       "ARG_MAX",       "CAST",              "TRANSPOSE",    "FLOOR_MOD",
                       "TransformLandmarks", "Landmarks2TransformMatrix",
                       "TransformTensorBilinear", "Convolution2DTransposeBias"};
  const std::vector<CaseRow> rows = ReadCases();
  for (const char* op : ops) {
    bool found = false;
    for (const CaseRow& r : rows) found = found || r.op == op;
    EXPECT_TRUE(found) << op;
  }
  EXPECT_EQ(sizeof(ops) / sizeof(ops[0]), 28u);
}

TEST(MlInterpreter, PlansMemoryWithBufferReuse) {
  std::string error;
  std::unique_ptr<TfliteModel> model =
      TfliteModel::FromFile(kCases + "/chain_conv_pool_concat.tflite", &error);
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<Interpreter> interp = Interpreter::Create(*model, &error);
  ASSERT_NE(interp, nullptr) << error;
  size_t activations = 0;
  for (const TensorDef& t : model->subgraphs()[0].tensors) {
    if (t.data == nullptr) activations += kernels::NumElements(t.shape) * sizeof(float);
  }
  EXPECT_LT(interp->arena_bytes(), activations);
  EXPECT_EQ(interp->num_planned_ops(), 6u);
}

// ---- RM-1 / review M4: hostile models --------------------------------------------------------------

struct RejectRow {
  std::string name;
  std::string expect;
};

std::vector<RejectRow> ReadRejects() {
  std::vector<RejectRow> rows;
  std::ifstream in(kCases + "/rejects.tsv");
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    RejectRow r;
    std::getline(ls, r.name, '\t');
    std::getline(ls, r.expect, '\t');
    rows.push_back(r);
  }
  return rows;
}

// Well-formed flatbuffers whose constants or shapes would overflow 32-bit arithmetic or ask for an
// absurd allocation. Before the hardening the first one passed Create and crashed Invoke with a
// segmentation fault (reviewer reproduction, int64 paddings 2^31 and 2^31 + 1).
TEST(MlHardening, CraftedModelsAreRejectedAtCreateWithAReason) {
  const std::vector<RejectRow> rows = ReadRejects();
  ASSERT_GE(rows.size(), 16u);
  for (const RejectRow& r : rows) {
    std::string error;
    std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(kCases + "/" + r.name + ".tflite", &error);
    ASSERT_NE(model, nullptr) << r.name << " must parse (it is well-formed): " << error;
    EXPECT_EQ(Interpreter::Create(*model, &error), nullptr) << r.name << " was accepted";
    EXPECT_NE(error.find(r.expect), std::string::npos)
        << r.name << ": expected '" << r.expect << "' in '" << error << "'";
  }
}

// FLOOR_MOD of two runtime integers: a divisor of -1 must not evaluate INT_MIN % -1, and a divisor
// of 0 is reported by Invoke instead of trapping (TFLite: "Division by 0").
TEST(MlHardening, FloorModWithDataDependentDivisor) {
  std::string error;
  std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(kCases + "/floor_mod_dynamic.tflite", &error);
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<Interpreter> it = Interpreter::Create(*model, &error);
  ASSERT_NE(it, nullptr) << error;
  ASSERT_EQ(it->num_inputs(), 2u);
  const float a[4] = {7.0f, -7.0f, -2147483648.0f, 5.0f};
  float b[4] = {3.0f, 3.0f, -1.0f, 2.0f};
  std::memcpy(it->input_data(0), a, sizeof(a));
  std::memcpy(it->input_data(1), b, sizeof(b));
  ASSERT_TRUE(it->Invoke(&error)) << error;
  const TensorView y = it->output(0);
  ASSERT_EQ(y.count, 4u);
  EXPECT_EQ(y.i32[0], 1);   // 7 mod 3
  EXPECT_EQ(y.i32[1], 2);   // -7 mod 3 (sign of the divisor)
  EXPECT_EQ(y.i32[2], 0);   // INT_MIN mod -1
  EXPECT_EQ(y.i32[3], 1);   // 5 mod 2
  b[1] = 0.0f;
  std::memcpy(it->input_data(1), b, sizeof(b));
  EXPECT_FALSE(it->Invoke(&error));
  EXPECT_NE(error.find("division by 0"), std::string::npos) << error;
}

struct FamilyRow {
  std::string name;
  double lo = 0.0;
  double hi = 0.0;
};

std::vector<FamilyRow> ReadFamily() {
  std::vector<FamilyRow> rows;
  std::ifstream in(kCases + "/family.tsv");
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    FamilyRow r;
    std::string lo;
    std::string hi;
    std::getline(ls, r.name, '\t');
    std::getline(ls, lo, '\t');
    std::getline(ls, hi, '\t');
    r.lo = std::stod(lo);
    r.hi = std::stod(hi);
    rows.push_back(r);
  }
  return rows;
}

// Fills every graph input with a deterministic pattern in [0.25, 0.75].
void FillInputs(Interpreter* it) {
  for (size_t k = 0; k < it->num_inputs(); ++k) {
    float* in = it->input_data(k);
    for (size_t j = 0; j < it->input_count(k); ++j) {
      in[j] = 0.25f + 0.5f * static_cast<float>((j * 7919u + k * 31u) % 1000u) / 999.0f;
    }
  }
}

// RM-1b, review MAJOR-1: models whose declared shapes, constants and options were changed together
// (gen_op_cases.py _family_models). Before the fix the legacy RESIZE_BILINEAR rows read one element
// past the input (ASan: heap-buffer-overflow in kernels.cc ResizeBilinear). A model is either
// rejected with a message or runs, and then every output is finite and inside the stated range.
TEST(MlHardening, StructureAwareFamilyModelsAreRejectedOrRunInBounds) {
  const std::vector<FamilyRow> rows = ReadFamily();
  ASSERT_GE(rows.size(), 20u);
  size_t rejected = 0;
  size_t ran = 0;
  for (const FamilyRow& r : rows) {
    std::string error;
    std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(kCases + "/" + r.name + ".tflite", &error);
    ASSERT_NE(model, nullptr) << r.name << " must parse: " << error;
    std::unique_ptr<Interpreter> it = Interpreter::Create(*model, &error);
    if (!it) {
      EXPECT_FALSE(error.empty()) << r.name;
      ++rejected;
      continue;
    }
    EXPECT_LE(it->memory_use().total(), kernels::kMaxTotalBytes) << r.name;
    FillInputs(it.get());
    ASSERT_TRUE(it->Invoke(&error)) << r.name << ": " << error;
    for (size_t o = 0; o < it->num_outputs(); ++o) {
      const TensorView v = it->output(o);
      ASSERT_NE(v.f32, nullptr) << r.name;
      size_t outside = 0;
      size_t first = 0;
      for (size_t i = 0; i < v.count; ++i) {
        const double x = v.f32[i];
        if (!(x >= r.lo && x <= r.hi)) {  // also catches NaN
          if (outside == 0) first = i;
          ++outside;
        }
      }
      EXPECT_EQ(outside, 0u) << r.name << ": " << outside << " outputs outside [" << r.lo << ", " << r.hi
                             << "], first at " << first << " = " << v.f32[first];
    }
    ++ran;
  }
  // Accepted and rejected models both exist (the memory limit is exercised from both sides).
  EXPECT_GE(rejected, 3u);
  EXPECT_GE(ran, 15u);
  RecordProperty("family_rejected", static_cast<int>(rejected));
  RecordProperty("family_ran", static_cast<int>(ran));
}

// The exact reproduction of the review: [1,1,1,1] resized to [1, 2^25]. The output alone is 128 MiB,
// so the total memory limit refuses it at Create (the defect itself is covered by the family above).
TEST(MlHardening, ReviewerResizeReproductionIsRefusedByTheMemoryLimit) {
  std::string error;
  std::unique_ptr<TfliteModel> model =
      TfliteModel::FromFile(kCases + "/reject_resize_legacy_2pow25_outputs.tflite", &error);
  ASSERT_NE(model, nullptr) << error;
  EXPECT_EQ(Interpreter::Create(*model, &error), nullptr);
  EXPECT_NE(error.find("exceeds the limit"), std::string::npos) << error;
}

// ResizeBilinear against the TFLite reference formula written out independently (reference_ops.h
// ResizeBilinear without the clamp added in RM-1b): wherever the reference index is in range, the
// kernel is bit-identical; the index is in range for every size pair tested (so the clamp never
// changes a result there).
TEST(MlKernels, ResizeBilinearClampDoesNotChangeInRangeResults) {
  struct Mode {
    bool align_corners;
    bool half_pixel;
  };
  size_t pairs = 0;
  size_t out_of_range = 0;
  for (const Mode mode : {Mode{false, false}, Mode{true, false}, Mode{false, true}}) {
    for (int in_h = 1; in_h <= 12; ++in_h) {
      for (int out_h = 1; out_h <= 14; ++out_h) {
        for (int in_w = 1; in_w <= 12; ++in_w) {
          for (int out_w = 1; out_w <= 14; ++out_w) {
            std::vector<float> in(static_cast<size_t>(in_h) * in_w);
            for (size_t i = 0; i < in.size(); ++i) in[i] = 0.1f * static_cast<float>(i % 17) - 0.7f;
            std::vector<float> got(static_cast<size_t>(out_h) * out_w);
            kernels::ResizeBilinear(in.data(), {1, in_h, in_w, 1}, out_h, out_w, mode.align_corners,
                                    mode.half_pixel, got.data());
            float hs = static_cast<float>(in_h) / out_h;
            float ws = static_cast<float>(in_w) / out_w;
            if (mode.align_corners && out_h > 1) hs = static_cast<float>(in_h - 1) / (out_h - 1);
            if (mode.align_corners && out_w > 1) ws = static_cast<float>(in_w - 1) / (out_w - 1);
            auto interp = [&](int v, float scale, int size, float* sv, int* lo, int* hi) {
              *sv = mode.half_pixel ? (static_cast<float>(v) + 0.5f) * scale - 0.5f
                                    : static_cast<float>(v) * scale;
              *lo = std::max(static_cast<int>(std::floor(*sv)), 0);
              *hi = std::min(static_cast<int>(std::ceil(*sv)), size - 1);
            };
            for (int y = 0; y < out_h; ++y) {
              float iy;
              int y0;
              int y1;
              interp(y, hs, in_h, &iy, &y0, &y1);
              for (int x = 0; x < out_w; ++x) {
                float ix;
                int x0;
                int x1;
                interp(x, ws, in_w, &ix, &x0, &x1);
                if (y0 >= in_h || x0 >= in_w) {
                  ++out_of_range;
                  continue;
                }
                const float dy = iy - y0;
                const float dx = ix - x0;
                const float want = in[y0 * in_w + x0] * (1 - dy) * (1 - dx) +
                                   in[y1 * in_w + x0] * dy * (1 - dx) +
                                   in[y0 * in_w + x1] * (1 - dy) * dx + in[y1 * in_w + x1] * dy * dx;
                const float have = got[static_cast<size_t>(y) * out_w + x];
                ASSERT_EQ(std::memcmp(&want, &have, sizeof(float)), 0)
                    << in_h << "x" << in_w << " -> " << out_h << "x" << out_w << " at " << y << "," << x;
              }
            }
            ++pairs;
          }
        }
      }
    }
  }
  EXPECT_EQ(out_of_range, 0u);
  RecordProperty("size_pairs", static_cast<int>(pairs));
}

// ---- RM-5: the deterministic GEMM -----------------------------------------------------------------

// The accumulation order promised in gemm.h, written as plain scalar code.
void ScalarGemm(const float* a, int m, int k, const float* w, int n, float* c) {
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      float total = 0.0f;
      for (int k0 = 0; k0 < k; k0 += gemm::kKBlock) {
        const int k1 = std::min(k, k0 + gemm::kKBlock);
        float block = 0.0f;
        for (int kk = k0; kk < k1; ++kk) block = block + a[i * k + kk] * w[j * k + kk];
        total = total + block;
      }
      c[i * n + j] = total;
    }
  }
}

TEST(MlGemm, MatchesTheScalarOrderBitForBitForEveryTileShape) {
  uint32_t state = 12345u;
  auto rnd = [&state]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int32_t>(state >> 8) - (1 << 23)) / static_cast<float>(1 << 23);
  };
  size_t cases = 0;
  for (const int m : {1, 2, 3, 4, 5, 7, 8, 63, 64, 65, 130}) {
    for (const int n : {1, 3, 7, 8, 9, 16, 17, 24}) {
      for (const int k : {1, 2, 31, 32, 33, 63, 64, 65, 100, 288}) {
        std::vector<float> a(static_cast<size_t>(m) * k);
        std::vector<float> w(static_cast<size_t>(n) * k);
        for (float& v : a) v = rnd();
        for (float& v : w) v = rnd();
        std::vector<float> packed(gemm::PackedSize(n, k));
        gemm::PackWeights(w.data(), n, k, static_cast<size_t>(k), packed.data());
        std::vector<float> want(static_cast<size_t>(m) * n);
        std::vector<float> got(static_cast<size_t>(m) * n, -123.0f);
        ScalarGemm(a.data(), m, k, w.data(), n, want.data());
        gemm::Gemm(a.data(), m, k, static_cast<size_t>(k), packed.data(), n, got.data(),
                   static_cast<size_t>(n));
        ASSERT_EQ(std::memcmp(want.data(), got.data(), want.size() * sizeof(float)), 0)
            << "m=" << m << " n=" << n << " k=" << k;
        // One row at a time takes the other micro-kernel path: same bits.
        for (int i = 0; i < m; ++i) {
          std::vector<float> row(static_cast<size_t>(n), -1.0f);
          gemm::Gemm(a.data() + static_cast<size_t>(i) * k, 1, k, static_cast<size_t>(k), packed.data(), n,
                     row.data(), static_cast<size_t>(n));
          ASSERT_EQ(std::memcmp(row.data(), want.data() + static_cast<size_t>(i) * n, sizeof(float) * n), 0)
              << "row " << i << " m=" << m << " n=" << n << " k=" << k;
        }
        ++cases;
      }
    }
  }
  RecordProperty("gemm_shapes", static_cast<int>(cases));
}

// A fused multiply-add rounds once where the promised order rounds twice. With a = {1, 1 + 2^-12} and
// w = {-(1 + 2^-11), 1 + 2^-12} the second step is (-(1 + 2^-11)) + (1 + 2^-12)^2: rounded separately
// the product is 1 + 2^-11 and the sum is exactly 0; fused it is 2^-24.
TEST(MlGemm, MultiplyAndAddAreRoundedSeparately) {
  const float a[2] = {1.0f, 1.0f + 0x1p-12f};
  const float w[2] = {-(1.0f + 0x1p-11f), 1.0f + 0x1p-12f};
  float packed[gemm::kNr * 2];
  gemm::PackWeights(w, 1, 2, 2, packed);
  float c[1] = {-1.0f};
  gemm::Gemm(a, 1, 2, 2, packed, 1, c, 1);
  EXPECT_EQ(c[0], 0.0f) << "the build fuses multiply and add (-ffp-contract=off is missing for gemm.cc)";
  // Same property through the 4-row micro kernel.
  float a4[8];
  for (int r = 0; r < 4; ++r) {
    a4[2 * r] = a[0];
    a4[2 * r + 1] = a[1];
  }
  float c4[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
  gemm::Gemm(a4, 4, 2, 2, packed, 1, c4, 1);
  for (int r = 0; r < 4; ++r) EXPECT_EQ(c4[r], 0.0f) << r;
}

// DEPTHWISE_CONV_2D against the loop it replaced (accumulate the in-image taps in (ky, kx) order,
// then bias and clamp): bit-identical, incl. padding, stride, dilation and channel counts that
// exercise the 16-, 4- and 1-channel paths.
TEST(MlKernels, DepthwiseMatchesTheScalarLoopBitForBit) {
  uint32_t state = 777u;
  auto rnd = [&state]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int32_t>(state >> 8) - (1 << 23)) / static_cast<float>(1 << 23);
  };
  size_t cases = 0;
  for (const int channels : {1, 3, 4, 5, 16, 17, 20, 36}) {
    for (const int kernel : {1, 3, 5}) {
      for (const int stride : {1, 2}) {
        for (const int dilation : {1, 2}) {
          for (const bool same : {true, false}) {
            kernels::ConvGeometry g;
            g.batch = 1;
            g.in_h = 9;
            g.in_w = 7;
            g.in_c = channels;
            g.out_c = channels;
            g.kernel_h = kernel;
            g.kernel_w = kernel;
            g.stride_h = stride;
            g.stride_w = stride;
            g.dilation_h = dilation;
            g.dilation_w = dilation;
            const Padding pad = same ? Padding::kSame : Padding::kValid;
            g.out_h = kernels::ComputeOutSize(pad, g.in_h, kernel, stride, dilation);
            g.out_w = kernels::ComputeOutSize(pad, g.in_w, kernel, stride, dilation);
            if (g.out_h <= 0 || g.out_w <= 0) continue;
            g.pad_top = kernels::ComputePadBefore(pad, g.in_h, g.out_h, kernel, stride, dilation);
            g.pad_left = kernels::ComputePadBefore(pad, g.in_w, g.out_w, kernel, stride, dilation);
            std::vector<float> in(static_cast<size_t>(g.in_h) * g.in_w * channels);
            std::vector<float> w(static_cast<size_t>(kernel) * kernel * channels);
            std::vector<float> bias(static_cast<size_t>(channels));
            for (float& v : in) v = rnd();
            for (float& v : w) v = rnd();
            for (float& v : bias) v = rnd();
            const kernels::ActivationRange act{0.0f, 6.0f};
            std::vector<float> got(static_cast<size_t>(g.out_h) * g.out_w * channels);
            kernels::DepthwiseConv2D(g, 1, in.data(), w.data(), bias.data(), act, got.data());
            for (int oy = 0; oy < g.out_h; ++oy) {
              for (int ox = 0; ox < g.out_w; ++ox) {
                for (int c = 0; c < channels; ++c) {
                  float acc = 0.0f;
                  for (int ky = 0; ky < kernel; ++ky) {
                    const int iy = oy * stride - g.pad_top + ky * dilation;
                    if (iy < 0 || iy >= g.in_h) continue;
                    for (int kx = 0; kx < kernel; ++kx) {
                      const int ix = ox * stride - g.pad_left + kx * dilation;
                      if (ix < 0 || ix >= g.in_w) continue;
                      acc += in[(static_cast<size_t>(iy) * g.in_w + ix) * channels + c] *
                             w[(static_cast<size_t>(ky) * kernel + kx) * channels + c];
                    }
                  }
                  const float want = std::min(std::max(acc + bias[c], act.min), act.max);
                  const float have = got[(static_cast<size_t>(oy) * g.out_w + ox) * channels + c];
                  ASSERT_EQ(std::memcmp(&want, &have, sizeof(float)), 0)
                      << "c=" << channels << " k=" << kernel << " s=" << stride << " d=" << dilation
                      << " same=" << same << " at " << oy << "," << ox << "," << c;
                }
              }
            }
            ++cases;
          }
        }
      }
    }
  }
  RecordProperty("depthwise_configs", static_cast<int>(cases));
}

// ---- RM-1 / RM-1b: fuzzing helpers -----------------------------------------------------------------

// The family models allocate up to 128 MiB per variant, so their byte flips cost minutes: they are
// part of the fuzz only when SR_ML_FUZZ_FAMILY is set (the sanitizer runs of docs/design/ml-runtime.md
// section 8.9 set it); StructureAwareFamilyModelsAreRejectedOrRunInBounds always covers them.
std::vector<std::string> AllCaseModelNames() {
  std::vector<std::string> names;
  for (const CaseRow& r : ReadCases()) names.push_back(r.name);
  for (const RejectRow& r : ReadRejects()) names.push_back(r.name);
  if (std::getenv("SR_ML_FUZZ_FAMILY") != nullptr) {
    for (const FamilyRow& r : ReadFamily()) names.push_back(r.name);
  }
  for (const char* n : {"unsupported_builtin", "unsupported_custom", "bad_shape",
                        "floor_mod_dynamic"}) {
    names.push_back(n);
  }
  return names;
}

// Every byte of every checked-in model flipped three ways (XOR 0xff, 0x01, 0x80) and every
// truncation: parsing and Create either fail with a message or yield a model that runs. Without a
// sanitizer a stray read or write shows up only if it crashes; the ASan / UBSan build of this
// test (docs/design/ml-runtime.md section 8.6) is what makes it a memory-safety check.
//
// RM-1b: a mutated model is always run when its activations are at most 32 MiB; a bigger one is run
// while the sum of such activations stays within SR_ML_FUZZ_BUDGET_MIB (default 1024). The old rule
// skipped every model above 32 MiB, which is where the resize defect lived. Files above 16 KiB (the
// shared-buffer rejection model) are not byte-flipped, each variant would cost milliseconds.
TEST(MlParser, EveryCaseModelSurvivesByteFlipsAndTruncation) {
  const char* budget_env = std::getenv("SR_ML_FUZZ_BUDGET_MIB");
  uint64_t budget = (budget_env != nullptr ? std::strtoull(budget_env, nullptr, 10) : 1024ull) << 20;
  size_t variants = 0;
  size_t parsed = 0;
  size_t created = 0;
  size_t invoked = 0;
  size_t invoked_large = 0;
  size_t skipped_large = 0;
  size_t skipped_files = 0;
  for (const std::string& name : AllCaseModelNames()) {
    const std::vector<uint8_t> bytes = ReadFile(kCases + "/" + name + ".tflite");
    ASSERT_FALSE(bytes.empty()) << name;
    if (bytes.size() > 16384) {
      ++skipped_files;
      continue;
    }
    for (size_t n = 0; n < bytes.size(); ++n) {
      std::string error;
      const std::unique_ptr<TfliteModel> m =
          TfliteModel::FromBytes(std::vector<uint8_t>(bytes.begin(), bytes.begin() + n), &error);
      ++variants;
      if (!m) {
        ASSERT_FALSE(error.empty()) << name << " truncated to " << n;
      } else {
        ++parsed;
        Interpreter::Create(*m, &error);
      }
    }
    for (const uint8_t flip : {uint8_t{0xff}, uint8_t{0x01}, uint8_t{0x80}}) {
      for (size_t i = 0; i < bytes.size(); ++i) {
        std::vector<uint8_t> mutated = bytes;
        mutated[i] ^= flip;
        std::string error;
        std::unique_ptr<TfliteModel> m = TfliteModel::FromBytes(std::move(mutated), &error);
        ++variants;
        if (!m) {
          ASSERT_FALSE(error.empty()) << name << " byte " << i;
          continue;
        }
        ++parsed;
        std::unique_ptr<Interpreter> it = Interpreter::Create(*m, &error);
        if (!it) {
          ASSERT_FALSE(error.empty()) << name << " byte " << i;
          continue;
        }
        ++created;
        const uint64_t cost = it->memory_use().activations;
        if (cost > (uint64_t{32} << 20)) {
          if (cost > budget) {
            ++skipped_large;  // valid but beyond the remaining budget; Create alone was exercised
            continue;
          }
          budget -= cost;
          ++invoked_large;
        }
        for (size_t k = 0; k < it->num_inputs(); ++k) {
          float* in = it->input_data(k);
          for (size_t j = 0; j < it->input_count(k); ++j) in[j] = 0.5f;
        }
        if (!it->Invoke(&error)) {
          EXPECT_NE(error.find("division by 0"), std::string::npos) << name << " byte " << i << ": " << error;
        }
        ++invoked;
      }
    }
  }
  RecordProperty("variants", static_cast<int>(variants));
  RecordProperty("parsed", static_cast<int>(parsed));
  RecordProperty("created", static_cast<int>(created));
  RecordProperty("invoked", static_cast<int>(invoked));
  RecordProperty("invoked_above_32mib", static_cast<int>(invoked_large));
  RecordProperty("skipped_over_budget", static_cast<int>(skipped_large));
  RecordProperty("skipped_files_over_16kib", static_cast<int>(skipped_files));
  EXPECT_GT(created, 1000u);
}

// RM-1b, review MINOR-1: the exactness class compares bits. +0 and -0 differ; two NaNs agree.
TEST(MlCompare, BitComparisonSeesTheSignOfZeroAndTreatsAllNaNsAlike) {
  const float nan_neg = std::copysign(std::nanf(""), -1.0f);
  const float got_values[6] = {0.0f, -0.0f, 1.5f, nan_neg, INFINITY, -0.0f};
  TensorView got;
  got.type = TensorType::kFloat32;
  got.shape = {6};
  got.count = 6;
  got.f32 = got_values;
  BundleTensor ref;
  ref.type = TensorType::kFloat64;
  ref.shape = {6};
  ref.values = {0.0, 0.0, 1.5, std::nan(""), INFINITY, -0.0};
  const OutputError e = CompareOutput(got, ref);
  ASSERT_TRUE(e.shape_ok && e.type_ok);
  EXPECT_EQ(e.bit_mismatches, 1u);        // element 1: -0 against +0
  EXPECT_EQ(e.zero_sign_mismatches, 1u);
  EXPECT_FALSE(PassesExact(e));
  ref.values[1] = -0.0;
  EXPECT_TRUE(PassesExact(CompareOutput(got, ref)));
}

// ---- RM-1: the relative regression gate table ---------------------------------------------------

TEST(MlRegressionGate, TableCoversEveryOutputAndAppliesTheRegisteredRule) {
  std::vector<GateRow> rows;
  std::string error;
  ASSERT_TRUE(LoadRegressionGate(kGateTsv, &rows, &error)) << error;
  EXPECT_EQ(rows.size(), 38u);  // 7 models x 2 inputs x their outputs (4+14+6+2+4+4+4)
  for (const GateRow& r : rows) {
    EXPECT_TRUE(std::isfinite(r.baseline_max_abs) && r.baseline_max_abs >= 0.0) << r.model;
    EXPECT_TRUE(std::isfinite(r.baseline_rms) && r.baseline_rms >= 0.0) << r.model;
    EXPECT_GT(r.ref_max_abs, 0.0) << r.model << " " << r.output;
    // The resolution is never below one float32 rounding step of the output scale (OR-3 floor).
    EXPECT_GE(r.resolution_max_abs, kGateFloat32Eps * r.ref_max_abs) << r.model << " " << r.output;
    EXPECT_GE(r.resolution_rms, kGateFloat32Eps * r.ref_rms) << r.model << " " << r.output;
  }
  const GateRow* row = FindGateRow(rows, "face-model", 0, "regressors");
  ASSERT_NE(row, nullptr);
  OutputError e;
  e.shape_ok = e.type_ok = true;
  e.ref_max_abs = row->ref_max_abs;
  e.ref_rms = row->ref_rms;
  // The error may reach baseline + 3 x value exactly ...
  const double max_limit = row->baseline_max_abs + 3.0 * row->resolution_max_abs;
  const double rms_limit = row->baseline_rms + 3.0 * row->resolution_rms;
  e.max_abs = max_limit;
  e.rms = rms_limit;
  EXPECT_TRUE(EvaluateGate(*row, e).pass);
  // ... one step more fails, for either statistic.
  e.max_abs = std::nextafter(max_limit, INFINITY);
  EXPECT_FALSE(EvaluateGate(*row, e).pass);
  e.max_abs = row->baseline_max_abs;
  e.rms = std::nextafter(rms_limit, INFINITY);
  EXPECT_FALSE(EvaluateGate(*row, e).pass);
  // A wrong shape or type never passes.
  e.rms = row->baseline_rms;
  e.shape_ok = false;
  EXPECT_FALSE(EvaluateGate(*row, e).pass);
  // The observed reference scales must equal the table's (a regenerated reference differs by 7e-15).
  e.shape_ok = e.type_ok = true;
  e.rms = row->baseline_rms;
  e.ref_max_abs = row->ref_max_abs * (1.0 + 1e-14);
  e.ref_rms = row->ref_rms;
  EXPECT_TRUE(EvaluateGate(*row, e).reference_ok);
  e.ref_max_abs = row->ref_max_abs * (1.0 + 1e-9);
  EXPECT_FALSE(EvaluateGate(*row, e).reference_ok);
  EXPECT_FALSE(EvaluateGate(*row, e).pass);
  // An output without a row is not found (the model test then fails; there is no default).
  EXPECT_EQ(FindGateRow(rows, "face-model", 0, "no-such-output"), nullptr);
}

TEST(MlRegressionGate, AggregateRuleMatchesTheTable) {
  std::vector<GateRow> rows;
  GateAggregateLimits limits;
  std::string error;
  ASSERT_TRUE(LoadRegressionGate(kGateTsv, &rows, &error)) << error;
  ASSERT_TRUE(LoadAggregateLimits(kGateAggregateTsv, &limits, &error)) << error;
  size_t in_aggregate = 0;
  for (const GateRow& r : rows) in_aggregate += InAggregate(r) ? 1 : 0;
  EXPECT_EQ(in_aggregate, limits.rows);
  EXPECT_EQ(in_aggregate, 32u);
  // A limit below 1 would fail the registered baseline itself.
  EXPECT_GT(limits.max_abs_limit, 1.0);
  EXPECT_GT(limits.rms_limit, 1.0);
  // The accumulator: all rows at exactly the baseline give 1; all rows at twice the baseline give 2.
  AggregateAccumulator acc;
  AggregateAccumulator doubled;
  for (const GateRow& r : rows) {
    OutputError e;
    e.shape_ok = e.type_ok = true;
    e.max_abs = r.baseline_max_abs;
    e.rms = r.baseline_rms;
    acc.Add(r, e);
    e.max_abs *= 2.0;
    e.rms *= 2.0;
    doubled.Add(r, e);
  }
  EXPECT_EQ(acc.rows, limits.rows);
  EXPECT_NEAR(acc.RmsRatio(), 1.0, 1e-12);
  EXPECT_NEAR(acc.MaxAbsRatio(), 1.0, 1e-12);
  EXPECT_NEAR(doubled.RmsRatio(), 2.0, 1e-12);
  EXPECT_GT(doubled.RmsRatio(), limits.rms_limit);
  EXPECT_GT(doubled.MaxAbsRatio(), limits.max_abs_limit);
}

}  // namespace
}  // namespace ml
}  // namespace slam_refact
