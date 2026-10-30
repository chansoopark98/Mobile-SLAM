// Platform-independent hardening run of the ML interpreter, built for native and WASM (node).
// RM-1d (review ML m3, RM-1 "WASM unit and fuzz not run"): the GoogleTest suite is native only
// (CMakeLists.txt: WASM tests are not GTest), so this driver repeats the three hardening checks of
// tests/ml/ml_ops_test.cc without GoogleTest and prints deterministic counters plus one FNV-1a
// digest of every outcome and every (NaN-canonicalised) output bit pattern. A native and a WASM
// run of the same data are expected to print the same lines; a difference points at a platform
// dependent path (wasm32 size_t, 32-bit overflow, uninitialised memory) or at libm: the L2TM
// custom op calls std::atan2/cos/sin (custom_ops.cc), and one byte-flip variant
// (landmarks_to_matrix_odd_subset byte 635) was observed to differ by 1 ULP between native and
// WASM (ml-runtime.md 8.11; the libm cause is INFERRED, not isolated). So identical lines are an
// observation for this data set, not a property of every libm-using op. It is a REPORT-grade
// diagnostic, not a registered gate (docs/design/ml-lane-closure.md).
//
//   sr_ml_harden --repo <repo root> [--trace <file>]   (rejects, family, byte-flip fuzz; --trace
//   writes one `label digest` line per invoked fuzz variant, to locate a native / WASM difference)
//
// Checks (data: slam_refact/engine/tests/ml/data/op_cases):
//   1. rejects.tsv  every model parses, Create() refuses it, and the message holds the stated text.
//   2. family.tsv   a model is refused with a message, or runs with every output inside [lo, hi].
//   3. fuzz         every byte of every checked-in case model (files up to 16 KiB) XORed with 0xff,
//                   0x01 and 0x80, and every truncation: parse / Create refuse with a message, or
//                   the model runs (activations up to 32 MiB always; larger ones within a 1024 MiB
//                   running budget). Same rules as MlParser.EveryCaseModelSurvivesByteFlipsAndTruncation.
// Exit status 0 only when no check failed. Sanitizers are what make 3 a memory-safety check on
// native; under WASM an out-of-bounds access is a trap or silently wrong, which the digest shows.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ml/interpreter.h"
#include "ml/kernels.h"
#include "ml/tflite_model.h"

namespace {

using slam_refact::ml::Interpreter;
using slam_refact::ml::TensorType;
using slam_refact::ml::TensorView;
using slam_refact::ml::TfliteModel;

uint64_t g_digest = 1469598103934665603ull;  // FNV-1a 64 offset basis
std::FILE* g_trace = nullptr;                // --trace <file>: one line per invoked variant
size_t g_failures = 0;

void Mix(uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    g_digest ^= (v >> (8 * i)) & 0xffu;
    g_digest *= 1099511628211ull;
  }
}

void Fail(const std::string& what) {
  ++g_failures;
  std::fprintf(stderr, "FAIL: %s\n", what.c_str());
}

std::vector<uint8_t> ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
}

std::vector<std::vector<std::string>> ReadTsv(const std::string& path) {
  std::vector<std::vector<std::string>> rows;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> cols;
    std::istringstream ls(line);
    std::string col;
    while (std::getline(ls, col, '\t')) cols.push_back(col);
    rows.push_back(cols);
  }
  return rows;
}

void FillInputs(Interpreter* it) {
  for (size_t k = 0; k < it->num_inputs(); ++k) {
    float* in = it->input_data(k);
    for (size_t j = 0; j < it->input_count(k); ++j) {
      // uint64_t, not size_t: on wasm32 `j * 7919u` wraps for inputs above 542k elements and the
      // WASM run would be fed other data than the native one (found by this driver's first run).
      const uint64_t h = (static_cast<uint64_t>(j) * 7919u + static_cast<uint64_t>(k) * 31u) % 1000u;
      in[j] = 0.25f + 0.5f * static_cast<float>(h) / 999.0f;
    }
  }
}

// Digest of the outputs; every NaN counts as the same pattern (sign and payload of a NaN made by
// an invalid operation are platform dependent, ml-runtime.md section 8.8). Returns the digest of
// these outputs alone (independent of the running digest) and mixes it into the running digest.
uint64_t MixOutputs(const Interpreter& it) {
  uint64_t local = 1469598103934665603ull;
  auto add = [&local](uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      local ^= (v >> (8 * i)) & 0xffu;
      local *= 1099511628211ull;
    }
  };
  for (size_t o = 0; o < it.num_outputs(); ++o) {
    const TensorView v = it.output(o);
    add(v.count);
    for (size_t i = 0; i < v.count; ++i) {
      if (v.f32 != nullptr) {
        uint32_t bits;
        std::memcpy(&bits, &v.f32[i], 4);
        if ((bits & 0x7f800000u) == 0x7f800000u && (bits & 0x007fffffu) != 0) bits = 0x7fc00000u;
        add(bits);
      } else if (v.i32 != nullptr) {
        add(static_cast<uint32_t>(v.i32[i]));
      } else if (v.i64 != nullptr) {
        add(static_cast<uint64_t>(v.i64[i]));
      }
    }
  }
  Mix(local);
  return local;
}

struct Tally {
  size_t variants = 0, parsed = 0, created = 0, invoked = 0, invoked_large = 0, skipped_large = 0;
  size_t skipped_files = 0, invoke_false = 0;
};

void CheckRejects(const std::string& dir) {
  size_t n = 0;
  for (const auto& row : ReadTsv(dir + "/rejects.tsv")) {
    if (row.size() < 2) continue;
    std::string error;
    std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(dir + "/" + row[0] + ".tflite", &error);
    if (!model) {
      Fail("rejects: " + row[0] + " must parse: " + error);
      continue;
    }
    std::unique_ptr<Interpreter> it = Interpreter::Create(*model, &error);
    if (it) {
      Fail("rejects: " + row[0] + " was accepted");
      Mix(1);
      continue;
    }
    if (error.find(row[1]) == std::string::npos) {
      Fail("rejects: " + row[0] + ": expected '" + row[1] + "' in '" + error + "'");
    }
    Mix(2);
    ++n;
  }
  std::printf("rejects: %zu refused at Create with the stated reason\n", n);
}

void CheckFamily(const std::string& dir) {
  size_t rejected = 0;
  size_t ran = 0;
  for (const auto& row : ReadTsv(dir + "/family.tsv")) {
    if (row.size() < 3) continue;
    const double lo = std::strtod(row[1].c_str(), nullptr);
    const double hi = std::strtod(row[2].c_str(), nullptr);
    std::string error;
    std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(dir + "/" + row[0] + ".tflite", &error);
    if (!model) {
      Fail("family: " + row[0] + " must parse: " + error);
      continue;
    }
    std::unique_ptr<Interpreter> it = Interpreter::Create(*model, &error);
    if (!it) {
      if (error.empty()) Fail("family: " + row[0] + " refused without a message");
      Mix(3);
      ++rejected;
      continue;
    }
    if (it->memory_use().total() > slam_refact::ml::kernels::kMaxTotalBytes) {
      Fail("family: " + row[0] + " exceeds the total memory limit");
    }
    FillInputs(it.get());
    if (!it->Invoke(&error)) {
      Fail("family: " + row[0] + " Invoke: " + error);
      continue;
    }
    bool in_range = true;  // C-10: only a model whose outputs all stay in range counts as "ran"
    for (size_t o = 0; o < it->num_outputs(); ++o) {
      const TensorView v = it->output(o);
      if (v.f32 == nullptr) {
        Fail("family: " + row[0] + " has a non-float output");
        in_range = false;
        continue;
      }
      size_t outside = 0;
      for (size_t i = 0; i < v.count; ++i) {
        const double x = v.f32[i];
        if (!(x >= lo && x <= hi)) ++outside;  // also catches NaN
      }
      if (outside != 0) {
        Fail("family: " + row[0] + ": " + std::to_string(outside) + " outputs outside [" +
             row[1] + ", " + row[2] + "]");
        in_range = false;
      }
    }
    Mix(4);
    const uint64_t out = MixOutputs(*it);
    if (g_trace != nullptr) {
      std::fprintf(g_trace, "family %s %016llx\n", row[0].c_str(),
                   static_cast<unsigned long long>(out));
    }
    if (in_range) ++ran;
  }
  std::printf("family: %zu refused, %zu ran inside their range\n", rejected, ran);
}

std::vector<std::string> CaseModelNames(const std::string& dir) {
  std::vector<std::string> names;
  for (const char* f : {"/cases.tsv", "/rejects.tsv"}) {
    for (const auto& row : ReadTsv(dir + f)) names.push_back(row[0]);
  }
  for (const char* n : {"unsupported_builtin", "unsupported_custom", "bad_shape",
                        "floor_mod_dynamic"}) {
    names.push_back(n);
  }
  return names;
}

void Variant(const std::string& label, std::vector<uint8_t> bytes, uint64_t* budget, Tally* t) {
  std::string error;
  ++t->variants;
  std::unique_ptr<TfliteModel> m = TfliteModel::FromBytes(std::move(bytes), &error);
  if (!m) {
    if (error.empty()) Fail("fuzz: " + label + " refused at parse without a message");
    Mix(5);
    return;
  }
  ++t->parsed;
  std::unique_ptr<Interpreter> it = Interpreter::Create(*m, &error);
  if (!it) {
    if (error.empty()) Fail("fuzz: " + label + " refused at Create without a message");
    Mix(6);
    return;
  }
  ++t->created;
  const uint64_t cost = it->memory_use().activations;
  if (cost > (uint64_t{32} << 20)) {
    if (cost > *budget) {
      ++t->skipped_large;
      Mix(7);
      return;
    }
    *budget -= cost;
    ++t->invoked_large;
  }
  for (size_t k = 0; k < it->num_inputs(); ++k) {
    float* in = it->input_data(k);
    for (size_t j = 0; j < it->input_count(k); ++j) in[j] = 0.5f;
  }
  if (!it->Invoke(&error)) {
    ++t->invoke_false;
    if (error.find("division by 0") == std::string::npos) {
      Fail("fuzz: " + label + " Invoke failed with an unexpected message: " + error);
    }
    Mix(8);
    return;
  }
  ++t->invoked;
  Mix(9);
  const uint64_t out = MixOutputs(*it);
  if (g_trace != nullptr) {
    std::fprintf(g_trace, "%s %016llx\n", label.c_str(), static_cast<unsigned long long>(out));
  }
}

void CheckFuzz(const std::string& dir) {
  Tally t;
  uint64_t budget = uint64_t{1024} << 20;
  for (const std::string& name : CaseModelNames(dir)) {
    const std::vector<uint8_t> bytes = ReadFile(dir + "/" + name + ".tflite");
    if (bytes.empty()) {
      Fail("fuzz: cannot read " + name);
      continue;
    }
    if (bytes.size() > 16384) {
      ++t.skipped_files;
      continue;
    }
    for (size_t n = 0; n < bytes.size(); ++n) {
      std::string error;
      ++t.variants;
      std::unique_ptr<TfliteModel> m =
          TfliteModel::FromBytes(std::vector<uint8_t>(bytes.begin(), bytes.begin() + n), &error);
      if (!m) {
        if (error.empty()) Fail("fuzz: " + name + " truncated to " + std::to_string(n));
        Mix(5);
      } else {
        ++t.parsed;
        Mix(Interpreter::Create(*m, &error) ? 6 : 10);
      }
    }
    for (const uint8_t flip : {uint8_t{0xff}, uint8_t{0x01}, uint8_t{0x80}}) {
      for (size_t i = 0; i < bytes.size(); ++i) {
        std::vector<uint8_t> mutated = bytes;
        mutated[i] ^= flip;
        Variant(name + " byte " + std::to_string(i), std::move(mutated), &budget, &t);
      }
    }
  }
  std::printf(
      "fuzz: variants=%zu parsed=%zu created=%zu invoked=%zu invoked_above_32MiB=%zu "
      "skipped_over_budget=%zu invoke_false=%zu skipped_files_over_16KiB=%zu\n",
      t.variants, t.parsed, t.created, t.invoked, t.invoked_large, t.skipped_large, t.invoke_false,
      t.skipped_files);
  if (t.created <= 1000) Fail("fuzz: fewer than 1000 variants created");
}

}  // namespace

int main(int argc, char** argv) {
  std::string repo;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], "--repo") == 0) repo = argv[i + 1];
    if (std::strcmp(argv[i], "--trace") == 0) g_trace = std::fopen(argv[i + 1], "w");
  }
  if (repo.empty()) {
    std::fprintf(stderr, "usage: sr_ml_harden --repo <repo root>\n");
    return 2;
  }
  const std::string dir = repo + "/slam_refact/engine/tests/ml/data/op_cases";
  CheckRejects(dir);
  CheckFamily(dir);
  CheckFuzz(dir);
  std::printf("digest: %016llx\n", static_cast<unsigned long long>(g_digest));
  std::printf("failures: %zu\n", g_failures);
  if (g_trace != nullptr) std::fclose(g_trace);
  return g_failures == 0 ? 0 : 1;
}
