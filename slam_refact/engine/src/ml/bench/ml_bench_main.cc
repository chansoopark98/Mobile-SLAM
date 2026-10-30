// sr_ml_bench: per-model inference time (p50/p95) and accuracy against the numpy reference, for
// native and WASM (node) builds of the ML engine (docs/design/ml-runtime.md §7, §8).
//
//   sr_ml_bench --repo <repo root> --ref-dir <ml_ref run-all dir> [--models a,b] [--iters 30]
//               [--warmup 3] [--json <out>]
//
// Desktop timings are reference records only; they are not mobile performance (§2).
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ml/compare.h"
#include "ml/interpreter.h"
#include "ml/tensor_bundle.h"
#include "ml/tflite_model.h"

namespace {

using slam_refact::ml::BundleTensor;
using slam_refact::ml::Interpreter;
using slam_refact::ml::OutputError;
using slam_refact::ml::TfliteModel;

struct ModelEntry {
  const char* name;
  const char* path;
};

// Same table as tools/ml_ref/ml_ref.py MODELS.
const ModelEntry kModels[] = {
    {"face-model", "assets/references/xr_reference/engine/resources/face-model.tflite"},
    {"face-mesh-model", "assets/references/xr_reference/engine/resources/face-mesh-model.tflite"},
    {"face-ear-model", "assets/references/xr_reference/engine/resources/face-ear-model.tflite"},
    {"semantics-model", "assets/references/xr_reference/engine/resources/semantics-model.tflite"},
    {"palm_detection",
     "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/palm_detection.tflite"},
    {"hand_landmark",
     "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/hand_landmark.tflite"},
    {"hand_landmark_3d",
     "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/"
     "hand_landmark_3d.tflite"},
};

double Seconds(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
  return std::chrono::duration<double>(b - a).count();
}

// Insertion sort (small N); keeps the bench off the std::sort watch list (check_std_algos.py).
void SortAscending(std::vector<double>* v) {
  for (size_t i = 1; i < v->size(); ++i) {
    const double x = (*v)[i];
    size_t j = i;
    while (j > 0 && (*v)[j - 1] > x) {
      (*v)[j] = (*v)[j - 1];
      --j;
    }
    (*v)[j] = x;
  }
}

// Nearest-rank percentile of a sorted vector.
double Percentile(const std::vector<double>& sorted, double q) {
  if (sorted.empty()) return 0.0;
  size_t rank = static_cast<size_t>(q * static_cast<double>(sorted.size()) + 0.999999);
  if (rank < 1) rank = 1;
  if (rank > sorted.size()) rank = sorted.size();
  return sorted[rank - 1];
}

std::string JsonEscape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') o.push_back('\\');
    o.push_back(c);
  }
  return o;
}

bool LoadInputs(Interpreter* interp, const std::string& path, std::string* error) {
  std::vector<BundleTensor> in;
  if (!slam_refact::ml::ReadTensorBundle(path, &in, error)) return false;
  if (in.size() != interp->num_inputs()) {
    *error = path + ": input count differs from the model";
    return false;
  }
  for (size_t k = 0; k < in.size(); ++k) {
    if (in[k].shape != interp->input_shape(k) || in[k].values.size() != interp->input_count(k)) {
      *error = path + ": input shape differs from the model";
      return false;
    }
    float* dst = interp->input_data(k);
    for (size_t i = 0; i < in[k].values.size(); ++i) dst[i] = static_cast<float>(in[k].values[i]);
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::string repo = ".";
  std::string ref_dir;
  std::string json_path;
  std::string only;
  int iters = 30;
  int warmup = 3;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
    if (a == "--repo") {
      repo = next();
    } else if (a == "--ref-dir") {
      ref_dir = next();
    } else if (a == "--json") {
      json_path = next();
    } else if (a == "--models") {
      only = "," + next() + ",";
    } else if (a == "--iters") {
      iters = std::atoi(next().c_str());
    } else if (a == "--warmup") {
      warmup = std::atoi(next().c_str());
    } else {
      std::fprintf(stderr, "unknown argument %s\n", a.c_str());
      return 2;
    }
  }
  if (ref_dir.empty() || iters <= 0 || warmup < 0) {
    std::fprintf(stderr,
                 "usage: sr_ml_bench --repo <root> --ref-dir <dir> [--models a,b] [--iters N] "
                 "[--warmup N] [--json out]\n");
    return 2;
  }
#ifdef __EMSCRIPTEN__
  const char* platform = "wasm";
#else
  const char* platform = "native";
#endif
  std::ostringstream json;
  json.precision(17);  // error statistics are compared against the regression gate (RM-1b)
  json << "{\"platform\": \"" << platform << "\", \"iters\": " << iters
       << ", \"warmup\": " << warmup << ", \"models\": [";
  bool first_model = true;
  bool all_ok = true;
  for (const ModelEntry& m : kModels) {
    if (!only.empty() && only.find("," + std::string(m.name) + ",") == std::string::npos) continue;
    std::string error;
    const auto t0 = std::chrono::steady_clock::now();
    std::unique_ptr<TfliteModel> model = TfliteModel::FromFile(repo + "/" + m.path, &error);
    const auto t1 = std::chrono::steady_clock::now();
    if (!model) {
      std::fprintf(stderr, "%s: %s\n", m.name, error.c_str());
      return 1;
    }
    std::unique_ptr<Interpreter> interp = Interpreter::Create(*model, &error);
    const auto t2 = std::chrono::steady_clock::now();
    if (!interp) {
      std::fprintf(stderr, "%s: %s\n", m.name, error.c_str());
      return 1;
    }
    // Accuracy on both reference inputs.
    std::ostringstream outs;
    outs.precision(17);
    bool first_out = true;
    for (int k = 0; k < 2; ++k) {
      const std::string base = ref_dir + "/" + m.name + "/";
      if (!LoadInputs(interp.get(), base + "inputs-" + std::to_string(k) + ".srt", &error) ||
          !interp->Invoke(&error)) {
        std::fprintf(stderr, "%s: %s\n", m.name, error.c_str());
        return 1;
      }
      std::vector<BundleTensor> ref;
      if (!slam_refact::ml::ReadTensorBundle(base + "ref-" + std::to_string(k) + ".srt", &ref,
                                             &error) ||
          ref.size() != interp->num_outputs()) {
        std::fprintf(stderr, "%s: reference: %s\n", m.name, error.c_str());
        return 1;
      }
      for (size_t o = 0; o < ref.size(); ++o) {
        const OutputError e = slam_refact::ml::CompareOutput(interp->output(o), ref[o]);
        const bool pass = slam_refact::ml::PassesModelTolerance(e);
        all_ok = all_ok && e.shape_ok && e.type_ok;
        outs << (first_out ? "" : ", ") << "{\"input\": " << k << ", \"name\": \""
             << JsonEscape(ref[o].name) << "\", \"max_abs\": " << e.max_abs
             << ", \"ref_max_abs\": " << e.ref_max_abs << ", \"rms\": " << e.rms
             << ", \"ref_rms\": " << e.ref_rms << ", \"bit_mismatches\": " << e.bit_mismatches
             << ", \"count\": " << e.count << ", \"within_model_tolerance\": "
             << (pass ? "true" : "false") << "}";
        first_out = false;
      }
    }
    // Timing on input 0.
    if (!LoadInputs(interp.get(), ref_dir + "/" + m.name + "/inputs-0.srt", &error)) {
      std::fprintf(stderr, "%s: %s\n", m.name, error.c_str());
      return 1;
    }
    for (int w = 0; w < warmup; ++w) interp->Invoke(&error);
    std::vector<double> ms;
    for (int it = 0; it < iters; ++it) {
      const auto a = std::chrono::steady_clock::now();
      interp->Invoke(&error);
      const auto b = std::chrono::steady_clock::now();
      ms.push_back(1e3 * Seconds(a, b));
    }
    double mean = 0.0;
    for (double v : ms) mean += v;
    mean /= static_cast<double>(ms.size());
    SortAscending(&ms);
    // One profiled run for the per-op split.
    interp->reset_profile();
    interp->set_profiling(true);
    interp->Invoke(&error);
    interp->set_profiling(false);
    std::ostringstream prof;
    bool first_p = true;
    for (const auto& kv : interp->profile()) {
      prof << (first_p ? "" : ", ") << "\"" << JsonEscape(kv.first) << "\": " << 1e3 * kv.second;
      first_p = false;
    }
    std::printf("%-17s p50 %8.3f ms  p95 %8.3f ms  min %8.3f  max %8.3f  (parse %.1f ms, create "
                "%.1f ms, arena %.1f MB, memory %.1f MB, ops %zu + folded %zu)\n",
                m.name, Percentile(ms, 0.50), Percentile(ms, 0.95), ms.front(), ms.back(),
                1e3 * Seconds(t0, t1), 1e3 * Seconds(t1, t2),
                static_cast<double>(interp->arena_bytes()) / (1024.0 * 1024.0),
                static_cast<double>(interp->memory_use().total()) / (1024.0 * 1024.0),
                interp->num_planned_ops(), interp->num_folded_ops());
    std::fflush(stdout);
    json << (first_model ? "" : ", ") << "{\"name\": \"" << m.name << "\", \"path\": \"" << m.path
         << "\", \"parse_ms\": " << 1e3 * Seconds(t0, t1)
         << ", \"create_ms\": " << 1e3 * Seconds(t1, t2)
         << ", \"arena_bytes\": " << interp->arena_bytes()
         << ", \"memory_bytes\": {\"constants\": " << interp->memory_use().constants
         << ", \"packed_weights\": " << interp->memory_use().packed_weights
         << ", \"scratch\": " << interp->memory_use().scratch
         << ", \"activations\": " << interp->memory_use().activations
         << ", \"total\": " << interp->memory_use().total() << "}"
         << ", \"planned_ops\": " << interp->num_planned_ops()
         << ", \"folded_ops\": " << interp->num_folded_ops()
         << ", \"p50_ms\": " << Percentile(ms, 0.50) << ", \"p95_ms\": " << Percentile(ms, 0.95)
         << ", \"p99_ms\": " << Percentile(ms, 0.99) << ", \"min_ms\": " << ms.front()
         << ", \"max_ms\": " << ms.back() << ", \"mean_ms\": " << mean
         << ", \"outputs\": [" << outs.str() << "], \"profile_ms\": {" << prof.str() << "}}";
    first_model = false;
  }
  json << "]}\n";
  if (!json_path.empty()) {
    std::ofstream f(json_path);
    f << json.str();
    if (!f) {
      std::fprintf(stderr, "cannot write %s\n", json_path.c_str());
      return 1;
    }
  }
  return all_ok ? 0 : 1;
}
