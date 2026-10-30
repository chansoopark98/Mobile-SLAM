// Relative regression gate v3 of the seven-model end-to-end test (RM-1b; docs/design/ml-runtime.md
// section 5.3, which also says what v1 and the never-run v2 were and why they were superseded). The
// absolute tolerances of section 5.2 stay as they are; this gate is added on top.
//
// Row rule, for every (model, input, output) row and each of the two statistics (max|c-r| and
// RMS(c-r)):
//
//     observed <= baseline + 3 * max(resolution, allowance)
//
// baseline    the engine's own error when the gate was registered (data/model_regression_gate_v3.tsv,
//             written once by tools/ml_ref/make_regression_gate_v3.py)
// resolution  one float32 rounding step of the output scale, 2^-23 * scale (the null-run noise of the
//             engine itself, same binary on P and E cores and in WASM, is 0)
// allowance   the largest change of that statistic under implementations of the same accuracy class
//             (K blocks of 8, 16 and 64, and the previous Eigen kernel), the null-run N3. This is a
//             tolerance policy, not a measurement resolution.
// The table stores max(resolution, allowance) in the two resolution columns. The OR-3 rule
// "tolerance >= 3 x resolution" holds because the tolerance is 3 x max(resolution, allowance).
//
// Reference rule: the observed reference scales (max|r|, RMS(r)) must equal the table's within 1e-12
// relative. The numpy reference is regenerated at every ctest run and differs by up to 7e-15 between
// runs; a larger change means the reference or the inputs changed, which this gate does not judge.
//
// Aggregate rule (data/model_regression_gate_v3_aggregate.tsv): over the rows with a non-zero
// baseline RMS and ref_rms >= kAggregateMinRefRms (32 rows) the geometric mean of observed/baseline
// must stay <= the registered limit, for each statistic. It sees a systematic loss of accuracy that
// is too small for any single row.
// The registration is build/slam-refact-full-20261019/preregistration/rm1-regression-gate.v3.md/.manifest.json.
#ifndef SLAM_REFACT_TESTS_ML_REGRESSION_GATE_H_
#define SLAM_REFACT_TESTS_ML_REGRESSION_GATE_H_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ml/compare.h"

namespace slam_refact {
namespace ml {

// The registered rule: error <= baseline + kGateResolutionFactor * resolution.
constexpr double kGateResolutionFactor = 3.0;
constexpr double kGateFloat32Eps = 1.1920928955078125e-07;  // 2^-23
constexpr double kGateRefRelTolerance = 1e-12;
constexpr double kAggregateMinRefRms = 1e-3;

struct GateRow {
  std::string model;
  int input = 0;
  std::string output;
  double baseline_max_abs = 0.0;
  double baseline_rms = 0.0;
  double ref_max_abs = 0.0;
  double ref_rms = 0.0;
  double resolution_max_abs = 0.0;  // max(2^-23 * ref_max_abs, allowance)
  double resolution_rms = 0.0;      // max(2^-23 * ref_rms, allowance)
};

struct GateVerdict {
  bool pass = false;
  bool reference_ok = false;   // the observed reference scales equal the table's
  double max_abs_limit = 0.0;
  double rms_limit = 0.0;
  double max_abs_ratio = 0.0;  // observed / limit
  double rms_ratio = 0.0;
};

inline bool LoadRegressionGate(const std::string& path, std::vector<GateRow>* rows,
                               std::string* error) {
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> f;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, '\t')) f.push_back(cell);
    if (f.size() != 9) {
      *error = "bad row (expected 9 tab-separated fields): " + line;
      return false;
    }
    GateRow r;
    r.model = f[0];
    r.input = std::stoi(f[1]);
    r.output = f[2];
    r.baseline_max_abs = std::stod(f[3]);
    r.baseline_rms = std::stod(f[4]);
    r.ref_max_abs = std::stod(f[5]);
    r.ref_rms = std::stod(f[6]);
    r.resolution_max_abs = std::stod(f[7]);
    r.resolution_rms = std::stod(f[8]);
    rows->push_back(r);
  }
  return true;
}

inline const GateRow* FindGateRow(const std::vector<GateRow>& rows, const std::string& model,
                                  int input, const std::string& output) {
  for (const GateRow& r : rows) {
    if (r.model == model && r.input == input && r.output == output) return &r;
  }
  return nullptr;
}

inline bool RelativelyEqual(double a, double b, double tolerance) {
  return std::fabs(a - b) <= tolerance * std::fabs(b);
}

inline GateVerdict EvaluateGate(const GateRow& row, const OutputError& e) {
  GateVerdict v;
  v.max_abs_limit = row.baseline_max_abs + kGateResolutionFactor * row.resolution_max_abs;
  v.rms_limit = row.baseline_rms + kGateResolutionFactor * row.resolution_rms;
  v.max_abs_ratio = v.max_abs_limit > 0 ? e.max_abs / v.max_abs_limit : (e.max_abs > 0 ? 1e30 : 0);
  v.rms_ratio = v.rms_limit > 0 ? e.rms / v.rms_limit : (e.rms > 0 ? 1e30 : 0);
  v.reference_ok = RelativelyEqual(e.ref_max_abs, row.ref_max_abs, kGateRefRelTolerance) &&
                   RelativelyEqual(e.ref_rms, row.ref_rms, kGateRefRelTolerance);
  v.pass = e.shape_ok && e.type_ok && v.reference_ok && e.max_abs <= v.max_abs_limit &&
           e.rms <= v.rms_limit;
  return v;
}

// ---- aggregate rule --------------------------------------------------------------------------------

struct GateAggregateLimits {
  double max_abs_limit = 0.0;
  double rms_limit = 0.0;
  size_t rows = 0;
};

inline bool LoadAggregateLimits(const std::string& path, GateAggregateLimits* out,
                                std::string* error) {
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  std::string line;
  int seen = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> f;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, '\t')) f.push_back(cell);
    if (f.size() != 3) {
      *error = "bad aggregate row (expected statistic, limit, rows): " + line;
      return false;
    }
    if (f[0] == "max_abs") {
      out->max_abs_limit = std::stod(f[1]);
    } else if (f[0] == "rms") {
      out->rms_limit = std::stod(f[1]);
    } else {
      *error = "unknown aggregate statistic " + f[0];
      return false;
    }
    out->rows = static_cast<size_t>(std::stoul(f[2]));
    ++seen;
  }
  if (seen != 2) {
    *error = "aggregate file must have one max_abs and one rms line";
    return false;
  }
  return true;
}

inline bool InAggregate(const GateRow& row) {
  return row.baseline_rms > 0.0 && row.baseline_max_abs > 0.0 && row.ref_rms >= kAggregateMinRefRms;
}

// Geometric means of observed / baseline over the aggregate rows.
struct AggregateAccumulator {
  double sum_log_max_abs = 0.0;
  double sum_log_rms = 0.0;
  size_t rows = 0;
  void Add(const GateRow& row, const OutputError& e) {
    if (!InAggregate(row)) return;
    sum_log_max_abs += std::log(e.max_abs / row.baseline_max_abs);
    sum_log_rms += std::log(e.rms / row.baseline_rms);
    ++rows;
  }
  double MaxAbsRatio() const { return rows ? std::exp(sum_log_max_abs / static_cast<double>(rows)) : 0.0; }
  double RmsRatio() const { return rows ? std::exp(sum_log_rms / static_cast<double>(rows)) : 0.0; }
};

}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_TESTS_ML_REGRESSION_GATE_H_
