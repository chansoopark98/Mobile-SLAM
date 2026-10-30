// Output comparison against the numpy reference (docs/design/ml-runtime.md §5). The tolerances
// below are the pre-registered values; they are shared by the GTests and sr_ml_bench.
#ifndef SLAM_REFACT_ML_COMPARE_H_
#define SLAM_REFACT_ML_COMPARE_H_

#include <cstddef>
#include <string>

#include "ml/interpreter.h"
#include "ml/tensor_bundle.h"

namespace slam_refact {
namespace ml {

// §5.1 class F: max|c - r| <= kOpTolerance * max(1, max|r|).
constexpr double kOpTolerance = 1e-5;
// §5.2: max|c - r| <= kModelMaxAbsTolerance * max(1, max|r|) and
//       RMS(c - r) <= kModelRmsTolerance * max(1, RMS(r)).
constexpr double kModelMaxAbsTolerance = 1e-3;
constexpr double kModelRmsTolerance = 1e-4;

struct OutputError {
  bool shape_ok = false;
  bool type_ok = false;
  size_t count = 0;
  double max_abs = 0.0;     // max |c - r|
  double rms = 0.0;         // RMS(c - r)
  double ref_max_abs = 0.0; // max |r|
  double ref_rms = 0.0;     // RMS(r)
  // float: the bits of c differ from the bits of float32(r), with every NaN equal to every NaN (a NaN
  // payload or sign carries no meaning here); integer: c != r. +0 and -0 are different (RM-1b: the
  // float comparison `==` of the earlier version could not see the sign of zero).
  size_t bit_mismatches = 0;
  // The part of bit_mismatches where c and float32(r) are both zero with opposite signs.
  size_t zero_sign_mismatches = 0;
  size_t worst_index = 0;
};

// Compares a FLOAT32 / INT32 / INT64 view with a reference bundle tensor (float64 or integer).
OutputError CompareOutput(const TensorView& got, const BundleTensor& ref);

double ScaleFloor(double v);  // max(1, v)
bool PassesOpTolerance(const OutputError& e);
bool PassesModelTolerance(const OutputError& e);
bool PassesExact(const OutputError& e);
std::string DescribeError(const OutputError& e);

}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_COMPARE_H_
