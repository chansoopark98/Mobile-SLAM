#include "ml/compare.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace slam_refact {
namespace ml {

double ScaleFloor(double v) { return v > 1.0 ? v : 1.0; }

OutputError CompareOutput(const TensorView& got, const BundleTensor& ref) {
  OutputError e;
  e.shape_ok = got.shape == ref.shape && got.count == ref.count();
  const bool ref_float = ref.type == TensorType::kFloat32 || ref.type == TensorType::kFloat64;
  const bool got_float = got.type == TensorType::kFloat32;
  e.type_ok = ref_float == got_float &&
              (got_float || (got.type == TensorType::kInt32 && got.i32 != nullptr) ||
               (got.type == TensorType::kInt64 && got.i64 != nullptr));
  if (!e.shape_ok || !e.type_ok) return e;
  e.count = got.count;
  double sum_sq = 0.0;
  double ref_sq = 0.0;
  for (size_t i = 0; i < got.count; ++i) {
    double c;
    double r;
    bool same;
    if (got_float) {
      c = got.f32[i];
      r = ref.values[i];
      const float rf = static_cast<float>(ref.values[i]);
      uint32_t got_bits;
      uint32_t ref_bits;
      std::memcpy(&got_bits, &got.f32[i], sizeof(got_bits));
      std::memcpy(&ref_bits, &rf, sizeof(ref_bits));
      same = got_bits == ref_bits || (std::isnan(got.f32[i]) && std::isnan(rf));
      if (!same && got.f32[i] == 0.0f && rf == 0.0f) ++e.zero_sign_mismatches;
    } else {
      const int64_t ci = got.type == TensorType::kInt32 ? got.i32[i] : got.i64[i];
      c = static_cast<double>(ci);
      r = static_cast<double>(ref.ints[i]);
      same = ci == ref.ints[i];
    }
    // IEEE special values (RM-1): equal infinities and NaN against NaN are an exact match; a
    // NaN or infinity against anything else is an infinite error. Finite data is unaffected.
    const double d = (c == r || (std::isnan(c) && std::isnan(r))) ? 0.0 : std::fabs(c - r);
    if (!(d <= e.max_abs)) {  // also catches NaN
      if (std::isnan(d)) {
        e.max_abs = INFINITY;
      } else {
        e.max_abs = d;
      }
      e.worst_index = i;
    }
    sum_sq += d * d;
    if (std::isfinite(r)) {  // the tolerance scale comes from the finite reference values
      ref_sq += r * r;
      e.ref_max_abs = std::max(e.ref_max_abs, std::fabs(r));
    }
    if (!same) ++e.bit_mismatches;
  }
  if (got.count > 0) {
    e.rms = std::sqrt(sum_sq / static_cast<double>(got.count));
    e.ref_rms = std::sqrt(ref_sq / static_cast<double>(got.count));
  }
  return e;
}

bool PassesOpTolerance(const OutputError& e) {
  return e.shape_ok && e.type_ok && e.max_abs <= kOpTolerance * ScaleFloor(e.ref_max_abs);
}

bool PassesModelTolerance(const OutputError& e) {
  return e.shape_ok && e.type_ok &&
         e.max_abs <= kModelMaxAbsTolerance * ScaleFloor(e.ref_max_abs) &&
         e.rms <= kModelRmsTolerance * ScaleFloor(e.ref_rms);
}

bool PassesExact(const OutputError& e) { return e.shape_ok && e.type_ok && e.bit_mismatches == 0; }

std::string DescribeError(const OutputError& e) {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "shape_ok=%d type_ok=%d n=%zu max_abs=%.3e (ref max %.3e) rms=%.3e (ref rms "
                "%.3e) bit_mismatches=%zu (zero sign %zu) worst=%zu",
                e.shape_ok ? 1 : 0, e.type_ok ? 1 : 0, e.count, e.max_abs, e.ref_max_abs, e.rms,
                e.ref_rms, e.bit_mismatches, e.zero_sign_mismatches, e.worst_index);
  return buf;
}

}  // namespace ml
}  // namespace slam_refact
