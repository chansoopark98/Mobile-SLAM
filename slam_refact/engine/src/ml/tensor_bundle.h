// Tensor bundle (.srt) files exchanged with the numpy reference (slam_refact/tools/ml_ref/
// tensor_bundle.py; format in docs/design/ml-runtime.md §6). Used by the ML tests and the bench.
#ifndef SLAM_REFACT_ML_TENSOR_BUNDLE_H_
#define SLAM_REFACT_ML_TENSOR_BUNDLE_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ml/tflite_model.h"

namespace slam_refact {
namespace ml {

struct BundleTensor {
  std::string name;
  TensorType type = TensorType::kFloat32;  // kFloat32, kInt32, kInt64 or kFloat64
  std::vector<int32_t> shape;
  std::vector<double> values;   // float types (float32 values widened exactly)
  std::vector<int64_t> ints;    // integer types
  size_t count() const;
};

bool ReadTensorBundle(const std::string& path, std::vector<BundleTensor>* out, std::string* error);
bool WriteTensorBundle(const std::string& path, const std::vector<BundleTensor>& tensors,
                       std::string* error);

}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_TENSOR_BUNDLE_H_
