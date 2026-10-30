// Minimal FlexBuffers reader for TFLite custom op options (US-016).
//
// Reads a root map whose values are scalars (int, uint, float, bool), strings or vectors of
// scalars, which is what the custom ops of the seven reference runtime / MediaPipe models use
// (TransformTensorBilinear, Landmarks2TransformMatrix). Every read is bounds checked; malformed
// input makes ParseFlexMap return false. No flatbuffers library (docs/design/ml-runtime.md §1).
#ifndef SLAM_REFACT_ML_FLEXBUFFERS_H_
#define SLAM_REFACT_ML_FLEXBUFFERS_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace slam_refact {
namespace ml {

struct FlexValue {
  enum class Kind { kNull, kInt, kFloat, kBool, kString, kNumberVector };
  Kind kind = Kind::kNull;
  int64_t int_value = 0;
  double float_value = 0.0;
  std::string string_value;
  // kNumberVector: every element as double, and as int64 when the elements are integers.
  std::vector<double> numbers;
  std::vector<int64_t> ints;
  bool integer_vector = false;
};

using FlexMap = std::map<std::string, FlexValue>;

// Parses `data` (the whole FlexBuffers blob) as a root map. Returns false with `error` set on
// malformed input or on value types outside FlexValue (nested maps, vectors of non-scalars).
bool ParseFlexMap(const uint8_t* data, size_t size, FlexMap* out, std::string* error);

// Typed lookups. Return false when the key is absent or has another kind.
bool FlexGetInt(const FlexMap& map, const std::string& key, int64_t* out);
bool FlexGetFloat(const FlexMap& map, const std::string& key, double* out);
bool FlexGetIntVector(const FlexMap& map, const std::string& key, std::vector<int64_t>* out);

}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_FLEXBUFFERS_H_
