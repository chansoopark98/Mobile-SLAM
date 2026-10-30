// TFLite model parser without the flatbuffers library (US-016; docs/design/ml-runtime.md §1).
//
// Field indices follow tensorflow/lite/schema/schema.fbs (local copy:
// assets/references/xr_reference/official/tensorflow-fork/tensorflow/lite/schema/schema.fbs). Every
// offset, vtable and vector is bounds checked against the file; a malformed file makes the factory
// return nullptr with an error message (no exceptions, no aborts). The model owns a copy of the
// file bytes; TensorDef::data points into it.
#ifndef SLAM_REFACT_ML_TFLITE_MODEL_H_
#define SLAM_REFACT_ML_TFLITE_MODEL_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace slam_refact {
namespace ml {

// schema.fbs TensorType.
enum class TensorType : int8_t {
  kFloat32 = 0,
  kFloat16 = 1,
  kInt32 = 2,
  kUint8 = 3,
  kInt64 = 4,
  kString = 5,
  kBool = 6,
  kInt16 = 7,
  kComplex64 = 8,
  kInt8 = 9,
  kFloat64 = 10,
  kComplex128 = 11,
  kUint64 = 12,
  kResource = 13,
  kVariant = 14,
  kUint32 = 15,
};

// Name as in schema.fbs ("FLOAT32", ...); "UNKNOWN" outside the enum.
const char* TensorTypeName(TensorType type);
// Element size in bytes of fixed-size types, 0 for STRING / RESOURCE / VARIANT / unknown.
size_t TensorTypeSize(TensorType type);

// schema.fbs BuiltinOperator values used by the seven models.
enum BuiltinCode : int32_t {
  kOpAdd = 0,
  kOpConcatenation = 2,
  kOpConv2D = 3,
  kOpDepthwiseConv2D = 4,
  kOpDequantize = 6,
  kOpLogistic = 14,
  kOpMaxPool2D = 17,
  kOpMul = 18,
  kOpRelu = 19,
  kOpReshape = 22,
  kOpResizeBilinear = 23,
  kOpCustom = 32,
  kOpPad = 34,
  kOpTranspose = 39,
  kOpMean = 40,
  kOpSub = 41,
  kOpStridedSlice = 45,
  kOpCast = 53,
  kOpPrelu = 54,
  kOpArgMax = 56,
  kOpMinimum = 57,
  kOpReduceMax = 82,
  kOpFloorMod = 95,
  kOpLeakyRelu = 98,
  kOpHardSwish = 117,
};

// Builtin name of `code` ("CONV_2D", ...), or "BUILTIN_<code>" for codes outside BuiltinCode.
std::string BuiltinOpName(int32_t code);

// schema.fbs BuiltinOptions union member indices parsed by this reader.
enum OptionsType : uint8_t {
  kOptNone = 0,
  kOptConv2D = 1,
  kOptDepthwiseConv2D = 2,
  kOptPool2D = 5,
  kOptConcatenation = 10,
  kOptAdd = 11,
  kOptResizeBilinear = 15,
  kOptReshape = 17,
  kOptMul = 21,
  kOptPad = 22,
  kOptTranspose = 26,
  kOptReducer = 27,
  kOptSub = 28,
  kOptStridedSlice = 32,
  kOptCast = 37,
  kOptDequantize = 38,
  kOptMaximumMinimum = 39,
  kOptArgMax = 40,
  kOptFloorMod = 72,
  kOptLeakyRelu = 75,
  kOptHardSwish = 91,
};

enum class Padding : int8_t { kSame = 0, kValid = 1 };
enum class Activation : int8_t {
  kNone = 0,
  kRelu = 1,
  kReluN1To1 = 2,
  kRelu6 = 3,
  kTanh = 4,
  kSignBit = 5,
};

// Union of the builtin option fields the reader understands. `type` is the union member index;
// fields of other members keep their defaults. `known` is false for union members the reader does
// not parse (the interpreter rejects such ops).
struct BuiltinOptions {
  uint8_t type = kOptNone;
  bool known = true;
  Padding padding = Padding::kSame;
  int32_t stride_w = 0;
  int32_t stride_h = 0;
  int32_t dilation_w = 1;
  int32_t dilation_h = 1;
  int32_t depth_multiplier = 0;
  int32_t filter_w = 0;
  int32_t filter_h = 0;
  Activation activation = Activation::kNone;
  int32_t axis = 0;
  bool align_corners = false;
  bool half_pixel_centers = false;
  bool keep_dims = false;
  bool has_new_shape = false;
  std::vector<int32_t> new_shape;
  int32_t begin_mask = 0;
  int32_t end_mask = 0;
  int32_t ellipsis_mask = 0;
  int32_t new_axis_mask = 0;
  int32_t shrink_axis_mask = 0;
  bool offset = false;
  TensorType output_type = TensorType::kInt64;
  TensorType cast_in_type = TensorType::kFloat32;
  TensorType cast_out_type = TensorType::kFloat32;
  float alpha = 0.0f;
};

struct QuantizationParams {
  std::vector<float> min;
  std::vector<float> max;
  std::vector<float> scale;
  std::vector<int64_t> zero_point;
  int32_t quantized_dimension = 0;
};

struct TensorDef {
  std::string name;
  std::vector<int32_t> shape;
  TensorType type = TensorType::kFloat32;
  uint32_t buffer = 0;
  const uint8_t* data = nullptr;  // constant data (into TfliteModel::bytes()), nullptr if none
  size_t data_size = 0;
  bool has_quantization = false;
  QuantizationParams quantization;
  bool is_variable = false;
  bool has_sparsity = false;
};

struct OperatorDef {
  uint32_t opcode_index = 0;
  int32_t builtin_code = 0;
  std::string custom_code;  // non-empty for CUSTOM ops
  int32_t version = 1;
  std::vector<int32_t> inputs;  // -1 = optional input absent
  std::vector<int32_t> outputs;
  BuiltinOptions options;
  std::vector<uint8_t> custom_options;
  int8_t custom_options_format = 0;
};

struct SubgraphDef {
  std::string name;
  std::vector<TensorDef> tensors;
  std::vector<int32_t> inputs;
  std::vector<int32_t> outputs;
  std::vector<OperatorDef> ops;
};

class TfliteModel {
 public:
  // Parses `bytes`. Returns nullptr and sets `error` if the file is not a well-formed TFLite
  // model (identifier "TFL3", in-bounds tables, tensor data sizes matching their shapes).
  static std::unique_ptr<TfliteModel> FromBytes(std::vector<uint8_t> bytes, std::string* error);
  static std::unique_ptr<TfliteModel> FromFile(const std::string& path, std::string* error);

  TfliteModel(const TfliteModel&) = delete;
  TfliteModel& operator=(const TfliteModel&) = delete;

  uint32_t version() const { return version_; }
  const std::string& description() const { return description_; }
  const std::vector<SubgraphDef>& subgraphs() const { return subgraphs_; }
  size_t num_buffers() const { return num_buffers_; }
  const std::vector<uint8_t>& bytes() const { return bytes_; }

  // Name of an operator: builtin name or the custom code.
  static std::string OpName(const OperatorDef& op);

 private:
  TfliteModel() = default;
  bool Parse(std::string* error);

  std::vector<uint8_t> bytes_;
  uint32_t version_ = 0;
  std::string description_;
  std::vector<SubgraphDef> subgraphs_;
  size_t num_buffers_ = 0;
};

// Canonical text of subgraph 0 (tensors, ops with options, inputs, outputs). Same format as
// slam_refact/tools/ml_ref/ml_ref.py graph_text(); the model test compares the two line by line.
std::string CanonicalGraphText(const TfliteModel& model);

}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_TFLITE_MODEL_H_
