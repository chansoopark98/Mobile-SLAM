// Graph executor for TFLite models parsed by TfliteModel (US-017; docs/design/ml-runtime.md).
//
// Create() validates every op of subgraph 0 (supported op and options, tensor types, constant
// parameter inputs, and that each computed output shape equals the declared one), decodes FP16
// constants to float32, folds DEQUANTIZE of constants, and plans activation memory (a buffer is
// reused once its tensor's last consumer has run). Unsupported models are rejected with an error;
// nothing is silently skipped. Single-threaded, float32.
//
// Resource limits (kernels.h kMax*): a model whose tensor, total memory (constants, packed weights,
// scratch and activations together), stride / dilation / window / size parameter or constant integer
// argument is outside them is rejected by Create().
// Everything that depends on the model alone is validated there, so Invoke() cannot index outside
// a buffer; the only run-time failure left is an integer FLOOR_MOD with a divisor of 0 coming from
// data.
#ifndef SLAM_REFACT_ML_INTERPRETER_H_
#define SLAM_REFACT_ML_INTERPRETER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ml/tflite_model.h"

namespace slam_refact {
namespace ml {

struct TensorView {
  std::string name;
  TensorType type = TensorType::kFloat32;
  std::vector<int32_t> shape;
  size_t count = 0;
  const float* f32 = nullptr;    // FLOAT32
  const int32_t* i32 = nullptr;  // INT32
  const int64_t* i64 = nullptr;  // INT64
};

// Memory the interpreter allocated, in bytes (RM-1b: charged against kernels::kMaxTotalBytes).
struct MemoryUse {
  size_t constants = 0;       // decoded constant tensors, incl. DEQUANTIZE-folded copies
  size_t packed_weights = 0;  // CONV_2D filters in the GEMM panel layout
  size_t scratch = 0;         // largest im2col / transpose-conv / runtime-packing need
  size_t activations = 0;     // arena slots and integer activations
  size_t total() const { return constants + packed_weights + scratch + activations; }
};

class Interpreter {
 public:
  // `model` must outlive the interpreter. Returns nullptr with `error` set if the model uses an
  // op, option or tensor type outside the supported set or is inconsistent.
  static std::unique_ptr<Interpreter> Create(const TfliteModel& model, std::string* error);
  ~Interpreter();

  Interpreter(const Interpreter&) = delete;
  Interpreter& operator=(const Interpreter&) = delete;

  size_t num_inputs() const;
  size_t num_outputs() const;
  // Graph inputs are FLOAT32; write `input_count(i)` floats before Invoke().
  float* input_data(size_t i);
  size_t input_count(size_t i) const;
  const std::vector<int32_t>& input_shape(size_t i) const;
  // Valid after a successful Invoke() until the next Invoke().
  TensorView output(size_t i) const;
  // View of any tensor that currently holds data (constants, or activations while their buffer is
  // not reused; for debugging and tests).
  TensorView tensor(int index) const;

  // False (with `error`) only for an integer FLOOR_MOD whose non-constant divisor is 0.
  bool Invoke(std::string* error);

  // Ops executed per Invoke (after constant folding) and ops folded at Create.
  size_t num_planned_ops() const;
  size_t num_folded_ops() const;
  // Bytes of activation buffers after memory planning.
  size_t arena_bytes() const;
  // Everything the interpreter holds (the model file itself is not included).
  MemoryUse memory_use() const;

  // Per-op-name wall time (seconds, summed over Invokes since the last reset).
  void set_profiling(bool enabled);
  std::vector<std::pair<std::string, double>> profile() const;
  void reset_profile();

 private:
  struct Impl;
  explicit Interpreter(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_INTERPRETER_H_
