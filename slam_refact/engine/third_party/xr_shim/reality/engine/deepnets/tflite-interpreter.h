// TFLite-free shim of reality/engine/deepnets/tflite-interpreter.h (US-018).
//
// The vendored detectors (faces/face-detector-{global,local}, ears/ear-detector-local) hold a
// TFLiteInterpreter and use only this surface of the original wrapper (tflite-interpreter.h:47-101):
//   TFLiteInterpreter(modelData, modelSize, cacheSizeInBytes, numThreads)
//   interpreter_->typed_input_tensor<float>(0), interpreter_->Invoke(),
//   interpreter_->typed_output_tensor<float>(i)
// The original runs TFLite 2.11 + the XNNPACK delegate; this shim runs the same .tflite bytes on
// the slam_refact Eigen interpreter (src/ml, docs/design/ml-runtime.md). operator-> returns the
// wrapper itself, which provides the three tflite::Interpreter calls above. Output pointers are
// const (the vendored code only reads them). cacheSizeInBytes and numThreads are accepted and
// ignored (single-threaded, no XNNPACK weight cache). The implementation is
// third_party/xr_shim/tflite-interpreter.cc.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "c8/vector.h"

namespace c8 {

constexpr int DEFAULT_NUM_THREADS = 2;

class TFLiteInterpreter {
public:
  // Aborts (C8_THROW) if the bytes are not a model the slam_refact interpreter accepts, as the
  // original aborts on a failed build. Callers validate the model first (src/face).
  TFLiteInterpreter(
    const uint8_t *modelData,
    int modelSize,
    size_t cacheSizeInBytes,
    int numThreads = DEFAULT_NUM_THREADS);
  TFLiteInterpreter(
    const Vector<uint8_t> &modelData, size_t cacheSizeInBytes, int numThreads = DEFAULT_NUM_THREADS)
      : TFLiteInterpreter(modelData.data(), modelData.size(), cacheSizeInBytes, numThreads) {}

  TFLiteInterpreter *operator->() { return this; }

  TFLiteInterpreter(TFLiteInterpreter &&) noexcept;
  TFLiteInterpreter &operator=(TFLiteInterpreter &&) noexcept;
  TFLiteInterpreter(const TFLiteInterpreter &) = delete;
  TFLiteInterpreter &operator=(const TFLiteInterpreter &) = delete;
  ~TFLiteInterpreter();

  // tflite::Interpreter subset. Only T = float is defined.
  template <typename T>
  T *typed_input_tensor(int index);
  template <typename T>
  const T *typed_output_tensor(int index) const;
  // Returns 0 (kTfLiteOk). An execution error aborts (C8_THROW); the vendored callers ignore the
  // status, and the supported models never fail at Invoke.
  int Invoke();

  std::vector<int> getInputTensorDims(size_t index = 0);
  std::vector<int> getOutputTensorDims(size_t index = 0);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

template <>
float *TFLiteInterpreter::typed_input_tensor<float>(int index);
template <>
const float *TFLiteInterpreter::typed_output_tensor<float>(int index) const;

}  // namespace c8
