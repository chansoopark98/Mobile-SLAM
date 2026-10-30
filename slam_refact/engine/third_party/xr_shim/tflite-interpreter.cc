// Implementation of the TFLiteInterpreter shim (reality/engine/deepnets/tflite-interpreter.h) on
// the slam_refact Eigen interpreter (src/ml). Replaces PUB:reality/engine/deepnets/
// tflite-interpreter.cc (TFLite 2.11 + XNNPACK); the custom ops registered there
// (Landmarks2TransformMatrix, TransformTensorBilinear, TransformLandmarks, all version 2) are
// built into slam_refact::ml::Interpreter.

#include "reality/engine/deepnets/tflite-interpreter.h"

#include <string>
#include <utility>

#include "c8/exceptions.h"
#include "ml/interpreter.h"
#include "ml/tflite_model.h"

namespace c8 {

struct TFLiteInterpreter::Impl {
  std::unique_ptr<slam_refact::ml::TfliteModel> model;
  std::unique_ptr<slam_refact::ml::Interpreter> interpreter;
};

TFLiteInterpreter::TFLiteInterpreter(
  const uint8_t *modelData, int modelSize, size_t /*cacheSizeInBytes*/, int /*numThreads*/)
    : impl_(new Impl()) {
  if (modelData == nullptr || modelSize <= 0) {
    C8_THROW("[tflite-interpreter] empty model");
  }
  std::string error;
  impl_->model = slam_refact::ml::TfliteModel::FromBytes(
    std::vector<uint8_t>(modelData, modelData + modelSize), &error);
  if (impl_->model == nullptr) {
    C8_THROW("[tflite-interpreter] model parse failed: %s", error.c_str());
  }
  impl_->interpreter = slam_refact::ml::Interpreter::Create(*impl_->model, &error);
  if (impl_->interpreter == nullptr) {
    C8_THROW("[tflite-interpreter] interpreter create failed: %s", error.c_str());
  }
}

TFLiteInterpreter::TFLiteInterpreter(TFLiteInterpreter &&) noexcept = default;
TFLiteInterpreter &TFLiteInterpreter::operator=(TFLiteInterpreter &&) noexcept = default;
TFLiteInterpreter::~TFLiteInterpreter() = default;

template <>
float *TFLiteInterpreter::typed_input_tensor<float>(int index) {
  if (index < 0 || static_cast<size_t>(index) >= impl_->interpreter->num_inputs()) {
    C8_THROW("[tflite-interpreter] input index %d out of range", index);
  }
  return impl_->interpreter->input_data(static_cast<size_t>(index));
}

template <>
const float *TFLiteInterpreter::typed_output_tensor<float>(int index) const {
  if (index < 0 || static_cast<size_t>(index) >= impl_->interpreter->num_outputs()) {
    C8_THROW("[tflite-interpreter] output index %d out of range", index);
  }
  const auto view = impl_->interpreter->output(static_cast<size_t>(index));
  if (view.f32 == nullptr) {
    C8_THROW("[tflite-interpreter] output %d is not FLOAT32 or not computed", index);
  }
  return view.f32;
}

int TFLiteInterpreter::Invoke() {
  std::string error;
  if (!impl_->interpreter->Invoke(&error)) {
    C8_THROW("[tflite-interpreter] invoke failed: %s", error.c_str());
  }
  return 0;
}

std::vector<int> TFLiteInterpreter::getInputTensorDims(size_t index) {
  if (index >= impl_->interpreter->num_inputs()) {
    return {};
  }
  const auto &shape = impl_->interpreter->input_shape(index);
  return std::vector<int>(shape.begin(), shape.end());
}

std::vector<int> TFLiteInterpreter::getOutputTensorDims(size_t index) {
  if (index >= impl_->interpreter->num_outputs()) {
    return {};
  }
  const auto view = impl_->interpreter->output(index);
  return std::vector<int>(view.shape.begin(), view.shape.end());
}

}  // namespace c8
