// Sky model identity and the semantics inference step (US-020; docs/spec/09-sky.md §2.3).
//
// SkyModel = the original semantics-worker.cc + semantics-worker.js on the slam_refact ML engine:
// the vendored public SemanticsClassifier (reality/engine/semantics/semantics-classifier.cc, with
// tflite-multiclassifier.cc: RGBA / 255 -> float32 NHWC, class 0 of the [1, 256, 144, 20] output)
// running the .tflite bytes through the TFLiteInterpreter shim (src/ml). The deployed model is
// assets/references/xr_reference/engine/resources/semantics-model.tflite (never copied into the repo).
#ifndef SLAM_REFACT_SKY_SKY_MODEL_H_
#define SLAM_REFACT_SKY_SKY_MODEL_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "sky/sky_types.h"

namespace slam_refact::sky {

// sha256 of the deployed model; equals the git-lfs oid of PUB:reality/engine/semantics/data/
// semantics_mobilenetv3_separable_quarter_fp16.tflite.
inline constexpr char kSemanticsModelSha256[] =
    "b25bebe3ec1b7390feec937d1e4eafb3d6f2104d584a738c0bdf6486030b70a0";
inline constexpr char kSemanticsModelPath[] =
    "assets/references/xr_reference/engine/resources/semantics-model.tflite";

std::string Sha256Hex(const uint8_t* data, size_t size);
inline std::string Sha256Hex(const std::vector<uint8_t>& d) { return Sha256Hex(d.data(), d.size()); }
bool ReadFileBytes(const std::string& path, std::vector<uint8_t>* out, std::string* error);

// semantics-worker.js `invoke` result: the class-0 score plane and the share of pixels above 0.4.
struct SkyInference {
  std::vector<float> data;  // kModelInputHeight x kModelInputWidth, row-major
  double percentage = 0.0;
};

class SkyModel {
 public:
  SkyModel();
  ~SkyModel();
  SkyModel(const SkyModel&) = delete;
  SkyModel& operator=(const SkyModel&) = delete;

  // Loads model bytes (sha256 checked against kSemanticsModelSha256 when `check_sha`).
  bool Load(const std::vector<uint8_t>& model, bool check_sha, std::string* error);
  bool loaded() const { return impl_ != nullptr; }

  // `rgba` = kModelInputHeight rows x kModelInputWidth cols RGBA8 (the resized pixels of
  // layers-controller.ts). Returns false before Load().
  bool Invoke(const uint8_t* rgba, SkyInference* out);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_SKY_MODEL_H_
