// Loaded image targets (contract FULL-3, US-013): the opaque public slam_refact::ImageTargetSet.
//
// Equivalent of EngineData::detectionImages_ of the original tracking controller
// (PUB:reality/app/xr/js/tracking-controller.cc:93-94, 476-510, 543-546): it lives in the glue
// (it survives rebuildEngine) and every XREngine reads it through a pointer (setDetectionImages).
// Loading uses TargetLoader (CPU pyramid port of DetectionImageLoader) with the device intrinsics
// table (Intrinsics::getCameraIntrinsics(deviceModel), tracking-controller.cc:438-447).

#ifndef SLAM_REFACT_IMAGE_TARGET_IMAGE_TARGET_SET_H_
#define SLAM_REFACT_IMAGE_TARGET_IMAGE_TARGET_SET_H_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "image_target/image_target_spec.h"
#include "reality/engine/imagedetection/detection-image-tracker.h"
#include "slam_refact/image_targets.h"
#include "slam_refact/types.h"

namespace slam_refact {

class ImageTargetSet {
 public:
  ImageTargetSet() = default;
  ImageTargetSet(const ImageTargetSet&) = delete;
  ImageTargetSet& operator=(const ImageTargetSet&) = delete;

  // setActiveImageTargets semantics (tracking-controller.ts:1130-1180, keyed by imagePath there;
  // here by name + imagePath): every input is parsed and loaded before anything changes; a target
  // whose name is loaded already is kept as it is, also when its imagePath differs (the original
  // C++ inserts into a map and keeps the existing entry; tracking-controller.cc:483). Errors: more than
  // kMaxActiveImageTargets inputs, duplicate names, JSON errors, empty luminance. Curvy
  // (CYLINDER / CONICAL) targets load like flat ones since US-014. Returns "" or the error message
  // (then nothing changed).
  std::string Set(const std::vector<ImageTargetInput>& inputs, const PinholeIntrinsics& device_k,
                  ShaderNumericModel model);
  // c8EmAsm_unloadDetectionImages.
  void Unload(const std::vector<std::string>& names);
  // c8EmAsm_clearDetectionImages: empties the set and bumps clear_generation() so the engine
  // resets the image tracking state (clearImageTargetState: setImageTrackingDisabled(true/false)).
  void Clear();

  bool empty() const { return images_.empty(); }
  size_t size() const { return images_.size(); }
  std::vector<std::string> names() const;
  uint64_t clear_generation() const { return clear_generation_; }

  // DetectionImagePtrMap of the loaded images (the 938 argument). The pointers stay valid until
  // the next Set / Unload / Clear (std::map nodes do not move).
  const DetectionImagePtrMap& ptr_map() const { return ptrs_; }
  // Loaded image by name; aborts (SR_CHECK) when absent.
  const c8::DetectionImage& at(const std::string& name) const;
  // Loaded image by name, nullptr when absent (the non-aborting form of at()).
  const c8::DetectionImage* find(const std::string& name) const;

 private:
  void RebuildPtrMap();

  DetectionImageMap images_;
  std::map<std::string, std::string> image_paths_;
  DetectionImagePtrMap ptrs_;
  uint64_t clear_generation_ = 0;
};

namespace image_target {

// ImageTargetTypeMsg of a loaded image and DetectionImage::toCurvyGeometry as a plain struct.
DetectedImageType ToDetectedImageType(c8::DetectionImageType t);
CurvyGeometryOut CurvyGeometryOf(const c8::DetectionImage& im);

}  // namespace image_target
}  // namespace slam_refact

#endif  // SLAM_REFACT_IMAGE_TARGET_IMAGE_TARGET_SET_H_
