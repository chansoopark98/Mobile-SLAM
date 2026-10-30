// Loaded image targets (US-013). Line references are to
// PUB:reality/app/xr/js/tracking-controller.cc and PUB:reality/app/xr/js/src/tracking-controller.ts.

#include "image_target/image_target_set.h"

#include <set>
#include <utility>

#include "common/check.h"
#include "image_target/target_json.h"
#include "image_target/target_loader.h"
#include "reality/engine/api/reality.capnp.h"

namespace slam_refact {

std::string ImageTargetSet::Set(const std::vector<ImageTargetInput>& inputs,
                                const PinholeIntrinsics& device_k, ShaderNumericModel model) {
  if (inputs.size() > static_cast<size_t>(kMaxActiveImageTargets)) {
    return "too many active image targets: " + std::to_string(inputs.size()) + " > " +
           std::to_string(kMaxActiveImageTargets);
  }
  // Parse and validate everything first so an error leaves the set unchanged.
  std::vector<image_target::ImageTargetSpec> specs(inputs.size());
  std::set<std::string> seen;
  for (size_t i = 0; i < inputs.size(); ++i) {
    std::string error;
    if (!image_target::ParseImageTargetJson(inputs[i].json, &specs[i], &error)) {
      return "image target " + std::to_string(i) + ": " + error;
    }
    const image_target::ImageTargetSpec& s = specs[i];
    if (s.name.empty()) return "image target " + std::to_string(i) + ": empty name";
    if (!seen.insert(s.name).second) return "duplicate image target name: " + s.name;
    const GrayImageView& l = inputs[i].luminance;
    if (l.data == nullptr || l.width <= 0 || l.height <= 0 || l.stride < l.width) {
      return "image target " + s.name + ": luminance needs pixels, width, height > 0 and stride >= width";
    }
  }
  // Load the new targets (processNewDetectionImageTexture). A name that is loaded already keeps
  // its image whatever the imagePath: c8EmAsm_processNewDetectionImageTexture calls
  // detectionImages_.insert(name, image), which does not replace an existing entry
  // (tracking-controller.cc:483), so a different image under a loaded name is dropped; the JS layer
  // still records the new imagePath as loaded (setActiveImageTargets, currentTargetUrls_).
  DetectionImageMap loaded;
  for (size_t i = 0; i < inputs.size(); ++i) {
    const image_target::ImageTargetSpec& s = specs[i];
    if (images_.count(s.name) != 0) {
      image_paths_[s.name] = s.image_path;
      continue;
    }
    loaded.insert(std::make_pair(
        s.name, image_target::LoadDetectionImage(s, inputs[i].luminance, device_k, model)));
  }
  // Unload names that are no longer configured, then insert the new images.
  for (auto it = images_.begin(); it != images_.end();) {
    if (seen.count(it->first) == 0) {
      image_paths_.erase(it->first);
      it = images_.erase(it);
    } else {
      ++it;
    }
  }
  for (size_t i = 0; i < inputs.size(); ++i) {
    const auto node = loaded.find(specs[i].name);
    if (node == loaded.end()) continue;
    images_.insert(std::make_pair(node->first, std::move(node->second)));
    image_paths_[specs[i].name] = specs[i].image_path;
  }
  RebuildPtrMap();
  return "";
}

void ImageTargetSet::Unload(const std::vector<std::string>& names) {
  for (const std::string& n : names) {  // :502-510
    images_.erase(n);
    image_paths_.erase(n);
  }
  RebuildPtrMap();
}

void ImageTargetSet::Clear() {  // clearImageTargetState (:124-131)
  images_.clear();
  image_paths_.clear();
  ++clear_generation_;
  RebuildPtrMap();
}

std::vector<std::string> ImageTargetSet::names() const {
  std::vector<std::string> out;
  for (const auto& e : images_) out.push_back(e.first);
  return out;
}

const c8::DetectionImage& ImageTargetSet::at(const std::string& name) const {
  const auto it = images_.find(name);
  SR_CHECK(it != images_.end(), "ImageTargetSet: unknown image target");
  return it->second;
}

const c8::DetectionImage* ImageTargetSet::find(const std::string& name) const {
  const auto it = images_.find(name);
  return it == images_.end() ? nullptr : &it->second;
}

void ImageTargetSet::RebuildPtrMap() {
  ptrs_.clear();
  for (auto& e : images_) ptrs_.insert(std::make_pair(e.first, &e.second));
}

namespace image_target {

DetectedImageType ToDetectedImageType(c8::DetectionImageType t) {
  switch (t) {
    case c8::DetectionImageType::PLANAR: return DetectedImageType::kPlanar;
    case c8::DetectionImageType::CURVY: return DetectedImageType::kCurvy;
    default: return DetectedImageType::kUnspecified;
  }
}

CurvyGeometryOut CurvyGeometryOf(const c8::DetectionImage& im) {
  c8::CurvyGeometry::Data d;
  im.toCurvyGeometry(c8::CurvyGeometry::Builder(&d));
  return {d.curvyCircumferenceTop, d.curvyCircumferenceBottom, d.curvySideLength,
          d.targetCircumferenceTop, d.height, d.topRadius, d.bottomRadius, d.arcLengthRadians,
          d.arcStartRadians};
}

}  // namespace image_target
}  // namespace slam_refact
