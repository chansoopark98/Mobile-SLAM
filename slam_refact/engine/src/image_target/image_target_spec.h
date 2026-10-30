// Image target description consumed by the target loader (US-012).
//
// Plain-struct replacement of the capnp ImageTargetMetadata the original JS layer assembles from a
// target JSON (PUB:reality/app/xr/js/src/tracking-controller.ts:822-873,
// PUB:reality/engine/api/reality.capnp:408-428). The image size (imageWidth/imageHeight) is not
// part of the spec: the original takes it from the decoded image (img.naturalWidth/Height), here
// from the luminance image handed to the loader.

#ifndef SLAM_REFACT_IMAGE_TARGET_IMAGE_TARGET_SPEC_H_
#define SLAM_REFACT_IMAGE_TARGET_IMAGE_TARGET_SPEC_H_

#include <string>

namespace slam_refact::image_target {

// ImageTargetTypeMsg (reality.capnp): CYLINDER and CONICAL JSON targets are both CURVY.
enum class ImageTargetType {
  kUnspecified = 0,
  kPlanar = 1,
  kCurvy = 2,
};

struct ImageTargetSpec {
  std::string name;
  ImageTargetType type = ImageTargetType::kUnspecified;
  // JSON "type" as written ("PLANAR", "CYLINDER", "CONICAL", ...), kept for the API type.
  std::string type_name;
  // JSON "imagePath" (the luminance image URL of the CLI output).
  std::string image_path;

  // properties.originalWidth/originalHeight/left/top/width/height (crop in original pixels).
  int original_image_width = 0;
  int original_image_height = 0;
  int crop_original_image_x = 0;
  int crop_original_image_y = 0;
  int crop_original_image_width = 0;
  int crop_original_image_height = 0;
  bool is_rotated = false;

  // properties.staticOrientation (Studio/Desktop targets only).
  bool is_static_target = false;
  float roll_angle = 0.0f;
  float pitch_angle = 0.0f;

  // properties.cylinder* / targetCircumferenceTop (curvy targets only).
  float curvy_circumference_top = 0.0f;
  float curvy_circumference_bottom = 0.0f;
  float curvy_side_length = 0.0f;
  float target_circumference_top = 0.0f;
};

}  // namespace slam_refact::image_target

#endif  // SLAM_REFACT_IMAGE_TARGET_IMAGE_TARGET_SPEC_H_
