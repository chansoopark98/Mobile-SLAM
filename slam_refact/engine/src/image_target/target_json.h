// Target JSON reader (US-012): the CLI / Studio image target JSON -> ImageTargetSpec.
//
// Field mapping follows the original JS layer, which builds ImageTargetMetadata from the JSON
// (PUB:reality/app/xr/js/src/tracking-controller.ts:822-873):
//   name, type (CYLINDER|CONICAL -> CURVY, PLANAR -> PLANAR, else UNSPECIFIED),
//   properties.{originalWidth, originalHeight, left, top, width, height},
//   properties.isRotated (true or "true"), properties.staticOrientation.{rollAngle, pitchAngle},
//   properties.{cylinderCircumferenceTop, cylinderCircumferenceBottom, cylinderSideLength,
//   targetCircumferenceTop} for curvy targets.
// Integer fields are truncated toward zero like the capnp Int32 setters. Unlike the JS layer (which
// writes 0 or NaN for a missing field), a missing or mistyped field the engine uses is an error.

#ifndef SLAM_REFACT_IMAGE_TARGET_TARGET_JSON_H_
#define SLAM_REFACT_IMAGE_TARGET_TARGET_JSON_H_

#include <string>
#include <string_view>

#include "image_target/image_target_spec.h"

namespace slam_refact::image_target {

// Returns true and fills *out on success; returns false with a message in *error otherwise
// (*out is then unspecified). Input errors never abort.
bool ParseImageTargetJson(std::string_view json, ImageTargetSpec* out, std::string* error);

}  // namespace slam_refact::image_target

#endif  // SLAM_REFACT_IMAGE_TARGET_TARGET_JSON_H_
