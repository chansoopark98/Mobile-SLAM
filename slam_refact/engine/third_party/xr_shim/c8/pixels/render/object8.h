// Subset shim of c8/pixels/render/object8.h (US-019).
//
// ears/ear-types.cc includes object8.h only for the camera clip planes used to project the face
// mesh (ear-types.cc earLiftLandmarksTo3D). The original header pulls the renderer (materials,
// geometry, model data views); this shim keeps just the two constants, with the values of
// PUB:c8/pixels/render/object8.h:21-22.
#pragma once

namespace c8 {

constexpr const float CAM_PROJECTION_NEAR_CLIP = 1e-2f;
constexpr const float CAM_PROJECTION_FAR_CLIP = 1e3f;

}  // namespace c8
