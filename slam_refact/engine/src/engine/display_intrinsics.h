// Engine private header: display intrinsics / projection matrix of reality.intrinsics (spec 06 §1,
// contract FULL-1). Port of PUB:c8/geometry/intrinsics.cc (cropAndScaleIntrinsics, toClipSpace) and
// of the web branch of PUB:reality/engine/camera/camera-request-executor.cc.

#ifndef SLAM_REFACT_ENGINE_DISPLAY_INTRINSICS_H_
#define SLAM_REFACT_ENGINE_DISPLAY_INTRINSICS_H_

#include <array>

#include "slam_refact/engine_config.h"
#include "slam_refact/types.h"
#include "slam_refact/xr_controller.h"

namespace slam_refact {

// Intrinsics::cropAndScaleIntrinsics (intrinsics.cc:245-264).
PinholeIntrinsics CropAndScaleIntrinsics(const PinholeIntrinsics& k, int width, int height);

// Intrinsics::toClipSpace (intrinsics.cc:81-130), column-major 4x4.
std::array<float, 16> ToClipSpace(const PinholeIntrinsics& k, float near_clip, float far_clip);

// Capture intrinsics of the engine configuration with the rule the engine's K uses: device table
// (getCaptureIntrinsics, intrinsics.cc:270-273), ACC-4 accuracy focal, or the diagnostic
// intrinsics_override cropped and scaled to the capture size (the last two are not original).
PinholeIntrinsics CaptureIntrinsicsFor(const EngineConfig& cfg);

// CameraRequestExecutor::execute, web branch (camera-request-executor.cc:135-150, 192-198):
// toClipSpace(getDisplayIntrinsics(capture, graphics), near, far) when far > 0, else the fixed
// fallback matrix. graphics.texture_width/height must already be resolved (> 0).
std::array<float, 16> DisplayProjection(const PinholeIntrinsics& capture,
                                        const GraphicsIntrinsics& graphics);

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_DISPLAY_INTRINSICS_H_
