// Engine private header: reality.lighting exposure of the web path (spec 06 §4, contract FULL-1).
// Port of PUB:reality/engine/lighting/lighting-request-executor.cc:56-106 (pyramid branch). The
// score is the vendored c8::LightingEstimator (reality/engine/lighting/lighting-estimator.cc, which
// calls PUB:c8/pixels/pixel-transforms.cc:1927-1956 estimateExposureScore); before 2026-10-22 it
// was a hand port of that function.

#ifndef SLAM_REFACT_ENGINE_LIGHTING_H_
#define SLAM_REFACT_ENGINE_LIGHTING_H_

#include <cstdint>

#include "c8/pixels/gr8-pyramid.h"

namespace slam_refact {

// Temperature of every web lighting estimate (lighting-request-executor.cc:105; binary func 702,
// DCMP:64815 stores 1170939904 = 6500.0f).
inline constexpr float kWebLightingTemperature = 6500.0f;

// LightingEstimator::estimateLighting (-> estimateExposureScore) of a strided image whose first channel is
// scored every num_channels bytes.
float ExposureScore(int rows, int cols, int row_bytes, const uint8_t* pixels, int num_channels);

// LightingRequestExecutor web branch: ExposureScore over the R channel (4-channel stride) of the
// last pyramid level.
float EstimateExposure(const c8::Gr8Pyramid& pyramid);

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_LIGHTING_H_
