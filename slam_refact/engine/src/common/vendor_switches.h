// Runtime switches of the vendored reference code that are not part of the reference runtime.
// The definitions live in the vendored translation units (third_party/xr_source/patches/); the
// engine only declares them here so that the vendored headers stay untouched.

#ifndef SLAM_REFACT_COMMON_VENDOR_SWITCHES_H_
#define SLAM_REFACT_COMMON_VENDOR_SWITCHES_H_

#include "slam_refact/engine_config.h"

namespace c8 {

// P4 candidate V1 (v1-pose-estimation-exact-limit.patch, bundle-residual-analytic.cc). Default false:
// PoseEstimationAnalytic::Evaluate then computes the reference expression unchanged.
void setPoseEstimationAnalyticExactLimit(bool on);
bool poseEstimationAnalyticExactLimit();

}  // namespace c8

namespace slam_refact {

// Process-global like config::SetAccuracy(); called by the Engine constructor, so a later engine
// built with the switch off turns it off again.
inline void ApplyVendorAccuracy(const AccuracyConfig& accuracy) {
  c8::setPoseEstimationAnalyticExactLimit(accuracy.enabled && accuracy.v1_exact_limit);
}

}  // namespace slam_refact

#endif  // SLAM_REFACT_COMMON_VENDOR_SWITCHES_H_
