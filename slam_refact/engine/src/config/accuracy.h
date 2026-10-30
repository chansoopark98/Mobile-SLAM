// Accuracy-mode settings (EngineConfig::accuracy, contract-changes.md ACC-1).
//
// Process-global like globalParams() (architecture.md §4.6): Engine construction stores the
// AccuracyConfig here and every module reads it through Accuracy(). The default (enabled == false)
// keeps every original code path unchanged.

#ifndef SLAM_REFACT_CONFIG_ACCURACY_H_
#define SLAM_REFACT_CONFIG_ACCURACY_H_

#include "slam_refact/engine_config.h"

namespace slam_refact::config {

void SetAccuracy(const AccuracyConfig& cfg);
const AccuracyConfig& Accuracy();

}  // namespace slam_refact::config

#endif  // SLAM_REFACT_CONFIG_ACCURACY_H_
