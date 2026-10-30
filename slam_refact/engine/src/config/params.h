// Process-global parameters (architecture.md §3.1, §4.6). The store is the vendored
// c8::globalParams() (parameter-data shim), shared with the vendored public code, which reads its
// own keys (Bundle.*, PosePnP.*, FramePoint.*, GlobalMatcher.*) from the same store.

#ifndef SLAM_REFACT_CONFIG_PARAMS_H_
#define SLAM_REFACT_CONFIG_PARAMS_H_

#include <string>
#include <utility>
#include <vector>

#include "config/config_defaults.h"
#include "config/settings.h"
#include "slam_refact/types.h"

namespace slam_refact::config {

// Registers the binary default of every typed key that is not set yet (getOrSet). Keys already
// present keep their value. Idempotent. Lets every reader of a key see the binary default
// regardless of which component reads it first.
void RegisterDefaults();

// Sets EngineConfig::param_overrides in globalParams(). Aborts (SR_CHECK) on an unknown key, an
// unrecovered key, or a value whose alternative does not match the key's ParamKind.
void ApplyParamOverrides(const std::vector<std::pair<std::string, ParamValue>>& overrides);

// globalParams().version().
int ParamsVersion();

}  // namespace slam_refact::config

#endif  // SLAM_REFACT_CONFIG_PARAMS_H_
