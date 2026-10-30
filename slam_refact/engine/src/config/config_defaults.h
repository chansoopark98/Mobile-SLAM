// Table of every config key recovered from xr-slam.wasm (architecture.md §3.1 [M9]:
// 244 entries, 211 unique keys). Generated content lives in config_defaults.inc; this header
// does not depend on c8.

#ifndef SLAM_REFACT_CONFIG_CONFIG_DEFAULTS_H_
#define SLAM_REFACT_CONFIG_CONFIG_DEFAULTS_H_

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>

#include "slam_refact/types.h"

namespace slam_refact::config {

// Getter function of the binary: 881 = bool, 851 = int, 458 = float, 1131 = double.
enum class ParamKind : int32_t {
  kBool = 0,
  kInt = 1,
  kFloat = 2,
  kDouble = 3,
  kUnrecovered = 4,  // no typed default recoverable (VPS/debug keys, out of world-SLAM scope)
};

struct ConfigKeyInfo {
  const char* key;
  ParamKind kind;
  std::optional<ParamValue> default_value;  // empty iff kind == kUnrecovered
  int getter_func;                          // 0 iff kind == kUnrecovered
  int owner_func;                           // function that registers the key
  bool read_by_binary;                      // false: registered only (US-009 revision)
};

#define SR_CONFIG_BEGIN(prefix)
#define SR_CONFIG_KEY(prefix, field, key, kind, ctype, def, getter, owner, read) +1
#define SR_CONFIG_END(prefix)
#define SR_CONFIG_UNRECOVERED(prefix, key, func) +1
inline constexpr int kNumConfigKeys = 0
#include "config/config_defaults.inc"
    ;
#undef SR_CONFIG_BEGIN
#undef SR_CONFIG_KEY
#undef SR_CONFIG_END
#undef SR_CONFIG_UNRECOVERED

// All keys, grouped by prefix, sorted by key.
inline const std::array<ConfigKeyInfo, kNumConfigKeys>& ConfigKeyTable() {
#define SR_CONFIG_BEGIN(prefix)
#define SR_CONFIG_KEY(prefix, field, key, kind, ctype, def, getter, owner, read) \
  ConfigKeyInfo{key, ParamKind::kind, ParamValue(static_cast<ctype>(def)), getter, owner, read},
#define SR_CONFIG_END(prefix)
#define SR_CONFIG_UNRECOVERED(prefix, key, func) \
  ConfigKeyInfo{key, ParamKind::kUnrecovered, std::nullopt, 0, func, true},
  static const std::array<ConfigKeyInfo, kNumConfigKeys> table = {{
#include "config/config_defaults.inc"
  }};
#undef SR_CONFIG_BEGIN
#undef SR_CONFIG_KEY
#undef SR_CONFIG_END
#undef SR_CONFIG_UNRECOVERED
  return table;
}

// nullptr when the key is unknown.
inline const ConfigKeyInfo* FindConfigKey(const char* key) {
  for (const ConfigKeyInfo& info : ConfigKeyTable()) {
    if (std::strcmp(info.key, key) == 0) {
      return &info;
    }
  }
  return nullptr;
}

}  // namespace slam_refact::config

#endif  // SLAM_REFACT_CONFIG_CONFIG_DEFAULTS_H_
