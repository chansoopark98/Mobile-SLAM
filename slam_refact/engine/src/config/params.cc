#include "config/params.h"

#include <variant>

#include "c8/parameter-data.h"
#include "common/check.h"

namespace slam_refact::config {

// ---- Settings::Get(): version-cached getters (one per prefix) ----

#define SR_CONFIG_BEGIN(prefix)                                    \
  const prefix##Settings& prefix##Settings::Get() {                \
    static prefix##Settings cached;                                \
    static bool loaded = false;                                    \
    static int cached_version = 0;                                 \
    c8::ParameterData& params = c8::globalParams();                \
    if (loaded && cached_version == params.version()) {            \
      return cached;                                               \
    }                                                              \
    prefix##Settings fresh;
#define SR_CONFIG_KEY(prefix, field, key, kind, ctype, def, getter, owner, read) \
    fresh.field = params.getOrSet<ctype>(key, fresh.field);
#define SR_CONFIG_END(prefix)                                      \
    cached = fresh;                                                \
    cached_version = params.version();                             \
    loaded = true;                                                 \
    return cached;                                                 \
  }
#define SR_CONFIG_UNRECOVERED(prefix, key, func)
#include "config/config_defaults.inc"
#undef SR_CONFIG_BEGIN
#undef SR_CONFIG_KEY
#undef SR_CONFIG_END
#undef SR_CONFIG_UNRECOVERED

void RegisterDefaults() {
  c8::ParameterData& params = c8::globalParams();
  for (const ConfigKeyInfo& info : ConfigKeyTable()) {
    if (!info.default_value.has_value()) {
      continue;
    }
    std::visit([&](auto value) { params.getOrSet<decltype(value)>(info.key, value); },
               *info.default_value);
  }
}

void ApplyParamOverrides(const std::vector<std::pair<std::string, ParamValue>>& overrides) {
  c8::ParameterData& params = c8::globalParams();
  for (const auto& entry : overrides) {
    const std::string& key = entry.first;
    const ParamValue& value = entry.second;
    const ConfigKeyInfo* info = FindConfigKey(key.c_str());
    SR_CHECK(info != nullptr, "param override: unknown config key");
    SR_CHECK(info->default_value.has_value(), "param override: key has no recovered type");
    SR_CHECK(info->default_value->index() == value.index(),
             "param override: value type does not match the key's getter type");
    std::visit([&](auto v) { params.set<decltype(v)>(key.c_str(), v); }, value);
  }
}

int ParamsVersion() { return c8::globalParams().version(); }

}  // namespace slam_refact::config
