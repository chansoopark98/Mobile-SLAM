// Replacement for public xr_reference c8/parameter-data.cc without nlohmann::json.
//
// Origin: slam_refact/tools/re/public_build/shim_src/parameter-data-shim.cc (RE lane B), moved
// here by WP0a (architecture.md §2.2 item 4). Change from the lane-B file: the unsupported JSON
// entry points use C8_THROW (log + std::abort under C8_NO_EXCEPT, §2.5 D-15) instead of `throw`,
// so the file builds with -fno-exceptions.
//
// Keeps the header (c8/parameter-data.h) and its semantics for get/getOrSet/set/contains/
// deleteKey: values are stored per key in a std::variant. JSON merge/dump are not supported and
// abort so that any accidental use is visible.
#include <cstdint>
#include <map>
#include <variant>

#include "c8/exceptions.h"
#include "c8/parameter-data.h"

namespace c8 {

using ParamValue = std::variant<bool, int, int64_t, uint32_t, uint64_t, float, double, String>;

struct ParameterData::Parameters {
  std::map<String, ParamValue> values;
};

ParameterData::ParameterData() : parameters_(new Parameters()) {}
ParameterData::~ParameterData() { delete parameters_; }

template <typename T>
T ParameterData::get(const char *key) const {
  return std::get<T>(parameters_->values.at(key));
}

template <typename T>
T ParameterData::getOrSet(const char *key, T fallback) {
  auto it = parameters_->values.find(key);
  if (it != parameters_->values.end()) {
    return std::get<T>(it->second);
  }
  parameters_->values[key] = fallback;
  return fallback;
}

template <typename T>
void ParameterData::set(const char *key, T val) {
  ++version_;
  parameters_->values[key] = val;
}

bool ParameterData::contains(const char *key) const {
  return parameters_->values.count(key) > 0;
}

void ParameterData::deleteKey(const char *key) {
  ++version_;
  parameters_->values.erase(key);
}

void ParameterData::mergeJsonString(const String &) {
  C8_THROW("parameter-data-shim: JSON merge not supported");
}

String ParameterData::toJsonString() const {
  C8_THROW("parameter-data-shim: JSON dump not supported");
}

void ScopedParameterUpdates::mergeJsonString(const String &) {
  C8_THROW("parameter-data-shim: JSON merge not supported");
}

#define C8_PROBE_INSTANTIATE(T)                                 \
  template T ParameterData::get<T>(const char *key) const;      \
  template T ParameterData::getOrSet<T>(const char *key, T val); \
  template void ParameterData::set<T>(const char *key, T val);

C8_PROBE_INSTANTIATE(bool)
C8_PROBE_INSTANTIATE(int)
C8_PROBE_INSTANTIATE(int64_t)
C8_PROBE_INSTANTIATE(uint32_t)
C8_PROBE_INSTANTIATE(uint64_t)
C8_PROBE_INSTANTIATE(float)
C8_PROBE_INSTANTIATE(double)
C8_PROBE_INSTANTIATE(String)
#undef C8_PROBE_INSTANTIATE

ParameterData &globalParams() {
  static ParameterData d;
  return d;
}

}  // namespace c8
