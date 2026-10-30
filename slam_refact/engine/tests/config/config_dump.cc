// Dumps the compiled config defaults for check_config_defaults.py:
//   <key> TAB <kind> TAB <value>
// value: bool/int as decimal, float as 8 hex digits of the IEEE-754 bits, double as 16 hex
// digits. Values come from default-constructed Settings structs (what the engine uses), and
// unrecovered keys from the table.

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "config/config_defaults.h"
#include "config/settings.h"

namespace slam_refact::config {
namespace {

void Print(const char* key, bool v) { std::printf("%s\tbool\t%d\n", key, v ? 1 : 0); }
void Print(const char* key, int v) { std::printf("%s\tint\t%d\n", key, v); }
void Print(const char* key, float v) {
  uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  std::printf("%s\tfloat\t%08" PRIx32 "\n", key, bits);
}
void Print(const char* key, double v) {
  uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  std::printf("%s\tdouble\t%016" PRIx64 "\n", key, bits);
}

void DumpAll() {
#define SR_CONFIG_BEGIN(prefix) { const prefix##Settings s;
#define SR_CONFIG_KEY(prefix, field, key, kind, ctype, def, getter, owner, read) Print(key, s.field);
#define SR_CONFIG_END(prefix) }
#define SR_CONFIG_UNRECOVERED(prefix, key, func) std::printf("%s\tunrecovered\t-\n", key);
#include "config/config_defaults.inc"
#undef SR_CONFIG_BEGIN
#undef SR_CONFIG_KEY
#undef SR_CONFIG_END
#undef SR_CONFIG_UNRECOVERED
}

}  // namespace
}  // namespace slam_refact::config

int main() {
  slam_refact::config::DumpAll();
  return 0;
}
