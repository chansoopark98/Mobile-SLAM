// Probe shim for public xr_reference c8/stats/scope-timer.h.
//
// The original ScopeTimer pulls LatencySummarizer/LoggingContext, which depend on Cap'n Proto
// generated headers (c8/stats/api/*.capnp.h). Timing has no effect on algorithm output, so the
// compile probe replaces it with a no-op that keeps the same call surface used by
// reality/engine/{features,geometry,tracking}.
#pragma once

#include "c8/string.h"
#include "c8/vector.h"

namespace c8 {

class ScopeTimer {
public:
  static ScopeTimer *current() {
    static thread_local ScopeTimer dummy("probe-dummy");
    return &dummy;
  }
  static bool hasCurrent() { return false; }

  ScopeTimer(const String &, bool) noexcept {}
  explicit ScopeTimer(const String &) noexcept {}
  ScopeTimer(const char *) noexcept {}

  void addCounter(const String &, int, int = 0) {}

  ScopeTimer() = delete;
  ScopeTimer(ScopeTimer &&) = default;
  ScopeTimer &operator=(ScopeTimer &&) = default;
  ScopeTimer(const ScopeTimer &) = delete;
  ScopeTimer &operator=(const ScopeTimer &) = delete;
};

}  // namespace c8
