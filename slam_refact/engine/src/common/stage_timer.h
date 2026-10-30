// Stage timing hook (architecture.md §3.1). Names follow the original ScopeTimer scope names.
// The sink is optional and nothing in this repository installs one (the replay harness takes its
// per-stage times from StageTimings), so StageTimer is a no-op beyond two branch checks.
// Single-threaded by design (§4.6).

#ifndef SLAM_REFACT_COMMON_STAGE_TIMER_H_
#define SLAM_REFACT_COMMON_STAGE_TIMER_H_

#include <chrono>
#include <cstdint>

namespace slam_refact {

// Called once per finished scope with its elapsed wall time.
using StageTimerSink = void (*)(const char* name, int64_t elapsed_ns, void* user);

namespace internal {

struct StageTimerSinkSlot {
  StageTimerSink sink = nullptr;
  void* user = nullptr;
};

inline StageTimerSinkSlot& StageTimerSinkInstance() {
  static StageTimerSinkSlot slot;
  return slot;
}

}  // namespace internal

// Installs (or, with nullptr, removes) the process-global sink.
inline void SetStageTimerSink(StageTimerSink sink, void* user) {
  internal::StageTimerSinkInstance() = {sink, user};
}

class StageTimer {
 public:
  explicit StageTimer(const char* name) : name_(name) {
    if (internal::StageTimerSinkInstance().sink != nullptr) {
      active_ = true;
      start_ = std::chrono::steady_clock::now();
    }
  }

  ~StageTimer() {
    if (!active_) {
      return;
    }
    const internal::StageTimerSinkSlot& slot = internal::StageTimerSinkInstance();
    if (slot.sink == nullptr) {
      return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - start_;
    slot.sink(name_, std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count(),
              slot.user);
  }

  StageTimer(const StageTimer&) = delete;
  StageTimer& operator=(const StageTimer&) = delete;

 private:
  const char* name_;
  bool active_ = false;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_COMMON_STAGE_TIMER_H_
