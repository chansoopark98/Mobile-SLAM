// SR_CHECK: log and abort, the engine-side equivalent of C8_THROW under C8_NO_EXCEPT
// (PUB:c8/exceptions.h:62-69, architecture.md §3.1, D-15). The engine is built with
// -fno-exceptions; there is no recoverable error path.

#ifndef SLAM_REFACT_COMMON_CHECK_H_
#define SLAM_REFACT_COMMON_CHECK_H_

#include <cstdio>
#include <cstdlib>

namespace slam_refact::internal {

[[noreturn]] inline void CheckFailed(const char* file, int line, const char* expr,
                                     const char* msg) {
  std::fprintf(stderr, "SR_CHECK failed at %s:%d: (%s) %s\n", file, line, expr, msg);
  std::fflush(stderr);
  std::abort();
}

}  // namespace slam_refact::internal

// msg must be a const char*.
#define SR_CHECK(cond, msg)                                                       \
  do {                                                                            \
    if (!(cond)) {                                                                \
      ::slam_refact::internal::CheckFailed(__FILE__, __LINE__, #cond, (msg));     \
    }                                                                             \
  } while (false)

#endif  // SLAM_REFACT_COMMON_CHECK_H_
