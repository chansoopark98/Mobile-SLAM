// Writes the libcxx golden case digests as JSON to stdout.
//
//   gen_libcxx_golden --impl=std     (built with em++ 5.0.0 -> tests/data/compat/libcxx-golden.json)
//   gen_libcxx_golden --impl=compat  (same cases through slam_refact::libcxx, for diffing)
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <vector>

#include "libcxx_golden_cases.h"
#include "libcxx_golden_impls.h"

namespace {

template <class Impl>
void Print(const char* toolchain) {
  const std::vector<slam_refact::golden::Case> cases = slam_refact::golden::RunGoldenCases<Impl>();
  std::printf("{\n");
  std::printf("  \"format\": \"slam_refact libcxx golden v1\",\n");
  std::printf("  \"impl\": \"%s\",\n", Impl::kName);
  std::printf("  \"toolchain\": \"%s\",\n", toolchain);
  std::printf("  \"cplusplus\": %ld,\n", static_cast<long>(__cplusplus));
  std::printf("  \"sizeof_ptrdiff_t\": %d,\n", static_cast<int>(sizeof(std::ptrdiff_t)));
  std::printf("  \"case_count\": %d,\n", static_cast<int>(cases.size()));
  std::printf("  \"cases\": [\n");
  for (size_t i = 0; i < cases.size(); ++i) {
    std::printf("    {\"name\": \"%s\", \"digest\": \"%016" PRIx64 "\"}%s\n", cases[i].name.c_str(), cases[i].digest,
                i + 1 < cases.size() ? "," : "");
  }
  std::printf("  ]\n}\n");
}

}  // namespace

int main(int argc, char** argv) {
#if defined(__EMSCRIPTEN__)
  const char* toolchain = "em++ " __clang_version__;
#elif defined(__clang__)
  const char* toolchain = "clang++ " __clang_version__;
#else
  const char* toolchain = "g++ " __VERSION__;
#endif
  const char* impl = argc > 1 ? argv[1] : "--impl=std";
  if (std::strcmp(impl, "--impl=std") == 0) {
    Print<slam_refact::golden::StdImpl>(toolchain);
  } else if (std::strcmp(impl, "--impl=compat") == 0) {
    Print<slam_refact::golden::CompatImpl>(toolchain);
  } else {
    std::fprintf(stderr, "usage: %s --impl=std|compat\n", argv[0]);
    return 2;
  }
  return 0;
}
