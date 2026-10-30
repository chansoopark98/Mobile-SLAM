// Vendored public reference runtime closure checks (WP0a, architecture.md §7.3 "vendor").
//
// 1. The lane-B probe built from third_party/xr_source (with libcxx-compat.patch) prints the same
//    result lines as the em++/libc++ build of the unmodified public sources
//    (tests/data/vendor/probe-wasm.out). Only the probe's own result lines ("A.", "B.", "C.",
//    "D.") are compared: the header line carries __cplusplus and the remaining lines are Ceres
//    miniglog/VLOG output, which differs between builds.
// 2. C8_THROW logs and aborts (C8_NO_EXCEPT, D-15).
#include <csignal>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "c8/exceptions.h"
#include "gtest/gtest.h"

#ifndef SR_TEST_DATA_DIR
#error "SR_TEST_DATA_DIR must point at slam_refact/engine/tests/data"
#endif

namespace slam_refact {
namespace vendor_test {

int RunVendorProbe();

namespace {

std::vector<std::string> ResultLines(const std::string& text) {
  static const std::regex kResultLine("^[A-D]\\.");
  std::vector<std::string> out;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    if (std::regex_search(line, kResultLine)) out.push_back(line);
  }
  return out;
}

TEST(VendorProbe, MatchesWasmLibcxxReference) {
  std::ifstream ref_in(std::string(SR_TEST_DATA_DIR) + "/vendor/probe-wasm.out");
  ASSERT_TRUE(ref_in.good()) << "missing tests/data/vendor/probe-wasm.out";
  std::stringstream ref_text;
  ref_text << ref_in.rdbuf();
  const std::vector<std::string> expected = ResultLines(ref_text.str());
  ASSERT_GE(expected.size(), 30u);

  testing::internal::CaptureStdout();
  const int rc = RunVendorProbe();
  const std::string out = testing::internal::GetCapturedStdout();
  ASSERT_EQ(rc, 0);
  const std::vector<std::string> actual = ResultLines(out);

  ASSERT_EQ(actual.size(), expected.size()) << out;
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(actual[i], expected[i]) << "result line " << i;
  }
}

TEST(VendorDeathTest, C8ThrowLogsAndAborts) {
  EXPECT_EXIT(C8_THROW("slam_refact vendor death test"), testing::KilledBySignal(SIGABRT), "");
}

}  // namespace
}  // namespace vendor_test
}  // namespace slam_refact
