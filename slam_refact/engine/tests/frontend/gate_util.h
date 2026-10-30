// Shared gate procedures for the WP1 unit and evidence tests (F-H2, F-H3, F-H4, F-H7).

#ifndef SLAM_REFACT_TESTS_FRONTEND_GATE_UTIL_H_
#define SLAM_REFACT_TESTS_FRONTEND_GATE_UTIL_H_

#include <string>
#include <vector>

#include "c8/pixels/gr8-pyramid.h"
#include "frontend/frontend.h"
#include "reality/engine/features/image-point.h"
#include "test_util.h"

namespace slam_refact::frontend::testing {

// Engine CPU half per level: vendored keypointsForCropJs (func 917) + Gr8Gl::retainBest
// (func 918) with the level budgets of nKeypoints = 500 ([199, 138, 96, 67], S1 §5.7), edge 21.
std::vector<std::vector<c8::ImagePointLocation>> CpuHalfPerLevel(const c8::Gr8Pyramid& pyramid);

// F-H2: compares the engine lists with a binary_gr8_probe golden (all 9 fields, order included).
// Returns the number of levels that are not BIT_EXACT; *detail describes the first mismatch.
int CompareCpuHalfWithBinary(const std::vector<std::vector<c8::ImagePointLocation>>& engine,
                             const Json& binary_golden, std::string* detail);

// F-H7: level-0 pixel positions the detection path must emit, derived from the binary per-level
// lists with the public scaleToBaseImage arithmetic (gr8gl.cc:969-1071, non-ROI branch).
struct ExpectedPoint {
  float x;
  float y;
  int scale;
};
std::vector<ExpectedPoint> ExpectedBasePoints(const Json& binary_golden,
                                              const c8::Gr8Pyramid& pyramid);

// Gate verdicts of the isolation tests (S1 §8.3 H3/H4, tolerances-v2.json).
bool Stage23BitExact(const AtlasStats& st);
bool RebuildPass(const AtlasStats& st);

}  // namespace slam_refact::frontend::testing

#endif  // SLAM_REFACT_TESTS_FRONTEND_GATE_UTIL_H_
