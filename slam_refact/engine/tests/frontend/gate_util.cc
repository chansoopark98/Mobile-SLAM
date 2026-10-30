#include "gate_util.h"

#include <cstdint>
#include <cstring>
#include <sstream>

#include "reality/engine/features/gr8gl.h"

namespace c8 {
// External linkage in reality/engine/features/gr8gl.cc:601 (no header declaration).
void keypointsForCropJs(ConstRGBA8888PlanePixels s, uint8_t scale, int8_t roi, int edgeThreshold,
                        Vector<ImagePointLocation>& keypoints);
}  // namespace c8

namespace slam_refact::frontend::testing {
namespace {

constexpr int kBudgets[4] = {199, 138, 96, 67};
constexpr int kEdge = 21;

bool SameFloatBits(float a, float b) {
  uint32_t x = 0, y = 0;
  std::memcpy(&x, &a, 4);
  std::memcpy(&y, &b, 4);
  return x == y;
}

}  // namespace

std::vector<std::vector<c8::ImagePointLocation>> CpuHalfPerLevel(const c8::Gr8Pyramid& pyramid) {
  std::vector<std::vector<c8::ImagePointLocation>> out(pyramid.levels.size());
  for (size_t li = 0; li < pyramid.levels.size(); ++li) {
    c8::Vector<c8::ImagePointLocation> kps;
    c8::keypointsForCropJs(pyramid.level(static_cast<int>(li)), static_cast<uint8_t>(li), -1, kEdge,
                           kps);
    const auto level = pyramid.level(static_cast<int>(li));
    c8::Gr8Gl::retainBest(kps, kBudgets[li], level.cols(), level.rows(), kEdge, true);
    out[li].assign(kps.begin(), kps.end());
  }
  return out;
}

int CompareCpuHalfWithBinary(const std::vector<std::vector<c8::ImagePointLocation>>& engine,
                             const Json& binary_golden, std::string* detail) {
  const Json& levels = binary_golden["levels"];
  int bad = 0;
  detail->clear();
  if (levels.size() != engine.size()) {
    *detail = "level count differs";
    return static_cast<int>(engine.size());
  }
  for (size_t li = 0; li < engine.size(); ++li) {
    const Json& list = levels[li]["list"];
    bool ok = list.size() == engine[li].size();
    size_t first = 0;
    for (size_t i = 0; ok && i < list.size(); ++i) {
      const Json& r = list[i];
      const c8::ImagePointLocation& p = engine[li][i];
      ok = SameFloatBits(p.pt.x, static_cast<float>(r[0].number())) &&
           SameFloatBits(p.pt.y, static_cast<float>(r[1].number())) &&
           SameFloatBits(p.size, static_cast<float>(r[2].number())) &&
           static_cast<int>(p.scale) == static_cast<int>(r[3].number()) &&
           SameFloatBits(p.angle, static_cast<float>(r[4].number())) &&
           SameFloatBits(p.gravityAngle, static_cast<float>(r[5].number())) &&
           SameFloatBits(p.response, static_cast<float>(r[6].number())) &&
           static_cast<int>(p.roi) == static_cast<int>(r[7].number()) &&
           static_cast<int>(p.gravityPortion) == static_cast<int>(r[8].number());
      first = i;
    }
    if (!ok) {
      ++bad;
      if (detail->empty()) {
        std::ostringstream ss;
        ss << "level " << li << ": engine " << engine[li].size() << " kept vs binary "
           << list.size() << ", first difference at index " << first;
        *detail = ss.str();
      }
    }
  }
  return bad;
}

std::vector<ExpectedPoint> ExpectedBasePoints(const Json& binary_golden,
                                              const c8::Gr8Pyramid& pyramid) {
  const auto layer_scale = pyramid.mapLevelsToBase();
  const c8::LevelLayout b = pyramid.levels[0];
  std::vector<ExpectedPoint> out;
  const Json& levels = binary_golden["levels"];
  for (size_t li = 0; li < levels.size(); ++li) {
    const Json& list = levels[li]["list"];
    const c8::LevelLayout l = pyramid.levels[li];
    for (size_t i = 0; i < list.size(); ++i) {
      float x = static_cast<float>(list[i][0].number());
      float y = static_cast<float>(list[i][1].number());
      x = x * layer_scale[li][0] + layer_scale[li][1];
      y = y * layer_scale[li][2] + layer_scale[li][3];
      if (l.rotated) {
        const float tempx = x;
        x = b.w - 1 - y;
        y = tempx;
      }
      out.push_back({x, y, static_cast<int>(li)});
    }
  }
  return out;
}

bool Stage23BitExact(const AtlasStats& st) {
  for (int ch = 0; ch < 4; ++ch) {
    if (st.equal[ch] != st.pixels) {
      return false;
    }
  }
  return st.pixels > 0;
}

bool RebuildPass(const AtlasStats& st) {
  return st.identical_fraction(0) >= 0.9999 && st.identical_fraction(1) >= 0.9999 &&
         st.max_diff[0] <= 1 && st.max_diff[1] <= 1 && st.identical_fraction(2) >= 0.9999 &&
         st.identical_fraction(3) >= 0.9999;
}

}  // namespace slam_refact::frontend::testing
