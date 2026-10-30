// Test helpers for the WP1 frontend gates: minimal JSON reader, gzip / PGM file reading,
// SHA-256, capture loading and atlas region statistics. Test-only code.

#ifndef SLAM_REFACT_TESTS_FRONTEND_TEST_UTIL_H_
#define SLAM_REFACT_TESTS_FRONTEND_TEST_UTIL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "frontend/frontend.h"

namespace slam_refact::frontend::testing {

// ---- JSON (RFC 8259 subset sufficient for the golden files: no \u surrogate pairs) ----
class Json {
 public:
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

  static bool Parse(const std::string& text, Json* out, std::string* error);

  bool is_null() const { return type_ == Type::kNull; }
  double number() const { return number_; }
  const std::string& str() const { return str_; }
  size_t size() const { return type_ == Type::kArray ? array_.size() : object_.size(); }
  const Json& operator[](size_t i) const { return array_[i]; }
  // Returns a null value when the key is missing.
  const Json& operator[](const std::string& key) const;

 private:
  friend class JsonParser;
  Type type_ = Type::kNull;
  double number_ = 0.0;
  std::string str_;
  std::vector<Json> array_;
  std::vector<std::pair<std::string, Json>> object_;
};

bool ReadTextFile(const std::string& path, std::string* out);
bool ReadJsonFile(const std::string& path, Json* out);
bool ReadGzipFile(const std::string& path, std::vector<uint8_t>* out);
bool ReadPgm(const std::string& path, int* width, int* height, std::vector<uint8_t>* gray);

std::array<uint8_t, 32> Sha256(const uint8_t* data, size_t size);
std::string Hex(const uint8_t* data, size_t size);

// Captured GPU readback (1024 x rows RGBA, row 0 = GL row 0) as an RgbaAtlas.
bool LoadCaptureAtlas(const std::string& gz_path, int rows, RgbaAtlas* out);

// Per-channel statistics over all level rectangles (spec-lane CompareAtlas / run_gates.py).
struct AtlasStats {
  long pixels = 0;
  std::array<long, 4> equal{};
  std::array<long, 4> max_diff{};
  long candidates_a = 0;  // B != 0 inside the 21 px detection interior
  long candidates_b = 0;
  long candidates_both = 0;
  double identical_fraction(int ch) const {
    return pixels ? static_cast<double>(equal[ch]) / static_cast<double>(pixels) : 1.0;
  }
  double candidate_jaccard() const {
    const long uni = candidates_a + candidates_b - candidates_both;
    return uni ? static_cast<double>(candidates_both) / static_cast<double>(uni) : 1.0;
  }
};
AtlasStats CompareLevels(const RgbaAtlas& a, const RgbaAtlas& b, const PyramidLayout& layout);

// Paths compiled in by tests/frontend/CMakeLists.txt.
std::string TestDataDir();
std::string EvidenceRoot();

}  // namespace slam_refact::frontend::testing

#endif  // SLAM_REFACT_TESTS_FRONTEND_TEST_UTIL_H_
