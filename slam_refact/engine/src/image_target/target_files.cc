// Target files (US-012): binary PGM reader and the JSON + luminance pair.

#include "image_target/target_files.h"

#include <cstring>
#include <fstream>
#include <iterator>
#include <utility>

#include "image_target/target_json.h"

namespace slam_refact::image_target {
namespace {

constexpr long kMaxPgmSide = 16384;

void SkipPgmWhitespaceAndComments(std::string_view bytes, size_t* pos) {
  while (*pos < bytes.size()) {
    const char c = bytes[*pos];
    if (c == '#') {
      while (*pos < bytes.size() && bytes[*pos] != '\n') {
        ++*pos;
      }
    } else if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f') {
      ++*pos;
    } else {
      return;
    }
  }
}

bool ReadPgmInt(std::string_view bytes, size_t* pos, long* out) {
  SkipPgmWhitespaceAndComments(bytes, pos);
  long v = 0;
  size_t n = 0;
  while (*pos < bytes.size() && bytes[*pos] >= '0' && bytes[*pos] <= '9' && n < 9) {
    v = v * 10 + (bytes[*pos] - '0');
    ++*pos;
    ++n;
  }
  *out = v;
  return n > 0;
}

}  // namespace

bool ParsePgm(std::string_view bytes, GrayImage* out, std::string* error) {
  if (bytes.size() < 2 || bytes[0] != 'P' || bytes[1] != '5') {
    *error = "not a binary PGM (P5)";
    return false;
  }
  size_t pos = 2;
  long width = 0;
  long height = 0;
  long maxval = 0;
  if (!ReadPgmInt(bytes, &pos, &width) || !ReadPgmInt(bytes, &pos, &height) ||
      !ReadPgmInt(bytes, &pos, &maxval)) {
    *error = "malformed PGM header";
    return false;
  }
  if (width <= 0 || height <= 0 || width > kMaxPgmSide || height > kMaxPgmSide) {
    *error = "PGM size out of range";
    return false;
  }
  if (maxval != 255) {
    *error = "PGM maxval must be 255";
    return false;
  }
  // Exactly one whitespace byte separates the header from the raster (netpbm).
  if (pos >= bytes.size() ||
      !(bytes[pos] == ' ' || bytes[pos] == '\t' || bytes[pos] == '\n' || bytes[pos] == '\r')) {
    *error = "malformed PGM header terminator";
    return false;
  }
  ++pos;
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (bytes.size() - pos != n) {
    *error = "PGM raster size does not match the header";
    return false;
  }
  GrayImage image;
  image.width = static_cast<int>(width);
  image.height = static_cast<int>(height);
  image.pixels.resize(n);
  std::memcpy(image.pixels.data(), bytes.data() + pos, n);
  *out = std::move(image);
  return true;
}

bool ReadFileBytes(const std::string& path, std::string* bytes, std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  bytes->assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  if (in.bad()) {
    *error = "cannot read " + path;
    return false;
  }
  return true;
}

bool LoadImageTargetFiles(const std::string& json_path, const std::string& pgm_path,
                          ImageTargetSpec* spec, GrayImage* luminance, std::string* error) {
  std::string json;
  std::string pgm;
  std::string why;
  if (!ReadFileBytes(json_path, &json, error) || !ReadFileBytes(pgm_path, &pgm, error)) {
    return false;
  }
  if (!ParseImageTargetJson(json, spec, &why)) {
    *error = json_path + ": " + why;
    return false;
  }
  if (!ParsePgm(pgm, luminance, &why)) {
    *error = pgm_path + ": " + why;
    return false;
  }
  return true;
}

}  // namespace slam_refact::image_target
