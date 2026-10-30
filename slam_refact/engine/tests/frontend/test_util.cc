#include "test_util.h"

#include <zlib.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#ifndef SR_TEST_DATA_DIR
#error "SR_TEST_DATA_DIR must point at slam_refact/engine/tests/data"
#endif
#ifndef SR_EVIDENCE_ROOT
#error "SR_EVIDENCE_ROOT must point at the evidence root"
#endif

namespace slam_refact::frontend::testing {

std::string TestDataDir() { return SR_TEST_DATA_DIR; }
std::string EvidenceRoot() { return SR_EVIDENCE_ROOT; }

// ---- JSON ----

class JsonParser {
 public:
  explicit JsonParser(const std::string& s) : s_(s) {}

  bool ParseDocument(Json* out, std::string* error) {
    if (!ParseValue(out)) {
      *error = error_ + " at offset " + std::to_string(pos_);
      return false;
    }
    SkipSpace();
    if (pos_ != s_.size()) {
      *error = "trailing characters at offset " + std::to_string(pos_);
      return false;
    }
    return true;
  }

 private:
  void SkipSpace() {
    while (pos_ < s_.size() &&
           (s_[pos_] == ' ' || s_[pos_] == '\n' || s_[pos_] == '\r' || s_[pos_] == '\t')) {
      ++pos_;
    }
  }

  bool Fail(const char* msg) {
    error_ = msg;
    return false;
  }

  bool Literal(const char* lit) {
    const size_t n = std::strlen(lit);
    if (s_.compare(pos_, n, lit) != 0) {
      return Fail("bad literal");
    }
    pos_ += n;
    return true;
  }

  bool ParseString(std::string* out) {
    if (pos_ >= s_.size() || s_[pos_] != '"') {
      return Fail("expected string");
    }
    ++pos_;
    out->clear();
    while (pos_ < s_.size() && s_[pos_] != '"') {
      char c = s_[pos_++];
      if (c == '\\') {
        if (pos_ >= s_.size()) {
          return Fail("bad escape");
        }
        const char e = s_[pos_++];
        switch (e) {
          case '"': c = '"'; break;
          case '\\': c = '\\'; break;
          case '/': c = '/'; break;
          case 'b': c = '\b'; break;
          case 'f': c = '\f'; break;
          case 'n': c = '\n'; break;
          case 'r': c = '\r'; break;
          case 't': c = '\t'; break;
          case 'u': {
            if (pos_ + 4 > s_.size()) {
              return Fail("bad \\u escape");
            }
            const unsigned cp = static_cast<unsigned>(std::strtoul(s_.substr(pos_, 4).c_str(), nullptr, 16));
            pos_ += 4;
            if (cp < 0x80) {
              out->push_back(static_cast<char>(cp));
            } else if (cp < 0x800) {
              out->push_back(static_cast<char>(0xC0 | (cp >> 6)));
              out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
            } else {
              out->push_back(static_cast<char>(0xE0 | (cp >> 12)));
              out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
              out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
            }
            continue;
          }
          default:
            return Fail("bad escape");
        }
      }
      out->push_back(c);
    }
    if (pos_ >= s_.size()) {
      return Fail("unterminated string");
    }
    ++pos_;
    return true;
  }

  bool ParseValue(Json* out) {
    SkipSpace();
    if (pos_ >= s_.size()) {
      return Fail("unexpected end");
    }
    const char c = s_[pos_];
    if (c == '{') {
      ++pos_;
      out->type_ = Json::Type::kObject;
      SkipSpace();
      if (pos_ < s_.size() && s_[pos_] == '}') {
        ++pos_;
        return true;
      }
      while (true) {
        SkipSpace();
        std::string key;
        if (!ParseString(&key)) {
          return false;
        }
        SkipSpace();
        if (pos_ >= s_.size() || s_[pos_] != ':') {
          return Fail("expected ':'");
        }
        ++pos_;
        Json value;
        if (!ParseValue(&value)) {
          return false;
        }
        out->object_.emplace_back(std::move(key), std::move(value));
        SkipSpace();
        if (pos_ < s_.size() && s_[pos_] == ',') {
          ++pos_;
          continue;
        }
        if (pos_ < s_.size() && s_[pos_] == '}') {
          ++pos_;
          return true;
        }
        return Fail("expected ',' or '}'");
      }
    }
    if (c == '[') {
      ++pos_;
      out->type_ = Json::Type::kArray;
      SkipSpace();
      if (pos_ < s_.size() && s_[pos_] == ']') {
        ++pos_;
        return true;
      }
      while (true) {
        Json value;
        if (!ParseValue(&value)) {
          return false;
        }
        out->array_.push_back(std::move(value));
        SkipSpace();
        if (pos_ < s_.size() && s_[pos_] == ',') {
          ++pos_;
          continue;
        }
        if (pos_ < s_.size() && s_[pos_] == ']') {
          ++pos_;
          return true;
        }
        return Fail("expected ',' or ']'");
      }
    }
    if (c == '"') {
      out->type_ = Json::Type::kString;
      return ParseString(&out->str_);
    }
    if (c == 't') {
      out->type_ = Json::Type::kBool;
      return Literal("true");
    }
    if (c == 'f') {
      out->type_ = Json::Type::kBool;
      return Literal("false");
    }
    if (c == 'n') {
      out->type_ = Json::Type::kNull;
      return Literal("null");
    }
    // Number: strtod gives the correctly rounded double of the decimal text.
    const char* begin = s_.c_str() + pos_;
    char* end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end == begin) {
      return Fail("bad value");
    }
    pos_ += static_cast<size_t>(end - begin);
    out->type_ = Json::Type::kNumber;
    out->number_ = v;
    return true;
  }

  const std::string& s_;
  size_t pos_ = 0;
  std::string error_;
};

bool Json::Parse(const std::string& text, Json* out, std::string* error) {
  *out = Json();
  JsonParser parser(text);
  return parser.ParseDocument(out, error);
}

const Json& Json::operator[](const std::string& key) const {
  static const Json kNull;
  for (const auto& kv : object_) {
    if (kv.first == key) {
      return kv.second;
    }
  }
  return kNull;
}

// ---- files ----

bool ReadTextFile(const std::string& path, std::string* out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  *out = ss.str();
  return true;
}

bool ReadJsonFile(const std::string& path, Json* out) {
  std::string text;
  std::string error;
  if (!ReadTextFile(path, &text)) {
    std::fprintf(stderr, "cannot read %s\n", path.c_str());
    return false;
  }
  if (!Json::Parse(text, out, &error)) {
    std::fprintf(stderr, "cannot parse %s: %s\n", path.c_str(), error.c_str());
    return false;
  }
  return true;
}

bool ReadGzipFile(const std::string& path, std::vector<uint8_t>* out) {
  gzFile f = gzopen(path.c_str(), "rb");
  if (f == nullptr) {
    return false;
  }
  out->clear();
  std::vector<uint8_t> buf(1 << 20);
  while (true) {
    const int n = gzread(f, buf.data(), static_cast<unsigned>(buf.size()));
    if (n < 0) {
      gzclose(f);
      return false;
    }
    if (n == 0) {
      break;
    }
    out->insert(out->end(), buf.begin(), buf.begin() + n);
  }
  gzclose(f);
  return true;
}

bool ReadPgm(const std::string& path, int* width, int* height, std::vector<uint8_t>* gray) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (f == nullptr) {
    return false;
  }
  int maxv = 0;
  if (std::fscanf(f, "P5 %d %d %d", width, height, &maxv) != 3 || maxv != 255) {
    std::fclose(f);
    return false;
  }
  std::fgetc(f);
  gray->resize(static_cast<size_t>(*width) * *height);
  const size_t n = std::fread(gray->data(), 1, gray->size(), f);
  std::fclose(f);
  return n == gray->size();
}

bool LoadCaptureAtlas(const std::string& gz_path, int rows, RgbaAtlas* out) {
  constexpr int kSize = 1024;
  if (!ReadGzipFile(gz_path, &out->rgba)) {
    return false;
  }
  out->size = kSize;
  out->rows = rows;
  return out->rgba.size() == static_cast<size_t>(kSize) * rows * 4;
}

AtlasStats CompareLevels(const RgbaAtlas& a, const RgbaAtlas& b, const PyramidLayout& layout) {
  AtlasStats st;
  const int rows = std::min(a.rows, b.rows);
  for (const LevelLayout& l : layout.levels) {
    for (int y = 0; y < l.h && l.r + y < rows; ++y) {
      for (int x = 0; x < l.w; ++x) {
        const size_t oa = (static_cast<size_t>(l.r + y) * a.size + l.c + x) * 4;
        const size_t ob = (static_cast<size_t>(l.r + y) * b.size + l.c + x) * 4;
        ++st.pixels;
        for (int ch = 0; ch < 4; ++ch) {
          const long d = std::labs(static_cast<long>(a.rgba[oa + ch]) - b.rgba[ob + ch]);
          st.equal[ch] += d == 0;
          st.max_diff[ch] = std::max(st.max_diff[ch], d);
        }
        const bool interior = x >= 21 && y >= 21 && x < l.w - 21 && y < l.h - 21;
        if (interior) {
          st.candidates_a += a.rgba[oa + 2] != 0;
          st.candidates_b += b.rgba[ob + 2] != 0;
          st.candidates_both += a.rgba[oa + 2] != 0 && b.rgba[ob + 2] != 0;
        }
      }
    }
  }
  return st;
}

// ---- SHA-256 (FIPS 180-4) ----

namespace {

constexpr uint32_t kK[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
    0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
    0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
    0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
    0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
    0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
    0xc67178f2};

inline uint32_t Rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

void Block(const uint8_t* p, uint32_t h[8]) {
  uint32_t w[64];
  for (int i = 0; i < 16; ++i) {
    w[i] = (uint32_t{p[4 * i]} << 24) | (uint32_t{p[4 * i + 1]} << 16) |
           (uint32_t{p[4 * i + 2]} << 8) | uint32_t{p[4 * i + 3]};
  }
  for (int i = 16; i < 64; ++i) {
    const uint32_t s0 = Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const uint32_t s1 = Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
  for (int i = 0; i < 64; ++i) {
    const uint32_t s1 = Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25);
    const uint32_t ch = (e & f) ^ (~e & g);
    const uint32_t t1 = hh + s1 + ch + kK[i] + w[i];
    const uint32_t s0 = Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22);
    const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const uint32_t t2 = s0 + maj;
    hh = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  h[0] += a;
  h[1] += b;
  h[2] += c;
  h[3] += d;
  h[4] += e;
  h[5] += f;
  h[6] += g;
  h[7] += hh;
}

}  // namespace

std::array<uint8_t, 32> Sha256(const uint8_t* data, size_t size) {
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  size_t i = 0;
  for (; i + 64 <= size; i += 64) {
    Block(data + i, h);
  }
  uint8_t tail[128] = {0};
  const size_t rem = size - i;
  std::memcpy(tail, data + i, rem);
  tail[rem] = 0x80;
  const size_t tail_len = rem + 1 + 8 <= 64 ? 64 : 128;
  const uint64_t bits = static_cast<uint64_t>(size) * 8;
  for (int k = 0; k < 8; ++k) {
    tail[tail_len - 1 - k] = static_cast<uint8_t>(bits >> (8 * k));
  }
  Block(tail, h);
  if (tail_len == 128) {
    Block(tail + 64, h);
  }
  std::array<uint8_t, 32> out{};
  for (int k = 0; k < 8; ++k) {
    out[4 * k] = static_cast<uint8_t>(h[k] >> 24);
    out[4 * k + 1] = static_cast<uint8_t>(h[k] >> 16);
    out[4 * k + 2] = static_cast<uint8_t>(h[k] >> 8);
    out[4 * k + 3] = static_cast<uint8_t>(h[k]);
  }
  return out;
}

std::string Hex(const uint8_t* data, size_t size) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string s;
  s.reserve(2 * size);
  for (size_t i = 0; i < size; ++i) {
    s.push_back(kDigits[data[i] >> 4]);
    s.push_back(kDigits[data[i] & 15]);
  }
  return s;
}

}  // namespace slam_refact::frontend::testing
