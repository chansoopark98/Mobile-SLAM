// sr_held: the shared HELD device of the preregistered gates (docs/design/preregistration-protocol.md section 12.5,
// design and usage: docs/design/sr-held-design.md). C++ side; the node side is slam_refact/tools/prereg/sr_held.mjs.
// Both read the same manifest, designation file, one-run marker and seed file, and make the same decision for the same
// input (conformance: tools/prereg/sr_held_cases.json, read by tests/support/sr_held_conformance.cc and
// tools/prereg/sr_held_conformance.test.mjs).
//
// A first real run is refused (HELD) by CODE unless every condition holds; it can happen once per (gate, version, kind):
//   args, env       the caller passes the value of its environment variable; it must equal the sha256 of the designation file
//   manifest        <gate_id>.v<N>.manifest.json parses as strict JSON (duplicate keys are an error; values are read by
//                   their path, never by "the first key of that name"), names this gate and version, first_real_run_at null
//   names           every designated file name is a bare file name (no '/', no '\\', not "." or ".."); the gate file is
//                   <gate_id>.v<N>.md
//   time            created_at < review.at < now (audit: < first_real_run_at), parsed as instants, not compared as text
//   files           gate file, review file and designation file have the sha256 the manifest records
//   designation     names "<gate_id>.v<N>" exactly, first_real_run_permitted is the JSON value true, and its pinned values
//                   equal the gate file sha256, the review file sha256 and the canonical sha256 of the manifest without
//                   review / first_real_run_at / held_run (the form that verify_manifest.py --print-pin prints)
//   pins, inputs    every file the request presents has the registered sha256 and every registered pin is presented;
//                   the data directory has the registered tree sha256 (no symlinks, no special files)
//   seed            (only when the manifest registers one) the coordinator seed is validated, but handed out only by
//                   HeldGate(), after the marker exists; it never appears in a log line or a refusal reason
//   marker          <gate_id>.v<N>.first-run.<kind>.marker is created with O_EXCL; the code never removes it
// It is an accident guard, not an authentication (protocol 12.6): whoever can write the preregistration directory and the
// environment can satisfy every check. Pure functions of their arguments plus the files they name.

#ifndef SLAM_REFACT_TESTS_SUPPORT_SR_HELD_H_
#define SLAM_REFACT_TESTS_SUPPORT_SR_HELD_H_

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../common/sha256_hex.h"

namespace slam_refact::sr_held {

// ---------------------------------------------------------------------------------------------------------------------
// Files and hashes

inline bool ReadFile(const std::string& path, std::string* out) {
  struct stat st;
  if (::stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) return false;
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  std::stringstream ss;
  ss << in.rdbuf();
  if (in.bad()) return false;
  *out = ss.str();
  return true;
}

inline bool Sha256File(const std::string& path, std::string* sha) {
  std::string data;
  if (!ReadFile(path, &data)) return false;
  *sha = testutil::Sha256Hex(data);
  return true;
}

inline bool IsHex64(const std::string& s) {
  if (s.size() != 64) return false;
  return std::all_of(s.begin(), s.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

// A designated file is looked up in the preregistration directory by this name only.
inline bool IsBareName(const std::string& s) {
  if (s.empty() || s.size() > 255 || s == "." || s == "..") return false;
  return std::none_of(s.begin(), s.end(), [](char c) { return c == '/' || c == '\\' || static_cast<unsigned char>(c) < 0x20; });
}

inline bool IsId(const std::string& s) {
  if (s.empty() || s.size() > 64) return false;
  const auto alnum = [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); };
  if (!alnum(s[0])) return false;
  return std::all_of(s.begin(), s.end(), [&](char c) { return alnum(c) || c == '_' || c == '-'; });
}

inline bool IsDecimal(const std::string& s) {  // unsigned decimal, no leading zeros, at most 2^64-1
  if (s.empty() || s.size() > 20 || !std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; })) return false;
  if (s.size() > 1 && s[0] == '0') return false;
  return s.size() < 20 || s <= "18446744073709551615";
}

inline bool ValidUtf8(const std::string& s) {
  size_t i = 0;
  const size_t n = s.size();
  while (i < n) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    if (c < 0x80) {
      ++i;
      continue;
    }
    size_t len = 0;
    uint32_t cp = 0;
    if (c >= 0xC2 && c <= 0xDF) {
      len = 2;
      cp = c & 0x1Fu;
    } else if (c >= 0xE0 && c <= 0xEF) {
      len = 3;
      cp = c & 0x0Fu;
    } else if (c >= 0xF0 && c <= 0xF4) {
      len = 4;
      cp = c & 0x07u;
    } else {
      return false;
    }
    if (i + len > n) return false;
    for (size_t k = 1; k < len; ++k) {
      const unsigned char d = static_cast<unsigned char>(s[i + k]);
      if ((d & 0xC0) != 0x80) return false;
      cp = (cp << 6) | (d & 0x3Fu);
    }
    if (len == 3 && (cp < 0x800 || (cp >= 0xD800 && cp <= 0xDFFF))) return false;
    if (len == 4 && (cp < 0x10000 || cp > 0x10FFFF)) return false;
    i += len;
  }
  return true;
}

// sha256 over the sorted lines "<relative path>\t<file sha256>\n" of every regular file below `dir`. Symlinks and special
// files are refused (code INPUT_NONREGULAR), as is an empty or unreadable tree (INPUT_UNREADABLE).
inline bool ListTree(const std::string& root, const std::string& rel, std::vector<std::string>* files, std::string* code) {
  DIR* d = ::opendir((rel.empty() ? root : root + "/" + rel).c_str());
  if (d == nullptr) {
    *code = "INPUT_UNREADABLE";
    return false;
  }
  std::vector<std::string> names;
  while (const dirent* e = ::readdir(d)) {
    const std::string name = e->d_name;
    if (name != "." && name != "..") names.push_back(name);
  }
  ::closedir(d);
  for (const std::string& name : names) {
    const std::string r = rel.empty() ? name : rel + "/" + name;
    struct stat st;
    if (::lstat((root + "/" + r).c_str(), &st) != 0) {
      *code = "INPUT_UNREADABLE";
      return false;
    }
    if (S_ISDIR(st.st_mode)) {
      if (!ListTree(root, r, files, code)) return false;
    } else if (S_ISREG(st.st_mode)) {
      files->push_back(r);
    } else {
      *code = "INPUT_NONREGULAR";
      return false;
    }
  }
  return true;
}

inline bool TreeSha(const std::string& dir, std::string* sha, std::string* code) {
  struct stat st;
  if (::stat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
    *code = "INPUT_UNREADABLE";
    return false;
  }
  std::vector<std::string> files;
  if (!ListTree(dir, "", &files, code)) return false;
  if (files.empty()) {
    *code = "INPUT_UNREADABLE";
    return false;
  }
  std::sort(files.begin(), files.end());
  std::string lines;
  for (const std::string& f : files) {
    std::string h;
    if (!Sha256File(dir + "/" + f, &h)) {
      *code = "INPUT_UNREADABLE";
      return false;
    }
    lines += f + "\t" + h + "\n";
  }
  *sha = testutil::Sha256Hex(lines);
  return true;
}

inline std::string SelfExe() {
  char buf[4096];
  const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  return n > 0 ? std::string(buf, static_cast<size_t>(n)) : std::string();
}

// ---------------------------------------------------------------------------------------------------------------------
// Strict JSON (RFC 8259 value grammar; a duplicate key in one object, a BOM, invalid UTF-8, a lone surrogate escape, NaN
// and Infinity are errors). Values are read by path through Get(); numbers keep their raw token.

struct Json {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };
  Type type = Type::kNull;
  bool boolean = false;
  std::string text;               // string value, or the raw number token
  std::vector<Json> items;        // array elements
  std::vector<std::string> keys;  // object keys in file order (unique)
  std::vector<Json> values;       // object values, parallel to keys

  const Json* Get(const std::string& key) const {
    if (type != Type::kObject) return nullptr;
    for (size_t i = 0; i < keys.size(); ++i) {
      if (keys[i] == key) return &values[i];
    }
    return nullptr;
  }
};

inline void AppendUtf8(uint32_t cp, std::string* out) {
  if (cp < 0x80) {
    out->push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out->push_back(static_cast<char>(0xC0 | (cp >> 6)));
    out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp < 0x10000) {
    out->push_back(static_cast<char>(0xE0 | (cp >> 12)));
    out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out->push_back(static_cast<char>(0xF0 | (cp >> 18)));
    out->push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

class JsonParser {
 public:
  explicit JsonParser(const std::string& text) : s_(text) {}

  bool Parse(Json* out, std::string* err) {
    if (!ValidUtf8(s_)) return Fail(err, "invalid UTF-8");
    if (s_.compare(0, 3, "\xEF\xBB\xBF") == 0) return Fail(err, "byte order mark");
    SkipWs();
    if (!Value(out, 0)) return Fail(err, err_);
    SkipWs();
    if (i_ != s_.size()) return Fail(err, "trailing characters");
    return true;
  }

 private:
  static bool Fail(std::string* err, const std::string& why) {
    *err = why;
    return false;
  }
  bool Err(const std::string& why) {
    err_ = why + " at byte " + std::to_string(i_);
    return false;
  }
  void SkipWs() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) ++i_;
  }
  bool Literal(const char* word) {
    const size_t n = std::char_traits<char>::length(word);
    if (s_.compare(i_, n, word) != 0) return Err("bad literal");
    i_ += n;
    return true;
  }
  bool Hex4(uint32_t* v) {
    if (i_ + 4 > s_.size()) return Err("short \\u escape");
    *v = 0;
    for (int k = 0; k < 4; ++k) {
      const char c = s_[i_ + static_cast<size_t>(k)];
      uint32_t d = 0;
      if (c >= '0' && c <= '9') {
        d = static_cast<uint32_t>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        d = static_cast<uint32_t>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        d = static_cast<uint32_t>(c - 'A' + 10);
      } else {
        return Err("bad \\u escape");
      }
      *v = (*v << 4) | d;
    }
    i_ += 4;
    return true;
  }
  bool String(std::string* out) {
    ++i_;  // opening quote
    out->clear();
    while (true) {
      if (i_ >= s_.size()) return Err("unterminated string");
      const unsigned char c = static_cast<unsigned char>(s_[i_]);
      if (c == '"') {
        ++i_;
        return true;
      }
      if (c < 0x20) return Err("control character in string");
      if (c != '\\') {
        out->push_back(static_cast<char>(c));
        ++i_;
        continue;
      }
      ++i_;
      if (i_ >= s_.size()) return Err("unterminated escape");
      const char e = s_[i_++];
      switch (e) {
        case '"': out->push_back('"'); break;
        case '\\': out->push_back('\\'); break;
        case '/': out->push_back('/'); break;
        case 'b': out->push_back('\b'); break;
        case 'f': out->push_back('\f'); break;
        case 'n': out->push_back('\n'); break;
        case 'r': out->push_back('\r'); break;
        case 't': out->push_back('\t'); break;
        case 'u': {
          uint32_t cp = 0;
          if (!Hex4(&cp)) return false;
          if (cp >= 0xD800 && cp <= 0xDBFF) {
            uint32_t lo = 0;
            if (s_.compare(i_, 2, "\\u") != 0) return Err("lone high surrogate");
            i_ += 2;
            if (!Hex4(&lo)) return false;
            if (lo < 0xDC00 || lo > 0xDFFF) return Err("lone high surrogate");
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
            return Err("lone low surrogate");
          }
          AppendUtf8(cp, out);
          break;
        }
        default: return Err("bad escape");
      }
    }
  }
  bool Number(Json* out) {
    const size_t start = i_;
    if (s_[i_] == '-') ++i_;
    if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return Err("bad number");
    if (s_[i_] == '0') {
      ++i_;
    } else {
      while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    if (i_ < s_.size() && s_[i_] == '.') {
      ++i_;
      if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return Err("bad fraction");
      while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
      ++i_;
      if (i_ < s_.size() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
      if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return Err("bad exponent");
      while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    out->type = Json::Type::kNumber;
    out->text = s_.substr(start, i_ - start);
    return true;
  }
  bool Value(Json* out, int depth) {
    if (depth > 64) return Err("nesting too deep");
    if (i_ >= s_.size()) return Err("unexpected end");
    const char c = s_[i_];
    if (c == '{') {
      out->type = Json::Type::kObject;
      ++i_;
      SkipWs();
      if (i_ < s_.size() && s_[i_] == '}') {
        ++i_;
        return true;
      }
      while (true) {
        SkipWs();
        if (i_ >= s_.size() || s_[i_] != '"') return Err("object key expected");
        std::string key;
        if (!String(&key)) return false;
        if (std::find(out->keys.begin(), out->keys.end(), key) != out->keys.end()) return Err("duplicate key \"" + key + "\"");
        SkipWs();
        if (i_ >= s_.size() || s_[i_] != ':') return Err("':' expected");
        ++i_;
        SkipWs();
        Json v;
        if (!Value(&v, depth + 1)) return false;
        out->keys.push_back(key);
        out->values.push_back(std::move(v));
        SkipWs();
        if (i_ < s_.size() && s_[i_] == ',') {
          ++i_;
          continue;
        }
        if (i_ < s_.size() && s_[i_] == '}') {
          ++i_;
          return true;
        }
        return Err("',' or '}' expected");
      }
    }
    if (c == '[') {
      out->type = Json::Type::kArray;
      ++i_;
      SkipWs();
      if (i_ < s_.size() && s_[i_] == ']') {
        ++i_;
        return true;
      }
      while (true) {
        SkipWs();
        Json v;
        if (!Value(&v, depth + 1)) return false;
        out->items.push_back(std::move(v));
        SkipWs();
        if (i_ < s_.size() && s_[i_] == ',') {
          ++i_;
          continue;
        }
        if (i_ < s_.size() && s_[i_] == ']') {
          ++i_;
          return true;
        }
        return Err("',' or ']' expected");
      }
    }
    if (c == '"') {
      out->type = Json::Type::kString;
      return String(&out->text);
    }
    if (c == 't') {
      out->type = Json::Type::kBool;
      out->boolean = true;
      return Literal("true");
    }
    if (c == 'f') {
      out->type = Json::Type::kBool;
      out->boolean = false;
      return Literal("false");
    }
    if (c == 'n') {
      out->type = Json::Type::kNull;
      return Literal("null");
    }
    if (c == '-' || (c >= '0' && c <= '9')) return Number(out);
    return Err("unexpected character");
  }

  const std::string& s_;
  size_t i_ = 0;
  std::string err_;
};

inline bool ParseJson(const std::string& text, Json* out, std::string* err) {
  *out = Json();
  return JsonParser(text).Parse(out, err);
}

// ---------------------------------------------------------------------------------------------------------------------
// Canonical form of the manifest: json.dumps(sort_keys=True, separators=(",", ":"), ensure_ascii=False) of Python, over
// the manifest without the keys the coordinator and the runner fill in afterwards. This is the value
// `verify_manifest.py --print-pin` prints (protocol section 6) and the designation pins as manifest_before_review_sha256.

inline void CanonString(const std::string& s, std::string* out) {
  out->push_back('"');
  for (const char ch : s) {
    const unsigned char c = static_cast<unsigned char>(ch);
    switch (c) {
      case '"': *out += "\\\""; break;
      case '\\': *out += "\\\\"; break;
      case '\n': *out += "\\n"; break;
      case '\r': *out += "\\r"; break;
      case '\t': *out += "\\t"; break;
      case '\b': *out += "\\b"; break;
      case '\f': *out += "\\f"; break;
      default:
        if (c < 0x20) {
          static const char kHex[] = "0123456789abcdef";
          *out += "\\u00";
          out->push_back(kHex[c >> 4]);
          out->push_back(kHex[c & 15]);
        } else {
          out->push_back(ch);
        }
    }
  }
  out->push_back('"');
}

// Python: an integer token prints as the integer ("-0" is 0); a token with '.', 'e' or 'E' is a float and prints as
// repr(float): the shortest round-trip digits, exponent form when the decimal exponent is < -4 or >= 16.
inline bool CanonNumber(const std::string& tok, std::string* out) {
  if (tok.find_first_of(".eE") == std::string::npos) {
    *out += tok == "-0" ? std::string("0") : tok;
    return true;
  }
  const double d = std::strtod(tok.c_str(), nullptr);
  if (!std::isfinite(d)) return false;
  if (d == 0.0) {
    *out += std::signbit(d) ? "-0.0" : "0.0";
    return true;
  }
  char buf[64];
  const auto res = std::to_chars(buf, buf + sizeof(buf), d, std::chars_format::scientific);
  const std::string sci(buf, res.ptr);  // [-]d[.ddd]e[+-]xx
  const size_t epos = sci.find('e');
  std::string mant = sci.substr(0, epos);
  const int e10 = std::atoi(sci.c_str() + epos + 1);
  std::string sign;
  if (mant[0] == '-') {
    sign = "-";
    mant.erase(0, 1);
  }
  std::string digits;
  for (const char c : mant) {
    if (c != '.') digits.push_back(c);
  }
  const int decpt = e10 + 1;
  const int n = static_cast<int>(digits.size());
  std::string r;
  if (decpt <= -4 || decpt > 16) {
    r = digits.substr(0, 1) + (n > 1 ? "." + digits.substr(1) : std::string());
    const int ae = e10 < 0 ? -e10 : e10;
    r += std::string("e") + (e10 < 0 ? "-" : "+") + (ae < 10 ? "0" : "") + std::to_string(ae);
  } else if (decpt <= 0) {
    r = "0." + std::string(static_cast<size_t>(-decpt), '0') + digits;
  } else if (decpt >= n) {
    r = digits + std::string(static_cast<size_t>(decpt - n), '0') + ".0";
  } else {
    r = digits.substr(0, static_cast<size_t>(decpt)) + "." + digits.substr(static_cast<size_t>(decpt));
  }
  *out += sign + r;
  return true;
}

inline bool CanonValue(const Json& v, const std::set<std::string>* skip, std::string* out) {
  switch (v.type) {
    case Json::Type::kNull: *out += "null"; return true;
    case Json::Type::kBool: *out += v.boolean ? "true" : "false"; return true;
    case Json::Type::kNumber: return CanonNumber(v.text, out);
    case Json::Type::kString: CanonString(v.text, out); return true;
    case Json::Type::kArray: {
      out->push_back('[');
      for (size_t i = 0; i < v.items.size(); ++i) {
        if (i > 0) out->push_back(',');
        if (!CanonValue(v.items[i], nullptr, out)) return false;
      }
      out->push_back(']');
      return true;
    }
    case Json::Type::kObject: {
      std::vector<size_t> order;
      for (size_t i = 0; i < v.keys.size(); ++i) {
        if (skip == nullptr || skip->count(v.keys[i]) == 0) order.push_back(i);
      }
      std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return v.keys[a] < v.keys[b]; });
      out->push_back('{');
      for (size_t k = 0; k < order.size(); ++k) {
        if (k > 0) out->push_back(',');
        CanonString(v.keys[order[k]], out);
        out->push_back(':');
        if (!CanonValue(v.values[order[k]], nullptr, out)) return false;
      }
      out->push_back('}');
      return true;
    }
  }
  return false;
}

inline const std::set<std::string>& PinBlanked() {
  static const std::set<std::string> kBlanked = {"review", "first_real_run_at", "held_run"};
  return kBlanked;
}

inline bool CanonManifestText(const Json& manifest, std::string* text) {
  text->clear();
  return CanonValue(manifest, &PinBlanked(), text);
}

inline bool CanonManifestSha(const Json& manifest, std::string* sha) {
  std::string text;
  if (!CanonManifestText(manifest, &text)) return false;
  *sha = testutil::Sha256Hex(text);
  return true;
}

// ---------------------------------------------------------------------------------------------------------------------
// Time: "YYYY-MM-DDTHH:MM:SS[.f{1,9}](Z|+HH:MM|-HH:MM)", the time zone is required (`date -Is` form).

struct Instant {
  int64_t sec = 0;
  int32_t ns = 0;
};

inline bool Before(const Instant& a, const Instant& b) { return a.sec != b.sec ? a.sec < b.sec : a.ns < b.ns; }

inline bool ParseInstant(const std::string& t, Instant* out) {
  auto num = [&](size_t pos, size_t count, int* v) {
    if (pos + count > t.size()) return false;
    *v = 0;
    for (size_t k = 0; k < count; ++k) {
      const char c = t[pos + k];
      if (c < '0' || c > '9') return false;
      *v = *v * 10 + (c - '0');
    }
    return true;
  };
  int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
  if (t.size() < 20 || t[4] != '-' || t[7] != '-' || t[10] != 'T' || t[13] != ':' || t[16] != ':') return false;
  if (!num(0, 4, &y) || !num(5, 2, &mo) || !num(8, 2, &d) || !num(11, 2, &h) || !num(14, 2, &mi) || !num(17, 2, &s)) return false;
  size_t i = 19;
  int32_t ns = 0;
  if (t[i] == '.') {
    size_t digits = 0;
    ++i;
    while (i < t.size() && t[i] >= '0' && t[i] <= '9') {
      if (digits >= 9) return false;
      ns = ns * 10 + (t[i] - '0');
      ++digits;
      ++i;
    }
    if (digits == 0) return false;
    for (; digits < 9; ++digits) ns *= 10;
  }
  int64_t offset = 0;
  if (i < t.size() && t[i] == 'Z' && i + 1 == t.size()) {
    offset = 0;
  } else if (i + 6 == t.size() && (t[i] == '+' || t[i] == '-') && t[i + 3] == ':') {
    int oh = 0, om = 0;
    if (!num(i + 1, 2, &oh) || !num(i + 4, 2, &om) || oh > 23 || om > 59) return false;
    offset = (oh * 3600 + om * 60) * (t[i] == '-' ? -1 : 1);
  } else {
    return false;
  }
  const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
  static const int kDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (y < 1 || mo < 1 || mo > 12 || d < 1 || d > kDays[mo - 1] + (mo == 2 && leap ? 1 : 0)) return false;
  if (h > 23 || mi > 59 || s > 59) return false;
  // days from civil (H. Hinnant)
  const int64_t yy = y - (mo <= 2 ? 1 : 0);
  const int64_t era = (yy >= 0 ? yy : yy - 399) / 400;
  const int64_t yoe = yy - era * 400;
  const int64_t doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = era * 146097 + doe - 719468;
  out->sec = days * 86400 + h * 3600 + mi * 60 + s - offset;
  out->ns = ns;
  return true;
}

inline Instant SystemNow() {
  const auto n = std::chrono::system_clock::now().time_since_epoch();
  const auto s = std::chrono::duration_cast<std::chrono::seconds>(n);
  Instant t;
  t.sec = s.count();
  t.ns = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(n - s).count());
  return t;
}

// ---------------------------------------------------------------------------------------------------------------------
// The device

struct Request {
  std::string prereg_dir;          // manifest, designation file, review file, gate file, seed file and markers live here
  std::string gate_id;             // [A-Za-z0-9][A-Za-z0-9_-]*
  int version = 0;                 // N of <gate_id>.v<N>
  std::string kind = "main";       // one-run kind; one marker per kind, the kinds are registered in manifest held.kinds
  std::string env_value;           // value of the gate's environment variable (the sha256 of the designation file)
  std::vector<std::string> pins;   // files that must have the sha256 of manifest held.pins[<basename>] (e.g. SelfExe())
  std::string data_dir;            // directory whose tree sha256 must equal held.inputs[kind] ("" = the kind has no data)
  bool want_seed = false;          // the gate expects the coordinator seed (must equal manifest held.seed)
  std::string now;                 // ISO instant for tests; "" = the system clock
};

struct Outcome {
  bool ok = false;
  std::string code;                // "" when ok, else a stable refusal code (design doc section 5)
  std::string why;                 // human readable; never contains the seed
  std::vector<std::string> log;    // "[SR-HELD] ..." lines for the gate output; never contains the seed
  std::string designation_sha256;  // actual sha256 of the designation file (set once it was read)
  std::string seed;                // HeldGate() only: the coordinator seed, decimal, after the marker exists
  std::string marker;              // HeldGate() only: path of the marker this call created
};

inline std::string ManifestName(const Request& rq) { return rq.gate_id + ".v" + std::to_string(rq.version) + ".manifest.json"; }
inline std::string GateFileName(const Request& rq) { return rq.gate_id + ".v" + std::to_string(rq.version) + ".md"; }
inline std::string SeedFileName(const Request& rq) { return rq.gate_id + ".v" + std::to_string(rq.version) + ".coordinator-seed.txt"; }
inline std::string MarkerName(const Request& rq) {
  return rq.gate_id + ".v" + std::to_string(rq.version) + ".first-run." + rq.kind + ".marker";
}

namespace internal {

enum class Mode { kRelease, kAudit };

inline bool Fail(Outcome* o, const char* code, const std::string& why) {
  o->ok = false;
  o->code = code;
  o->why = why;
  return false;
}

inline void Log(Outcome* o, const std::string& line) { o->log.push_back("[SR-HELD] " + line); }

inline const Json* Member(const Json* object, const char* key, Json::Type type) {
  if (object == nullptr) return nullptr;
  const Json* v = object->Get(key);
  return v != nullptr && v->type == type ? v : nullptr;
}

inline std::string Basename(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

// Shape of the manifest fields the device reads (types, not values).
inline bool CheckManifestShape(const Json& m, Outcome* o) {
  const Json* review = Member(&m, "review", Json::Type::kObject);
  const Json* held = Member(&m, "held", Json::Type::kObject);
  if (Member(&m, "file", Json::Type::kString) == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest file is not a string");
  if (Member(&m, "sha256", Json::Type::kString) == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest sha256 is not a string");
  if (Member(&m, "created_at", Json::Type::kString) == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest created_at is not a string");
  if (review == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest review is not an object");
  if (held == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest held is not an object");
  const Json* at = review->Get("at");
  if (at == nullptr || (at->type != Json::Type::kNull && at->type != Json::Type::kString)) return Fail(o, "MANIFEST_INVALID", "manifest review.at is neither null nor a string");
  const Json* run = m.Get("first_real_run_at");
  if (run == nullptr || (run->type != Json::Type::kNull && run->type != Json::Type::kString)) return Fail(o, "MANIFEST_INVALID", "manifest first_real_run_at is neither null nor a string");
  for (const char* key : {"file", "sha256", "designation_file", "designation_file_sha256", "reviewer"}) {
    const Json* v = Member(review, key, Json::Type::kString);
    if (v == nullptr || v->text.empty()) return Fail(o, "MANIFEST_INVALID", std::string("manifest review.") + key + " is not a non-empty string");
  }
  const Json* kinds = Member(held, "kinds", Json::Type::kArray);
  if (kinds == nullptr || kinds->items.empty()) return Fail(o, "MANIFEST_INVALID", "manifest held.kinds is not a non-empty array");
  std::set<std::string> seen;
  for (const Json& k : kinds->items) {
    if (k.type != Json::Type::kString || !IsId(k.text) || !seen.insert(k.text).second) return Fail(o, "MANIFEST_INVALID", "manifest held.kinds has a bad or repeated kind");
  }
  const Json* pins = Member(held, "pins", Json::Type::kObject);
  if (pins == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest held.pins is not an object");
  for (size_t i = 0; i < pins->keys.size(); ++i) {
    if (!IsBareName(pins->keys[i]) || pins->values[i].type != Json::Type::kString || !IsHex64(pins->values[i].text)) return Fail(o, "MANIFEST_INVALID", "manifest held.pins has a bad entry");
  }
  const Json* inputs = Member(held, "inputs", Json::Type::kObject);
  if (inputs == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest held.inputs is not an object");
  for (size_t i = 0; i < inputs->keys.size(); ++i) {
    if (seen.count(inputs->keys[i]) == 0 || inputs->values[i].type != Json::Type::kString || !IsHex64(inputs->values[i].text)) return Fail(o, "MANIFEST_INVALID", "manifest held.inputs has a bad entry (a kind that is not in held.kinds, or a bad sha256)");
  }
  if (Member(held, "seed", Json::Type::kBool) == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest held.seed is not a boolean");
  if (const Json* dev = held->Get("dev_seeds")) {
    if (dev->type != Json::Type::kArray) return Fail(o, "MANIFEST_INVALID", "manifest held.dev_seeds is not an array");
    for (const Json& s : dev->items) {
      if (s.type != Json::Type::kString || !IsDecimal(s.text)) return Fail(o, "MANIFEST_INVALID", "manifest held.dev_seeds has a bad entry");
    }
  }
  if (const Json* sealed = held->Get("sealed_file_sha256")) {
    if (sealed->type != Json::Type::kArray) return Fail(o, "MANIFEST_INVALID", "manifest held.sealed_file_sha256 is not an array");
    for (const Json& s : sealed->items) {
      if (s.type != Json::Type::kString || !IsHex64(s.text)) return Fail(o, "MANIFEST_INVALID", "manifest held.sealed_file_sha256 has a bad entry");
    }
  }
  return true;
}

// The coordinator seed: validated before the marker is claimed (a bad seed must not burn the run), returned only by
// HeldGate(). The value never goes to the log or to a reason; only the sha256 of the seed file does.
inline bool LoadSeed(const Request& rq, const Json& held, const Json& pinned, Outcome* o, std::string* seed) {
  std::string text;
  if (!ReadFile(rq.prereg_dir + "/" + SeedFileName(rq), &text)) return Fail(o, "SEED_MISSING", "the coordinator seed file " + SeedFileName(rq) + " is missing");
  std::string token;
  std::istringstream in(text);
  in >> token;
  if (!IsDecimal(token)) return Fail(o, "SEED_INVALID", "the first token of the coordinator seed file is not an unsigned decimal integer");
  const Json* dev = held.Get("dev_seeds");
  if (dev != nullptr) {
    for (const Json& s : dev->items) {
      if (s.text == token) return Fail(o, "SEED_FORBIDDEN", "the coordinator seed equals a seed the executor ran before the review (held.dev_seeds)");
    }
  }
  const std::string file_sha = testutil::Sha256Hex(text);
  Log(o, "seed_file " + SeedFileName(rq) + " sha256=" + file_sha + " (the value is not logged)");
  const Json* pin = pinned.Get("seed_file_sha256");
  if (pin != nullptr && (pin->type != Json::Type::kString || pin->text != file_sha)) return Fail(o, "SEED_PIN", "the coordinator seed file differs from designation pinned.seed_file_sha256");
  *seed = token;
  return true;
}

inline bool Evaluate(const Request& rq, Mode mode, Outcome* o, std::string* seed) {
  *o = Outcome();
  // 1 args
  if (!IsId(rq.gate_id)) return Fail(o, "ARGS", "gate_id is not [A-Za-z0-9][A-Za-z0-9_-]{0,63}");
  if (rq.version < 1 || rq.version > 999999) return Fail(o, "ARGS", "version is not in 1..999999");
  if (!IsId(rq.kind)) return Fail(o, "ARGS", "kind is not [A-Za-z0-9][A-Za-z0-9_-]{0,63}");
  if (rq.prereg_dir.empty()) return Fail(o, "ARGS", "prereg_dir is empty");
  Instant now = SystemNow();
  if (!rq.now.empty() && !ParseInstant(rq.now, &now)) return Fail(o, "ARGS", "now is not an ISO instant with a time zone");
  // 2 env
  if (mode == Mode::kRelease && rq.env_value.empty()) return Fail(o, "ENV_MISSING", "HELD: the environment value is not set (the first real run of " + rq.gate_id + " needs the coordinator-designated review)");
  // 3 manifest
  std::string manifest_text;
  if (!ReadFile(rq.prereg_dir + "/" + ManifestName(rq), &manifest_text)) return Fail(o, "MANIFEST_UNREADABLE", "HELD: manifest " + ManifestName(rq) + " is unreadable");
  Json m;
  std::string err;
  if (!ParseJson(manifest_text, &m, &err)) return Fail(o, "MANIFEST_INVALID", "manifest is not strict JSON: " + err);
  if (m.type != Json::Type::kObject) return Fail(o, "MANIFEST_INVALID", "manifest is not a JSON object");
  // 4 identity and shape
  const Json* id = Member(&m, "gate_id", Json::Type::kString);
  const Json* ver = Member(&m, "version", Json::Type::kNumber);
  if (id == nullptr || ver == nullptr) return Fail(o, "MANIFEST_INVALID", "manifest gate_id / version missing or mistyped");
  if (id->text != rq.gate_id) return Fail(o, "MANIFEST_IDENTITY", "manifest names gate_id " + id->text + ", not " + rq.gate_id);
  if (ver->text != std::to_string(rq.version)) return Fail(o, "MANIFEST_IDENTITY", "manifest names version " + ver->text + ", not " + std::to_string(rq.version));
  if (!CheckManifestShape(m, o)) return false;
  const Json* review = m.Get("review");
  const Json* held = m.Get("held");
  // 5 names
  const std::string gate_name = m.Get("file")->text, review_name = review->Get("file")->text, designation_name = review->Get("designation_file")->text;
  if (!IsBareName(gate_name) || gate_name != GateFileName(rq)) return Fail(o, "NAME_UNSAFE", "manifest file must be " + GateFileName(rq));
  if (!IsBareName(review_name)) return Fail(o, "NAME_UNSAFE", "manifest review.file is not a bare file name");
  if (!IsBareName(designation_name)) return Fail(o, "NAME_UNSAFE", "manifest review.designation_file is not a bare file name");
  // 6 time
  Instant created, review_at, run_at;
  if (!ParseInstant(m.Get("created_at")->text, &created)) return Fail(o, "MANIFEST_INVALID", "manifest created_at is not an ISO instant with a time zone");
  if (review->Get("at")->type == Json::Type::kNull) return Fail(o, "REVIEW_MISSING", "HELD: manifest review.at is null: the review is not recorded");
  if (!ParseInstant(review->Get("at")->text, &review_at)) return Fail(o, "MANIFEST_INVALID", "manifest review.at is not an ISO instant with a time zone");
  const Json* run = m.Get("first_real_run_at");
  if (mode == Mode::kRelease && run->type != Json::Type::kNull) return Fail(o, "ALREADY_RUN", "manifest first_real_run_at is already set: this gate was run before (one run per registration)");
  if (mode == Mode::kAudit && run->type == Json::Type::kNull) return Fail(o, "AUDIT_NOT_RUN", "manifest first_real_run_at is null: there is no run to audit");
  if (mode == Mode::kAudit && !ParseInstant(run->text, &run_at)) return Fail(o, "MANIFEST_INVALID", "manifest first_real_run_at is not an ISO instant with a time zone");
  if (!Before(created, review_at)) return Fail(o, "ORDER", "created_at < review.at violated");
  if (mode == Mode::kRelease && !Before(review_at, now)) return Fail(o, "ORDER", "review.at < now violated (the review is dated in the future)");
  if (mode == Mode::kAudit && !Before(review_at, run_at)) return Fail(o, "ORDER", "review.at < first_real_run_at violated (the run is not after the review)");
  // 7 gate file, review file, designation file
  std::string gate_sha, review_sha, designation_text;
  if (!Sha256File(rq.prereg_dir + "/" + gate_name, &gate_sha)) return Fail(o, "GATE_FILE_MISSING", "HELD: gate file " + gate_name + " is unreadable");
  Log(o, "gate_file " + gate_name + " sha256 actual=" + gate_sha + " registered=" + m.Get("sha256")->text);
  if (gate_sha != m.Get("sha256")->text) return Fail(o, "GATE_SHA_MISMATCH", "gate file sha256 differs from the manifest");
  if (!Sha256File(rq.prereg_dir + "/" + review_name, &review_sha)) return Fail(o, "REVIEW_FILE_MISSING", "HELD: review file " + review_name + " is unreadable");
  Log(o, "review_file " + review_name + " sha256 actual=" + review_sha + " registered=" + review->Get("sha256")->text);
  if (review_sha != review->Get("sha256")->text) return Fail(o, "REVIEW_FILE_SHA", "review file sha256 differs from the manifest");
  if (!ReadFile(rq.prereg_dir + "/" + designation_name, &designation_text)) return Fail(o, "DESIGNATION_MISSING", "HELD: designation file " + designation_name + " is unreadable");
  o->designation_sha256 = testutil::Sha256Hex(designation_text);
  Log(o, "designation_file " + designation_name + " sha256 actual=" + o->designation_sha256 + " manifest=" + review->Get("designation_file_sha256")->text);
  if (o->designation_sha256 != review->Get("designation_file_sha256")->text) return Fail(o, "DESIGNATION_SHA", "designation file sha256 differs from the manifest");
  if (mode == Mode::kRelease && rq.env_value != o->designation_sha256) return Fail(o, "ENV_MISMATCH", "HELD: the environment value is not the sha256 of the registered designation file");
  // 8 designation
  Json d;
  if (!ParseJson(designation_text, &d, &err)) return Fail(o, "DESIGNATION_INVALID", "designation file is not strict JSON: " + err);
  if (d.type != Json::Type::kObject) return Fail(o, "DESIGNATION_INVALID", "designation file is not a JSON object");
  const Json* pinned = Member(&d, "pinned", Json::Type::kObject);
  if (pinned == nullptr) return Fail(o, "DESIGNATION_INVALID", "designation pinned is not an object");
  const Json* dgate = Member(&d, "gate", Json::Type::kString);
  if (dgate == nullptr || dgate->text != rq.gate_id + ".v" + std::to_string(rq.version)) return Fail(o, "DESIGNATION_GATE", "designation file does not name " + rq.gate_id + ".v" + std::to_string(rq.version));
  const Json* permitted = d.Get("first_real_run_permitted");
  if (permitted == nullptr || permitted->type != Json::Type::kBool || !permitted->boolean) return Fail(o, "DESIGNATION_NOT_PERMITTED", "designation file: first_real_run_permitted is not true");
  const Json* pin_gate = Member(pinned, "gate_md_sha256", Json::Type::kString);
  const Json* pin_review = Member(pinned, "review_file_sha256", Json::Type::kString);
  const Json* pin_manifest = Member(pinned, "manifest_before_review_sha256", Json::Type::kString);
  if (pin_gate == nullptr || pin_review == nullptr || pin_manifest == nullptr) return Fail(o, "DESIGNATION_INVALID", "designation pinned lacks gate_md_sha256 / review_file_sha256 / manifest_before_review_sha256");
  std::string canon_sha;
  if (!CanonManifestSha(m, &canon_sha)) return Fail(o, "MANIFEST_INVALID", "manifest has a number that has no canonical form");
  Log(o, "manifest_before_review_sha256 actual=" + canon_sha + " designation=" + pin_manifest->text);
  if (pin_gate->text != gate_sha) return Fail(o, "DESIGNATION_PIN", "designation pinned.gate_md_sha256 differs from the gate file");
  if (pin_review->text != review_sha) return Fail(o, "DESIGNATION_PIN", "designation pinned.review_file_sha256 differs from the review file");
  if (pin_manifest->text != canon_sha) return Fail(o, "DESIGNATION_PIN", "manifest differs from the one the designation pinned (edited beyond review / first_real_run_at / held_run)");
  if (mode == Mode::kAudit) {
    struct stat st;
    if (::lstat((rq.prereg_dir + "/" + MarkerName(rq)).c_str(), &st) != 0) return Fail(o, "MARKER_MISSING", "no one-run marker " + MarkerName(rq) + " for a manifest that records a run");
    o->ok = true;
    return true;
  }
  // 9 kind, pins, inputs
  bool kind_ok = false;
  for (const Json& k : held->Get("kinds")->items) kind_ok = kind_ok || k.text == rq.kind;
  if (!kind_ok) return Fail(o, "KIND_UNREGISTERED", "kind " + rq.kind + " is not in manifest held.kinds");
  const Json* pins = held->Get("pins");
  std::set<std::string> presented;
  for (const std::string& path : rq.pins) {
    const std::string base = Basename(path);
    if (!presented.insert(base).second) return Fail(o, "PIN_DUPLICATE", "two presented pin files have the base name " + base);
    const Json* want = pins->Get(base);
    if (want == nullptr) return Fail(o, "PIN_UNREGISTERED", "manifest held.pins has no entry for " + base);
    std::string have;
    if (!Sha256File(path, &have)) return Fail(o, "PIN_UNREADABLE", "pinned file " + base + " is unreadable");
    Log(o, "pin " + base + " sha256 actual=" + have + " registered=" + want->text);
    if (have != want->text) return Fail(o, "PIN_MISMATCH", "held file " + base + " sha256 differs from the registered one");
  }
  for (const std::string& key : pins->keys) {
    if (presented.count(key) == 0) return Fail(o, "PIN_NOT_PRESENTED", "manifest held.pins registers " + key + " but the request does not present it");
  }
  const Json* registered = held->Get("inputs")->Get(rq.kind);
  if (registered != nullptr && rq.data_dir.empty()) return Fail(o, "INPUT_NOT_SUPPLIED", "manifest registers input data for kind " + rq.kind + " but the request supplies no data_dir");
  if (registered == nullptr && !rq.data_dir.empty()) return Fail(o, "INPUT_UNREGISTERED", "the request supplies a data_dir but the manifest registers no input data for kind " + rq.kind);
  if (registered != nullptr) {
    std::string have, code;
    if (!TreeSha(rq.data_dir, &have, &code)) return Fail(o, code.c_str(), "input data " + rq.data_dir + " cannot be hashed (" + code + ")");
    Log(o, "inputs " + rq.kind + " tree sha256 actual=" + have + " registered=" + registered->text);
    if (have != registered->text) return Fail(o, "INPUT_MISMATCH", "input data " + rq.data_dir + " differs from the registered data (tree sha256)");
  }
  // 10 seed
  if (held->Get("seed")->boolean != rq.want_seed) return Fail(o, "SEED_POLICY", "the manifest registers held.seed=" + std::string(held->Get("seed")->boolean ? "true" : "false") + " but the request says otherwise");
  if (rq.want_seed && !LoadSeed(rq, *held, *pinned, o, seed)) return false;
  o->ok = true;
  return true;
}

}  // namespace internal

// Every check, no side effect (no marker, no seed handed out). Use it for the "release" decision that does not start a run.
inline Outcome CheckRelease(const Request& rq) {
  Outcome o;
  std::string seed;
  internal::Evaluate(rq, internal::Mode::kRelease, &o, &seed);
  return o;
}

// Post-run audit of a manifest that records first_real_run_at: created_at < review.at < first_real_run_at, the pinned files,
// and the marker of this kind. No env value, no pins, no data.
inline Outcome Audit(const Request& rq) {
  Outcome o;
  std::string seed;
  internal::Evaluate(rq, internal::Mode::kAudit, &o, &seed);
  return o;
}

// The complete gate: CheckRelease, then the one-run marker (O_EXCL, never removed). Only here the seed is handed out.
// A refusal before the marker is not a run; a call that created the marker is a run, whatever happens afterwards.
// Keep the returned Outcome: a second call for the same (gate, version, kind) is refused (MARKER_EXISTS).
inline Outcome HeldGate(const Request& rq) {
  Outcome o;
  std::string seed;
  if (!internal::Evaluate(rq, internal::Mode::kRelease, &o, &seed)) return o;
  const std::string path = rq.prereg_dir + "/" + MarkerName(rq);
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0444);
  if (fd < 0) {
    internal::Fail(&o, errno == EEXIST ? "MARKER_EXISTS" : "MARKER_FAILED",
                   "HELD: the first-run marker " + path + " exists (or cannot be created): this kind was run before");
    return o;
  }
  const std::string body = "{\"gate\":\"" + rq.gate_id + ".v" + std::to_string(rq.version) + "\",\"kind\":\"" + rq.kind +
                           "\",\"designation_sha256\":\"" + o.designation_sha256 + "\",\"pid\":" + std::to_string(::getpid()) + "}\n";
  const ssize_t n = ::write(fd, body.data(), body.size());
  (void)n;
  ::fsync(fd);
  ::close(fd);
  internal::Log(&o, "first-run marker created: " + path);
  o.seed = seed;
  o.marker = path;
  return o;
}

// Report mode must never measure registered gate data. Decided by what the directory CONTAINS, not by its name or path:
// its tree sha256 equals a registered held.inputs value, or a file in it has a sha256 listed in held.sealed_file_sha256.
// A manifest that cannot be read or parsed gives "sealed" (fail closed). ok == true means "not registered data".
inline Outcome RefuseSealed(const Request& rq) {
  Outcome o;
  auto refuse = [&o](const char* code, const std::string& why) {
    internal::Fail(&o, code, why);
    return o;
  };
  std::string text, err;
  Json m;
  if (!IsId(rq.gate_id) || rq.version < 1 || rq.version > 999999 || rq.data_dir.empty()) return refuse("ARGS", "gate_id / version / data_dir invalid");
  if (!ReadFile(rq.prereg_dir + "/" + ManifestName(rq), &text) || !ParseJson(text, &m, &err) || m.type != Json::Type::kObject) {
    return refuse("SEALED_DATA", "the manifest cannot be read, so the data cannot be told from registered data (fail closed)");
  }
  const Json* held = internal::Member(&m, "held", Json::Type::kObject);
  const Json* inputs = internal::Member(held, "inputs", Json::Type::kObject);
  if (inputs == nullptr) return refuse("SEALED_DATA", "manifest held.inputs is missing (fail closed)");
  std::string tree, code;
  if (!TreeSha(rq.data_dir, &tree, &code)) return refuse("SEALED_DATA", rq.data_dir + " cannot be hashed (" + code + "), so it cannot be told from registered data (fail closed)");
  for (const Json& v : inputs->values) {
    if (v.type == Json::Type::kString && v.text == tree) return refuse("SEALED_DATA", "the files of " + rq.data_dir + " have the tree sha256 of registered gate data");
  }
  std::set<std::string> sealed_files;
  if (const Json* arr = held->Get("sealed_file_sha256")) {
    for (const Json& s : arr->items) sealed_files.insert(s.text);
  }
  std::vector<std::string> files;
  if (!sealed_files.empty() && ListTree(rq.data_dir, "", &files, &code)) {  // cannot fail here: TreeSha listed the same tree
    for (const std::string& f : files) {
      std::string h;
      if (Sha256File(rq.data_dir + "/" + f, &h) && sealed_files.count(h) != 0) return refuse("SEALED_DATA", rq.data_dir + "/" + f + " has the sha256 of a sealed file");
    }
  }
  o.ok = true;
  return o;
}

}  // namespace slam_refact::sr_held

#endif  // SLAM_REFACT_TESTS_SUPPORT_SR_HELD_H_
