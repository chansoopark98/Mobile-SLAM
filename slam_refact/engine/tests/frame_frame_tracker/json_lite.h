// Minimal JSON reader for the WP2 golden files (tests only; numbers are parsed with strtod so a
// float32 value printed by JS round-trips exactly through double -> float).

#ifndef SLAM_REFACT_TESTS_FRAME_FRAME_TRACKER_JSON_LITE_H_
#define SLAM_REFACT_TESTS_FRAME_FRAME_TRACKER_JSON_LITE_H_

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace slam_refact::testing {

struct JsonValue {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };
  Type type = Type::kNull;
  bool b = false;
  double num = 0.0;
  std::string str;
  std::vector<JsonValue> arr;
  std::map<std::string, JsonValue> obj;

  const JsonValue& operator[](size_t i) const { return arr.at(i); }
  const JsonValue& operator[](const std::string& key) const { return obj.at(key); }
  size_t size() const { return type == Type::kArray ? arr.size() : obj.size(); }
  double AsDouble() const { return num; }
  float AsFloat() const { return static_cast<float>(num); }
  long long AsInt() const { return static_cast<long long>(num); }
};

class JsonReader {
 public:
  explicit JsonReader(const std::string& text) : s_(text) {}

  bool Parse(JsonValue* out) {
    SkipWs();
    if (!Value(out)) {
      return false;
    }
    SkipWs();
    return p_ == s_.size();
  }

 private:
  void SkipWs() {
    while (p_ < s_.size() && (s_[p_] == ' ' || s_[p_] == '\n' || s_[p_] == '\r' || s_[p_] == '\t')) {
      ++p_;
    }
  }
  bool Literal(const char* lit) {
    const std::string l(lit);
    if (s_.compare(p_, l.size(), l) != 0) {
      return false;
    }
    p_ += l.size();
    return true;
  }
  bool String(std::string* out) {
    if (s_[p_] != '"') {
      return false;
    }
    ++p_;
    while (p_ < s_.size() && s_[p_] != '"') {
      if (s_[p_] == '\\') {
        ++p_;
        if (p_ >= s_.size()) {
          return false;
        }
        const char c = s_[p_];
        out->push_back(c == 'n' ? '\n' : c == 't' ? '\t' : c);
      } else {
        out->push_back(s_[p_]);
      }
      ++p_;
    }
    if (p_ >= s_.size()) {
      return false;
    }
    ++p_;
    return true;
  }
  bool Value(JsonValue* v) {
    if (p_ >= s_.size()) {
      return false;
    }
    const char c = s_[p_];
    if (c == '{') {
      v->type = JsonValue::Type::kObject;
      ++p_;
      SkipWs();
      if (s_[p_] == '}') {
        ++p_;
        return true;
      }
      while (true) {
        SkipWs();
        std::string key;
        if (!String(&key)) {
          return false;
        }
        SkipWs();
        if (s_[p_] != ':') {
          return false;
        }
        ++p_;
        SkipWs();
        JsonValue child;
        if (!Value(&child)) {
          return false;
        }
        v->obj.emplace(key, std::move(child));
        SkipWs();
        if (s_[p_] == ',') {
          ++p_;
          continue;
        }
        if (s_[p_] == '}') {
          ++p_;
          return true;
        }
        return false;
      }
    }
    if (c == '[') {
      v->type = JsonValue::Type::kArray;
      ++p_;
      SkipWs();
      if (s_[p_] == ']') {
        ++p_;
        return true;
      }
      while (true) {
        SkipWs();
        JsonValue child;
        if (!Value(&child)) {
          return false;
        }
        v->arr.push_back(std::move(child));
        SkipWs();
        if (s_[p_] == ',') {
          ++p_;
          continue;
        }
        if (s_[p_] == ']') {
          ++p_;
          return true;
        }
        return false;
      }
    }
    if (c == '"') {
      v->type = JsonValue::Type::kString;
      return String(&v->str);
    }
    if (Literal("true")) {
      v->type = JsonValue::Type::kBool;
      v->b = true;
      return true;
    }
    if (Literal("false")) {
      v->type = JsonValue::Type::kBool;
      return true;
    }
    if (Literal("null")) {
      return true;
    }
    const char* begin = s_.c_str() + p_;
    char* end = nullptr;
    v->num = std::strtod(begin, &end);
    if (end == begin) {
      return false;
    }
    v->type = JsonValue::Type::kNumber;
    p_ += static_cast<size_t>(end - begin);
    return true;
  }

  const std::string& s_;
  size_t p_ = 0;
};

inline bool ReadJson(const std::string& path, JsonValue* out) {
  std::ifstream f(path);
  if (!f) {
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string text = ss.str();
  JsonReader r(text);
  return r.Parse(out);
}

}  // namespace slam_refact::testing

#endif  // SLAM_REFACT_TESTS_FRAME_FRAME_TRACKER_JSON_LITE_H_
