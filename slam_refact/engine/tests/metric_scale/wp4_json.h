// Minimal JSON reader for the WP4 golden files (tests only). Aborts on malformed input
// (tests build with -fno-exceptions). Numbers are parsed with strtod, so a float written as its
// shortest double repr round-trips exactly.

#ifndef SLAM_REFACT_TESTS_METRIC_SCALE_WP4_JSON_H_
#define SLAM_REFACT_TESTS_METRIC_SCALE_WP4_JSON_H_

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace wp4_json {

struct Value {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };
  Type type = Type::kNull;
  bool b = false;
  double num = 0.0;
  std::string str;
  std::vector<Value> arr;
  std::map<std::string, Value> obj;

  bool is_null() const { return type == Type::kNull; }
  const Value& operator[](const std::string& k) const {
    auto it = obj.find(k);
    if (it == obj.end()) {
      std::fprintf(stderr, "wp4_json: missing key %s\n", k.c_str());
      std::abort();
    }
    return it->second;
  }
  const Value& operator[](size_t i) const { return arr.at(i); }
  size_t size() const { return type == Type::kArray ? arr.size() : obj.size(); }
  double d() const { return num; }
  float f() const { return static_cast<float>(num); }
  long long ll() const { return static_cast<long long>(num); }
  std::vector<float> floats() const {
    std::vector<float> out;
    for (const Value& v : arr) out.push_back(v.f());
    return out;
  }
};

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}
  Value Parse() {
    Value v = ParseValue();
    Ws();
    return v;
  }

 private:
  [[noreturn]] void Fail() {
    std::fprintf(stderr, "wp4_json: parse error at %zu\n", i_);
    std::abort();
  }
  void Ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\n' || s_[i_] == '\r' || s_[i_] == '\t')) ++i_;
  }
  bool Lit(const char* lit) {
    size_t n = std::string(lit).size();
    if (s_.compare(i_, n, lit) == 0) {
      i_ += n;
      return true;
    }
    return false;
  }
  std::string ParseString() {
    if (s_[i_] != '"') Fail();
    ++i_;
    std::string out;
    while (i_ < s_.size() && s_[i_] != '"') {
      if (s_[i_] == '\\') {
        ++i_;
        const char c = s_[i_];
        if (c == 'n') out.push_back('\n');
        else if (c == 't') out.push_back('\t');
        else if (c == 'u') { out.push_back('?'); i_ += 4; }
        else out.push_back(c);
        ++i_;
      } else {
        out.push_back(s_[i_++]);
      }
    }
    if (i_ >= s_.size()) Fail();
    ++i_;
    return out;
  }
  Value ParseValue() {
    Ws();
    if (i_ >= s_.size()) Fail();
    Value v;
    const char c = s_[i_];
    if (c == '{') {
      v.type = Value::Type::kObject;
      ++i_;
      Ws();
      if (s_[i_] == '}') { ++i_; return v; }
      while (true) {
        Ws();
        std::string k = ParseString();
        Ws();
        if (s_[i_] != ':') Fail();
        ++i_;
        v.obj[k] = ParseValue();
        Ws();
        if (s_[i_] == ',') { ++i_; continue; }
        if (s_[i_] == '}') { ++i_; break; }
        Fail();
      }
    } else if (c == '[') {
      v.type = Value::Type::kArray;
      ++i_;
      Ws();
      if (s_[i_] == ']') { ++i_; return v; }
      while (true) {
        v.arr.push_back(ParseValue());
        Ws();
        if (s_[i_] == ',') { ++i_; continue; }
        if (s_[i_] == ']') { ++i_; break; }
        Fail();
      }
    } else if (c == '"') {
      v.type = Value::Type::kString;
      v.str = ParseString();
    } else if (Lit("true")) {
      v.type = Value::Type::kBool;
      v.b = true;
    } else if (Lit("false")) {
      v.type = Value::Type::kBool;
    } else if (Lit("null")) {
      v.type = Value::Type::kNull;
    } else if (Lit("NaN")) {
      v.type = Value::Type::kNumber;
      v.num = std::strtod("nan", nullptr);
    } else {
      v.type = Value::Type::kNumber;
      char* end = nullptr;
      v.num = std::strtod(s_.c_str() + i_, &end);
      if (end == s_.c_str() + i_) Fail();
      i_ = static_cast<size_t>(end - s_.c_str());
    }
    return v;
  }
  const std::string& s_;
  size_t i_ = 0;
};

inline Value Load(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "wp4_json: cannot open %s\n", path.c_str());
    std::abort();
  }
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();
  return Parser(text).Parse();
}

}  // namespace wp4_json

#endif  // SLAM_REFACT_TESTS_METRIC_SCALE_WP4_JSON_H_
