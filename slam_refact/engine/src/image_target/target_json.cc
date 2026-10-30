// Target JSON reader (US-012). A small RFC 8259 parser (objects, arrays, strings with escapes,
// numbers, true/false/null) followed by the field mapping of tracking-controller.ts:822-873.

#include "image_target/target_json.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace slam_refact::image_target {
namespace {

struct JsonValue {
  enum class Kind { kNull, kBool, kNumber, kString, kArray, kObject };
  Kind kind = Kind::kNull;
  bool boolean = false;
  double number = 0.0;
  std::string string;
  std::vector<JsonValue> array;
  std::vector<std::pair<std::string, JsonValue>> object;

  // Last member with this key (duplicate keys: the last one wins, like JSON.parse).
  const JsonValue* Find(std::string_view key) const {
    const JsonValue* found = nullptr;
    for (const auto& member : object) {
      if (member.first == key) {
        found = &member.second;
      }
    }
    return found;
  }
};

constexpr int kMaxDepth = 64;

class JsonParser {
 public:
  explicit JsonParser(std::string_view text) : text_(text) {}

  bool Parse(JsonValue* out, std::string* error) {
    SkipWhitespace();
    if (!ParseValue(out, 0)) {
      *error = error_;
      return false;
    }
    SkipWhitespace();
    if (pos_ != text_.size()) {
      *error = Fail("trailing characters after the JSON value");
      return false;
    }
    return true;
  }

 private:
  std::string Fail(const char* what) {
    error_ = std::string("JSON parse error at byte ") + std::to_string(pos_) + ": " + what;
    return error_;
  }

  void SkipWhitespace() {
    while (pos_ < text_.size() &&
           (text_[pos_] == ' ' || text_[pos_] == '\t' || text_[pos_] == '\n' ||
            text_[pos_] == '\r')) {
      ++pos_;
    }
  }

  bool Consume(std::string_view literal) {
    if (text_.substr(pos_, literal.size()) != literal) {
      return false;
    }
    pos_ += literal.size();
    return true;
  }

  bool ParseValue(JsonValue* out, int depth) {
    if (depth > kMaxDepth) {
      Fail("nesting too deep");
      return false;
    }
    if (pos_ >= text_.size()) {
      Fail("unexpected end of input");
      return false;
    }
    const char c = text_[pos_];
    if (c == '{') {
      return ParseObject(out, depth);
    }
    if (c == '[') {
      return ParseArray(out, depth);
    }
    if (c == '"') {
      out->kind = JsonValue::Kind::kString;
      return ParseString(&out->string);
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
      return ParseNumber(out);
    }
    if (Consume("true")) {
      out->kind = JsonValue::Kind::kBool;
      out->boolean = true;
      return true;
    }
    if (Consume("false")) {
      out->kind = JsonValue::Kind::kBool;
      out->boolean = false;
      return true;
    }
    if (Consume("null")) {
      out->kind = JsonValue::Kind::kNull;
      return true;
    }
    Fail("unexpected character");
    return false;
  }

  bool ParseObject(JsonValue* out, int depth) {
    out->kind = JsonValue::Kind::kObject;
    ++pos_;  // '{'
    SkipWhitespace();
    if (pos_ < text_.size() && text_[pos_] == '}') {
      ++pos_;
      return true;
    }
    while (true) {
      SkipWhitespace();
      if (pos_ >= text_.size() || text_[pos_] != '"') {
        Fail("expected a string key");
        return false;
      }
      std::string key;
      if (!ParseString(&key)) {
        return false;
      }
      SkipWhitespace();
      if (pos_ >= text_.size() || text_[pos_] != ':') {
        Fail("expected ':'");
        return false;
      }
      ++pos_;
      SkipWhitespace();
      JsonValue value;
      if (!ParseValue(&value, depth + 1)) {
        return false;
      }
      out->object.emplace_back(std::move(key), std::move(value));
      SkipWhitespace();
      if (pos_ < text_.size() && text_[pos_] == ',') {
        ++pos_;
        continue;
      }
      if (pos_ < text_.size() && text_[pos_] == '}') {
        ++pos_;
        return true;
      }
      Fail("expected ',' or '}'");
      return false;
    }
  }

  bool ParseArray(JsonValue* out, int depth) {
    out->kind = JsonValue::Kind::kArray;
    ++pos_;  // '['
    SkipWhitespace();
    if (pos_ < text_.size() && text_[pos_] == ']') {
      ++pos_;
      return true;
    }
    while (true) {
      SkipWhitespace();
      JsonValue value;
      if (!ParseValue(&value, depth + 1)) {
        return false;
      }
      out->array.push_back(std::move(value));
      SkipWhitespace();
      if (pos_ < text_.size() && text_[pos_] == ',') {
        ++pos_;
        continue;
      }
      if (pos_ < text_.size() && text_[pos_] == ']') {
        ++pos_;
        return true;
      }
      Fail("expected ',' or ']'");
      return false;
    }
  }

  bool ParseHex4(uint32_t* out) {
    if (pos_ + 4 > text_.size()) {
      Fail("truncated \\u escape");
      return false;
    }
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      const char h = text_[pos_ + i];
      v <<= 4;
      if (h >= '0' && h <= '9') {
        v |= static_cast<uint32_t>(h - '0');
      } else if (h >= 'a' && h <= 'f') {
        v |= static_cast<uint32_t>(h - 'a' + 10);
      } else if (h >= 'A' && h <= 'F') {
        v |= static_cast<uint32_t>(h - 'A' + 10);
      } else {
        Fail("invalid \\u escape");
        return false;
      }
    }
    pos_ += 4;
    *out = v;
    return true;
  }

  static void AppendUtf8(uint32_t cp, std::string* out) {
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

  bool ParseString(std::string* out) {
    ++pos_;  // opening quote
    out->clear();
    while (pos_ < text_.size()) {
      const char c = text_[pos_++];
      if (c == '"') {
        return true;
      }
      if (static_cast<unsigned char>(c) < 0x20) {
        Fail("control character in string");
        return false;
      }
      if (c != '\\') {
        out->push_back(c);
        continue;
      }
      if (pos_ >= text_.size()) {
        break;
      }
      const char e = text_[pos_++];
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
          if (!ParseHex4(&cp)) {
            return false;
          }
          if (cp >= 0xD800 && cp <= 0xDBFF) {
            uint32_t low = 0;
            if (!Consume("\\u") || !ParseHex4(&low) || low < 0xDC00 || low > 0xDFFF) {
              Fail("unpaired surrogate in \\u escape");
              return false;
            }
            cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
          } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
            Fail("unpaired surrogate in \\u escape");
            return false;
          }
          AppendUtf8(cp, out);
          break;
        }
        default:
          Fail("invalid escape");
          return false;
      }
    }
    Fail("unterminated string");
    return false;
  }

  bool ParseNumber(JsonValue* out) {
    const size_t start = pos_;
    if (text_[pos_] == '-') {
      ++pos_;
    }
    auto digits = [this]() {
      const size_t begin = pos_;
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') {
        ++pos_;
      }
      return pos_ - begin;
    };
    if (pos_ < text_.size() && text_[pos_] == '0') {
      ++pos_;
    } else if (digits() == 0) {
      Fail("invalid number");
      return false;
    }
    if (pos_ < text_.size() && text_[pos_] == '.') {
      ++pos_;
      if (digits() == 0) {
        Fail("invalid number fraction");
        return false;
      }
    }
    if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
      ++pos_;
      if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) {
        ++pos_;
      }
      if (digits() == 0) {
        Fail("invalid number exponent");
        return false;
      }
    }
    const std::string literal(text_.substr(start, pos_ - start));
    out->kind = JsonValue::Kind::kNumber;
    out->number = std::strtod(literal.c_str(), nullptr);
    return true;
  }

  std::string_view text_;
  size_t pos_ = 0;
  std::string error_;
};

bool GetNumber(const JsonValue& object, std::string_view key, const char* where, double* out,
               std::string* error) {
  const JsonValue* v = object.Find(key);
  if (v == nullptr || v->kind != JsonValue::Kind::kNumber || !std::isfinite(v->number)) {
    *error = std::string(where) + std::string(key) + " must be a finite number";
    return false;
  }
  *out = v->number;
  return true;
}

// capnp setInt32 of a JS number: truncation toward zero (values outside int32 are rejected).
bool GetInt(const JsonValue& object, std::string_view key, int* out, std::string* error) {
  double v = 0.0;
  if (!GetNumber(object, key, "properties.", &v, error)) {
    return false;
  }
  const double t = std::trunc(v);
  if (t < static_cast<double>(INT32_MIN) || t > static_cast<double>(INT32_MAX)) {
    *error = "properties." + std::string(key) + " is outside the int32 range";
    return false;
  }
  *out = static_cast<int>(t);
  return true;
}

bool GetFloat(const JsonValue& object, std::string_view key, const char* where, float* out,
              std::string* error) {
  double v = 0.0;
  if (!GetNumber(object, key, where, &v, error)) {
    return false;
  }
  *out = static_cast<float>(v);
  return true;
}

}  // namespace

bool ParseImageTargetJson(std::string_view json, ImageTargetSpec* out, std::string* error) {
  JsonValue root;
  if (!JsonParser(json).Parse(&root, error)) {
    return false;
  }
  if (root.kind != JsonValue::Kind::kObject) {
    *error = "image target JSON must be an object";
    return false;
  }
  ImageTargetSpec spec;

  const JsonValue* name = root.Find("name");
  if (name == nullptr || name->kind != JsonValue::Kind::kString || name->string.empty()) {
    *error = "name must be a non-empty string";
    return false;
  }
  spec.name = name->string;

  const JsonValue* type = root.Find("type");
  if (type == nullptr || type->kind != JsonValue::Kind::kString) {
    *error = "type must be a string";
    return false;
  }
  spec.type_name = type->string;
  if (type->string == "CYLINDER" || type->string == "CONICAL") {
    spec.type = ImageTargetType::kCurvy;
  } else if (type->string == "PLANAR") {
    spec.type = ImageTargetType::kPlanar;
  } else {
    spec.type = ImageTargetType::kUnspecified;
  }

  const JsonValue* image_path = root.Find("imagePath");
  if (image_path != nullptr && image_path->kind == JsonValue::Kind::kString) {
    spec.image_path = image_path->string;
  }

  const JsonValue* props = root.Find("properties");
  if (props == nullptr || props->kind != JsonValue::Kind::kObject) {
    *error = "properties must be an object";
    return false;
  }
  if (!GetInt(*props, "originalWidth", &spec.original_image_width, error) ||
      !GetInt(*props, "originalHeight", &spec.original_image_height, error) ||
      !GetInt(*props, "left", &spec.crop_original_image_x, error) ||
      !GetInt(*props, "top", &spec.crop_original_image_y, error) ||
      !GetInt(*props, "width", &spec.crop_original_image_width, error) ||
      !GetInt(*props, "height", &spec.crop_original_image_height, error)) {
    return false;
  }

  // properties.isRotated === true || properties.isRotated === 'true'.
  const JsonValue* rotated = props->Find("isRotated");
  spec.is_rotated = rotated != nullptr &&
                    ((rotated->kind == JsonValue::Kind::kBool && rotated->boolean) ||
                     (rotated->kind == JsonValue::Kind::kString && rotated->string == "true"));

  // `if (properties.staticOrientation)`: an object is truthy, null/absent is not.
  const JsonValue* orientation = props->Find("staticOrientation");
  if (orientation != nullptr && orientation->kind == JsonValue::Kind::kObject) {
    spec.is_static_target = true;
    if (!GetFloat(*orientation, "rollAngle", "properties.staticOrientation.", &spec.roll_angle,
                  error) ||
        !GetFloat(*orientation, "pitchAngle", "properties.staticOrientation.", &spec.pitch_angle,
                  error)) {
      return false;
    }
  } else if (orientation != nullptr && orientation->kind != JsonValue::Kind::kNull) {
    *error = "properties.staticOrientation must be an object or null";
    return false;
  }

  if (spec.type == ImageTargetType::kCurvy) {
    if (!GetFloat(*props, "cylinderCircumferenceTop", "properties.",
                  &spec.curvy_circumference_top, error) ||
        !GetFloat(*props, "cylinderCircumferenceBottom", "properties.",
                  &spec.curvy_circumference_bottom, error) ||
        !GetFloat(*props, "cylinderSideLength", "properties.", &spec.curvy_side_length, error) ||
        !GetFloat(*props, "targetCircumferenceTop", "properties.", &spec.target_circumference_top,
                  error)) {
      return false;
    }
  }
  *out = std::move(spec);
  return true;
}

}  // namespace slam_refact::image_target
