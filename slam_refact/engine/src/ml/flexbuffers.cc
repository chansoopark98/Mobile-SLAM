// FlexBuffers layout (flatbuffers/flexbuffers.h, format spec "FlexBuffers"): the root is the last
// bytes of the blob: [value][packed type][root byte width]. A packed type is (type << 2) | log2
// width. Offsets are unsigned and point backwards from where they are stored. A map stores, right
// before its values: keys vector offset, keys byte width, size; after its values one type byte per
// value.
#include "ml/flexbuffers.h"

#include <cstring>

namespace slam_refact {
namespace ml {
namespace {

enum FlexType : int {
  kFbtNull = 0,
  kFbtInt = 1,
  kFbtUint = 2,
  kFbtFloat = 3,
  kFbtKey = 4,
  kFbtString = 5,
  kFbtIndirectInt = 6,
  kFbtIndirectUint = 7,
  kFbtIndirectFloat = 8,
  kFbtMap = 9,
  kFbtVector = 10,
  kFbtVectorInt = 11,
  kFbtVectorUint = 12,
  kFbtVectorFloat = 13,
  kFbtVectorKey = 14,
  kFbtVectorInt2 = 16,
  kFbtVectorFloat4 = 24,
  kFbtBool = 26,
  kFbtVectorBool = 36,
};

class Reader {
 public:
  Reader(const uint8_t* data, size_t size) : data_(data), size_(size) {}

  bool ok() const { return ok_; }
  const std::string& error() const { return error_; }

  void Fail(const std::string& message) {
    if (ok_) error_ = message;
    ok_ = false;
  }

  bool Has(size_t pos, size_t n) const { return pos <= size_ && n <= size_ - pos; }

  uint64_t Uint(size_t pos, size_t width) {
    if (!(width == 1 || width == 2 || width == 4 || width == 8) || !Has(pos, width)) {
      Fail("flexbuffer read outside the blob");
      return 0;
    }
    uint64_t v = 0;
    for (size_t i = 0; i < width; ++i) v |= static_cast<uint64_t>(data_[pos + i]) << (8 * i);
    return v;
  }

  int64_t Int(size_t pos, size_t width) {
    const uint64_t u = Uint(pos, width);
    if (width == 8) return static_cast<int64_t>(u);
    const uint64_t sign = uint64_t{1} << (8 * width - 1);
    return static_cast<int64_t>((u ^ sign) - sign);
  }

  double Float(size_t pos, size_t width) {
    if (width == 4) {
      const uint32_t bits = static_cast<uint32_t>(Uint(pos, 4));
      float f;
      std::memcpy(&f, &bits, sizeof(f));
      return f;
    }
    if (width == 8) {
      const uint64_t bits = Uint(pos, 8);
      double d;
      std::memcpy(&d, &bits, sizeof(d));
      return d;
    }
    Fail("flexbuffer float width is not 4 or 8");
    return 0.0;
  }

  // Follows an offset stored at `pos` (width `width`) backwards.
  size_t Indirect(size_t pos, size_t width) {
    const uint64_t off = Uint(pos, width);
    if (!ok_ || off > pos) {
      Fail("flexbuffer offset points before the blob");
      return 0;
    }
    return pos - static_cast<size_t>(off);
  }

  uint8_t Byte(size_t pos) {
    if (!Has(pos, 1)) {
      Fail("flexbuffer read outside the blob");
      return 0;
    }
    return data_[pos];
  }

  std::string CString(size_t pos) {
    std::string s;
    while (true) {
      if (!Has(pos, 1)) {
        Fail("unterminated flexbuffer key");
        return std::string();
      }
      const char c = static_cast<char>(data_[pos++]);
      if (c == '\0') break;
      s.push_back(c);
    }
    return s;
  }

  // Scalar of element type `type` stored at pos with byte width `width`.
  bool Scalar(size_t pos, size_t width, int type, double* number, int64_t* integer,
              bool* is_integer) {
    switch (type) {
      case kFbtInt:
        *integer = Int(pos, width);
        *number = static_cast<double>(*integer);
        *is_integer = true;
        return ok_;
      case kFbtUint:
      case kFbtBool:
        *integer = static_cast<int64_t>(Uint(pos, width));
        *number = static_cast<double>(*integer);
        *is_integer = true;
        return ok_;
      case kFbtFloat:
        *number = Float(pos, width);
        *integer = 0;
        *is_integer = false;
        return ok_;
      default:
        Fail("flexbuffer element is not a scalar");
        return false;
    }
  }

  // Value stored at `pos` in a parent of byte width `parent_width` with packed type `packed`.
  bool Value(size_t pos, size_t parent_width, uint8_t packed, FlexValue* out) {
    const int type = packed >> 2;
    const size_t width = size_t{1} << (packed & 3);
    double number = 0.0;
    int64_t integer = 0;
    bool is_integer = false;
    switch (type) {
      case kFbtNull:
        out->kind = FlexValue::Kind::kNull;
        return true;
      case kFbtInt:
      case kFbtUint:
      case kFbtFloat:
      case kFbtBool:
        if (!Scalar(pos, parent_width, type, &number, &integer, &is_integer)) return false;
        if (type == kFbtBool) {
          out->kind = FlexValue::Kind::kBool;
          out->int_value = integer;
        } else if (is_integer) {
          out->kind = FlexValue::Kind::kInt;
          out->int_value = integer;
          out->float_value = number;
        } else {
          out->kind = FlexValue::Kind::kFloat;
          out->float_value = number;
        }
        return true;
      default:
        break;
    }
    const size_t target = Indirect(pos, parent_width);
    if (!ok_) return false;
    switch (type) {
      case kFbtIndirectInt:
      case kFbtIndirectUint:
      case kFbtIndirectFloat: {
        const int scalar = type - kFbtIndirectInt + kFbtInt;
        if (!Scalar(target, width, scalar, &number, &integer, &is_integer)) return false;
        out->kind = is_integer ? FlexValue::Kind::kInt : FlexValue::Kind::kFloat;
        out->int_value = integer;
        out->float_value = number;
        return true;
      }
      case kFbtString: {
        if (target < width) {
          Fail("flexbuffer string outside the blob");
          return false;
        }
        const uint64_t n = Uint(target - width, width);
        if (!ok_ || n > size_ || !Has(target, static_cast<size_t>(n))) {
          Fail("flexbuffer string outside the blob");
          return false;
        }
        out->kind = FlexValue::Kind::kString;
        out->string_value.assign(reinterpret_cast<const char*>(data_ + target),
                                 static_cast<size_t>(n));
        return true;
      }
      default:
        break;
    }
    // Vectors of scalars.
    size_t count = 0;
    int elem_type = -1;
    bool typed = true;
    if (type >= kFbtVectorInt2 && type <= kFbtVectorFloat4) {
      count = static_cast<size_t>((type - kFbtVectorInt2) / 3 + 2);
      elem_type = (type - kFbtVectorInt2) % 3 + kFbtInt;
    } else if ((type >= kFbtVectorInt && type <= kFbtVectorFloat) || type == kFbtVectorBool ||
               type == kFbtVector) {
      if (target < width) {
        Fail("flexbuffer vector size outside the blob");
        return false;
      }
      count = static_cast<size_t>(Uint(target - width, width));
      if (type == kFbtVector) {
        typed = false;
      } else {
        elem_type = type == kFbtVectorBool ? kFbtBool : type - kFbtVectorInt + kFbtInt;
      }
    } else {
      Fail("unsupported flexbuffer value type " + std::to_string(type));
      return false;
    }
    if (!ok_ || count > size_ || !Has(target, count * width) ||
        (!typed && !Has(target + count * width, count))) {
      Fail("flexbuffer vector outside the blob");
      return false;
    }
    out->kind = FlexValue::Kind::kNumberVector;
    out->numbers.clear();
    out->ints.clear();
    out->integer_vector = true;
    for (size_t i = 0; i < count; ++i) {
      int et = elem_type;
      if (!typed) {
        const uint8_t elem_packed = Byte(target + count * width + i);
        et = elem_packed >> 2;
      }
      if (!Scalar(target + i * width, width, et, &number, &integer, &is_integer)) return false;
      out->numbers.push_back(number);
      out->ints.push_back(integer);
      out->integer_vector = out->integer_vector && is_integer;
    }
    return true;
  }

 private:
  const uint8_t* data_;
  size_t size_;
  bool ok_ = true;
  std::string error_;
};

}  // namespace

bool ParseFlexMap(const uint8_t* data, size_t size, FlexMap* out, std::string* error) {
  out->clear();
  if (data == nullptr || size < 3) {
    *error = "flexbuffer shorter than 3 bytes";
    return false;
  }
  Reader r(data, size);
  const size_t root_width = data[size - 1];
  const uint8_t root_packed = data[size - 2];
  if (!(root_width == 1 || root_width == 2 || root_width == 4 || root_width == 8) ||
      size < 2 + root_width) {
    *error = "bad flexbuffer root width";
    return false;
  }
  if ((root_packed >> 2) != kFbtMap) {
    *error = "flexbuffer root is not a map";
    return false;
  }
  const size_t root_pos = size - 2 - root_width;
  const size_t map = r.Indirect(root_pos, root_width);
  const size_t width = size_t{1} << (root_packed & 3);
  if (!r.ok() || map < 3 * width) {
    *error = r.ok() ? "flexbuffer map header outside the blob" : r.error();
    return false;
  }
  const size_t count = static_cast<size_t>(r.Uint(map - width, width));
  const size_t keys_width = static_cast<size_t>(r.Uint(map - 2 * width, width));
  const size_t keys = r.Indirect(map - 3 * width, width);
  if (!r.ok() || count > size || !r.Has(map, count * width) ||
      !r.Has(map + count * width, count) || !r.Has(keys, count * keys_width)) {
    *error = r.ok() ? "flexbuffer map outside the blob" : r.error();
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    const size_t key_at = r.Indirect(keys + i * keys_width, keys_width);
    const std::string key = r.CString(key_at);
    FlexValue value;
    const uint8_t packed = r.Byte(map + count * width + i);
    if (!r.ok() || !r.Value(map + i * width, width, packed, &value)) {
      *error = r.error();
      return false;
    }
    (*out)[key] = value;
  }
  if (!r.ok()) {
    *error = r.error();
    return false;
  }
  return true;
}

bool FlexGetInt(const FlexMap& map, const std::string& key, int64_t* out) {
  const auto it = map.find(key);
  if (it == map.end() || it->second.kind != FlexValue::Kind::kInt) return false;
  *out = it->second.int_value;
  return true;
}

bool FlexGetFloat(const FlexMap& map, const std::string& key, double* out) {
  const auto it = map.find(key);
  if (it == map.end()) return false;
  if (it->second.kind == FlexValue::Kind::kFloat || it->second.kind == FlexValue::Kind::kInt) {
    *out = it->second.float_value;
    return true;
  }
  return false;
}

bool FlexGetIntVector(const FlexMap& map, const std::string& key, std::vector<int64_t>* out) {
  const auto it = map.find(key);
  if (it == map.end() || it->second.kind != FlexValue::Kind::kNumberVector ||
      !it->second.integer_vector) {
    return false;
  }
  *out = it->second.ints;
  return true;
}

}  // namespace ml
}  // namespace slam_refact
