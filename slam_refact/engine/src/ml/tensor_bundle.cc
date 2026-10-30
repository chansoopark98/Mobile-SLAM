#include "ml/tensor_bundle.h"

#include <cstring>
#include <fstream>
#include <iterator>

namespace slam_refact {
namespace ml {
namespace {

constexpr char kMagic[] = "SRMLT001";

template <typename T>
bool Take(const std::vector<uint8_t>& b, size_t* pos, T* out) {
  if (*pos > b.size() || sizeof(T) > b.size() - *pos) return false;
  std::memcpy(out, b.data() + *pos, sizeof(T));
  *pos += sizeof(T);
  return true;
}

template <typename T>
void Put(std::vector<uint8_t>* b, T v) {
  const size_t at = b->size();
  b->resize(at + sizeof(T));
  std::memcpy(b->data() + at, &v, sizeof(T));
}

}  // namespace

size_t BundleTensor::count() const {
  size_t n = 1;
  for (int32_t d : shape) n *= static_cast<size_t>(d);
  return n;
}

bool ReadTensorBundle(const std::string& path, std::vector<BundleTensor>* out,
                      std::string* error) {
  out->clear();
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  const std::vector<uint8_t> b((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
  if (b.size() < 12 || std::memcmp(b.data(), kMagic, 8) != 0) {
    *error = path + ": not a tensor bundle";
    return false;
  }
  size_t pos = 8;
  uint32_t count = 0;
  Take(b, &pos, &count);
  for (uint32_t k = 0; k < count; ++k) {
    BundleTensor t;
    uint32_t name_len = 0;
    if (!Take(b, &pos, &name_len) || name_len > b.size() - pos) {
      *error = path + ": truncated name";
      return false;
    }
    t.name.assign(reinterpret_cast<const char*>(b.data() + pos), name_len);
    pos += name_len;
    uint8_t type = 0;
    uint32_t rank = 0;
    if (!Take(b, &pos, &type) || !Take(b, &pos, &rank) || rank > 8) {
      *error = path + ": bad header of " + t.name;
      return false;
    }
    t.type = static_cast<TensorType>(type);
    size_t elems = 1;
    for (uint32_t r = 0; r < rank; ++r) {
      int32_t d = 0;
      if (!Take(b, &pos, &d) || d < 0) {
        *error = path + ": bad dims of " + t.name;
        return false;
      }
      t.shape.push_back(d);
      elems *= static_cast<size_t>(d);
    }
    const size_t esize = TensorTypeSize(t.type);
    if (!(t.type == TensorType::kFloat32 || t.type == TensorType::kFloat64 ||
          t.type == TensorType::kInt32 || t.type == TensorType::kInt64) ||
        elems > (b.size() - pos) / esize) {
      *error = path + ": bad type or truncated data of " + t.name;
      return false;
    }
    for (size_t i = 0; i < elems; ++i) {
      if (t.type == TensorType::kFloat32) {
        float v = 0;
        Take(b, &pos, &v);
        t.values.push_back(v);
      } else if (t.type == TensorType::kFloat64) {
        double v = 0;
        Take(b, &pos, &v);
        t.values.push_back(v);
      } else if (t.type == TensorType::kInt32) {
        int32_t v = 0;
        Take(b, &pos, &v);
        t.ints.push_back(v);
      } else {
        int64_t v = 0;
        Take(b, &pos, &v);
        t.ints.push_back(v);
      }
    }
    out->push_back(std::move(t));
  }
  if (pos != b.size()) {
    *error = path + ": trailing bytes";
    return false;
  }
  return true;
}

bool WriteTensorBundle(const std::string& path, const std::vector<BundleTensor>& tensors,
                       std::string* error) {
  std::vector<uint8_t> b(kMagic, kMagic + 8);
  Put<uint32_t>(&b, static_cast<uint32_t>(tensors.size()));
  for (const BundleTensor& t : tensors) {
    Put<uint32_t>(&b, static_cast<uint32_t>(t.name.size()));
    b.insert(b.end(), t.name.begin(), t.name.end());
    Put<uint8_t>(&b, static_cast<uint8_t>(t.type));
    Put<uint32_t>(&b, static_cast<uint32_t>(t.shape.size()));
    for (int32_t d : t.shape) Put<int32_t>(&b, d);
    const size_t n = t.count();
    for (size_t i = 0; i < n; ++i) {
      switch (t.type) {
        case TensorType::kFloat32:
          Put<float>(&b, static_cast<float>(t.values[i]));
          break;
        case TensorType::kFloat64:
          Put<double>(&b, t.values[i]);
          break;
        case TensorType::kInt32:
          Put<int32_t>(&b, static_cast<int32_t>(t.ints[i]));
          break;
        case TensorType::kInt64:
          Put<int64_t>(&b, t.ints[i]);
          break;
        default:
          *error = "unsupported bundle type for " + t.name;
          return false;
      }
    }
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    *error = "cannot write " + path;
    return false;
  }
  out.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
  return static_cast<bool>(out);
}

}  // namespace ml
}  // namespace slam_refact
