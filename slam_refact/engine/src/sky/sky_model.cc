// See sky_model.h.
#include "sky/sky_model.h"

#include <array>
#include <cstdio>
#include <cstring>

#include "c8/pixels/pixels.h"
#include "c8/vector.h"
#include "reality/engine/semantics/semantics-classifier.h"

namespace slam_refact::sky {
namespace {

constexpr std::array<uint32_t, 64> kK = {
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

uint32_t Rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

void Compress(const uint8_t* block, std::array<uint32_t, 8>* state) {
  std::array<uint32_t, 64> w{};
  for (int i = 0; i < 16; ++i) {
    w[i] = (static_cast<uint32_t>(block[4 * i]) << 24) |
           (static_cast<uint32_t>(block[4 * i + 1]) << 16) |
           (static_cast<uint32_t>(block[4 * i + 2]) << 8) | static_cast<uint32_t>(block[4 * i + 3]);
  }
  for (int i = 16; i < 64; ++i) {
    const uint32_t s0 = Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const uint32_t s1 = Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  std::array<uint32_t, 8> v = *state;
  for (int i = 0; i < 64; ++i) {
    const uint32_t s1 = Rotr(v[4], 6) ^ Rotr(v[4], 11) ^ Rotr(v[4], 25);
    const uint32_t ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
    const uint32_t t1 = v[7] + s1 + ch + kK[i] + w[i];
    const uint32_t s0 = Rotr(v[0], 2) ^ Rotr(v[0], 13) ^ Rotr(v[0], 22);
    const uint32_t maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
    const uint32_t t2 = s0 + maj;
    v[7] = v[6];
    v[6] = v[5];
    v[5] = v[4];
    v[4] = v[3] + t1;
    v[3] = v[2];
    v[2] = v[1];
    v[1] = v[0];
    v[0] = t1 + t2;
  }
  for (int i = 0; i < 8; ++i) {
    (*state)[i] += v[i];
  }
}

}  // namespace

std::string Sha256Hex(const uint8_t* data, size_t size) {
  std::array<uint32_t, 8> state = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  const size_t full = size / 64;
  for (size_t b = 0; b < full; ++b) {
    Compress(data + 64 * b, &state);
  }
  std::array<uint8_t, 128> tail{};
  const size_t rest = size - 64 * full;
  for (size_t i = 0; i < rest; ++i) {
    tail[i] = data[64 * full + i];
  }
  tail[rest] = 0x80;
  const size_t tail_len = rest + 9 <= 64 ? 64 : 128;
  const uint64_t bits = static_cast<uint64_t>(size) * 8u;
  for (int i = 0; i < 8; ++i) {
    tail[tail_len - 1 - i] = static_cast<uint8_t>(bits >> (8 * i));
  }
  for (size_t b = 0; b < tail_len; b += 64) {
    Compress(tail.data() + b, &state);
  }
  static const char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(64);
  for (uint32_t word : state) {
    for (int i = 28; i >= 0; i -= 4) {
      out.push_back(kHex[(word >> i) & 0xfu]);
    }
  }
  return out;
}

bool ReadFileBytes(const std::string& path, std::vector<uint8_t>* out, std::string* error) {
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (f == nullptr) {
    *error = "cannot open " + path;
    return false;
  }
  out->clear();
  std::array<uint8_t, 1 << 16> buf{};
  size_t n = 0;
  while ((n = std::fread(buf.data(), 1, buf.size(), f)) > 0) {
    out->insert(out->end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(n));
  }
  const bool failed = std::ferror(f) != 0;
  std::fclose(f);
  if (failed) {
    *error = "read error " + path;
    return false;
  }
  return true;
}

struct SkyModel::Impl {
  // The classifier keeps no pointer into the model bytes after construction (the interpreter
  // shim copies them).
  std::unique_ptr<c8::SemanticsClassifier> classifier;
};

SkyModel::SkyModel() = default;
SkyModel::~SkyModel() = default;

bool SkyModel::Load(const std::vector<uint8_t>& model, bool check_sha, std::string* error) {
  if (check_sha) {
    const std::string sha = Sha256Hex(model);
    if (sha != kSemanticsModelSha256) {
      *error = "semantics model sha256 " + sha + " != " + kSemanticsModelSha256;
      return false;
    }
  }
  auto impl = std::make_unique<Impl>();
  // SemanticsClassifier(modelData, modelSize) aborts (C8_THROW = abort with C8_NO_EXCEPT) on a model
  // the interpreter rejects; sha256 checked bytes are the deployed model.
  impl->classifier =
      std::make_unique<c8::SemanticsClassifier>(model.data(), static_cast<int>(model.size()));
  impl_ = std::move(impl);
  return true;
}

bool SkyModel::Invoke(const uint8_t* rgba, SkyInference* out) {
  if (impl_ == nullptr) {
    return false;
  }
  // semantics-worker.cc processStagedSemanticsWorkerFrame(bytes, rows = 256, cols = 144).
  c8::ConstRGBA8888PlanePixels input(kModelInputHeight, kModelInputWidth, kModelInputWidth * 4,
                                     rgba);
  const c8::ConstOneChannelFloatPixels sky = impl_->classifier->obtainSemantics(input);
  const size_t count = static_cast<size_t>(kModelInputWidth) * kModelInputHeight;
  out->data.assign(sky.pixels(), sky.pixels() + count);
  // semantics-worker.js invoke(): percentage = validCount / length, a score counts when > 0.4.
  size_t valid = 0;
  for (size_t i = 0; i < count; ++i) {
    if (static_cast<double>(out->data[i]) > kModelThreshold) {
      ++valid;
    }
  }
  out->percentage = static_cast<double>(valid) / static_cast<double>(count);
  return true;
}

}  // namespace slam_refact::sky
