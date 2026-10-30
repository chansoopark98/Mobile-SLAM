// File and hash helpers of the hand module (US-021). Same SHA-256 as src/face and src/sky carry (each
// lane keeps its own copy so none depends on the other's heavy closure).
#ifndef SLAM_REFACT_HAND_HAND_MODELS_H_
#define SLAM_REFACT_HAND_HAND_MODELS_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace slam_refact::hand {

std::string Sha256Hex(const uint8_t* data, size_t size);
inline std::string Sha256Hex(const std::vector<uint8_t>& d) { return Sha256Hex(d.data(), d.size()); }
bool ReadFileBytes(const std::string& path, std::vector<uint8_t>* out, std::string* error);

}  // namespace slam_refact::hand

#endif  // SLAM_REFACT_HAND_HAND_MODELS_H_
