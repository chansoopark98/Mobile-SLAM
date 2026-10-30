// Face and ear model identity (US-018/US-019).
//
// The three deployed models of the reference runtime face chunk (assets/references/xr_reference/engine/resources,
// equal to analysis-runtime/standalone/resources; never copied into the repository). Their sha256
// equal the git-lfs oids of PUB:reality/app/xr/js/resources/face-{model,mesh-model,ear-model}.tflite,
// so the public JS source ships exactly these files; face-ear-model is
// PUB:reality/engine/ears/data/ear_model_v1.tflite (the current BUILD names v2, absent locally).
#ifndef SLAM_REFACT_FACE_FACE_MODELS_H_
#define SLAM_REFACT_FACE_FACE_MODELS_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace slam_refact::face {

inline constexpr char kFaceDetectModelSha256[] =
    "0d800dd37230bb0503e2bc84ce547a1328dc0e2d8c54a5a92df953710decc4ed";
inline constexpr char kFaceMeshModelSha256[] =
    "97cd5b1d16d1d205445134d0d8882a619068c1a49277652c468049d0fc7f2756";
inline constexpr char kFaceEarModelSha256[] =
    "4cd07c3ecc4ac3a19097ad5d8e2920113e8786d29302e48064b5e21b9f828552";

// Paths relative to the repository root.
inline constexpr char kFaceDetectModelPath[] =
    "assets/references/xr_reference/engine/resources/face-model.tflite";
inline constexpr char kFaceMeshModelPath[] =
    "assets/references/xr_reference/engine/resources/face-mesh-model.tflite";
inline constexpr char kFaceEarModelPath[] =
    "assets/references/xr_reference/engine/resources/face-ear-model.tflite";

// Lower-case hex SHA-256 (FIPS 180-4).
std::string Sha256Hex(const uint8_t* data, size_t size);
inline std::string Sha256Hex(const std::vector<uint8_t>& data) {
  return Sha256Hex(data.data(), data.size());
}

// Reads a whole file. Returns false (with `error`) if it cannot be opened or read.
bool ReadFileBytes(const std::string& path, std::vector<uint8_t>* out, std::string* error);

}  // namespace slam_refact::face

#endif  // SLAM_REFACT_FACE_FACE_MODELS_H_
