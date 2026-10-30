// Cap'n Proto free shim of the generated reality/engine/api/face.capnp.h (US-018).
//
// The vendored face and ear sources use two generated types:
//   - AttachmentPointMsg::AttachmentName (c8/geometry/face-types.h, faces/face-geometry.cc,
//     ears/ear-types.cc). Enumerators and values follow PUB:reality/engine/api/face.capnp:97-134
//     (capnp C++ names: UPPER_SNAKE_CASE of the schema names, enum class over uint16_t).
//   - DebugOptions::Reader (faces/face-tracker.{h,cc}, imported by face.capnp from
//     mesh.capnp:21-25). Only getOnlyDetectFaces() is read by the vendored code.
// Messages (FaceResponse, FaceOptions, ...) are replaced by plain structs in src/face
// (docs/spec/08-face-ear.md); Cap'n Proto serialization is out of scope (architecture.md §1.2).
#pragma once

#include <cstdint>

namespace c8 {

struct AttachmentPointMsg {
  enum class AttachmentName : uint16_t {
    UNSPECIFIED = 0,
    FOREHEAD = 1,
    RIGHT_EYEBROW_INNER = 2,
    RIGHT_EYEBROW_MIDDLE = 3,
    RIGHT_EYEBROW_OUTER = 4,
    LEFT_EYEBROW_INNER = 5,
    LEFT_EYEBROW_MIDDLE = 6,
    LEFT_EYEBROW_OUTER = 7,
    LEFT_EAR = 8,
    RIGHT_EAR = 9,
    LEFT_CHEEK = 10,
    RIGHT_CHEEK = 11,
    NOSE_BRIDGE = 12,
    NOSE_TIP = 13,
    LEFT_EYE = 14,
    RIGHT_EYE = 15,
    LEFT_EYE_OUTER_CORNER = 16,
    RIGHT_EYE_OUTER_CORNER = 17,
    UPPER_LIP = 18,
    LOWER_LIP = 19,
    MOUTH = 20,
    MOUTH_RIGHT_CORNER = 21,
    MOUTH_LEFT_CORNER = 22,
    CHIN = 23,
    LEFT_IRIS = 24,
    RIGHT_IRIS = 25,
    LEFT_UPPER_EYELID = 26,
    RIGHT_UPPER_EYELID = 27,
    LEFT_LOWER_EYELID = 28,
    RIGHT_LOWER_EYELID = 29,
    EAR_LEFT_HELIX = 30,
    EAR_LEFT_CANAL = 31,
    EAR_LEFT_LOBE = 32,
    EAR_RIGHT_HELIX = 33,
    EAR_RIGHT_CANAL = 34,
    EAR_RIGHT_LOBE = 35,
  };
};

// mesh.capnp:21-25 DebugOptions {extraOutput @0, onlyDetectFaces @1}. The Reader holds the value
// (a default-constructed Reader reads the schema default false, like an empty capnp message).
struct DebugOptions {
  class Reader {
  public:
    Reader() = default;
    explicit Reader(bool onlyDetectFaces) : onlyDetectFaces_(onlyDetectFaces) {}
    bool getOnlyDetectFaces() const { return onlyDetectFaces_; }

  private:
    bool onlyDetectFaces_ = false;
  };
};

}  // namespace c8
