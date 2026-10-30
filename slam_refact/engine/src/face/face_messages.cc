// Face message helpers (US-018/US-019). See face_messages.h.

#include "face/face_messages.h"

namespace slam_refact::face {

const char* AttachmentJsName(AttachmentName name) {
  // face-controller.ts AttachmentPoints (:35-72), keyed through AttachmentProtoEnums.
  switch (name) {
    case AttachmentName::FOREHEAD:
      return "forehead";
    case AttachmentName::RIGHT_EYEBROW_INNER:
      return "rightEyebrowInner";
    case AttachmentName::RIGHT_EYEBROW_MIDDLE:
      return "rightEyebrowMiddle";
    case AttachmentName::RIGHT_EYEBROW_OUTER:
      return "rightEyebrowOuter";
    case AttachmentName::LEFT_EYEBROW_INNER:
      return "leftEyebrowInner";
    case AttachmentName::LEFT_EYEBROW_MIDDLE:
      return "leftEyebrowMiddle";
    case AttachmentName::LEFT_EYEBROW_OUTER:
      return "leftEyebrowOuter";
    case AttachmentName::LEFT_EAR:
      return "leftEar";
    case AttachmentName::RIGHT_EAR:
      return "rightEar";
    case AttachmentName::LEFT_CHEEK:
      return "leftCheek";
    case AttachmentName::RIGHT_CHEEK:
      return "rightCheek";
    case AttachmentName::NOSE_BRIDGE:
      return "noseBridge";
    case AttachmentName::NOSE_TIP:
      return "noseTip";
    case AttachmentName::LEFT_EYE:
      return "leftEye";
    case AttachmentName::RIGHT_EYE:
      return "rightEye";
    case AttachmentName::LEFT_EYE_OUTER_CORNER:
      return "leftEyeOuterCorner";
    case AttachmentName::RIGHT_EYE_OUTER_CORNER:
      return "rightEyeOuterCorner";
    case AttachmentName::UPPER_LIP:
      return "upperLip";
    case AttachmentName::LOWER_LIP:
      return "lowerLip";
    case AttachmentName::MOUTH:
      return "mouth";
    case AttachmentName::MOUTH_RIGHT_CORNER:
      return "mouthRightCorner";
    case AttachmentName::MOUTH_LEFT_CORNER:
      return "mouthLeftCorner";
    case AttachmentName::CHIN:
      return "chin";
    case AttachmentName::LEFT_IRIS:
      return "leftIris";
    case AttachmentName::RIGHT_IRIS:
      return "rightIris";
    case AttachmentName::LEFT_UPPER_EYELID:
      return "leftUpperEyelid";
    case AttachmentName::RIGHT_UPPER_EYELID:
      return "rightUpperEyelid";
    case AttachmentName::LEFT_LOWER_EYELID:
      return "leftLowerEyelid";
    case AttachmentName::RIGHT_LOWER_EYELID:
      return "rightLowerEyelid";
    case AttachmentName::EAR_LEFT_HELIX:
      return "leftHelix";
    case AttachmentName::EAR_LEFT_CANAL:
      return "leftCanal";
    case AttachmentName::EAR_LEFT_LOBE:
      return "leftLobe";
    case AttachmentName::EAR_RIGHT_HELIX:
      return "rightHelix";
    case AttachmentName::EAR_RIGHT_CANAL:
      return "rightCanal";
    case AttachmentName::EAR_RIGHT_LOBE:
      return "rightLobe";
    case AttachmentName::UNSPECIFIED:
      break;
  }
  return "";
}

}  // namespace slam_refact::face
