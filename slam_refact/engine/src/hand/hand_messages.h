// Output structure of the hand module in the terms of the original hand.capnp (US-021;
// docs/spec/10-hand.md §4): HandMsg / HandAttachmentPointMsg of PUB:reality/engine/api/hand.capnp.
// Only the fields the substitute pipeline can fill are filled; the rest are listed as unsupported in
// the spec's mapping table (no value is invented).
#ifndef SLAM_REFACT_HAND_HAND_MESSAGES_H_
#define SLAM_REFACT_HAND_HAND_MESSAGES_H_

#include <optional>
#include <vector>

#include "hand/hand_tracker.h"
#include "hand/hand_types.h"

namespace slam_refact::hand {

// HandAttachmentPointMsg::AttachmentName values (hand.capnp, ordinals).
enum class AttachmentName : int {
  kUnspecified = 0,
  kWrist = 1,
  kThumbBaseJoint = 2, kThumbMidJoint = 3, kThumbTopJoint = 4, kThumbTip = 5,
  kIndexBaseJoint = 6, kIndexMidJoint = 7, kIndexTopJoint = 8, kIndexTip = 9,
  kMiddleBaseJoint = 10, kMiddleMidJoint = 11, kMiddleTopJoint = 12, kMiddleTip = 13,
  kRingBaseJoint = 14, kRingMidJoint = 15, kRingTopJoint = 16, kRingTip = 17,
  kPinkyBaseJoint = 18, kPinkyMidJoint = 19, kPinkyTopJoint = 20, kPinkyTip = 21,
  kPalm = 22,
  // 23..40 (thumbUpper, thumbNail, index/middle/ring/pinky Lower/Upper/Nail, wristTop/Bottom/Inner/Outer)
  // need the hand mesh that the removed hand_mesh_v2 model produced: never filled here.
};

enum class TrackingStatus : int { kUnspecified = 0, kFound = 1, kUpdated = 2, kLost = 3 };

struct HandAttachmentPoint {
  AttachmentName name = AttachmentName::kUnspecified;
  // IMAGE-space position: x, y normalised by the frame size, z in the landmark model's relative-depth
  // units. NOT the metric camera-space position of the original (no translation estimation).
  Landmark position;
};

struct HandMsg {
  int id = 1;  // a single hand is tracked
  TrackingStatus status = TrackingStatus::kUnspecified;
  std::vector<HandAttachmentPoint> attachment_points;  // wrist..pinkyTip (landmark i -> name i + 1) and palm
  NormRect hand_rect;                                   // ROI of the landmark model (extra, not in hand.capnp)
  float presence_score = 0.0f;                          // handflag (extra)
  // Fields of hand.capnp HandMsg this pipeline cannot provide (documented, left at defaults):
  // transform (3D position, rotation, scale), vertices, normals, handKind, wristTransform.
};

// Turns per-frame HandResults into found / updated / lost messages (hand.capnp TrackingStatus).
class HandEventTracker {
 public:
  // Returns a message when there is one: kFound on the frame the presence rises, kUpdated while it is
  // present, kLost once when it falls.
  std::optional<HandMsg> Update(const HandResult& result);
  void Reset() { present_ = false; }

 private:
  bool present_ = false;
};

// landmark index (0..20) -> attachment name (hand.capnp: "the first 21 points are aligned with hand
// landmarks"), and the palm point (mean of landmarks 0, 5, 9, 13, 17: the wrist and the four finger
// MCPs; a design choice of this substitute, INFERRED, not the original definition).
AttachmentName AttachmentForLandmark(int landmark_index);
Landmark PalmCenter(const std::vector<Landmark>& landmarks);

}  // namespace slam_refact::hand

#endif  // SLAM_REFACT_HAND_HAND_MESSAGES_H_
