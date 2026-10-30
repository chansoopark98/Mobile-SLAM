// See hand_messages.h.
#include "hand/hand_messages.h"

namespace slam_refact::hand {

AttachmentName AttachmentForLandmark(int landmark_index) {
  if (landmark_index < 0 || landmark_index >= kHandLandmarkCount) {
    return AttachmentName::kUnspecified;
  }
  return static_cast<AttachmentName>(landmark_index + 1);
}

Landmark PalmCenter(const std::vector<Landmark>& landmarks) {
  Landmark c;
  if (landmarks.size() < static_cast<size_t>(kHandLandmarkCount)) {
    return c;
  }
  for (int i : {0, 5, 9, 13, 17}) {
    c.x += landmarks[i].x;
    c.y += landmarks[i].y;
    c.z += landmarks[i].z;
  }
  c.x /= 5.0f;
  c.y /= 5.0f;
  c.z /= 5.0f;
  return c;
}

std::optional<HandMsg> HandEventTracker::Update(const HandResult& result) {
  std::optional<HandMsg> out;
  if (result.present) {
    HandMsg m;
    m.status = present_ ? TrackingStatus::kUpdated : TrackingStatus::kFound;
    for (int i = 0; i < kHandLandmarkCount && i < static_cast<int>(result.landmarks.size()); ++i) {
      m.attachment_points.push_back({AttachmentForLandmark(i), result.landmarks[i]});
    }
    m.attachment_points.push_back({AttachmentName::kPalm, PalmCenter(result.landmarks)});
    m.hand_rect = result.hand_rect;
    m.presence_score = result.presence_score;
    out = std::move(m);
  } else if (present_) {
    HandMsg m;
    m.status = TrackingStatus::kLost;
    m.presence_score = result.presence_score;
    out = std::move(m);
  }
  present_ = result.present;
  return out;
}

}  // namespace slam_refact::hand
