// FaceController event emission (US-018/US-019).
//
// Port of the event logic of XR8.FaceController, PUB:reality/app/xr/js/src/face-controller.ts
// (the shipped xr-face.js has the same code, minified): onProcessCpu (:1018-1060), mapFaces
// (:245-293), maybeEmitFaceStateUpdates (:403-491), the wink/blink timers (:374-402),
// removeExtraEarData (:493-497) and the faceloading/facescanning dispatch of onAttach (:862-864).
//
// The framework queues events (camera-loop.ts frameworkForModule: events.push) and delivers them
// after the module callbacks, so a facefound/faceupdated detail is seen after removeExtraEarData
// deleted the six ear point keys; the details here are written in that delivered form. Event names
// carry the module prefix ("facecontroller.facefound"). setTimeout is replaced by an explicit clock:
// timers due at or before `now_ms` fire (in due order, ties in creation order) before the events of
// the frame processed at `now_ms`, as browser timers fire between frames.
//
// JavaScript semantics kept: a stored expression flag starts undefined, so the first frame of a face
// emits one opened/closed (raised/lowered, earfound/earlost, earpointfound/earpointlost) event per
// flag; the stored IPD starts at 0 and only a change is emitted; the per-face state is cleared only
// on detach, not on facelost.
#ifndef SLAM_REFACT_FACE_FACE_EVENTS_H_
#define SLAM_REFACT_FACE_FACE_EVENTS_H_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "face/face_messages.h"

namespace slam_refact::face {

struct FaceEvent {
  std::string name;         // e.g. "facecontroller.mouthopened"
  std::string detail_json;  // JSON of the delivered detail object (key order as in JS)
};

// JSON of mapFaces(f) after removeExtraEarData (facefound / faceupdated detail).
std::string FaceDataJson(const FaceMsg& f);
// JSON of modelGeometry_ (faceloading / facescanning detail).
std::string ModelGeometryJson(const ModelGeometryMsg& m);

class FaceEventEmitter {
 public:
  // onAttach: faceloading then facescanning with the model geometry.
  std::vector<FaceEvent> OnAttach(const ModelGeometryMsg& model_geometry);
  // Fires the timers due at or before now_ms.
  std::vector<FaceEvent> AdvanceTo(double now_ms);
  // AdvanceTo(now_ms), then the events of onProcessCpu for `response` (nullptr: the JS returns
  // early, no event).
  std::vector<FaceEvent> OnProcessCpu(const FaceResponse* response, double now_ms);
  // onDetach: faceIdToFaceState_ = {} (pending timers then find no state).
  void OnDetach();

  size_t pending_timers() const { return timers_.size(); }

 private:
  struct FaceState {
    std::optional<bool> mouth_open;
    std::optional<bool> left_eye_open;
    std::optional<bool> right_eye_open;
    std::optional<bool> left_eyebrow_raised;
    std::optional<bool> right_eyebrow_raised;
    std::optional<bool> left_eye_winked;
    std::optional<bool> right_eye_winked;
    float interpupillary_distance_in_mm = 0.0f;
    std::optional<bool> left_ear_found;
    std::optional<bool> right_ear_found;
    std::optional<bool> ear_points[6];  // EarPointStateKeys order
  };
  enum class TimerKind { kWinkExpire, kWinkOrBlinkEnded };
  struct Timer {
    double due_ms = 0.0;
    uint64_t seq = 0;
    TimerKind kind = TimerKind::kWinkExpire;
    bool left = false;  // winkKey: leftEyeWinked (true) or rightEyeWinked
    int32_t face_id = 0;
  };

  void EmitFaceStateUpdates(const FaceMsg& face, double now_ms, std::vector<FaceEvent>* out);
  void AddTimer(double due_ms, TimerKind kind, bool left, int32_t face_id);

  std::map<int32_t, FaceState> face_state_;
  std::vector<Timer> timers_;
  uint64_t next_seq_ = 0;
};

}  // namespace slam_refact::face

#endif  // SLAM_REFACT_FACE_FACE_EVENTS_H_
