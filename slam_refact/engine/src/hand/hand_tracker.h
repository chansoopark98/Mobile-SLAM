// Single-hand tracking graph of MediaPipe (graphs/hand_tracking/hand_tracking_desktop.pbtxt with the
// HandDetectionSubgraph and HandLandmarkSubgraph of subgraphs/*_cpu.pbtxt) on the slam_refact ML
// engine (US-021; docs/spec/10-hand.md). CONFIGURED ceiling, see hand_types.h.
//
// One Process() call = one frame of the graph: the palm detector runs when the previous frame had no
// hand (PreviousLoopback + Gate), the hand rect is the palm rect or the previous frame's landmark rect
// (Merge), the landmark model runs on that rect every frame, presence is handflag > 0.1.
#ifndef SLAM_REFACT_HAND_HAND_TRACKER_H_
#define SLAM_REFACT_HAND_HAND_TRACKER_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hand/hand_geometry.h"
#include "hand/hand_types.h"

namespace slam_refact::hand {

// Models: public MediaPipe 2020 files under assets/references/xr_reference/official/mediapipe-fork/
// mediapipe/models (never copied into the repository). sha256 pinned.
inline constexpr char kPalmDetectionSha256[] =
    "ab2828eeeb871a1229699ac59f870f5d915b5de8582c5dd17330b83068c458ff";
inline constexpr char kHandLandmarkSha256[] =
    "06d03d84801cabd033ea56a366ff3dcd8cd40812a9e37c90df8dc1435d704018";
inline constexpr char kHandLandmark3dSha256[] =
    "12f500618bccaf7b208b1375e7fc60c66209fe570b5897defaeb5e3c592f33fa";
inline constexpr char kPalmDetectionPath[] =
    "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/palm_detection.tflite";
inline constexpr char kHandLandmarkPath[] =
    "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/hand_landmark.tflite";
inline constexpr char kHandLandmark3dPath[] =
    "assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/hand_landmark_3d.tflite";

inline constexpr int kHandLandmarkCount = 21;
inline constexpr int kModelInputSize = 256;
inline constexpr float kHandPresenceThreshold = 0.1f;  // ThresholdingCalculator in hand_landmark_cpu

// Which landmark model runs: hand_landmark.tflite (42 values, x y) or hand_landmark_3d.tflite (63,
// x y z).
enum class LandmarkModel { k2d, k3d };

struct HandStageTimes {  // milliseconds, 0 for a stage that did not run
  double palm_preprocess = 0, palm_inference = 0, palm_postprocess = 0;
  double landmark_preprocess = 0, landmark_inference = 0, landmark_postprocess = 0;
};

struct HandResult {
  bool palm_ran = false;
  std::vector<Detection> palm_detections;  // after NMS and letterbox removal
  NormRect rect_from_palm;                 // zero rect when nothing was detected
  NormRect hand_rect;                      // the rect the landmark model ran on
  float presence_score = 0.0f;             // handflag as the model outputs it
  bool present = false;                    // presence_score > 0.1
  std::vector<Landmark> landmarks;         // 21, normalised image coordinates (always filled)
  NormRect rect_for_next_frame;
  HandStageTimes times;
};

class HandTracker {
 public:
  HandTracker();
  ~HandTracker();
  HandTracker(const HandTracker&) = delete;
  HandTracker& operator=(const HandTracker&) = delete;

  // Loads the palm model and one landmark model (bytes of the files above; sha256 checked when
  // `check_sha`).
  bool Load(const std::vector<uint8_t>& palm, const std::vector<uint8_t>& landmark,
            LandmarkModel landmark_kind, bool check_sha, std::string* error);
  bool loaded() const;

  // One frame. `frame` is the camera image (the graph's input_video).
  bool Process(const Image3& frame, HandResult* result, std::string* error);
  // Forget the previous frame (presence false, no previous rect).
  void Reset();

  // The palm stage alone (no graph state): used by tests and benchmarks.
  bool RunPalmDetection(const Image3& frame, std::vector<Detection>* detections,
                        NormRect* rect_from_palm, std::string* error);
  // The landmark stage alone on a given rect.
  bool RunLandmarks(const Image3& frame, const NormRect& rect, std::vector<Landmark>* landmarks,
                    float* presence_score, NormRect* rect_for_next_frame, std::string* error);

  // Raw model outputs of the last palm / landmark run (tests compare the post-processing on them).
  const std::vector<float>& last_palm_boxes() const { return last_palm_boxes_; }
  const std::vector<float>& last_palm_scores() const { return last_palm_scores_; }
  const std::vector<Anchor>& anchors() const { return anchors_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<Anchor> anchors_;
  std::vector<float> last_palm_boxes_, last_palm_scores_;
  HandStageTimes last_times_;
  bool prev_presence_ = false;
  bool has_prev_rect_ = false;
  NormRect prev_rect_;
};

}  // namespace slam_refact::hand

#endif  // SLAM_REFACT_HAND_HAND_TRACKER_H_
