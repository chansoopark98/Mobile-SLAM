// See hand_tracker.h.
#include "hand/hand_tracker.h"

#include <chrono>
#include <cstring>

#include "hand/hand_image.h"
#include "hand/hand_models.h"
#include "ml/interpreter.h"
#include "ml/tflite_model.h"

namespace slam_refact::hand {
namespace {

using Clock = std::chrono::steady_clock;
double Ms(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

}  // namespace

struct HandTracker::Impl {
  std::unique_ptr<ml::TfliteModel> palm_model, landmark_model;
  std::unique_ptr<ml::Interpreter> palm, landmark;
  LandmarkModel kind = LandmarkModel::k3d;
  int palm_box_output = -1, palm_score_output = -1, landmark_output = -1, flag_output = -1;
};

HandTracker::HandTracker() = default;
HandTracker::~HandTracker() = default;

bool HandTracker::loaded() const { return impl_ != nullptr; }

bool HandTracker::Load(const std::vector<uint8_t>& palm, const std::vector<uint8_t>& landmark,
                       LandmarkModel kind, bool check_sha, std::string* error) {
  if (check_sha) {
    const std::string palm_sha = Sha256Hex(palm), landmark_sha = Sha256Hex(landmark);
    const char* want_landmark = kind == LandmarkModel::k3d ? kHandLandmark3dSha256 : kHandLandmarkSha256;
    if (palm_sha != kPalmDetectionSha256) {
      *error = "palm_detection sha256 " + palm_sha + " != " + kPalmDetectionSha256;
      return false;
    }
    if (landmark_sha != want_landmark) {
      *error = "hand landmark model sha256 " + landmark_sha + " != " + want_landmark;
      return false;
    }
  }
  auto impl = std::make_unique<Impl>();
  impl->kind = kind;
  impl->palm_model = ml::TfliteModel::FromBytes(palm, error);
  if (impl->palm_model == nullptr) return false;
  impl->palm = ml::Interpreter::Create(*impl->palm_model, error);
  if (impl->palm == nullptr) return false;
  impl->landmark_model = ml::TfliteModel::FromBytes(landmark, error);
  if (impl->landmark_model == nullptr) return false;
  impl->landmark = ml::Interpreter::Create(*impl->landmark_model, error);
  if (impl->landmark == nullptr) return false;
  // Output roles by shape (the graph relies on tensor order: boxes first / landmarks first).
  const PalmDecodeOptions po;
  for (size_t i = 0; i < impl->palm->num_outputs(); ++i) {
    const auto& shape = impl->palm->output(i).shape;
    if (shape.size() == 3 && shape[1] == po.num_boxes && shape[2] == po.num_coords) impl->palm_box_output = static_cast<int>(i);
    if (shape.size() == 3 && shape[1] == po.num_boxes && shape[2] == 1) impl->palm_score_output = static_cast<int>(i);
  }
  const int landmark_values = kind == LandmarkModel::k3d ? 63 : 42;
  for (size_t i = 0; i < impl->landmark->num_outputs(); ++i) {
    const auto v = impl->landmark->output(i);
    if (v.shape.size() == 2 && v.shape[1] == landmark_values) impl->landmark_output = static_cast<int>(i);
    if (v.shape.size() == 2 && v.shape[1] == 1) impl->flag_output = static_cast<int>(i);
  }
  if (impl->palm_box_output < 0 || impl->palm_score_output < 0 || impl->landmark_output < 0 ||
      impl->flag_output < 0) {
    *error = "hand models: unexpected output tensor shapes";
    return false;
  }
  if (impl->palm->input_count(0) != static_cast<size_t>(kModelInputSize) * kModelInputSize * 3 ||
      impl->landmark->input_count(0) != static_cast<size_t>(kModelInputSize) * kModelInputSize * 3) {
    *error = "hand models: unexpected input size";
    return false;
  }
  impl_ = std::move(impl);
  anchors_ = GenerateAnchors(SsdAnchorsOptions());
  Reset();
  return true;
}

void HandTracker::Reset() {
  prev_presence_ = false;
  has_prev_rect_ = false;
  prev_rect_ = NormRect();
}

bool HandTracker::RunPalmDetection(const Image3& frame, std::vector<Detection>* detections,
                                   NormRect* rect_from_palm, std::string* error) {
  if (impl_ == nullptr) {
    *error = "models not loaded";
    return false;
  }
  // ImageTransformation FIT 256x256 -> TfLiteConverter (zero_center) -> inference.
  const auto t_a = Clock::now();
  std::array<float, 4> padding{};
  const Image3 fitted = FitToSize(frame, kModelInputSize, kModelInputSize, &padding);
  ImageToTensor(fitted, /*zero_center=*/true, impl_->palm->input_data(0));
  const auto t_b = Clock::now();
  if (!impl_->palm->Invoke(error)) return false;
  const auto t_c = Clock::now();
  const auto boxes = impl_->palm->output(static_cast<size_t>(impl_->palm_box_output));
  const auto scores = impl_->palm->output(static_cast<size_t>(impl_->palm_score_output));
  last_palm_boxes_.assign(boxes.f32, boxes.f32 + boxes.count);
  last_palm_scores_.assign(scores.f32, scores.f32 + scores.count);
  const PalmDecodeOptions po;
  std::vector<Detection> decoded = DecodePalmDetections(last_palm_boxes_.data(), last_palm_scores_.data(), anchors_, po);
  std::vector<Detection> filtered = WeightedNonMaxSuppression(decoded, 0.3f, 0.5f);
  RemoveLetterbox(&filtered, padding);
  *detections = filtered;
  // DetectionsToRects (rotation keypoints 0 -> 2, 90 degrees, zero rect for empty) + RectTransformation.
  NormRect palm_rect;
  if (!filtered.empty()) {
    palm_rect = DetectionToNormRect(filtered[0], frame.width, frame.height, 0, 2, 90.0f);
  }
  RectTransformOptions ro;
  ro.scale_x = 2.6f;
  ro.scale_y = 2.6f;
  ro.shift_y = -0.5f;
  ro.square_long = true;
  *rect_from_palm = TransformNormRect(palm_rect, frame.width, frame.height, ro);
  const auto t_d = Clock::now();
  last_times_.palm_preprocess = Ms(t_a, t_b);
  last_times_.palm_inference = Ms(t_b, t_c);
  last_times_.palm_postprocess = Ms(t_c, t_d);
  return true;
}

bool HandTracker::RunLandmarks(const Image3& frame, const NormRect& rect, std::vector<Landmark>* landmarks,
                               float* presence_score, NormRect* rect_for_next_frame, std::string* error) {
  if (impl_ == nullptr) {
    *error = "models not loaded";
    return false;
  }
  const auto t_a = Clock::now();
  const Image3 cropped = CropNormRect(frame, rect);
  std::array<float, 4> padding{};
  const Image3 fitted = FitToSize(cropped, kModelInputSize, kModelInputSize, &padding);
  ImageToTensor(fitted, /*zero_center=*/false, impl_->landmark->input_data(0));
  const auto t_b = Clock::now();
  if (!impl_->landmark->Invoke(error)) return false;
  const auto t_c = Clock::now();
  const auto lm = impl_->landmark->output(static_cast<size_t>(impl_->landmark_output));
  const auto flag = impl_->landmark->output(static_cast<size_t>(impl_->flag_output));
  *presence_score = flag.f32[0];
  // TfLiteTensorsToLandmarks (NORM_LANDMARKS: x / 256, y / 256, z / normalize_z = 1).
  const int dims = impl_->kind == LandmarkModel::k3d ? 3 : 2;
  std::vector<Landmark> out(kHandLandmarkCount);
  for (int i = 0; i < kHandLandmarkCount; ++i) {
    const float* p = lm.f32 + i * dims;
    out[i].x = static_cast<float>(p[0]) / static_cast<float>(kModelInputSize);
    out[i].y = static_cast<float>(p[1]) / static_cast<float>(kModelInputSize);
    out[i].z = dims > 2 ? p[2] / 1.0f : 0.0f;
  }
  RemoveLetterbox(&out, padding);
  *landmarks = ProjectLandmarks(out, rect);
  // LandmarksToDetection -> DetectionsToRects (keypoints 0 -> 9, 90 degrees) -> RectTransformation
  // (scale 1.6, square_long).
  const Detection det = LandmarksToDetection(*landmarks);
  const NormRect from_landmarks = DetectionToNormRect(det, frame.width, frame.height, 0, 9, 90.0f);
  RectTransformOptions ro;
  ro.scale_x = 1.6f;
  ro.scale_y = 1.6f;
  ro.square_long = true;
  *rect_for_next_frame = TransformNormRect(from_landmarks, frame.width, frame.height, ro);
  const auto t_d = Clock::now();
  last_times_.landmark_preprocess = Ms(t_a, t_b);
  last_times_.landmark_inference = Ms(t_b, t_c);
  last_times_.landmark_postprocess = Ms(t_c, t_d);
  return true;
}

bool HandTracker::Process(const Image3& frame, HandResult* r, std::string* error) {
  *r = HandResult();
  r->palm_ran = !prev_presence_;  // Gate DISALLOW:prev_hand_presence, empty packets allow
  last_times_ = HandStageTimes();
  NormRect hand_rect;
  if (r->palm_ran) {
    if (!RunPalmDetection(frame, &r->palm_detections, &r->rect_from_palm, error)) return false;
    hand_rect = r->rect_from_palm;  // Merge: the palm rect is the first input
  } else {
    hand_rect = prev_rect_;  // Merge: the previous frame's landmark rect
  }
  r->hand_rect = hand_rect;
  if (!RunLandmarks(frame, hand_rect, &r->landmarks, &r->presence_score, &r->rect_for_next_frame, error)) {
    return false;
  }
  r->times = last_times_;
  r->present = static_cast<double>(r->presence_score) > static_cast<double>(kHandPresenceThreshold);
  prev_presence_ = r->present;
  prev_rect_ = r->rect_for_next_frame;
  has_prev_rect_ = true;
  return true;
}

}  // namespace slam_refact::hand
