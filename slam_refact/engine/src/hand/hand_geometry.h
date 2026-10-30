// Calculator math of the MediaPipe hand graph (US-021; docs/spec/10-hand.md §3). Plain functions on
// the types of hand_types.h; each cites the calculator it ports (PUB = assets/references/xr_reference/
// official/mediapipe-fork, mediapipe/calculators/...).
#ifndef SLAM_REFACT_HAND_HAND_GEOMETRY_H_
#define SLAM_REFACT_HAND_HAND_GEOMETRY_H_

#include <array>
#include <vector>

#include "hand/hand_types.h"

namespace slam_refact::hand {

// SsdAnchorsCalculatorOptions (tflite/ssd_anchors_calculator.proto), the fields the palm graph sets
// (graphs/hand_tracking/subgraphs/hand_detection_cpu.pbtxt).
struct SsdAnchorsOptions {
  int num_layers = 5;
  float min_scale = 0.1171875f;
  float max_scale = 0.75f;
  int input_size_height = 256;
  int input_size_width = 256;
  float anchor_offset_x = 0.5f;
  float anchor_offset_y = 0.5f;
  std::vector<int> strides = {8, 16, 32, 32, 32};
  std::vector<float> aspect_ratios = {1.0f};
  bool reduce_boxes_in_lowest_layer = false;
  float interpolated_scale_aspect_ratio = 1.0f;  // proto default
  bool fixed_anchor_size = true;
};

// SsdAnchorsCalculator::GenerateAnchors (ssd_anchors_calculator.cc:107-208).
std::vector<Anchor> GenerateAnchors(const SsdAnchorsOptions& options);

// TfLiteTensorsToDetectionsCalculatorOptions of the palm graph.
struct PalmDecodeOptions {
  int num_boxes = 2944;
  int num_coords = 18;
  int box_coord_offset = 0;
  int keypoint_coord_offset = 4;
  int num_keypoints = 7;
  int num_values_per_keypoint = 2;
  bool sigmoid_score = true;
  float score_clipping_thresh = 100.0f;
  bool reverse_output_order = true;
  float x_scale = 256.0f, y_scale = 256.0f, h_scale = 256.0f, w_scale = 256.0f;
  float min_score_thresh = 0.5f;
  int num_classes = 1;
};

// TfLiteTensorsToDetectionsCalculator::ProcessCPU for the two-tensor case: DecodeBoxes,
// sigmoid + clipping of the scores, ConvertToDetections (tflite_tensors_to_detections_calculator.cc
// :285-367, :589-722).
std::vector<Detection> DecodePalmDetections(const float* raw_boxes, const float* raw_scores,
                                            const std::vector<Anchor>& anchors,
                                            const PalmDecodeOptions& options);

// NonMaxSuppressionCalculator, algorithm WEIGHTED, overlap INTERSECTION_OVER_UNION
// (util/non_max_suppression_calculator.cc:171-370). Ties of the score sort use a stable order (the
// original's std::sort is unstable).
std::vector<Detection> WeightedNonMaxSuppression(const std::vector<Detection>& detections,
                                                 float min_suppression_threshold,
                                                 float min_score_threshold);

// DetectionLetterboxRemovalCalculator / LandmarkLetterboxRemovalCalculator; padding = {left, top,
// right, bottom}.
void RemoveLetterbox(std::vector<Detection>* detections, const std::array<float, 4>& padding);
void RemoveLetterbox(std::vector<Landmark>* landmarks, const std::array<float, 4>& padding);

// DetectionsToRectsCalculator::DetectionToNormalizedRect + ComputeRotation (util/
// detections_to_rects_calculator.cc:55-67, :256-271) with the rotation vector options.
NormRect DetectionToNormRect(const Detection& detection, int image_width, int image_height,
                             int start_keypoint, int end_keypoint, float target_angle_degrees);

// RectTransformationCalculator::TransformNormalizedRect (util/rect_transformation_calculator.cc:
// 218-259).
struct RectTransformOptions {
  float scale_x = 1.0f, scale_y = 1.0f, shift_x = 0.0f, shift_y = 0.0f;
  bool square_long = false;
};
NormRect TransformNormRect(NormRect rect, int image_width, int image_height,
                           const RectTransformOptions& options);

// LandmarksToDetectionCalculator (util/landmarks_to_detection_calculator.cc:35-62).
Detection LandmarksToDetection(const std::vector<Landmark>& landmarks);

// LandmarkProjectionCalculator (util/landmark_projection_calculator.cc:103-125), ignore_rotation
// false.
std::vector<Landmark> ProjectLandmarks(const std::vector<Landmark>& landmarks, const NormRect& rect);

// mediapipe NormalizeRadians (util/detections_to_rects_calculator.h): wraps to [-pi, pi).
float NormalizeRadians(float angle);

}  // namespace slam_refact::hand

#endif  // SLAM_REFACT_HAND_HAND_GEOMETRY_H_
