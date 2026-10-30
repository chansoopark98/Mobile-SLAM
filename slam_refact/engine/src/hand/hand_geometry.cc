// See hand_geometry.h.
#include "hand/hand_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace slam_refact::hand {
namespace {

constexpr double kPi = 3.14159265358979323846;

float CalculateScale(float min_scale, float max_scale, int stride_index, int num_strides) {
  return min_scale + (max_scale - min_scale) * 1.0f * stride_index / (num_strides - 1.0f);
}

struct Box {
  float xmin, ymin, xmax, ymax;
  float Area() const { return (xmax - xmin) * (ymax - ymin); }
};

// OverlapSimilarity(INTERSECTION_OVER_UNION, rect1, rect2) with mediapipe's Rectangle_f semantics.
float IntersectionOverUnion(const Box& r1, const Box& r2) {
  // Rectangle::Intersects: strict overlap.
  if (!(r1.xmin < r2.xmax && r2.xmin < r1.xmax && r1.ymin < r2.ymax && r2.ymin < r1.ymax)) {
    return 0.0f;
  }
  const float ix = std::min(r1.xmax, r2.xmax) - std::max(r1.xmin, r2.xmin);
  const float iy = std::min(r1.ymax, r2.ymax) - std::max(r1.ymin, r2.ymin);
  const float intersection_area = ix * iy;
  const float normalization = r1.Area() + r2.Area() - intersection_area;
  return normalization > 0.0f ? intersection_area / normalization : 0.0f;
}

Box BoxOf(const Detection& d) { return {d.xmin, d.ymin, d.xmin + d.width, d.ymin + d.height}; }

}  // namespace

float NormalizeRadians(float angle) {
  return static_cast<float>(angle - 2 * kPi * std::floor((angle - (-kPi)) / (2 * kPi)));
}

std::vector<Anchor> GenerateAnchors(const SsdAnchorsOptions& options) {
  std::vector<Anchor> anchors;
  int layer_id = 0;
  const int num_strides = static_cast<int>(options.strides.size());
  while (layer_id < num_strides) {
    std::vector<float> anchor_height, anchor_width, aspect_ratios, scales;
    int last_same_stride_layer = layer_id;
    while (last_same_stride_layer < num_strides &&
           options.strides[last_same_stride_layer] == options.strides[layer_id]) {
      const float scale =
          CalculateScale(options.min_scale, options.max_scale, last_same_stride_layer, num_strides);
      if (last_same_stride_layer == 0 && options.reduce_boxes_in_lowest_layer) {
        aspect_ratios.push_back(1.0f);
        aspect_ratios.push_back(2.0f);
        aspect_ratios.push_back(0.5f);
        scales.push_back(0.1f);
        scales.push_back(scale);
        scales.push_back(scale);
      } else {
        for (size_t a = 0; a < options.aspect_ratios.size(); ++a) {
          aspect_ratios.push_back(options.aspect_ratios[a]);
          scales.push_back(scale);
        }
        if (options.interpolated_scale_aspect_ratio > 0.0f) {
          const float scale_next =
              last_same_stride_layer == num_strides - 1
                  ? 1.0f
                  : CalculateScale(options.min_scale, options.max_scale, last_same_stride_layer + 1,
                                   num_strides);
          scales.push_back(std::sqrt(scale * scale_next));
          aspect_ratios.push_back(options.interpolated_scale_aspect_ratio);
        }
      }
      ++last_same_stride_layer;
    }
    for (size_t i = 0; i < aspect_ratios.size(); ++i) {
      const float ratio_sqrts = std::sqrt(aspect_ratios[i]);
      anchor_height.push_back(scales[i] / ratio_sqrts);
      anchor_width.push_back(scales[i] * ratio_sqrts);
    }
    const int stride = options.strides[layer_id];
    const int feature_map_height = static_cast<int>(std::ceil(1.0f * options.input_size_height / stride));
    const int feature_map_width = static_cast<int>(std::ceil(1.0f * options.input_size_width / stride));
    for (int y = 0; y < feature_map_height; ++y) {
      for (int x = 0; x < feature_map_width; ++x) {
        for (size_t anchor_id = 0; anchor_id < anchor_height.size(); ++anchor_id) {
          Anchor a;
          a.x_center = (x + options.anchor_offset_x) * 1.0f / feature_map_width;
          a.y_center = (y + options.anchor_offset_y) * 1.0f / feature_map_height;
          if (options.fixed_anchor_size) {
            a.w = 1.0f;
            a.h = 1.0f;
          } else {
            a.w = anchor_width[anchor_id];
            a.h = anchor_height[anchor_id];
          }
          anchors.push_back(a);
        }
      }
    }
    layer_id = last_same_stride_layer;
  }
  return anchors;
}

std::vector<Detection> DecodePalmDetections(const float* raw_boxes, const float* raw_scores,
                                            const std::vector<Anchor>& anchors,
                                            const PalmDecodeOptions& o) {
  const int nb = o.num_boxes, nc = o.num_coords;
  // DecodeBoxes.
  std::vector<float> boxes(static_cast<size_t>(nb) * nc);
  for (int i = 0; i < nb; ++i) {
    const int box_offset = i * nc + o.box_coord_offset;
    float y_center = raw_boxes[box_offset];
    float x_center = raw_boxes[box_offset + 1];
    float h = raw_boxes[box_offset + 2];
    float w = raw_boxes[box_offset + 3];
    if (o.reverse_output_order) {
      x_center = raw_boxes[box_offset];
      y_center = raw_boxes[box_offset + 1];
      w = raw_boxes[box_offset + 2];
      h = raw_boxes[box_offset + 3];
    }
    x_center = x_center / o.x_scale * anchors[i].w + anchors[i].x_center;
    y_center = y_center / o.y_scale * anchors[i].h + anchors[i].y_center;
    h = h / o.h_scale * anchors[i].h;  // apply_exponential_on_box_size is false
    w = w / o.w_scale * anchors[i].w;
    const float ymin = y_center - h / 2.f;
    const float xmin = x_center - w / 2.f;
    const float ymax = y_center + h / 2.f;
    const float xmax = x_center + w / 2.f;
    boxes[i * nc + 0] = ymin;
    boxes[i * nc + 1] = xmin;
    boxes[i * nc + 2] = ymax;
    boxes[i * nc + 3] = xmax;
    for (int k = 0; k < o.num_keypoints; ++k) {
      const int offset = i * nc + o.keypoint_coord_offset + k * o.num_values_per_keypoint;
      float keypoint_y = raw_boxes[offset];
      float keypoint_x = raw_boxes[offset + 1];
      if (o.reverse_output_order) {
        keypoint_x = raw_boxes[offset];
        keypoint_y = raw_boxes[offset + 1];
      }
      boxes[offset] = keypoint_x / o.x_scale * anchors[i].w + anchors[i].x_center;
      boxes[offset + 1] = keypoint_y / o.y_scale * anchors[i].h + anchors[i].y_center;
    }
  }
  // Scores: top class per box (one class), clipping + sigmoid.
  std::vector<Detection> out;
  for (int i = 0; i < nb; ++i) {
    float max_score = -std::numeric_limits<float>::max();
    for (int s = 0; s < o.num_classes; ++s) {
      float score = raw_scores[i * o.num_classes + s];
      if (o.sigmoid_score) {
        score = score < -o.score_clipping_thresh ? -o.score_clipping_thresh : score;
        score = score > o.score_clipping_thresh ? o.score_clipping_thresh : score;
        score = 1.0f / (1.0f + std::exp(-score));
      }
      if (max_score < score) {
        max_score = score;
      }
    }
    if (max_score < o.min_score_thresh) {
      continue;
    }
    const int box_offset = i * nc;
    Detection d;
    d.score = max_score;
    d.xmin = boxes[box_offset + 1];
    d.ymin = boxes[box_offset + 0];
    d.width = boxes[box_offset + 3] - boxes[box_offset + 1];
    d.height = boxes[box_offset + 2] - boxes[box_offset + 0];
    for (int kp = 0; kp < o.num_keypoints * o.num_values_per_keypoint; kp += o.num_values_per_keypoint) {
      const int idx = box_offset + o.keypoint_coord_offset + kp;
      d.keypoints.push_back({boxes[idx + 0], boxes[idx + 1]});
    }
    out.push_back(std::move(d));
  }
  return out;
}

std::vector<Detection> WeightedNonMaxSuppression(const std::vector<Detection>& detections,
                                                 float min_suppression_threshold,
                                                 float min_score_threshold) {
  std::vector<std::pair<int, float>> remained;
  for (size_t i = 0; i < detections.size(); ++i) {
    remained.emplace_back(static_cast<int>(i), detections[i].score);
  }
  std::stable_sort(remained.begin(), remained.end(),
                   [](const auto& a, const auto& b) { return a.second > b.second; });
  std::vector<Detection> output;
  std::vector<std::pair<int, float>> rest, candidates;
  while (!remained.empty()) {
    const Detection& detection = detections[remained[0].first];
    if (min_score_threshold > 0 && detection.score < min_score_threshold) {
      break;
    }
    rest.clear();
    candidates.clear();
    const Box location = BoxOf(detection);
    for (const auto& indexed : remained) {
      const float similarity = IntersectionOverUnion(BoxOf(detections[indexed.first]), location);
      if (similarity > min_suppression_threshold) {
        candidates.push_back(indexed);
      } else {
        rest.push_back(indexed);
      }
    }
    Detection weighted = detection;
    if (!candidates.empty()) {
      const size_t num_keypoints = detection.keypoints.size();
      std::vector<float> keypoints(num_keypoints * 2, 0.0f);
      float w_xmin = 0, w_ymin = 0, w_xmax = 0, w_ymax = 0, total_score = 0;
      for (const auto& c : candidates) {
        total_score += c.second;
        const Detection& d = detections[c.first];
        w_xmin += d.xmin * c.second;
        w_ymin += d.ymin * c.second;
        w_xmax += (d.xmin + d.width) * c.second;
        w_ymax += (d.ymin + d.height) * c.second;
        for (size_t i = 0; i < num_keypoints; ++i) {
          keypoints[i * 2] += d.keypoints[i].x * c.second;
          keypoints[i * 2 + 1] += d.keypoints[i].y * c.second;
        }
      }
      weighted.xmin = w_xmin / total_score;
      weighted.ymin = w_ymin / total_score;
      weighted.width = (w_xmax / total_score) - weighted.xmin;
      weighted.height = (w_ymax / total_score) - weighted.ymin;
      for (size_t i = 0; i < num_keypoints; ++i) {
        weighted.keypoints[i].x = keypoints[i * 2] / total_score;
        weighted.keypoints[i].y = keypoints[i * 2 + 1] / total_score;
      }
    }
    remained = std::move(rest);
    rest = {};
    output.push_back(std::move(weighted));
  }
  return output;
}

void RemoveLetterbox(std::vector<Detection>* detections, const std::array<float, 4>& padding) {
  const float left = padding[0], top = padding[1];
  const float left_and_right = padding[0] + padding[2];
  const float top_and_bottom = padding[1] + padding[3];
  for (auto& d : *detections) {
    const float xmin = (d.xmin - left) / (1.0f - left_and_right);
    const float ymin = (d.ymin - top) / (1.0f - top_and_bottom);
    d.width = d.width / (1.0f - left_and_right);
    d.height = d.height / (1.0f - top_and_bottom);
    d.xmin = xmin;
    d.ymin = ymin;
    for (auto& k : d.keypoints) {
      const float new_x = (k.x - left) / (1.0f - left_and_right);
      const float new_y = (k.y - top) / (1.0f - top_and_bottom);
      k.x = new_x;
      k.y = new_y;
    }
  }
}

void RemoveLetterbox(std::vector<Landmark>* landmarks, const std::array<float, 4>& padding) {
  const float left = padding[0], top = padding[1];
  const float left_and_right = padding[0] + padding[2];
  const float top_and_bottom = padding[1] + padding[3];
  for (auto& l : *landmarks) {
    const float new_x = (l.x - left) / (1.0f - left_and_right);
    const float new_y = (l.y - top) / (1.0f - top_and_bottom);
    l.x = new_x;
    l.y = new_y;  // z kept
  }
}

NormRect DetectionToNormRect(const Detection& detection, int image_width, int image_height,
                             int start_keypoint, int end_keypoint, float target_angle_degrees) {
  NormRect rect;
  rect.x_center = detection.xmin + detection.width / 2;
  rect.y_center = detection.ymin + detection.height / 2;
  rect.width = detection.width;
  rect.height = detection.height;
  const float target_angle = static_cast<float>(kPi * target_angle_degrees / 180.f);
  const float x0 = detection.keypoints[start_keypoint].x * image_width;
  const float y0 = detection.keypoints[start_keypoint].y * image_height;
  const float x1 = detection.keypoints[end_keypoint].x * image_width;
  const float y1 = detection.keypoints[end_keypoint].y * image_height;
  rect.rotation = NormalizeRadians(target_angle - std::atan2(-(y1 - y0), x1 - x0));
  return rect;
}

NormRect TransformNormRect(NormRect rect, int image_width, int image_height,
                           const RectTransformOptions& o) {
  float width = rect.width;
  float height = rect.height;
  const float rotation = rect.rotation;  // no rotation option is set by the hand graph
  if (rotation == 0.f) {
    rect.x_center = rect.x_center + width * o.shift_x;
    rect.y_center = rect.y_center + height * o.shift_y;
  } else {
    const float x_shift = (image_width * width * o.shift_x * std::cos(rotation) -
                           image_height * height * o.shift_y * std::sin(rotation)) /
                          image_width;
    const float y_shift = (image_width * width * o.shift_x * std::sin(rotation) +
                           image_height * height * o.shift_y * std::cos(rotation)) /
                          image_height;
    rect.x_center = rect.x_center + x_shift;
    rect.y_center = rect.y_center + y_shift;
  }
  if (o.square_long) {
    const float long_side = std::max(width * image_width, height * image_height);
    width = long_side / image_width;
    height = long_side / image_height;
  }
  rect.width = width * o.scale_x;
  rect.height = height * o.scale_y;
  return rect;
}

Detection LandmarksToDetection(const std::vector<Landmark>& landmarks) {
  Detection detection;
  float x_min = std::numeric_limits<float>::max();
  float x_max = std::numeric_limits<float>::min();  // smallest positive float, as in the original
  float y_min = std::numeric_limits<float>::max();
  float y_max = std::numeric_limits<float>::min();
  for (const auto& l : landmarks) {
    x_min = std::min(x_min, l.x);
    x_max = std::max(x_max, l.x);
    y_min = std::min(y_min, l.y);
    y_max = std::max(y_max, l.y);
    detection.keypoints.push_back({l.x, l.y});
  }
  detection.xmin = x_min;
  detection.ymin = y_min;
  detection.width = x_max - x_min;
  detection.height = y_max - y_min;
  return detection;
}

std::vector<Landmark> ProjectLandmarks(const std::vector<Landmark>& landmarks, const NormRect& rect) {
  std::vector<Landmark> out;
  out.reserve(landmarks.size());
  for (const auto& l : landmarks) {
    const float x = l.x - 0.5f;
    const float y = l.y - 0.5f;
    const float angle = rect.rotation;
    float new_x = std::cos(angle) * x - std::sin(angle) * y;
    float new_y = std::sin(angle) * x + std::cos(angle) * y;
    new_x = new_x * rect.width + rect.x_center;
    new_y = new_y * rect.height + rect.y_center;
    out.push_back({new_x, new_y, l.z});
  }
  return out;
}

}  // namespace slam_refact::hand
