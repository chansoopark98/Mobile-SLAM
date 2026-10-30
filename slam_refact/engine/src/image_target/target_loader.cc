// Image target loader on the CPU pyramid port (US-012). Line references are to the public
// PUB:reality/engine/imagedetection/detection-image-loader.cc unless noted.

#include "image_target/target_loader.h"

#include <cmath>
#include <utility>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hpoint.h"
#include "common/c8_adapters.h"
#include "common/check.h"
#include "config/settings.h"
#include "frontend/pyramid_source.h"
#include "reality/engine/features/detection-config.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/gr8gl.h"
#include "reality/engine/imagedetection/target-point.h"

namespace slam_refact::image_target {
namespace {

constexpr float kEpsilon = 1e-9f;  // :36
// M_PI (double) of the original, spelled out so the file does not depend on _USE_MATH_DEFINES.
constexpr double kPi = 3.14159265358979323846;
// Gr8Gl::create() of the loader (default arguments 1.44, 21, 2) and extractFeatures (:211-213).
constexpr int kTargetKeypoints = 2500;

}  // namespace

PinholeIntrinsics RotateCropAndScaleIntrinsics(const PinholeIntrinsics& intrinsics, int width,
                                               int height) {
  PinholeIntrinsics k = intrinsics;
  // If the camera feed is rotated with respect to our model, swap x/y (intrinsics.cc:171-181).
  if ((width > height) != (k.width > k.height)) {
    k = {k.height, k.width, k.cy, k.cx, k.fy, k.fx};
  }
  return frontend::CropAndScale(k, width, height);
}

TargetLoader::TargetLoader(const ImageTargetSpec& spec, int image_width, int image_height,
                           const PinholeIntrinsics& device_k)
    : name_(spec.name), image_width_(image_width), image_height_(image_height) {
  SR_CHECK(image_width > 0 && image_height > 0, "image target size must be positive");
  // :55-58. Upright 3:4 image whose pixels were rotated 90 degrees clockwise (xrhome / CLI).
  is_rotated_ = spec.is_rotated;
  rotate_features_ = is_rotated_;
  // :60-80. GORB only for static planar targets away from the XZ plane with zero pitch.
  if (spec.type == ImageTargetType::kPlanar && spec.is_static_target) {
    const float min_roll =
        config::DetectionImageLoaderSettings::Get().min_roll_angle_from_xz_plane_for_gorb;
    const float roll = spec.roll_angle;
    const float pitch = spec.pitch_angle;
    if (!(std::abs(roll - 90.0f) < min_roll || std::abs(roll - 270.0f) < min_roll) &&
        (std::abs(pitch) < kEpsilon)) {
      feature_type_ = c8::DescriptorType::GORB;
      rotate_features_ |= (image_width > image_height);
    } else {
      feature_type_ = c8::DescriptorType::ORB;
    }
  }

  // :82-88. GlRealityFrame::initialize(glShader, w, h, rotation_) = the CPU layout rule.
  rotation_ = image_width > image_height ? 90 : 0;
  layout_ = frontend::ComputePyramidLayout(image_width, image_height);

  // :90-92.
  const frontend::LevelLayout& l0 = layout_.levels[0];
  k_ = rotate_features_ ? RotateCropAndScaleIntrinsics(device_k, l0.h, l0.w)
                        : RotateCropAndScaleIntrinsics(device_k, l0.w, l0.h);

  // :94-113.
  switch (spec.type) {
    case ImageTargetType::kPlanar:
      planar_geom_ = c8::PlanarImageGeometry{rotate_features_};
      detection_image_type_ = c8::DetectionImageType::PLANAR;
      break;
    case ImageTargetType::kCurvy:
      SetGeometry(spec.crop_original_image_width, spec.crop_original_image_height,
                  BuildCurvySpec(spec));
      break;
    default:
      planar_geom_ = c8::PlanarImageGeometry{rotate_features_};
      detection_image_type_ = c8::DetectionImageType::UNSPECIFIED;
  }
}

// :127-147.
c8::HMatrix TargetLoader::BuildFullLabelToCroppedLabelPose(c8::CurvySpec s) {
  if (s.isRotated) {
    std::swap(s.crop.outerWidth, s.crop.outerHeight);
    std::swap(s.crop.cropStartX, s.crop.cropStartY);
    s.crop.cropStartY = 1.f - s.crop.cropStartY - (1.f / s.crop.outerHeight);
  }
  const c8::CurvyOuterCrop crop = s.crop;

  // Yaw rotation between a cropped label and a full label.
  const float inner_left_shift_to_target_center = crop.cropStartX + 0.5f / crop.outerWidth;
  const float shift_in_label_space = inner_left_shift_to_target_center - 0.5;
  const float shift_in_radian = 2 * kPi * shift_in_label_space * s.arc;
  const c8::HMatrix rot = c8::HMatrixGen::yRadians(shift_in_radian);

  // Pixel space has y pointing down but model space has y pointing up.
  const float translate_y =
      -1.0f * (crop.outerHeight * crop.cropStartY + 0.5f - crop.outerHeight / 2);
  return c8::HMatrixGen::translation(0, translate_y, 0) * rot;
}

// :149-174.
c8::CurvySpec TargetLoader::BuildCurvySpec(const ImageTargetSpec& spec) {
  const bool is_rotated = spec.is_rotated;
  const int original_width = spec.original_image_width;
  const int original_height = spec.original_image_height;
  const int x = spec.crop_original_image_x;
  const int y = spec.crop_original_image_y;
  const int crop_width = spec.crop_original_image_width;
  const int crop_height = spec.crop_original_image_height;
  const float circumference_top = spec.curvy_circumference_top;
  const float circumference_bottom = spec.curvy_circumference_bottom;
  const float target_circumference_top = spec.target_circumference_top;

  const float arc = target_circumference_top / circumference_top;
  const c8::CurvyOuterCrop crop{
      1.0f * original_width / crop_width,
      1.0f * original_height / crop_height,
      1.0f * x / original_width,
      1.0f * y / original_height,
  };
  const float base = circumference_top / circumference_bottom;
  return {arc, is_rotated, crop, base};
}

// :176-193.
void TargetLoader::SetGeometry(int crop_width, int crop_height, c8::CurvySpec spec) {
  detection_image_type_ = c8::DetectionImageType::CURVY;
  full_label_to_cropped_label_pose_ = BuildFullLabelToCroppedLabelPose(spec);
  c8::curvyForTarget(crop_width, crop_height, spec, &curvy_geom_, &curvy_full_geom_);
  curvy_full_geom_.srcRows = crop_height * spec.crop.outerHeight;
  curvy_full_geom_.srcCols = crop_width * spec.crop.outerWidth;

  // The image is not rotated in the pyramid (the feature points are), so w and h flip here.
  const frontend::LevelLayout& l0 = layout_.levels[0];
  if (is_rotated_) {
    curvy_geom_.srcRows = l0.w;
    curvy_geom_.srcCols = l0.h;
  } else {
    curvy_geom_.srcRows = l0.h;
    curvy_geom_.srcCols = l0.w;
  }
}

// processGpu + readDataToCpu (:195-202) + extractFeatures (:204-251).
c8::DetectionImage TargetLoader::ExtractFeatures(const GrayImageView& luminance,
                                                 ShaderNumericModel model) const {
  SR_CHECK(luminance.data != nullptr, "image target luminance has no pixels");
  SR_CHECK(luminance.width == image_width_ && luminance.height == image_height_,
           "image target luminance size differs from the loader size");
  frontend::PyramidSource source(image_width_, image_height_, frontend::ShaderParams{}, model,
                                 rotation_);
  const c8::Gr8Pyramid pyramid = source.Get(PyramidAtlasView{}, RgbaImageView{}, luminance);

  c8::Gr8Gl feature_detector = c8::Gr8Gl::create();
  c8::FrameWithPoints fake_frame({});
  c8::DetectionConfig image_detection_config;
  image_detection_config.allOrb = true;
  image_detection_config.allGorb = true;
  image_detection_config.nKeypoints = kTargetKeypoints;
  const float gravity_angle_offset = rotate_features_ ? 90.0f : 0.0f;
  c8::ImagePoints image_points = feature_detector.detectAndCompute(
      pyramid, fake_frame, image_detection_config, gravity_angle_offset);

  c8::TargetWithPoints frame_points{ToC8Intrinsics(k_)};
  frame_points.reserve(image_points.size());
  const int width = pyramid.levels[0].w;
  for (const auto& f : image_points) {
    const auto l = f.location();
    if (rotate_features_) {
      const float new_x = l.pt.y;
      const float new_y = width - 1 - l.pt.x;
      const float new_angle = l.angle - 90;
      const float new_gravity_angle = l.gravityAngle - 90;
      frame_points.addImagePixelPoint(c8::HPoint2(new_x, new_y), l.scale, new_angle,
                                      new_gravity_angle, f.features().clone());
    } else {
      frame_points.addImagePixelPoint(c8::HPoint2(l.pt.x, l.pt.y), l.scale, l.angle,
                                      l.gravityAngle, f.features().clone());
    }
  }
  frame_points.setExcludedEdgePixels(feature_detector.edgeThreshold());
  frame_points.setFeatureType(feature_type_);

  if (detection_image_type_ == c8::DetectionImageType::CURVY) {
    return c8::DetectionImage(name_, std::move(frame_points), rotation_, curvy_geom_,
                              curvy_full_geom_, full_label_to_cropped_label_pose_);
  }
  return c8::DetectionImage(name_, std::move(frame_points), rotation_, planar_geom_);
}

c8::DetectionImage LoadDetectionImage(const ImageTargetSpec& spec, const GrayImageView& luminance,
                                      const PinholeIntrinsics& device_k, ShaderNumericModel model) {
  return TargetLoader(spec, luminance.width, luminance.height, device_k)
      .ExtractFeatures(luminance, model);
}

}  // namespace slam_refact::image_target
