// Image target loader on the CPU pyramid port (US-012).
//
// Port of the public DetectionImageLoader (PUB:reality/engine/imagedetection/
// detection-image-loader.{h,cc}) with the GL stages replaced:
//   GlRealityFrame::initialize/draw/readPixels -> frontend::PyramidSource (CPU shader port, the
//   same layout rule as GlRealityFrame::initialize, stage-1 rotation = the loader rotation),
//   Gr8Gl::detectAndCompute (vendored, unchanged) on that pyramid.
// The texture upload of the original (texImage2D of the decoded RGBA image) is the gray path of
// PyramidSource (R = G = B = luminance, A = 255).

#ifndef SLAM_REFACT_IMAGE_TARGET_TARGET_LOADER_H_
#define SLAM_REFACT_IMAGE_TARGET_TARGET_LOADER_H_

#include <string>

#include "c8/geometry/parameterized-geometry.h"
#include "c8/hmatrix.h"
#include "frontend/frontend.h"
#include "image_target/image_target_spec.h"
#include "reality/engine/features/image-descriptor.h"
#include "reality/engine/imagedetection/detection-image.h"
#include "slam_refact/image_types.h"
#include "slam_refact/types.h"

namespace slam_refact::image_target {

// Intrinsics::rotateCropAndScaleIntrinsics (PUB:c8/geometry/intrinsics.cc:168-184) on top of
// frontend::CropAndScale (= Intrinsics::cropAndScaleIntrinsics, intrinsics.cc:245-263).
PinholeIntrinsics RotateCropAndScaleIntrinsics(const PinholeIntrinsics& k, int width, int height);

class TargetLoader {
 public:
  // DetectionImageLoader::initialize. image_width x image_height is the decoded luminance size;
  // device_k is Intrinsics::getCameraIntrinsics(device model) (frontend::FullIntrinsicsForDevice).
  TargetLoader(const ImageTargetSpec& spec, int image_width, int image_height,
               const PinholeIntrinsics& device_k);

  // processGpu + readDataToCpu + extractFeatures: CPU pyramid of the luminance image, then
  // Gr8Gl::detectAndCompute with allOrb + allGorb and 2500 keypoints. Aborts (SR_CHECK) when the
  // image is not image_width x image_height.
  c8::DetectionImage ExtractFeatures(const GrayImageView& luminance,
                                     ShaderNumericModel model) const;

  // DetectionImageLoader::buildCurvySpec / buildFullLabelToCroppedLabelPose.
  static c8::CurvySpec BuildCurvySpec(const ImageTargetSpec& spec);
  static c8::HMatrix BuildFullLabelToCroppedLabelPose(c8::CurvySpec spec);

  const std::string& name() const { return name_; }
  int rotation() const { return rotation_; }
  bool rotate_features() const { return rotate_features_; }
  c8::DescriptorType feature_type() const { return feature_type_; }
  // k_ of the original: the target intrinsics at pyramid level 0 (rotated when rotateFeatures).
  const PinholeIntrinsics& intrinsics() const { return k_; }
  const frontend::PyramidLayout& layout() const { return layout_; }

 private:
  // DetectionImageLoader::setGeometry.
  void SetGeometry(int crop_width, int crop_height, c8::CurvySpec spec);

  std::string name_;
  int image_width_ = 0;
  int image_height_ = 0;
  frontend::PyramidLayout layout_;
  PinholeIntrinsics k_;
  c8::DetectionImageType detection_image_type_ = c8::DetectionImageType::UNSPECIFIED;
  c8::PlanarImageGeometry planar_geom_;
  c8::CurvyImageGeometry curvy_geom_;
  c8::CurvyImageGeometry curvy_full_geom_;
  c8::HMatrix full_label_to_cropped_label_pose_ = c8::HMatrixGen::i();
  int rotation_ = 0;
  bool is_rotated_ = false;
  c8::DescriptorType feature_type_ = c8::DescriptorType::ORB;
  bool rotate_features_ = false;
};

// TargetLoader(spec, luminance size, device_k).ExtractFeatures(luminance, model).
c8::DetectionImage LoadDetectionImage(const ImageTargetSpec& spec, const GrayImageView& luminance,
                                      const PinholeIntrinsics& device_k, ShaderNumericModel model);

}  // namespace slam_refact::image_target

#endif  // SLAM_REFACT_IMAGE_TARGET_TARGET_LOADER_H_
