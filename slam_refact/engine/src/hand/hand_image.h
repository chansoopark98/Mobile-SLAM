// Image operations of the MediaPipe hand graph on the CPU (US-021; docs/spec/10-hand.md §2), written
// to reproduce the OpenCV calls the original calculators make:
//   ImageTransformationCalculator FIT (image_transformation_calculator.cc:378-422):
//       cv::resize INTER_LINEAR to round(size * min scale), cv::copyMakeBorder BORDER_CONSTANT 0,
//       LETTERBOX_PADDING (:566-588)
//   ImageCroppingCalculator RenderCpu (image_cropping_calculator.cc:215-262):
//       cv::RotatedRect + cv::boxPoints + cv::getPerspectiveTransform + cv::warpPerspective with
//       flags = 0 (INTER_NEAREST!) and BORDER_REPLICATE
//   TfLiteConverterCalculator NormalizeImage (tflite_converter_calculator.cc:641-675)
// The product has no OpenCV dependency; OpenCV (cv2 in the test tooling) is the independent oracle of
// these functions (tools/hand/gen_image_golden.py), compared to a registered tolerance.
#ifndef SLAM_REFACT_HAND_HAND_IMAGE_H_
#define SLAM_REFACT_HAND_HAND_IMAGE_H_

#include <array>

#include "hand/hand_types.h"

namespace slam_refact::hand {

// cv::resize(src, dst, Size(dw, dh)) with the default INTER_LINEAR on CV_8UC3: OpenCV's fixed-point
// path (11 coefficient bits, (x + 2^21) >> 22 rounding).
Image3 ResizeLinear(const Image3& src, int dw, int dh);

// ImageTransformationCalculator with scale_mode FIT and the default constant_padding = true.
// `padding` = LETTERBOX_PADDING {left, top, right, bottom} normalised by the output size.
Image3 FitToSize(const Image3& src, int out_w, int out_h, std::array<float, 4>* padding);

// ImageCroppingCalculator for a NORM_RECT input (BORDER_REPLICATE). An invalid rect (width or height
// <= 0) crops the whole image, as GetCropSpecs does.
Image3 CropNormRect(const Image3& src, const NormRect& rect);

// cv::RotatedRect::points (boxPoints) and cv::getPerspectiveTransform (the 3x3 matrix, row-major).
std::array<std::array<float, 2>, 4> BoxPoints(float cx, float cy, float w, float h, float angle_deg);
std::array<double, 9> GetPerspectiveTransform(const std::array<std::array<float, 2>, 4>& src,
                                              const std::array<std::array<float, 2>, 4>& dst);

// TfLiteConverterCalculator: x / 127.5 - 1 (zero_center) or x / 255, float32 NHWC.
void ImageToTensor(const Image3& img, bool zero_center, float* out);

}  // namespace slam_refact::hand

#endif  // SLAM_REFACT_HAND_HAND_IMAGE_H_
