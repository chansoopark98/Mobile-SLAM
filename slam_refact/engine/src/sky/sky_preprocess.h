// Input side of the LayersController (US-020; docs/spec/09-sky.md §2): the pixel helpers of
// PUB:reality/app/xr/js/src/pixel-transforms.ts and quaternion.ts, the output size rule of
// layers-controller.ts, and a CPU model of the GPU camera resizer (gl-renderer.ts GlTextureRenderer
// with toTexture + flipY).
#ifndef SLAM_REFACT_SKY_SKY_PREPROCESS_H_
#define SLAM_REFACT_SKY_SKY_PREPROCESS_H_

#include <cstdint>

#include "sky/sky_types.h"

namespace slam_refact::sky {

// c8::cropDimensionByAspectRatio (PUB:c8/pixels/pixel-transforms.cc:2963-2977, same float and int
// conversions); re-implemented because linking the vendored pixel-transforms.cc would need
// color-maps.cc and packed.cc (the face lane's sr_vendor_face_pixels) for symbols this module never
// calls.
void CropDimensionByAspectRatio(int input_width, int input_height, float aspect_ratio,
                                int* crop_width, int* crop_height);

// pixel-transforms.ts scaleFactorSameAspectRatioCrop.
double ScaleFactorSameAspectRatioCrop(double image_w, double image_h, double desired_w,
                                      double desired_h);

// layers-controller.ts calculateOutputSize: the model input size (portrait 144x256 when the video
// is taller than wide, else landscape 256x144) and the resized video size (double, as JS keeps it).
struct ModelInputGeometry {
  int model_input_width = 0;
  int model_input_height = 0;
  double video_resize_width = 0.0;
  double video_resize_height = 0.0;
};
ModelInputGeometry CalculateOutputSize(int video_width, int video_height);

// pixel-transforms.ts toImageArrayLetterbox / toImageArrayRotateCW90 / toImageArrayRotateCCW90.
// `dst` has dst_w * dst_h * 4 bytes and is written in place. The JS version writes through a
// Uint8Array with double-valued indices: an index that is not an integer in [0, length) is silently
// dropped (typed array semantics), and offsets may be negative or fractional (the deployed
// controller calls these with dst smaller than the source in the landscape cases). The port keeps
// exactly that behaviour.
void ToImageArrayLetterbox(const uint8_t* src, int src_w, int src_h, int dst_w, int dst_h,
                           uint8_t* dst);
void ToImageArrayRotateCW90(const uint8_t* src, int src_w, int src_h, int dst_w, int dst_h,
                            uint8_t* dst);
void ToImageArrayRotateCCW90(const uint8_t* src, int src_w, int src_h, int dst_w, int dst_h,
                             uint8_t* dst);

// GL_LINEAR / CLAMP_TO_EDGE sampling of an RGBA8 texture at normalized coordinates (u, v) (v = 0 is
// row 0). `out` = 4 floats in [0, 1].
void SampleBilinearClamp(const Rgba8Image& tex, double u, double v, float out[4]);

// Bilinear filter weight (fractional texel position) as the GPU sampler applies it: rounded to 8
// fractional bits. A weight of 1e-5 (float noise of an exactly texel-centred coordinate) becomes 0,
// as it does on the GPU; without this a texel that is only a rounding error away from its
// neighbour would blend them, which changes `prevCol.b == 0.0` in the projection shader.
float QuantizeFilterWeight(float w);

// float in [0, 1] -> GL RGBA8 normalized byte (round to nearest, clamped).
uint8_t ToUnorm8(float v);

// CPU model of cameraResizeRenderer_.render({renderTexture, viewport}) of layers-controller.ts
// onProcessGpu: GlTextureRenderer with toTexture {dest_w, dest_h} and flipY, drawing the camera
// texture through a viewport (vp_x, vp_y, vp_w, vp_h) (integers, vp_x/vp_y may be negative), with
// the noop shader (texture2D at the interpolated uv, GL_LINEAR). The flipY uv assignment cancels
// the quad's own orientation, so output row y samples texture rows from v = 0 upward: no vertical
// flip in memory order.
Rgba8Image ResizeCameraToModelInput(const Rgba8Image& camera, int dest_w, int dest_h, int vp_x,
                                    int vp_y, int vp_w, int vp_h);

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_SKY_PREPROCESS_H_
