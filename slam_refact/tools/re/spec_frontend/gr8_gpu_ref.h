// Reference CPU emulation of reference runtime GlRealityFrame::drawTexture (public xr_reference/xr_reference@4edfa484,
// reality/engine/features/gl-reality-frame.cc + gr8-*.frag/.vert), written for spec lane S1 only.
//
// Purpose: validate the frontend spec (slam_refact/docs/spec/01-frontend.md) by producing the same
// RGBA8 pyramid atlas the GPU reads back (R gray, G 7x7 Gaussian, B NMS score, A subpixel nibbles)
// so the public Gr8Gl CPU half can consume it unchanged. Every shader expression is evaluated in
// IEEE float32 in source order; textures are RGBA8 (unorm, round-to-nearest on write).
// No OpenCV. Not engine code (engine lives under slam_refact/engine).
#ifndef SLAM_REFACT_TOOLS_RE_SPEC_FRONTEND_GR8_GPU_REF_H_
#define SLAM_REFACT_TOOLS_RE_SPEC_FRONTEND_GR8_GPU_REF_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace spec_frontend {

struct Layout {
  int r = 0;
  int c = 0;
  int w = 0;
  int h = 0;
  bool rotated = false;
};

struct Gr8GpuParams {
  int pyramid_size = 1024;     // GlRealityFrame.PyramidSize
  float gain = 1.0f;           // Gr8FeatureShader::featureGain_
  float score_thresh = 4.0f;   // Gr8FeatureShader::scoreThresh_
  float nms_tolerance = 1.0f;  // 0 = NMS, 1 = NMES (spatial tie break), other = no NMS
  bool skip_subpixel = false;  // GlRealityFrame::skipSubpixel_
  int rotation = 0;            // engineInit rotation (screen orientation angle)
  int flip_y = 0;              // GlRealityFrame::flipY_
  // 0: exact float bilinear weights. N>0: weights quantized to N fractional bits (GPU fixed point
  // filtering model). Used only for LINEAR textures (camera texture, filtered input).
  int linear_weight_bits = 0;
  // false: normative IEEE model (texel = b / 255, mix = x*(1-a) + y*a, unorm write = floor(255 f + 0.5)).
  // true: SwiftShader-approximate model fitted on the lane S1 capture (texel = b * (1/255.f),
  // mix = x + a*(y - x), unorm write = round-half-even(255 f)). See 01-frontend.md section 8.
  bool swiftshader_approx = false;
  // true (default): evaluate the shader expressions in the form embedded in xr-slam.wasm (SPIRV-Cross
  // output, xr-slam-strings.txt 7973-9167: reassociated pyramid blur sum, simplified subpixel algebra,
  // multiply by 0.90909087 instead of / 1.1). false: the public .frag source form.
  bool binary_expression_forms = true;
};

// RGBA8 texture, row 0 = GL row 0 (texture t = 0, readPixels row 0).
struct Tex {
  int w = 0;
  int h = 0;
  std::vector<uint8_t> px;
  void Init(int width, int height) {
    w = width;
    h = height;
    px.assign(static_cast<size_t>(w) * h * 4, 0);  // WebGL zero-initialises textures.
  }
  uint8_t *At(int x, int y) { return &px[(static_cast<size_t>(y) * w + x) * 4]; }
  const uint8_t *At(int x, int y) const { return &px[(static_cast<size_t>(y) * w + x) * 4]; }
};

class Gr8GpuRef {
 public:
  // src_w/src_h: camera texture size (videoWidth/videoHeight).
  void Initialize(int src_w, int src_h, const Gr8GpuParams &params);
  // rgba: camera texture bytes, src_w x src_h, row 0 = first uploaded row (top of the video).
  void Draw(const uint8_t *rgba, int stride_bytes);
  // Isolation tests on a captured readback (R of the readback == stage-1/decimated gray of dest1).
  // rebuild_levels = false: dest1 R := captured R everywhere, then stage2 + stage3 only.
  // rebuild_levels = true:  dest1 R := captured R inside level 0 and the ROI slots only, then the
  //                         blur/decimate chain rebuilds levels 1-3, then stage2 + stage3.
  void DrawFromCapturedGray(const uint8_t *readback, int rows, bool rebuild_levels);

  const Tex &atlas() const { return dest3_; }        // what readPixels returns
  const Tex &stage1_atlas() const { return dest1_; }  // gray pyramid before stage2/3
  const Tex &filtered() const { return filtered_; }
  const std::vector<Layout> &levels() const { return levels_; }
  const std::vector<Layout> &roi_layouts() const { return roi_layouts_; }
  int filtered_w() const { return filtered_w_; }
  int filtered_h() const { return filtered_h_; }
  int input_w() const { return input_w_; }
  int input_h() const { return input_h_; }
  static std::array<float, 6> SubpixelParams();

 private:
  void InitializeLevels(int src_w, int src_h, int out_w, int out_h);
  void InputStage();
  void Stage1();
  void PyramidBlur(int level);
  void PyramidDecimate(int level);
  void Stage2();
  void Stage3();
  void SampleLinear(const Tex &t, float u, float v, float out[4]) const;

  Gr8GpuParams params_;
  int input_w_ = 0, input_h_ = 0;
  int filtered_w_ = 0, filtered_h_ = 0;
  int tex_w_ = 0, tex_h_ = 0;
  std::vector<Layout> levels_;
  std::vector<Layout> roi_layouts_;
  Tex camera_, filtered_, dest1_, dest2_, dest3_;
};

// GLSL helpers exposed for the self test.
float Max9FullScore(const float d[16]);  // shader-literal scoreGr8 on 16 circle differences
float Fast9Textbook(const float d[16]);  // max over 16 arcs of min over 9 contiguous, both signs

}  // namespace spec_frontend

#endif  // SLAM_REFACT_TOOLS_RE_SPEC_FRONTEND_GR8_GPU_REF_H_
