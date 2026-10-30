// Frontend private header: where the per-frame Gr8Pyramid comes from (S1 §7 "pyramid 주입 hook",
// architecture.md §3.0 `pyramid = in.pyramid ?: CpuGr8Pyramid::Build(in.image)`, §3.2 point 2,
// H7). Engine::Track (WP5) owns one PyramidSource per engine instance.

#ifndef SLAM_REFACT_FRONTEND_PYRAMID_SOURCE_H_
#define SLAM_REFACT_FRONTEND_PYRAMID_SOURCE_H_

#include <cstdint>
#include <vector>

#include "c8/pixels/gr8-pyramid.h"
#include "c8/pixels/image-roi.h"
#include "frontend/frontend.h"
#include "frontend/gr8_cpu_shader.h"
#include "slam_refact/image_types.h"
#include "slam_refact/types.h"

namespace slam_refact::frontend {

// c8::Gr8Pyramid view over an RGBA atlas (size x rows, row bytes 4 * size, row 0 = GL row 0) with
// the layout's four levels and no ROI (world SLAM, S1 §2.3). No copy: the atlas bytes must
// outlive the returned pyramid.
c8::Gr8Pyramid MakeGr8Pyramid(const uint8_t* rgba, int size, int rows,
                              const PyramidLayout& layout);

// H7 injection: validates an external readback atlas against the layout (same texture size,
// every level inside the given rows) and wraps it without a copy. Aborts (SR_CHECK) otherwise.
c8::Gr8Pyramid PyramidFromInjectedAtlas(const PyramidAtlasView& view, const PyramidLayout& layout);

class PyramidSource {
 public:
  // camera_w x camera_h is the engineInit size (portrait in world SLAM, R16). rotation_deg is the
  // stage-1 screen rotation (engineInit rotation).
  PyramidSource(int camera_w, int camera_h, ShaderParams params, ShaderNumericModel model,
                int rotation_deg = 0, int pyramid_size = kPyramidSizeDefault);

  const PyramidLayout& layout() const { return layout_; }

  // FrameInput priority (engine.h): an injected pyramid wins; otherwise the CPU shader port runs
  // on the RGBA image, otherwise on the gray image uploaded as R = G = B, A = 255 (the camera
  // texture of a gray frame, S1 §2.1 VERIFIED H1). The returned view stays valid until the next
  // call (CPU path) or as long as the injected bytes live (injection path). Aborts (SR_CHECK) when
  // the image used is not camera_w x camera_h.
  c8::Gr8Pyramid Get(const PyramidAtlasView& injected, const RgbaImageView& rgba,
                     const GrayImageView& gray);

  // FULL-3: Get on the CPU path plus the ROI slots of this draw (GlRealityFrame nextRois, at most
  // 4; DrawRoisIntoAtlas) attached as pyramid.rois with their slot layouts (readPixels:
  // rois_ = drawnRois_). With no ROI this is Get.
  // FULL-7 (RM-4b): an injected atlas with ROIs is used as drawn by its producer (the ROI slots of
  // PyramidLayout::roi_slots, stage 1 from the camera texture through the slot's mvp, then stages 2
  // and 3 over the whole texture: R gray, B NMES score, A subpixel) and gets the same pyramid.rois.
  // It must have at least RoiAtlasRows(layout) rows (SR_CHECK); the producer takes the slot order
  // and mvp from GlueRunner::NextDrawRois().
  c8::Gr8Pyramid GetWithRois(const PyramidAtlasView& injected, const RgbaImageView& rgba,
                             const GrayImageView& gray, const std::vector<c8::ImageRoi>& rois);

  // Atlas of the last CPU build (empty before the first CPU build).
  const RgbaAtlas& last_cpu_atlas() const { return atlas_; }

 private:
  PyramidLayout layout_;
  CpuGr8Pyramid cpu_;
  int rotation_deg_;
  int camera_w_;
  int camera_h_;
  RgbaAtlas atlas_;
  std::vector<uint8_t> gray_rgba_;
  ShaderNumericModel model_;
};

}  // namespace slam_refact::frontend

#endif  // SLAM_REFACT_FRONTEND_PYRAMID_SOURCE_H_
