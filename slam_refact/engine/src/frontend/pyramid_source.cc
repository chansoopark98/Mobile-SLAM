// Pyramid source: injection (H7) or CPU shader port (S1 §7, architecture.md §3.2).

#include "frontend/pyramid_source.h"

#include "common/check.h"
#include "frontend/roi_draw.h"

namespace slam_refact::frontend {

c8::Gr8Pyramid MakeGr8Pyramid(const uint8_t* rgba, int size, int rows,
                              const PyramidLayout& layout) {
  c8::Gr8Pyramid pyramid;
  pyramid.data = c8::ConstRGBA8888PlanePixels(rows, size, 4 * size, rgba);
  for (const LevelLayout& l : layout.levels) {
    c8::LevelLayout level;
    level.r = l.r;
    level.c = l.c;
    level.w = l.w;
    level.h = l.h;
    level.rotated = l.rotated;
    pyramid.levels.push_back(level);
  }
  return pyramid;
}

c8::Gr8Pyramid PyramidFromInjectedAtlas(const PyramidAtlasView& view, const PyramidLayout& layout) {
  SR_CHECK(view.rgba != nullptr, "injected pyramid has no pixels");
  SR_CHECK(view.size == layout.texture_size, "injected pyramid size differs from the layout");
  SR_CHECK(view.rows > 0 && view.rows <= view.size, "injected pyramid rows out of range");
  for (const LevelLayout& l : layout.levels) {
    SR_CHECK(l.r + l.h <= view.rows && l.c + l.w <= view.size,
             "injected pyramid does not contain every level of the layout");
  }
  return MakeGr8Pyramid(view.rgba, view.size, view.rows, layout);
}

PyramidSource::PyramidSource(int camera_w, int camera_h, ShaderParams params,
                             ShaderNumericModel model, int rotation_deg, int pyramid_size)
    : layout_(ComputePyramidLayout(camera_w, camera_h, pyramid_size)),
      cpu_(layout_, params, model),
      rotation_deg_(rotation_deg),
      camera_w_(camera_w),
      camera_h_(camera_h),
      model_(model) {}

c8::Gr8Pyramid PyramidSource::Get(const PyramidAtlasView& injected, const RgbaImageView& rgba,
                                  const GrayImageView& gray) {
  if (injected.rgba != nullptr) {
    return PyramidFromInjectedAtlas(injected, layout_);
  }
  // The layout, the intrinsics (K rule on the capture size) and the shader texture coordinates
  // all assume the engineInit size; a frame of another size would be resampled silently onto a
  // pyramid whose K no longer matches (S2 review MEDIUM-3). The original re-runs engineInit when
  // the camera size changes, so a mismatching frame is an input error here.
  if (rgba.data != nullptr) {
    SR_CHECK(rgba.width == camera_w_ && rgba.height == camera_h_,
             "rgba frame size differs from the capture geometry (engineInit size)");
    cpu_.Build(rgba, rotation_deg_, &atlas_);
  } else {
    SR_CHECK(gray.data != nullptr && gray.width > 0 && gray.height > 0,
             "frame has neither an injected pyramid nor an image");
    SR_CHECK(gray.width == camera_w_ && gray.height == camera_h_,
             "gray frame size differs from the capture geometry (engineInit size)");
    SR_CHECK(gray.stride >= gray.width, "gray stride smaller than width");
    gray_rgba_.resize(static_cast<size_t>(gray.width) * gray.height * 4);
    for (int y = 0; y < gray.height; ++y) {
      const uint8_t* src = gray.data + static_cast<size_t>(y) * gray.stride;
      uint8_t* dst = &gray_rgba_[static_cast<size_t>(y) * gray.width * 4];
      for (int x = 0; x < gray.width; ++x) {
        dst[4 * x + 0] = src[x];
        dst[4 * x + 1] = src[x];
        dst[4 * x + 2] = src[x];
        dst[4 * x + 3] = 255;
      }
    }
    const RgbaImageView view{gray_rgba_.data(), gray.width, gray.height, gray.width * 4};
    cpu_.Build(view, rotation_deg_, &atlas_);
  }
  return MakeGr8Pyramid(atlas_.rgba.data(), atlas_.size, atlas_.rows, layout_);
}

namespace {

// pyramid.rois = the ROIs of this draw with their slot layouts (readPixels: rois_ = drawnRois_).
void AttachRoiSlots(const PyramidLayout& layout, const std::vector<c8::ImageRoi>& rois,
                    c8::Gr8Pyramid* pyramid) {
  for (size_t i = 0; i < rois.size(); ++i) {
    const LevelLayout& s = layout.roi_slots[i];
    c8::ROI r;
    r.roi = rois[i];
    r.layout.r = s.r;
    r.layout.c = s.c;
    r.layout.w = s.w;
    r.layout.h = s.h;
    r.layout.rotated = s.rotated;
    pyramid->rois.push_back(r);
  }
}

}  // namespace

c8::Gr8Pyramid PyramidSource::GetWithRois(const PyramidAtlasView& injected,
                                          const RgbaImageView& rgba, const GrayImageView& gray,
                                          const std::vector<c8::ImageRoi>& rois) {
  if (injected.rgba != nullptr && !rois.empty()) {
    // FULL-7 (RM-4b): the producer of an injected atlas draws the ROI slots itself (stage 1 into
    // the slots, then stages 2 and 3 over the whole texture, like the GPU does); the engine reads
    // them as drawn and only attaches the slot layouts. The atlas must contain every slot.
    SR_CHECK(rois.size() <= layout_.roi_slots.size(), "injected pyramid: more ROIs than slots");
    SR_CHECK(injected.rows >= RoiAtlasRows(layout_),
             "injected pyramid does not contain the ROI slots (rows < RoiAtlasRows)");
    c8::Gr8Pyramid pyramid = PyramidFromInjectedAtlas(injected, layout_);
    AttachRoiSlots(layout_, rois, &pyramid);
    return pyramid;
  }
  if (injected.rgba != nullptr || rois.empty()) {
    return Get(injected, rgba, gray);
  }
  Get(injected, rgba, gray);  // levels into atlas_ (and the gray upload into gray_rgba_)
  // roiSrc = the camera texture (drawTexture -> drawInitialStage(filtered, cameraTexture, ...)).
  const RgbaImageView camera =
      rgba.data != nullptr ? rgba
                           : RgbaImageView{gray_rgba_.data(), gray.width, gray.height, gray.width * 4};
  DrawRoisIntoAtlas(layout_, cpu_, model_, camera, rotation_deg_, rois, &atlas_);
  c8::Gr8Pyramid pyramid = MakeGr8Pyramid(atlas_.rgba.data(), atlas_.size, atlas_.rows, layout_);
  AttachRoiSlots(layout_, rois, &pyramid);
  return pyramid;
}

}  // namespace slam_refact::frontend
