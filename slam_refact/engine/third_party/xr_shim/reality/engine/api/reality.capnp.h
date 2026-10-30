// Cap'n Proto free shim of the generated reality/engine/api/reality.capnp.h (US-011).
//
// The vendored image target core uses one generated type: CurvyGeometry::Builder, the argument of
// DetectionImage::toCurvyGeometry (PUB:reality/engine/imagedetection/detection-image.{h,cc}).
// This header provides that type as a plain struct with the same field names and setters as the
// schema (PUB:reality/engine/api/reality.capnp:393-406, namespace c8 by $Cxx.namespace), so the
// vendored files compile unchanged. A Builder writes into a caller-owned CurvyGeometry::Data and
// a Reader reads it. Cap'n Proto serialization is out of scope (architecture.md §1.2).
#pragma once

namespace c8 {

struct CurvyGeometry {
  // Field order and names follow reality.capnp:393-406 (@0..@8).
  struct Data {
    float curvyCircumferenceTop = 0.0f;
    float curvyCircumferenceBottom = 0.0f;
    float curvySideLength = 0.0f;
    float targetCircumferenceTop = 0.0f;
    float height = 0.0f;
    float topRadius = 0.0f;
    float bottomRadius = 0.0f;
    float arcLengthRadians = 0.0f;
    float arcStartRadians = 0.0f;
  };

  class Reader {
  public:
    explicit Reader(const Data *data) : data_(data) {}
    float getCurvyCircumferenceTop() const { return data_->curvyCircumferenceTop; }
    float getCurvyCircumferenceBottom() const { return data_->curvyCircumferenceBottom; }
    float getCurvySideLength() const { return data_->curvySideLength; }
    float getTargetCircumferenceTop() const { return data_->targetCircumferenceTop; }
    float getHeight() const { return data_->height; }
    float getTopRadius() const { return data_->topRadius; }
    float getBottomRadius() const { return data_->bottomRadius; }
    float getArcLengthRadians() const { return data_->arcLengthRadians; }
    float getArcStartRadians() const { return data_->arcStartRadians; }

  private:
    const Data *data_;
  };

  class Builder {
  public:
    explicit Builder(Data *data) : data_(data) {}
    void setCurvyCircumferenceTop(float v) { data_->curvyCircumferenceTop = v; }
    void setCurvyCircumferenceBottom(float v) { data_->curvyCircumferenceBottom = v; }
    void setCurvySideLength(float v) { data_->curvySideLength = v; }
    void setTargetCircumferenceTop(float v) { data_->targetCircumferenceTop = v; }
    void setHeight(float v) { data_->height = v; }
    void setTopRadius(float v) { data_->topRadius = v; }
    void setBottomRadius(float v) { data_->bottomRadius = v; }
    void setArcLengthRadians(float v) { data_->arcLengthRadians = v; }
    void setArcStartRadians(float v) { data_->arcStartRadians = v; }
    Reader asReader() const { return Reader(data_); }

  private:
    Data *data_;
  };
};

}  // namespace c8
