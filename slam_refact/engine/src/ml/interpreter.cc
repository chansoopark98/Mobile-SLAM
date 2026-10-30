#include "ml/interpreter.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstring>
#include <map>

#include "ml/custom_ops.h"
#include "ml/gemm.h"
#include "ml/kernels.h"

namespace slam_refact {
namespace ml {
namespace {

using kernels::Shape;

enum class Kind {
  kConv2D,
  kDepthwiseConv2D,
  kMaxPool2D,
  kBinary,
  kFloorMod,
  kRelu,
  kLeakyRelu,
  kHardSwish,
  kLogistic,
  kPad,
  kCopy,  // RESHAPE
  kConcatenation,
  kStridedSlice,
  kResizeBilinear,
  kMean,
  kReduceMax,
  kArgMax,
  kCast,
  kTranspose,
  kTransformLandmarks,
  kLandmarksToTransformMatrix,
  kTransformTensorBilinear,
  kTransposeConvBias,
};

struct Tensor {
  std::string name;
  TensorType type = TensorType::kFloat32;
  Shape shape;
  size_t count = 0;
  bool is_const = false;
  bool defined = false;  // constant, graph input, or produced by an earlier op
  std::vector<float> f32_const;
  std::vector<int32_t> i32;
  std::vector<int64_t> i64;
  float* f32 = nullptr;
  int first_def = -1;   // plan index of the producer (-1: graph input)
  int last_use = -1;    // plan index of the last consumer (INT_MAX: graph output)
  int slot = -1;
};

struct Plan {
  Kind kind = Kind::kCopy;
  int op_index = 0;
  std::string name;
  std::vector<int> in;
  std::vector<int> out;
  kernels::ConvGeometry geo;
  kernels::ActivationRange act{0.0f, 0.0f};
  int depth_multiplier = 1;
  kernels::BinaryOp binary = kernels::BinaryOp::kAdd;
  std::vector<int32_t> ints;
  std::vector<int> axes;
  int axis = 0;
  std::vector<kernels::SliceAxis> slice;
  int out_h = 0;
  int out_w = 0;
  bool align_corners = false;
  bool half_pixel_centers = false;
  float alpha = 0.0f;
  int dims = 0;
  custom::LandmarksToTransformMatrixParams l2t;
  // CONV_2D with a constant filter: gemm::PackWeights of [out_c][kh*kw*in_c], made at Create.
  // A non-constant filter is packed at every Invoke into Impl::pack_scratch.
  std::vector<float> packed;
};

std::string ShapeStr(const Shape& s) {
  std::string out = "[";
  for (size_t i = 0; i < s.size(); ++i) {
    if (i) out += ",";
    out += std::to_string(s[i]);
  }
  return out + "]";
}

}  // namespace

struct Interpreter::Impl {
  const TfliteModel* model = nullptr;
  std::vector<Tensor> tensors;
  std::vector<int> inputs;
  std::vector<int> outputs;
  std::vector<Plan> plans;
  size_t folded = 0;
  std::vector<std::vector<float>> slots;
  std::vector<float> scratch;       // im2col / transpose-conv taps, shared by all ops
  std::vector<float> pack_scratch;  // packed filter of a CONV_2D whose filter is not constant
  // Memory accounting (RM-1b): everything the interpreter allocates is charged against
  // kernels::kMaxTotalBytes before it is allocated (constants incl. folded copies and packed
  // weights, the largest scratch need, activations). The scratch buffers are allocated once, at
  // their charged size, at the end of Create.
  uint64_t used_bytes = 0;
  size_t const_bytes = 0;
  size_t packed_bytes = 0;
  size_t scratch_floats = 0;
  size_t pack_scratch_floats = 0;
  size_t activation_bytes = 0;
  bool profiling = false;
  std::map<std::string, double> profile;

  bool Fail(std::string* error, int op_index, const std::string& op_name,
            const std::string& what) {
    *error = "op " + std::to_string(op_index) + " (" + op_name + "): " + what;
    return false;
  }

  // Adds `bytes` to the memory budget; false (with `error`) if the total would exceed
  // kernels::kMaxTotalBytes. Called before the memory is allocated.
  bool Charge(uint64_t bytes, std::string* error) {
    used_bytes += bytes;
    if (used_bytes > kernels::kMaxTotalBytes) {
      *error = "memory of at least " + std::to_string(used_bytes) + " bytes exceeds the limit of " +
               std::to_string(kernels::kMaxTotalBytes) + " bytes";
      return false;
    }
    return true;
  }

  bool Build(std::string* error);
  bool PrepareOp(int op_index, const OperatorDef& op, std::string* error);
  bool PlanMemory(std::string* error);
  bool Run(const Plan& p, std::string* error);
};

namespace {

bool IsFloat(const Tensor& t) { return t.type == TensorType::kFloat32; }

bool FitsInt32(int64_t v) { return v >= INT32_MIN && v <= INT32_MAX; }
bool FitsParam(int64_t v) { return v >= 0 && v <= kernels::kMaxGeometryParam; }

// Every dimension >= 1 (convolutions, pooling, resize and the custom image ops do not define
// zero-sized tensors; requiring it also bounds their loops by the checked element counts).
bool AllPositive(const Shape& s) {
  for (int32_t d : s) {
    if (d < 1) return false;
  }
  return true;
}

bool ConstInts(const Tensor& t, std::vector<int64_t>* out) {
  if (!t.is_const) return false;
  out->clear();
  if (t.type == TensorType::kInt32) {
    out->assign(t.i32.begin(), t.i32.end());
    return true;
  }
  if (t.type == TensorType::kInt64) {
    out->assign(t.i64.begin(), t.i64.end());
    return true;
  }
  return false;
}

}  // namespace

bool Interpreter::Impl::Build(std::string* error) {
  if (model->subgraphs().size() != 1) {
    *error = "expected exactly one subgraph, got " + std::to_string(model->subgraphs().size());
    return false;
  }
  const SubgraphDef& sg = model->subgraphs().front();
  tensors.resize(sg.tensors.size());
  for (size_t i = 0; i < sg.tensors.size(); ++i) {
    const TensorDef& d = sg.tensors[i];
    Tensor& t = tensors[i];
    t.name = d.name;
    t.type = d.type;
    t.shape = d.shape;
    for (int32_t dim : d.shape) {
      if (dim < 0) {
        *error = "tensor " + std::to_string(i) + " (" + d.name + ") has a dynamic dimension";
        return false;
      }
    }
    if (!kernels::CheckedNumElements(d.shape, &t.count)) {
      *error = "tensor " + std::to_string(i) + " (" + d.name + ") shape " + ShapeStr(d.shape) +
               " has more than " + std::to_string(kernels::kMaxTensorElements) + " elements";
      return false;
    }
    if (d.is_variable || d.has_sparsity) {
      *error = "tensor " + std::to_string(i) + " is variable or sparse (unsupported)";
      return false;
    }
    if (d.has_quantization && !d.quantization.scale.empty()) {
      *error = "tensor " + std::to_string(i) + " (" + d.name + ") is quantized (unsupported)";
      return false;
    }
    const bool supported = d.type == TensorType::kFloat32 || d.type == TensorType::kFloat16 ||
                           d.type == TensorType::kInt32 || d.type == TensorType::kInt64;
    if (!supported) {
      *error = "tensor " + std::to_string(i) + " (" + d.name + ") has unsupported type " +
               TensorTypeName(d.type);
      return false;
    }
    if (d.data != nullptr) {
      t.is_const = true;
      t.defined = true;
      // Each tensor gets its own decoded copy, also when several tensors share one flatbuffer
      // buffer (a 4 MiB buffer referenced by 100 tensors costs 400 MiB): charge before copying.
      const uint64_t elem = d.type == TensorType::kInt64 ? sizeof(int64_t) : sizeof(float);
      if (!Charge(static_cast<uint64_t>(t.count) * elem, error)) return false;
      const_bytes += t.count * elem;
      switch (d.type) {
        case TensorType::kFloat32:
          t.f32_const.resize(t.count);
          std::memcpy(t.f32_const.data(), d.data, t.count * sizeof(float));
          t.f32 = t.f32_const.data();
          break;
        case TensorType::kFloat16: {
          t.f32_const.resize(t.count);
          for (size_t k = 0; k < t.count; ++k) {
            uint16_t h;
            std::memcpy(&h, d.data + 2 * k, sizeof(h));
            t.f32_const[k] = kernels::HalfToFloat(h);
          }
          t.f32 = t.f32_const.data();
          break;
        }
        case TensorType::kInt32:
          t.i32.resize(t.count);
          std::memcpy(t.i32.data(), d.data, t.count * sizeof(int32_t));
          break;
        case TensorType::kInt64:
          t.i64.resize(t.count);
          std::memcpy(t.i64.data(), d.data, t.count * sizeof(int64_t));
          break;
        default:
          break;
      }
    } else if (d.type == TensorType::kFloat16) {
      *error = "tensor " + std::to_string(i) + " (" + d.name + ") is a non-constant FLOAT16";
      return false;
    }
  }
  inputs = sg.inputs;
  outputs = sg.outputs;
  for (int idx : inputs) {
    Tensor& t = tensors[idx];
    if (t.is_const || !IsFloat(t)) {
      *error = "graph input " + t.name + " must be a non-constant FLOAT32 tensor";
      return false;
    }
    t.defined = true;
  }
  for (size_t oi = 0; oi < sg.ops.size(); ++oi) {
    if (!PrepareOp(static_cast<int>(oi), sg.ops[oi], error)) return false;
  }
  for (int idx : outputs) {
    if (!tensors[idx].defined) {
      *error = "graph output " + tensors[idx].name + " is never produced";
      return false;
    }
  }
  return PlanMemory(error);
}

bool Interpreter::Impl::PrepareOp(int op_index, const OperatorDef& op, std::string* error) {
  const std::string name = TfliteModel::OpName(op);
  auto fail = [&](const std::string& what) { return Fail(error, op_index, name, what); };
  for (int idx : op.inputs) {
    if (idx >= 0 && !tensors[idx].defined) {
      return fail("input tensor " + std::to_string(idx) + " is used before it is produced");
    }
  }
  for (int idx : op.outputs) {
    if (tensors[idx].defined) {
      return fail("output tensor " + std::to_string(idx) + " is constant or already produced");
    }
  }
  if (op.outputs.empty()) return fail("op has no output");
  if (!op.options.known) return fail("unsupported builtin options type");

  Plan p;
  p.op_index = op_index;
  p.name = name;
  for (int idx : op.inputs) p.in.push_back(idx);
  p.out = op.outputs;
  auto in = [&](size_t k) -> Tensor& { return tensors[op.inputs[k]]; };
  Tensor& out = tensors[op.outputs[0]];
  auto expect_inputs = [&](size_t lo, size_t hi) {
    return op.inputs.size() >= lo && op.inputs.size() <= hi;
  };
  auto present = [&](size_t k) { return k < op.inputs.size() && op.inputs[k] >= 0; };
  auto check_out = [&](const Shape& shape, TensorType type) {
    if (out.type != type) {
      return fail(std::string("output type ") + TensorTypeName(out.type) + ", expected " +
                  TensorTypeName(type));
    }
    if (out.shape != shape) {
      return fail("computed output shape " + ShapeStr(shape) + " != declared " +
                  ShapeStr(out.shape));
    }
    return true;
  };
  auto activation = [&](Activation a, kernels::ActivationRange* r) {
    if (!kernels::RangeFor(a, r)) return fail("unsupported fused activation");
    return true;
  };
  auto finish = [&]() {
    for (int idx : op.outputs) tensors[idx].defined = true;
    plans.push_back(std::move(p));
    return true;
  };
  if (op.outputs.size() != 1) return fail("expected one output");
  const BuiltinOptions& o = op.options;

  if (op.builtin_code == kOpCustom) {
    if (name == "TransformLandmarks") {
      if (op.version != 2) return fail("only version 2 is supported");
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& lm = in(0);
      const Tensor& m = in(1);
      if (!IsFloat(lm) || lm.shape.size() != 3 || lm.shape[0] != 1 || !IsFloat(m) ||
          m.count != 16) {
        return fail("expected FLOAT32 [1,N,D] landmarks and a 4x4 matrix");
      }
      // ParseTransformLandmarksV2Attributes: dimensions = output channels (= D).
      p.dims = lm.shape[2];
      if (p.dims != 2 && p.dims != 3) return fail("landmark dimensions must be 2 or 3");
      if (!check_out(lm.shape, TensorType::kFloat32)) return false;
      p.kind = Kind::kTransformLandmarks;
      return finish();
    }
    if (name == "Landmarks2TransformMatrix") {
      if (op.version != 2) return fail("only version 2 is supported");
      if (!expect_inputs(1, 1) || !present(0)) return fail("expected 1 input");
      const Tensor& lm = in(0);
      if (!IsFloat(lm) || lm.shape.size() != 3 || lm.shape[2] != 3) {
        return fail("expected FLOAT32 [1,N,3] landmarks");
      }
      std::string perr;
      if (!custom::ParseLandmarksToTransformMatrix(op.custom_options,
                                                   static_cast<int>(lm.count / 3), &p.l2t,
                                                   &perr)) {
        return fail(perr);
      }
      if (!check_out({1, 4, 4}, TensorType::kFloat32)) return false;
      p.kind = Kind::kLandmarksToTransformMatrix;
      return finish();
    }
    if (name == "TransformTensorBilinear") {
      if (op.version != 2) return fail("only version 2 is supported");
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& x = in(0);
      const Tensor& m = in(1);
      if (!IsFloat(x) || x.shape.size() != 4 || x.shape[0] != 1 || !IsFloat(m) ||
          m.count != 16) {
        return fail("expected FLOAT32 [1,H,W,C] input and a 4x4 matrix");
      }
      if (!AllPositive(x.shape)) return fail("zero-sized input dimension");
      std::string perr;
      if (!custom::ParseTransformTensorBilinear(op.custom_options, &p.out_h, &p.out_w, &perr)) {
        return fail(perr);
      }
      if (!check_out({1, p.out_h, p.out_w, x.shape[3]}, TensorType::kFloat32)) return false;
      p.kind = Kind::kTransformTensorBilinear;
      return finish();
    }
    if (name == "Convolution2DTransposeBias") {
      if (op.version != 1) return fail("only version 1 is supported");
      if (!expect_inputs(3, 3) || !present(0) || !present(1) || !present(2)) {
        return fail("expected 3 inputs");
      }
      const Tensor& x = in(0);
      const Tensor& w = in(1);
      const Tensor& b = in(2);
      if (!IsFloat(x) || !IsFloat(w) || !IsFloat(b) || x.shape.size() != 4 ||
          w.shape.size() != 4 || b.shape.size() != 1 || w.shape[3] != x.shape[3] ||
          b.shape[0] != w.shape[0]) {
        return fail("expected FLOAT32 input NHWC, weights OHWI, bias [O]");
      }
      bool same = false;
      int sw = 0;
      int sh = 0;
      std::string perr;
      if (!custom::ParseTransposeConvBias(op.custom_options, &same, &sw, &sh, &perr)) {
        return fail(perr);
      }
      if (!AllPositive(x.shape) || !AllPositive(w.shape)) return fail("zero-sized dimension");
      if (!FitsParam(sw) || !FitsParam(sh)) return fail("stride out of range");
      int oh = 0;
      int ow = 0;
      int ph = 0;
      int pw = 0;
      if (!custom::TransposeConvBiasOutputSize(x.shape[1], x.shape[2], w.shape[1], w.shape[2],
                                               sh, sw, same, &oh, &ow, &ph, &pw)) {
        return fail("output size out of range");
      }
      // The kernel keeps one [in_h*in_w, out_c] GEMM result.
      size_t taps = 0;
      if (!kernels::CheckedMul(static_cast<size_t>(x.shape[1]) * static_cast<size_t>(x.shape[2]),
                               static_cast<size_t>(w.shape[0]), kernels::kMaxTensorElements,
                               &taps)) {
        return fail("scratch buffer too large");
      }
      if (!check_out({x.shape[0], oh, ow, w.shape[0]}, TensorType::kFloat32)) return false;
      {
        // Scratch: the GEMM result of one tap and the packed filter of one tap.
        const size_t need = taps + gemm::PackedSize(w.shape[0], x.shape[3]);
        if (!Charge((need > scratch_floats ? need - scratch_floats : 0) * sizeof(float), error)) {
          return fail(*error);
        }
        scratch_floats = std::max(scratch_floats, need);
      }
      p.kind = Kind::kTransposeConvBias;
      p.geo.batch = x.shape[0];
      p.geo.in_h = x.shape[1];
      p.geo.in_w = x.shape[2];
      p.geo.in_c = x.shape[3];
      p.geo.out_h = oh;
      p.geo.out_w = ow;
      p.geo.out_c = w.shape[0];
      p.geo.kernel_h = w.shape[1];
      p.geo.kernel_w = w.shape[2];
      p.geo.stride_h = sh;
      p.geo.stride_w = sw;
      p.geo.pad_top = ph / 2;
      p.geo.pad_left = pw / 2;
      return finish();
    }
    return fail("unsupported custom op");
  }

  switch (op.builtin_code) {
    case kOpConv2D:
    case kOpDepthwiseConv2D: {
      const bool dw = op.builtin_code == kOpDepthwiseConv2D;
      if (o.type != (dw ? kOptDepthwiseConv2D : kOptConv2D)) return fail("missing options");
      if (!expect_inputs(2, 3) || !present(0) || !present(1)) return fail("expected 2-3 inputs");
      const Tensor& x = in(0);
      const Tensor& w = in(1);
      if (!IsFloat(x) || !IsFloat(w) || x.shape.size() != 4 || w.shape.size() != 4) {
        return fail("expected FLOAT32 4-D input and filter");
      }
      if (o.stride_w <= 0 || o.stride_h <= 0 || o.dilation_w <= 0 || o.dilation_h <= 0) {
        return fail("invalid stride or dilation");
      }
      if (!AllPositive(x.shape) || !AllPositive(w.shape)) return fail("zero-sized dimension");
      if (!FitsParam(o.stride_w) || !FitsParam(o.stride_h) || !FitsParam(o.dilation_w) ||
          !FitsParam(o.dilation_h) ||
          !FitsParam((static_cast<int64_t>(w.shape[1]) - 1) * o.dilation_h + 1) ||
          !FitsParam((static_cast<int64_t>(w.shape[2]) - 1) * o.dilation_w + 1)) {
        return fail("stride, dilation or effective filter size out of range");
      }
      int out_c = 0;
      if (dw) {
        if (w.shape[0] != 1 || x.shape[3] == 0 || w.shape[3] % x.shape[3] != 0) {
          return fail("depthwise filter must be [1,KH,KW,C*M]");
        }
        out_c = w.shape[3];
        p.depth_multiplier = w.shape[3] / x.shape[3];
      } else {
        if (w.shape[3] != x.shape[3]) return fail("filter input channels differ from the input");
        out_c = w.shape[0];
      }
      if (present(2)) {
        const Tensor& b = in(2);
        if (!IsFloat(b) || b.count != static_cast<size_t>(out_c)) return fail("bad bias");
      }
      if (!activation(o.activation, &p.act)) return false;
      kernels::ConvGeometry& g = p.geo;
      g.batch = x.shape[0];
      g.in_h = x.shape[1];
      g.in_w = x.shape[2];
      g.in_c = x.shape[3];
      g.kernel_h = w.shape[1];
      g.kernel_w = w.shape[2];
      g.stride_h = o.stride_h;
      g.stride_w = o.stride_w;
      g.dilation_h = o.dilation_h;
      g.dilation_w = o.dilation_w;
      g.out_c = out_c;
      g.out_h = kernels::ComputeOutSize(o.padding, g.in_h, g.kernel_h, g.stride_h, g.dilation_h);
      g.out_w = kernels::ComputeOutSize(o.padding, g.in_w, g.kernel_w, g.stride_w, g.dilation_w);
      if (g.out_h <= 0 || g.out_w <= 0) return fail("empty output");
      // The im2col buffer holds out_h*out_w rows of kernel_h*kernel_w*in_c values.
      size_t rows = 0;
      size_t k = 0;
      size_t col = 0;
      if (!dw) {
        if (!kernels::CheckedMul(static_cast<size_t>(g.out_h), static_cast<size_t>(g.out_w),
                                 kernels::kMaxTensorElements, &rows) ||
            !kernels::CheckedMul(static_cast<size_t>(g.kernel_h) * static_cast<size_t>(g.kernel_w),
                                 static_cast<size_t>(g.in_c), kernels::kMaxTensorElements, &k) ||
            !kernels::CheckedMul(rows, k, kernels::kMaxTensorElements, &col)) {
          return fail("im2col buffer too large");
        }
      }
      g.pad_top = kernels::ComputePadBefore(o.padding, g.in_h, g.out_h, g.kernel_h, g.stride_h,
                                            g.dilation_h);
      g.pad_left = kernels::ComputePadBefore(o.padding, g.in_w, g.out_w, g.kernel_w, g.stride_w,
                                             g.dilation_w);
      if (!check_out({g.batch, g.out_h, g.out_w, out_c}, TensorType::kFloat32)) return false;
      if (!dw) {
        // Scratch memory the kernel will need: the im2col buffer unless the convolution is a
        // plain matrix product (kernels::Conv2DIsPointwise), and the packed filter.
        if (!kernels::Conv2DIsPointwise(g)) {
          if (!Charge((col > scratch_floats ? col - scratch_floats : 0) * sizeof(float), error)) {
            return fail(*error);
          }
          scratch_floats = std::max(scratch_floats, col);
        }
        const size_t packed_floats = gemm::PackedSize(out_c, static_cast<int>(k));
        if (w.is_const) {
          if (!Charge(packed_floats * sizeof(float), error)) return fail(*error);
          packed_bytes += packed_floats * sizeof(float);
          p.packed.resize(packed_floats);
          gemm::PackWeights(w.f32, out_c, static_cast<int>(k), k, p.packed.data());
        } else {
          if (!Charge((packed_floats > pack_scratch_floats ? packed_floats - pack_scratch_floats : 0) *
                          sizeof(float),
                      error)) {
            return fail(*error);
          }
          pack_scratch_floats = std::max(pack_scratch_floats, packed_floats);
        }
      }
      p.kind = dw ? Kind::kDepthwiseConv2D : Kind::kConv2D;
      return finish();
    }
    case kOpMaxPool2D: {
      if (o.type != kOptPool2D) return fail("missing options");
      if (!expect_inputs(1, 1) || !present(0)) return fail("expected 1 input");
      const Tensor& x = in(0);
      if (!IsFloat(x) || x.shape.size() != 4) return fail("expected FLOAT32 4-D input");
      if (o.stride_w <= 0 || o.stride_h <= 0 || o.filter_w <= 0 || o.filter_h <= 0) {
        return fail("invalid stride or window");
      }
      if (!AllPositive(x.shape)) return fail("zero-sized dimension");
      if (!FitsParam(o.stride_w) || !FitsParam(o.stride_h) || !FitsParam(o.filter_w) ||
          !FitsParam(o.filter_h)) {
        return fail("stride or window out of range");
      }
      if (!activation(o.activation, &p.act)) return false;
      kernels::ConvGeometry& g = p.geo;
      g.batch = x.shape[0];
      g.in_h = x.shape[1];
      g.in_w = x.shape[2];
      g.in_c = x.shape[3];
      g.out_c = x.shape[3];
      g.kernel_h = o.filter_h;
      g.kernel_w = o.filter_w;
      g.stride_h = o.stride_h;
      g.stride_w = o.stride_w;
      g.out_h = kernels::ComputeOutSize(o.padding, g.in_h, g.kernel_h, g.stride_h, 1);
      g.out_w = kernels::ComputeOutSize(o.padding, g.in_w, g.kernel_w, g.stride_w, 1);
      if (g.out_h <= 0 || g.out_w <= 0) return fail("empty output");
      g.pad_top = kernels::ComputePadBefore(o.padding, g.in_h, g.out_h, g.kernel_h, g.stride_h, 1);
      g.pad_left = kernels::ComputePadBefore(o.padding, g.in_w, g.out_w, g.kernel_w, g.stride_w, 1);
      if (!check_out({g.batch, g.out_h, g.out_w, g.out_c}, TensorType::kFloat32)) return false;
      p.kind = Kind::kMaxPool2D;
      return finish();
    }
    case kOpAdd:
    case kOpSub:
    case kOpMul:
    case kOpMinimum:
    case kOpPrelu: {
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& a = in(0);
      const Tensor& b = in(1);
      if (!IsFloat(a) || !IsFloat(b)) return fail("expected FLOAT32 inputs");
      Shape shape;
      if (!kernels::BroadcastShape(a.shape, b.shape, &shape)) {
        return fail("shapes " + ShapeStr(a.shape) + " and " + ShapeStr(b.shape) +
                    " do not broadcast");
      }
      Activation act = Activation::kNone;
      if (op.builtin_code == kOpAdd || op.builtin_code == kOpSub || op.builtin_code == kOpMul) {
        if (o.type != kOptNone && o.type != kOptAdd && o.type != kOptSub && o.type != kOptMul) {
          return fail("unexpected options type");
        }
        act = o.activation;
      }
      if (!activation(act, &p.act)) return false;
      p.binary = op.builtin_code == kOpAdd   ? kernels::BinaryOp::kAdd
                 : op.builtin_code == kOpSub ? kernels::BinaryOp::kSub
                 : op.builtin_code == kOpMul ? kernels::BinaryOp::kMul
                 : op.builtin_code == kOpMinimum ? kernels::BinaryOp::kMinimum
                                                 : kernels::BinaryOp::kPrelu;
      if (!check_out(shape, TensorType::kFloat32)) return false;
      p.kind = Kind::kBinary;
      return finish();
    }
    case kOpFloorMod: {
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& a = in(0);
      const Tensor& b = in(1);
      if (a.type != b.type ||
          !(a.type == TensorType::kInt32 || a.type == TensorType::kInt64 || IsFloat(a))) {
        return fail("inputs must share an INT32, INT64 or FLOAT32 type");
      }
      Shape shape;
      if (!kernels::BroadcastShape(a.shape, b.shape, &shape)) return fail("shapes do not broadcast");
      if (b.is_const) {
        for (int32_t v : b.i32) {
          if (v == 0) return fail("constant divisor contains 0");
        }
        for (int64_t v : b.i64) {
          if (v == 0) return fail("constant divisor contains 0");
        }
      }
      if (!check_out(shape, a.type)) return false;
      p.kind = Kind::kFloorMod;
      return finish();
    }
    case kOpRelu:
    case kOpHardSwish:
    case kOpLogistic:
    case kOpLeakyRelu: {
      if (!expect_inputs(1, 1) || !present(0)) return fail("expected 1 input");
      const Tensor& x = in(0);
      if (!IsFloat(x)) return fail("expected FLOAT32 input");
      if (op.builtin_code == kOpLeakyRelu) {
        if (o.type != kOptLeakyRelu) return fail("missing options");
        p.alpha = o.alpha;
      }
      if (!check_out(x.shape, TensorType::kFloat32)) return false;
      p.kind = op.builtin_code == kOpRelu        ? Kind::kRelu
               : op.builtin_code == kOpHardSwish ? Kind::kHardSwish
               : op.builtin_code == kOpLogistic  ? Kind::kLogistic
                                                 : Kind::kLeakyRelu;
      return finish();
    }
    case kOpPad: {
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& x = in(0);
      std::vector<int64_t> pads;
      if (!IsFloat(x)) return fail("expected FLOAT32 input");
      if (!ConstInts(in(1), &pads) || pads.size() != 2 * x.shape.size() || x.shape.size() > 6) {
        return fail("paddings must be a constant [rank,2] integer tensor (rank <= 6)");
      }
      Shape shape = x.shape;
      for (size_t d = 0; d < x.shape.size(); ++d) {
        if (pads[2 * d] < 0 || pads[2 * d + 1] < 0) return fail("negative padding");
        // Padded extent in 64 bits; it must fit the int32 dimension (and thereby the checked
        // element count of the declared output) before anything is narrowed.
        const int64_t extent = static_cast<int64_t>(x.shape[d]) + pads[2 * d] + pads[2 * d + 1];
        if (pads[2 * d] > INT32_MAX || pads[2 * d + 1] > INT32_MAX || !FitsInt32(extent)) {
          return fail("padding out of the int32 range");
        }
        shape[d] = static_cast<int32_t>(extent);
        p.ints.push_back(static_cast<int32_t>(pads[2 * d]));
        p.ints.push_back(static_cast<int32_t>(pads[2 * d + 1]));
      }
      if (!check_out(shape, TensorType::kFloat32)) return false;
      p.kind = Kind::kPad;
      return finish();
    }
    case kOpReshape: {
      if (!expect_inputs(1, 2) || !present(0)) return fail("expected 1-2 inputs");
      const Tensor& x = in(0);
      if (!IsFloat(x)) return fail("expected FLOAT32 input");
      std::vector<int64_t> target;
      if (present(1) && in(1).count > 0) {
        if (!ConstInts(in(1), &target)) return fail("shape input must be a constant integer tensor");
      } else if (o.type == kOptReshape && o.has_new_shape) {
        target.assign(o.new_shape.begin(), o.new_shape.end());
      } else {
        return fail("no target shape");
      }
      Shape shape;
      int infer = -1;
      size_t known = 1;
      for (size_t d = 0; d < target.size(); ++d) {
        if (target[d] == -1) {
          if (infer >= 0) return fail("more than one -1 in the target shape");
          infer = static_cast<int>(d);
          shape.push_back(1);
        } else if (target[d] < 0) {
          return fail("negative target dimension");
        } else {
          if (target[d] > static_cast<int64_t>(kernels::kMaxTensorElements) ||
              !kernels::CheckedMul(known, static_cast<size_t>(target[d]),
                                   kernels::kMaxTensorElements, &known)) {
            return fail("target shape has too many elements");
          }
          shape.push_back(static_cast<int32_t>(target[d]));
        }
      }
      if (infer >= 0) {
        if (known == 0 || x.count % known != 0) return fail("cannot infer the -1 dimension");
        shape[infer] = static_cast<int32_t>(x.count / known);
      }
      size_t target_count = 0;
      if (!kernels::CheckedNumElements(shape, &target_count) || target_count != x.count) {
        return fail("element count changes");
      }
      if (!check_out(shape, TensorType::kFloat32)) return false;
      p.kind = Kind::kCopy;
      return finish();
    }
    case kOpConcatenation: {
      if (o.type != kOptConcatenation) return fail("missing options");
      if (op.inputs.empty()) return fail("no inputs");
      if (o.activation != Activation::kNone) return fail("fused activation is not supported");
      const Shape& s0 = in(0).shape;
      const int rank = static_cast<int>(s0.size());
      const int axis = o.axis < 0 ? o.axis + rank : o.axis;
      if (axis < 0 || axis >= rank) return fail("axis out of range");
      Shape shape = s0;
      int64_t axis_total = 0;
      for (size_t k = 0; k < op.inputs.size(); ++k) {
        if (!present(k) || !IsFloat(in(k)) || in(k).shape.size() != s0.size()) {
          return fail("inputs must be FLOAT32 of equal rank");
        }
        for (int d = 0; d < rank; ++d) {
          if (d != axis && in(k).shape[d] != s0[d]) return fail("non-axis dimensions differ");
        }
        axis_total += in(k).shape[axis];
        if (!FitsInt32(axis_total)) return fail("concatenated extent out of the int32 range");
      }
      shape[axis] = static_cast<int32_t>(axis_total);
      if (!check_out(shape, TensorType::kFloat32)) return false;
      p.axis = axis;
      p.kind = Kind::kConcatenation;
      return finish();
    }
    case kOpDequantize: {
      if (!expect_inputs(1, 1) || !present(0)) return fail("expected 1 input");
      const Tensor& x = in(0);
      if (x.type != TensorType::kFloat16 || !x.is_const) {
        return fail("only constant FLOAT16 inputs are supported");
      }
      if (!check_out(x.shape, TensorType::kFloat32)) return false;
      // Constant folding: fp16 -> fp32 is exact.
      if (!Charge(static_cast<uint64_t>(out.count) * sizeof(float), error)) return fail(*error);
      const_bytes += out.count * sizeof(float);
      out.f32_const = x.f32_const;
      out.f32 = out.f32_const.data();
      out.is_const = true;
      out.defined = true;
      ++folded;
      return true;
    }
    case kOpStridedSlice: {
      if (o.type != kOptStridedSlice) return fail("missing options");
      if (!expect_inputs(4, 4)) return fail("expected 4 inputs");
      const Tensor& x = in(0);
      if (!IsFloat(x) || x.shape.size() > 6) return fail("expected FLOAT32 input of rank <= 6");
      if (o.ellipsis_mask || o.new_axis_mask || o.offset) {
        return fail("ellipsis / new axis masks and offset are not supported");
      }
      std::vector<int64_t> b;
      std::vector<int64_t> e;
      std::vector<int64_t> s;
      if (!ConstInts(in(1), &b) || !ConstInts(in(2), &e) || !ConstInts(in(3), &s) ||
          b.size() != x.shape.size() || e.size() != x.shape.size() ||
          s.size() != x.shape.size()) {
        return fail("begin / end / strides must be constant integer vectors of the input rank");
      }
      for (int64_t v : s) {
        if (v == 0) return fail("zero stride");
      }
      // Narrowing int64 constants to int32 must not change their value (a stride of 2^32 would
      // become 0, begin / end of 2^32 + k would become k).
      for (size_t d = 0; d < b.size(); ++d) {
        if (!FitsInt32(b[d]) || !FitsInt32(e[d]) || !FitsInt32(s[d])) {
          return fail("begin / end / stride out of the int32 range");
        }
      }
      std::vector<int32_t> b32(b.begin(), b.end());
      std::vector<int32_t> e32(e.begin(), e.end());
      std::vector<int32_t> s32(s.begin(), s.end());
      p.slice = kernels::ResolveStridedSlice(x.shape, b32, e32, s32, o.begin_mask, o.end_mask,
                                             o.shrink_axis_mask);
      Shape shape;
      for (size_t d = 0; d < x.shape.size(); ++d) {
        if (!(o.shrink_axis_mask & (1 << d))) {
          shape.push_back(kernels::SliceLength(p.slice[d]));
        } else if (p.slice[d].start < 0 || p.slice[d].start >= x.shape[d] ||
                   kernels::SliceLength(p.slice[d]) != 1) {
          // The element of a shrunk axis must exist; begin == size would read one past the end
          // (found by the byte-flip test under ASan, slice_reverse_shrink).
          return fail("shrink axis index out of range");
        }
      }
      if (!check_out(shape, TensorType::kFloat32)) return false;
      p.kind = Kind::kStridedSlice;
      return finish();
    }
    case kOpResizeBilinear: {
      if (o.type != kOptResizeBilinear) return fail("missing options");
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& x = in(0);
      std::vector<int64_t> size;
      if (!IsFloat(x) || x.shape.size() != 4) return fail("expected FLOAT32 4-D input");
      if (!ConstInts(in(1), &size) || size.size() != 2 || size[0] <= 0 || size[1] <= 0) {
        return fail("size must be a constant positive [2] integer tensor");
      }
      if (!FitsParam(size[0]) || !FitsParam(size[1]) || !AllPositive(x.shape)) {
        return fail("size out of range or zero-sized input dimension");
      }
      if (o.align_corners && o.half_pixel_centers) {
        return fail("align_corners and half_pixel_centers are exclusive");
      }
      p.out_h = static_cast<int>(size[0]);
      p.out_w = static_cast<int>(size[1]);
      p.align_corners = o.align_corners;
      p.half_pixel_centers = o.half_pixel_centers;
      if (!check_out({x.shape[0], p.out_h, p.out_w, x.shape[3]}, TensorType::kFloat32)) {
        return false;
      }
      p.kind = Kind::kResizeBilinear;
      return finish();
    }
    case kOpMean:
    case kOpReduceMax: {
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& x = in(0);
      std::vector<int64_t> axes;
      if (!IsFloat(x) || x.shape.size() > 6) return fail("expected FLOAT32 input of rank <= 6");
      if (!ConstInts(in(1), &axes) || axes.empty()) return fail("axes must be constant integers");
      const int rank = static_cast<int>(x.shape.size());
      std::vector<bool> reduced(rank, false);
      for (int64_t a : axes) {
        const int64_t n = a < 0 ? a + rank : a;
        if (n < 0 || n >= rank) return fail("axis out of range");
        if (!reduced[n]) p.axes.push_back(static_cast<int>(n));
        reduced[n] = true;
      }
      const bool keep = o.type == kOptReducer && o.keep_dims;
      Shape shape;
      for (int d = 0; d < rank; ++d) {
        if (!reduced[d]) {
          shape.push_back(x.shape[d]);
        } else if (keep) {
          shape.push_back(1);
        }
      }
      if (!check_out(shape, TensorType::kFloat32)) return false;
      p.kind = op.builtin_code == kOpMean ? Kind::kMean : Kind::kReduceMax;
      return finish();
    }
    case kOpArgMax: {
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& x = in(0);
      std::vector<int64_t> axis;
      if (!IsFloat(x)) return fail("expected FLOAT32 input");
      if (!ConstInts(in(1), &axis) || axis.size() != 1) return fail("axis must be a constant scalar");
      const int rank = static_cast<int>(x.shape.size());
      const int64_t a = axis[0] < 0 ? axis[0] + rank : axis[0];
      if (a < 0 || a >= rank || x.shape[a] == 0) return fail("axis out of range");
      if (out.type != TensorType::kInt32 && out.type != TensorType::kInt64) {
        return fail("output must be INT32 or INT64");
      }
      if (o.type == kOptArgMax && o.output_type != out.type) {
        return fail("output_type option differs from the output tensor type");
      }
      Shape shape;
      for (int d = 0; d < rank; ++d) {
        if (d != a) shape.push_back(x.shape[d]);
      }
      if (!check_out(shape, out.type)) return false;
      p.axis = static_cast<int>(a);
      p.kind = Kind::kArgMax;
      return finish();
    }
    case kOpCast: {
      if (!expect_inputs(1, 1) || !present(0)) return fail("expected 1 input");
      const Tensor& x = in(0);
      auto ok_type = [](TensorType t) {
        return t == TensorType::kFloat32 || t == TensorType::kInt32 || t == TensorType::kInt64;
      };
      if (!ok_type(x.type) || !ok_type(out.type)) return fail("unsupported cast types");
      if (o.type == kOptCast && (o.cast_in_type != x.type || o.cast_out_type != out.type)) {
        return fail("cast options differ from the tensor types");
      }
      if (!check_out(x.shape, out.type)) return false;
      p.kind = Kind::kCast;
      return finish();
    }
    case kOpTranspose: {
      if (!expect_inputs(2, 2) || !present(0) || !present(1)) return fail("expected 2 inputs");
      const Tensor& x = in(0);
      std::vector<int64_t> perm;
      if (!IsFloat(x) || x.shape.size() > 6) return fail("expected FLOAT32 input of rank <= 6");
      if (!ConstInts(in(1), &perm) || perm.size() != x.shape.size()) {
        return fail("perm must be a constant integer vector of the input rank");
      }
      std::vector<bool> seen(perm.size(), false);
      Shape shape;
      for (int64_t v : perm) {
        if (v < 0 || v >= static_cast<int64_t>(perm.size()) || seen[v]) {
          return fail("perm is not a permutation");
        }
        seen[v] = true;
        shape.push_back(x.shape[v]);
        p.ints.push_back(static_cast<int32_t>(v));
      }
      if (!check_out(shape, TensorType::kFloat32)) return false;
      p.kind = Kind::kTranspose;
      return finish();
    }
    default:
      return fail("unsupported builtin op");
  }
}

bool Interpreter::Impl::PlanMemory(std::string* error) {
  // Lifetimes over plan indices.
  for (Tensor& t : tensors) {
    t.first_def = -1;
    t.last_use = -1;
  }
  for (size_t pi = 0; pi < plans.size(); ++pi) {
    for (int idx : plans[pi].in) {
      if (idx >= 0) tensors[idx].last_use = static_cast<int>(pi);
    }
    for (int idx : plans[pi].out) tensors[idx].first_def = static_cast<int>(pi);
  }
  for (int idx : outputs) tensors[idx].last_use = INT_MAX;
  // Graph inputs keep their buffers: callers fill them once and may Invoke() repeatedly.
  for (int idx : inputs) tensors[idx].last_use = INT_MAX;

  // Integer activations keep their own storage (few and small); counted against the arena limit
  // (review M4) before anything is allocated.
  uint64_t arena = 0;
  for (const Tensor& t : tensors) {
    if (t.is_const) continue;
    if (t.type == TensorType::kInt32) arena += static_cast<uint64_t>(t.count) * sizeof(int32_t);
    if (t.type == TensorType::kInt64) arena += static_cast<uint64_t>(t.count) * sizeof(int64_t);
  }
  if (!Charge(arena, error)) return false;
  activation_bytes += arena;
  for (Tensor& t : tensors) {
    if (t.is_const) continue;
    if (t.type == TensorType::kInt32) t.i32.assign(t.count, 0);
    if (t.type == TensorType::kInt64) t.i64.assign(t.count, 0);
  }
  // Float activations share slots: greedy best fit in execution order.
  std::vector<size_t> slot_size;
  std::vector<bool> slot_free;
  auto acquire = [&](Tensor& t) {
    int best = -1;
    for (size_t s = 0; s < slot_size.size(); ++s) {
      if (!slot_free[s] || slot_size[s] < t.count) continue;
      if (best < 0 || slot_size[s] < slot_size[best]) best = static_cast<int>(s);
    }
    if (best < 0) {
      // Grow the largest free slot, or open a new one.
      for (size_t s = 0; s < slot_size.size(); ++s) {
        if (slot_free[s] && (best < 0 || slot_size[s] > slot_size[best])) best = static_cast<int>(s);
      }
      if (best < 0) {
        slot_size.push_back(0);
        slot_free.push_back(true);
        best = static_cast<int>(slot_size.size()) - 1;
      }
      slot_size[best] = t.count;
    }
    slot_free[best] = false;
    t.slot = best;
  };
  auto is_float_activation = [](const Tensor& t) {
    return !t.is_const && t.type == TensorType::kFloat32;
  };
  for (int idx : inputs) acquire(tensors[idx]);
  for (size_t pi = 0; pi < plans.size(); ++pi) {
    for (int idx : plans[pi].out) {
      if (is_float_activation(tensors[idx])) acquire(tensors[idx]);
    }
    // Release tensors whose last consumer is this op (after the outputs took their slots).
    for (int idx : plans[pi].in) {
      if (idx < 0) continue;
      Tensor& t = tensors[idx];
      if (is_float_activation(t) && t.slot >= 0 && t.last_use == static_cast<int>(pi)) {
        slot_free[t.slot] = true;
      }
    }
    for (int idx : plans[pi].out) {
      Tensor& t = tensors[idx];
      if (is_float_activation(t) && t.last_use < 0) slot_free[t.slot] = true;  // never read
    }
  }
  uint64_t slot_bytes = 0;
  for (size_t sz : slot_size) slot_bytes += static_cast<uint64_t>(sz) * sizeof(float);
  if (!Charge(slot_bytes, error)) return false;
  activation_bytes += slot_bytes;
  slots.assign(slot_size.size(), std::vector<float>());
  for (size_t s = 0; s < slot_size.size(); ++s) slots[s].assign(slot_size[s], 0.0f);
  for (Tensor& t : tensors) {
    if (is_float_activation(t) && t.slot >= 0) t.f32 = slots[t.slot].data();
  }
  // Scratch buffers get their final size now, so Invoke never reallocates them (a growing vector
  // briefly holds the old and the new buffer, which the charged budget does not cover).
  scratch.assign(scratch_floats, 0.0f);
  pack_scratch.assign(pack_scratch_floats, 0.0f);
  return true;
}

bool Interpreter::Impl::Run(const Plan& p, std::string* error) {
  auto in = [&](size_t k) -> Tensor& { return tensors[p.in[k]]; };
  Tensor& out = tensors[p.out[0]];
  switch (p.kind) {
    case Kind::kConv2D: {
      const float* packed = p.packed.data();
      if (p.packed.empty()) {
        // The filter is not a constant: pack what the previous op produced.
        const int k = p.geo.kernel_h * p.geo.kernel_w * p.geo.in_c;
        pack_scratch.resize(gemm::PackedSize(p.geo.out_c, k));
        gemm::PackWeights(in(1).f32, p.geo.out_c, k, static_cast<size_t>(k), pack_scratch.data());
        packed = pack_scratch.data();
      }
      kernels::Conv2D(p.geo, in(0).f32, packed,
                      p.in.size() > 2 && p.in[2] >= 0 ? in(2).f32 : nullptr, p.act, out.f32,
                      &scratch);
      break;
    }
    case Kind::kDepthwiseConv2D:
      kernels::DepthwiseConv2D(p.geo, p.depth_multiplier, in(0).f32, in(1).f32,
                               p.in.size() > 2 && p.in[2] >= 0 ? in(2).f32 : nullptr, p.act,
                               out.f32);
      break;
    case Kind::kMaxPool2D:
      kernels::MaxPool2D(p.geo, p.act, in(0).f32, out.f32);
      break;
    case Kind::kBinary:
      kernels::BroadcastBinary(p.binary, in(0).f32, in(0).shape, in(1).f32, in(1).shape, out.f32,
                               out.shape, p.act);
      break;
    case Kind::kFloorMod:
      if (out.type == TensorType::kInt64 || out.type == TensorType::kInt32) {
        const bool ok =
            out.type == TensorType::kInt64
                ? kernels::BroadcastFloorMod(in(0).i64.data(), in(0).shape, in(1).i64.data(),
                                             in(1).shape, out.i64.data(), out.shape)
                : kernels::BroadcastFloorMod(in(0).i32.data(), in(0).shape, in(1).i32.data(),
                                             in(1).shape, out.i32.data(), out.shape);
        if (!ok) return Fail(error, p.op_index, p.name, "division by 0");
      } else {
        kernels::BroadcastFloorMod(in(0).f32, in(0).shape, in(1).f32, in(1).shape, out.f32,
                                   out.shape);
      }
      break;
    case Kind::kRelu:
      kernels::Relu(in(0).f32, out.count, out.f32);
      break;
    case Kind::kLeakyRelu:
      kernels::LeakyRelu(in(0).f32, out.count, p.alpha, out.f32);
      break;
    case Kind::kHardSwish:
      kernels::HardSwish(in(0).f32, out.count, out.f32);
      break;
    case Kind::kLogistic:
      kernels::Logistic(in(0).f32, out.count, out.f32);
      break;
    case Kind::kPad:
      kernels::Pad(in(0).f32, in(0).shape, p.ints, out.f32, out.shape);
      break;
    case Kind::kCopy:
      if (out.count != 0) std::memcpy(out.f32, in(0).f32, out.count * sizeof(float));
      break;
    case Kind::kConcatenation: {
      std::vector<const float*> ptrs;
      std::vector<Shape> shapes;
      for (size_t k = 0; k < p.in.size(); ++k) {
        ptrs.push_back(in(k).f32);
        shapes.push_back(in(k).shape);
      }
      kernels::Concatenation(ptrs, shapes, p.axis, out.f32);
      break;
    }
    case Kind::kStridedSlice:
      kernels::StridedSlice(in(0).f32, in(0).shape, p.slice, out.f32);
      break;
    case Kind::kResizeBilinear:
      kernels::ResizeBilinear(in(0).f32, in(0).shape, p.out_h, p.out_w, p.align_corners,
                              p.half_pixel_centers, out.f32);
      break;
    case Kind::kMean:
      kernels::ReduceMean(in(0).f32, in(0).shape, p.axes, out.f32, out.count);
      break;
    case Kind::kReduceMax:
      kernels::ReduceMax(in(0).f32, in(0).shape, p.axes, out.f32, out.count);
      break;
    case Kind::kArgMax:
      if (out.type == TensorType::kInt64) {
        kernels::ArgMax(in(0).f32, in(0).shape, p.axis, out.i64.data());
      } else {
        kernels::ArgMax(in(0).f32, in(0).shape, p.axis, out.i32.data());
      }
      break;
    case Kind::kCast: {
      const Tensor& x = in(0);
      for (size_t i = 0; i < out.count; ++i) {
        // Read as double (exact for float32 / int32; int64 beyond 2^53 is not produced by the
        // models), convert with C++ semantics (float -> int truncates toward zero).
        double v = 0.0;
        int64_t iv = 0;
        bool is_int = true;
        if (x.type == TensorType::kFloat32) {
          v = x.f32[i];
          is_int = false;
        } else if (x.type == TensorType::kInt32) {
          iv = x.i32[i];
        } else {
          iv = x.i64[i];
        }
        // float -> integer saturates and maps NaN to 0 (the plain conversion is undefined there);
        // integer -> integer narrows with the usual wrap.
        if (out.type == TensorType::kFloat32) {
          out.f32[i] = is_int ? static_cast<float>(iv) : static_cast<float>(v);
        } else if (out.type == TensorType::kInt32) {
          out.i32[i] = is_int ? static_cast<int32_t>(iv) : kernels::SaturatingFloatToInt32(x.f32[i]);
        } else {
          out.i64[i] = is_int ? iv : kernels::SaturatingFloatToInt64(x.f32[i]);
        }
      }
      break;
    }
    case Kind::kTranspose:
      kernels::Transpose(in(0).f32, in(0).shape, p.ints, out.f32);
      break;
    case Kind::kTransformLandmarks:
      custom::TransformLandmarks(in(0).f32, in(0).shape[1], p.dims, in(1).f32, out.f32);
      break;
    case Kind::kLandmarksToTransformMatrix:
      custom::LandmarksToTransformMatrix(p.l2t, in(0).f32, out.f32);
      break;
    case Kind::kTransformTensorBilinear:
      custom::TransformTensorBilinear(in(0).f32, in(0).shape[1], in(0).shape[2], in(0).shape[3],
                                      in(1).f32, p.out_h, p.out_w, out.f32);
      break;
    case Kind::kTransposeConvBias:
      custom::TransposeConvBias(in(0).f32, p.geo.batch, p.geo.in_h, p.geo.in_w, p.geo.in_c,
                                in(1).f32, p.geo.out_c, p.geo.kernel_h, p.geo.kernel_w,
                                in(2).f32, p.geo.stride_h, p.geo.stride_w, p.geo.pad_top,
                                p.geo.pad_left, p.geo.out_h, p.geo.out_w, out.f32, &scratch);
      break;
  }
  return true;
}

Interpreter::Interpreter(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Interpreter::~Interpreter() = default;

std::unique_ptr<Interpreter> Interpreter::Create(const TfliteModel& model, std::string* error) {
  std::unique_ptr<Impl> impl(new Impl());
  impl->model = &model;
  if (!impl->Build(error)) return nullptr;
  return std::unique_ptr<Interpreter>(new Interpreter(std::move(impl)));
}

size_t Interpreter::num_inputs() const { return impl_->inputs.size(); }
size_t Interpreter::num_outputs() const { return impl_->outputs.size(); }

float* Interpreter::input_data(size_t i) { return impl_->tensors[impl_->inputs[i]].f32; }

size_t Interpreter::input_count(size_t i) const {
  return impl_->tensors[impl_->inputs[i]].count;
}

const std::vector<int32_t>& Interpreter::input_shape(size_t i) const {
  return impl_->tensors[impl_->inputs[i]].shape;
}

TensorView Interpreter::tensor(int index) const {
  const Tensor& t = impl_->tensors[index];
  TensorView v;
  v.name = t.name;
  v.type = t.type == TensorType::kFloat16 ? TensorType::kFloat32 : t.type;
  v.shape = t.shape;
  v.count = t.count;
  v.f32 = t.f32;
  v.i32 = t.i32.empty() ? nullptr : t.i32.data();
  v.i64 = t.i64.empty() ? nullptr : t.i64.data();
  return v;
}

TensorView Interpreter::output(size_t i) const { return tensor(impl_->outputs[i]); }

bool Interpreter::Invoke(std::string* error) {
  // Create() rejects everything that depends on the model alone; the one failure left at run time
  // depends on data: an integer FLOOR_MOD whose (non-constant) divisor is 0.
  std::string ignored;
  if (error == nullptr) error = &ignored;
  for (const Plan& p : impl_->plans) {
    bool ok;
    if (impl_->profiling) {
      const auto t0 = std::chrono::steady_clock::now();
      ok = impl_->Run(p, error);
      const auto t1 = std::chrono::steady_clock::now();
      impl_->profile[p.name] += std::chrono::duration<double>(t1 - t0).count();
    } else {
      ok = impl_->Run(p, error);
    }
    if (!ok) return false;
  }
  return true;
}

size_t Interpreter::num_planned_ops() const { return impl_->plans.size(); }
size_t Interpreter::num_folded_ops() const { return impl_->folded; }

MemoryUse Interpreter::memory_use() const {
  MemoryUse u;
  u.constants = impl_->const_bytes;
  u.packed_weights = impl_->packed_bytes;
  u.scratch = (impl_->scratch_floats + impl_->pack_scratch_floats) * sizeof(float);
  u.activations = impl_->activation_bytes;
  return u;
}

size_t Interpreter::arena_bytes() const {
  size_t n = 0;
  for (const auto& s : impl_->slots) n += s.size() * sizeof(float);
  return n;
}

void Interpreter::set_profiling(bool enabled) { impl_->profiling = enabled; }

std::vector<std::pair<std::string, double>> Interpreter::profile() const {
  return std::vector<std::pair<std::string, double>>(impl_->profile.begin(),
                                                     impl_->profile.end());
}

void Interpreter::reset_profile() { impl_->profile.clear(); }

}  // namespace ml
}  // namespace slam_refact
