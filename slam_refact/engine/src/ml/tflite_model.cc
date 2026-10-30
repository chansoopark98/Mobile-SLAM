#include "ml/tflite_model.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <utility>

namespace slam_refact {
namespace ml {
namespace {

// Bounds-checked flatbuffer access. The first failure is kept; later reads return defaults.
class Fb {
 public:
  Fb(const uint8_t* data, size_t size) : data_(data), size_(size) {}

  struct Table {
    size_t pos = 0;
    size_t vtable = 0;
    uint16_t vtable_size = 0;
    bool valid = false;
  };

  bool ok() const { return ok_; }
  const std::string& error() const { return error_; }

  void Fail(const std::string& what) {
    if (ok_) error_ = what;
    ok_ = false;
  }

  bool Has(size_t pos, size_t n) const { return pos <= size_ && n <= size_ - pos; }

  template <typename T>
  T Read(size_t pos) {
    T v{};
    if (!Has(pos, sizeof(T))) {
      Fail("read outside the file at offset " + std::to_string(pos));
      return v;
    }
    std::memcpy(&v, data_ + pos, sizeof(T));  // little-endian host (x86-64, wasm32)
    return v;
  }

  Table TableAt(size_t pos) {
    Table t;
    if (!ok_) return t;
    const int32_t soffset = Read<int32_t>(pos);
    if (!ok_) return t;
    const int64_t vt = static_cast<int64_t>(pos) - soffset;
    if (vt < 0 || !Has(static_cast<size_t>(vt), 4)) {
      Fail("vtable outside the file for table at " + std::to_string(pos));
      return t;
    }
    t.vtable = static_cast<size_t>(vt);
    t.vtable_size = Read<uint16_t>(t.vtable);
    if (!ok_ || t.vtable_size < 4 || (t.vtable_size & 1) || !Has(t.vtable, t.vtable_size)) {
      Fail("bad vtable at " + std::to_string(t.vtable));
      return t;
    }
    t.pos = pos;
    t.valid = true;
    return t;
  }

  // Offset of field `index` relative to the table, 0 if absent.
  size_t FieldOffset(const Table& t, int index) {
    if (!t.valid) return 0;
    const size_t o = 4 + 2 * static_cast<size_t>(index);
    if (o + 2 > t.vtable_size) return 0;
    return Read<uint16_t>(t.vtable + o);
  }

  template <typename T>
  T Scalar(const Table& t, int index, T def) {
    const size_t o = FieldOffset(t, index);
    if (o == 0) return def;
    return Read<T>(t.pos + o);
  }

  bool HasField(const Table& t, int index) { return FieldOffset(t, index) != 0; }

  // Position of the object referenced by field `index`; 0 (never a valid object) if absent.
  size_t Ref(const Table& t, int index) {
    const size_t o = FieldOffset(t, index);
    if (o == 0) return 0;
    const size_t at = t.pos + o;
    const uint32_t off = Read<uint32_t>(at);
    if (!ok_) return 0;
    if (off == 0 || !Has(at, static_cast<size_t>(off)) || at + off >= size_) {
      Fail("reference outside the file at " + std::to_string(at));
      return 0;
    }
    return at + off;
  }

  Table SubTable(const Table& t, int index) {
    const size_t p = Ref(t, index);
    if (p == 0) return Table();
    return TableAt(p);
  }

  std::string String(const Table& t, int index) {
    const size_t p = Ref(t, index);
    if (p == 0) return std::string();
    const uint32_t n = Read<uint32_t>(p);
    if (!ok_ || !Has(p + 4, n)) {
      Fail("string outside the file at " + std::to_string(p));
      return std::string();
    }
    return std::string(reinterpret_cast<const char*>(data_ + p + 4), n);
  }

  // Vector field: element start and count; false if absent. Checks count * elem_size in bounds.
  bool Vector(const Table& t, int index, size_t elem_size, size_t* start, size_t* count) {
    const size_t p = Ref(t, index);
    if (p == 0) return false;
    const uint32_t n = Read<uint32_t>(p);
    if (!ok_) return false;
    if (n > (size_ - p) / (elem_size == 0 ? 1 : elem_size) || !Has(p + 4, n * elem_size)) {
      Fail("vector outside the file at " + std::to_string(p));
      return false;
    }
    *start = p + 4;
    *count = n;
    return true;
  }

  template <typename T>
  std::vector<T> ScalarVector(const Table& t, int index, bool* present = nullptr) {
    std::vector<T> out;
    size_t start = 0;
    size_t count = 0;
    const bool has = Vector(t, index, sizeof(T), &start, &count);
    if (present != nullptr) *present = has;
    if (!has) return out;
    out.resize(count);
    if (count) std::memcpy(out.data(), data_ + start, count * sizeof(T));
    return out;
  }

  std::vector<Table> TableVector(const Table& t, int index) {
    std::vector<Table> out;
    size_t start = 0;
    size_t count = 0;
    if (!Vector(t, index, 4, &start, &count)) return out;
    out.reserve(count);
    for (size_t k = 0; k < count && ok_; ++k) {
      const size_t at = start + 4 * k;
      const uint32_t off = Read<uint32_t>(at);
      if (!ok_ || off == 0 || !Has(at, static_cast<size_t>(off)) || at + off >= size_) {
        Fail("table vector element outside the file at " + std::to_string(at));
        break;
      }
      out.push_back(TableAt(at + off));
    }
    return out;
  }

  const uint8_t* data() const { return data_; }

 private:
  const uint8_t* data_;
  size_t size_;
  bool ok_ = true;
  std::string error_;
};

bool ValidPadding(int8_t v) { return v == 0 || v == 1; }
bool ValidActivation(int8_t v) { return v >= 0 && v <= 5; }
bool ValidTensorType(int8_t v) { return v >= 0 && v <= 15; }

// Builtin option tables (field order from schema.fbs).
void ParseOptions(Fb* fb, const Fb::Table& t, uint8_t type, BuiltinOptions* o) {
  o->type = type;
  auto padding = [&](int i) {
    const int8_t v = fb->Scalar<int8_t>(t, i, 0);
    if (!ValidPadding(v)) fb->Fail("invalid padding " + std::to_string(v));
    return static_cast<Padding>(v);
  };
  auto activation = [&](int i) {
    const int8_t v = fb->Scalar<int8_t>(t, i, 0);
    if (!ValidActivation(v)) fb->Fail("invalid fused activation " + std::to_string(v));
    return static_cast<Activation>(v);
  };
  auto tensor_type = [&](int i, int8_t def) {
    const int8_t v = fb->Scalar<int8_t>(t, i, def);
    if (!ValidTensorType(v)) fb->Fail("invalid tensor type " + std::to_string(v));
    return static_cast<TensorType>(v);
  };
  switch (type) {
    case kOptConv2D:
      o->padding = padding(0);
      o->stride_w = fb->Scalar<int32_t>(t, 1, 0);
      o->stride_h = fb->Scalar<int32_t>(t, 2, 0);
      o->activation = activation(3);
      o->dilation_w = fb->Scalar<int32_t>(t, 4, 1);
      o->dilation_h = fb->Scalar<int32_t>(t, 5, 1);
      break;
    case kOptDepthwiseConv2D:
      o->padding = padding(0);
      o->stride_w = fb->Scalar<int32_t>(t, 1, 0);
      o->stride_h = fb->Scalar<int32_t>(t, 2, 0);
      o->depth_multiplier = fb->Scalar<int32_t>(t, 3, 0);
      o->activation = activation(4);
      o->dilation_w = fb->Scalar<int32_t>(t, 5, 1);
      o->dilation_h = fb->Scalar<int32_t>(t, 6, 1);
      break;
    case kOptPool2D:
      o->padding = padding(0);
      o->stride_w = fb->Scalar<int32_t>(t, 1, 0);
      o->stride_h = fb->Scalar<int32_t>(t, 2, 0);
      o->filter_w = fb->Scalar<int32_t>(t, 3, 0);
      o->filter_h = fb->Scalar<int32_t>(t, 4, 0);
      o->activation = activation(5);
      break;
    case kOptConcatenation:
      o->axis = fb->Scalar<int32_t>(t, 0, 0);
      o->activation = activation(1);
      break;
    case kOptAdd:
    case kOptSub:
    case kOptMul:
      o->activation = activation(0);
      break;
    case kOptResizeBilinear:
      o->align_corners = fb->Scalar<uint8_t>(t, 2, 0) != 0;
      o->half_pixel_centers = fb->Scalar<uint8_t>(t, 3, 0) != 0;
      break;
    case kOptReshape:
      o->new_shape = fb->ScalarVector<int32_t>(t, 0, &o->has_new_shape);
      break;
    case kOptReducer:
      o->keep_dims = fb->Scalar<uint8_t>(t, 0, 0) != 0;
      break;
    case kOptStridedSlice:
      o->begin_mask = fb->Scalar<int32_t>(t, 0, 0);
      o->end_mask = fb->Scalar<int32_t>(t, 1, 0);
      o->ellipsis_mask = fb->Scalar<int32_t>(t, 2, 0);
      o->new_axis_mask = fb->Scalar<int32_t>(t, 3, 0);
      o->shrink_axis_mask = fb->Scalar<int32_t>(t, 4, 0);
      o->offset = fb->Scalar<uint8_t>(t, 5, 0) != 0;
      break;
    case kOptCast:
      o->cast_in_type = tensor_type(0, 0);
      o->cast_out_type = tensor_type(1, 0);
      break;
    case kOptArgMax:
      o->output_type = tensor_type(0, 0);
      break;
    case kOptLeakyRelu:
      o->alpha = fb->Scalar<float>(t, 0, 0.0f);
      break;
    case kOptPad:
    case kOptTranspose:
    case kOptDequantize:
    case kOptMaximumMinimum:
    case kOptFloorMod:
    case kOptHardSwish:
      break;
    default:
      o->known = false;
      break;
  }
}

std::string JoinInts(const std::vector<int32_t>& v) {
  std::string s;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) s += ',';
    s += std::to_string(v[i]);
  }
  return s;
}

const char* PaddingName(Padding p) { return p == Padding::kSame ? "SAME" : "VALID"; }

const char* ActivationName(Activation a) {
  switch (a) {
    case Activation::kNone:
      return "NONE";
    case Activation::kRelu:
      return "RELU";
    case Activation::kReluN1To1:
      return "RELU_N1_TO_1";
    case Activation::kRelu6:
      return "RELU6";
    case Activation::kTanh:
      return "TANH";
    case Activation::kSignBit:
      return "SIGN_BIT";
  }
  return "UNKNOWN";
}

std::string CanonicalOptions(const OperatorDef& op) {
  const BuiltinOptions& o = op.options;
  const bool has = o.type != kOptNone;
  char buf[256];
  switch (op.builtin_code) {
    case kOpConv2D:
      std::snprintf(buf, sizeof(buf), "pad=%s sw=%d sh=%d dw=%d dh=%d act=%s",
                    PaddingName(o.padding), o.stride_w, o.stride_h, o.dilation_w, o.dilation_h,
                    ActivationName(o.activation));
      return buf;
    case kOpDepthwiseConv2D:
      std::snprintf(buf, sizeof(buf), "pad=%s sw=%d sh=%d dm=%d dw=%d dh=%d act=%s",
                    PaddingName(o.padding), o.stride_w, o.stride_h, o.depth_multiplier,
                    o.dilation_w, o.dilation_h, ActivationName(o.activation));
      return buf;
    case kOpMaxPool2D:
      std::snprintf(buf, sizeof(buf), "pad=%s sw=%d sh=%d fw=%d fh=%d act=%s",
                    PaddingName(o.padding), o.stride_w, o.stride_h, o.filter_w, o.filter_h,
                    ActivationName(o.activation));
      return buf;
    case kOpConcatenation:
      std::snprintf(buf, sizeof(buf), "axis=%d act=%s", o.axis, ActivationName(o.activation));
      return buf;
    case kOpAdd:
    case kOpSub:
    case kOpMul:
      return has ? std::string("act=") + ActivationName(o.activation) : std::string();
    case kOpResizeBilinear:
      std::snprintf(buf, sizeof(buf), "ac=%d hp=%d", o.align_corners ? 1 : 0,
                    o.half_pixel_centers ? 1 : 0);
      return buf;
    case kOpReshape:
      if (!has) return std::string();
      return o.has_new_shape ? "new_shape=[" + JoinInts(o.new_shape) + "]"
                             : std::string("new_shape=none");
    case kOpMean:
    case kOpReduceMax:
      return has ? std::string("keep=") + (o.keep_dims ? "1" : "0") : std::string();
    case kOpStridedSlice:
      std::snprintf(buf, sizeof(buf), "bm=%d em=%d elm=%d nam=%d sam=%d", o.begin_mask,
                    o.end_mask, o.ellipsis_mask, o.new_axis_mask, o.shrink_axis_mask);
      return buf;
    case kOpArgMax:
      return has ? std::string("out=") + TensorTypeName(o.output_type) : std::string();
    case kOpLeakyRelu: {
      uint32_t bits;
      std::memcpy(&bits, &o.alpha, sizeof(bits));
      std::snprintf(buf, sizeof(buf), "alpha=0x%08x", bits);
      return buf;
    }
    case kOpCast:
      return has ? std::string("in=") + TensorTypeName(o.cast_in_type) +
                       " out=" + TensorTypeName(o.cast_out_type)
                 : std::string();
    default:
      return std::string();
  }
}

std::string Hex(const std::vector<uint8_t>& v) {
  static const char kDigits[] = "0123456789abcdef";
  std::string s;
  s.reserve(v.size() * 2);
  for (uint8_t b : v) {
    s.push_back(kDigits[b >> 4]);
    s.push_back(kDigits[b & 15]);
  }
  return s;
}

}  // namespace

const char* TensorTypeName(TensorType type) {
  switch (type) {
    case TensorType::kFloat32:
      return "FLOAT32";
    case TensorType::kFloat16:
      return "FLOAT16";
    case TensorType::kInt32:
      return "INT32";
    case TensorType::kUint8:
      return "UINT8";
    case TensorType::kInt64:
      return "INT64";
    case TensorType::kString:
      return "STRING";
    case TensorType::kBool:
      return "BOOL";
    case TensorType::kInt16:
      return "INT16";
    case TensorType::kComplex64:
      return "COMPLEX64";
    case TensorType::kInt8:
      return "INT8";
    case TensorType::kFloat64:
      return "FLOAT64";
    case TensorType::kComplex128:
      return "COMPLEX128";
    case TensorType::kUint64:
      return "UINT64";
    case TensorType::kResource:
      return "RESOURCE";
    case TensorType::kVariant:
      return "VARIANT";
    case TensorType::kUint32:
      return "UINT32";
  }
  return "UNKNOWN";
}

size_t TensorTypeSize(TensorType type) {
  switch (type) {
    case TensorType::kFloat32:
    case TensorType::kInt32:
    case TensorType::kUint32:
      return 4;
    case TensorType::kFloat16:
    case TensorType::kInt16:
      return 2;
    case TensorType::kUint8:
    case TensorType::kInt8:
    case TensorType::kBool:
      return 1;
    case TensorType::kInt64:
    case TensorType::kFloat64:
    case TensorType::kUint64:
    case TensorType::kComplex64:
      return 8;
    case TensorType::kComplex128:
      return 16;
    case TensorType::kString:
    case TensorType::kResource:
    case TensorType::kVariant:
      return 0;
  }
  return 0;
}

std::string BuiltinOpName(int32_t code) {
  switch (code) {
    case kOpAdd:
      return "ADD";
    case kOpConcatenation:
      return "CONCATENATION";
    case kOpConv2D:
      return "CONV_2D";
    case kOpDepthwiseConv2D:
      return "DEPTHWISE_CONV_2D";
    case kOpDequantize:
      return "DEQUANTIZE";
    case kOpLogistic:
      return "LOGISTIC";
    case kOpMaxPool2D:
      return "MAX_POOL_2D";
    case kOpMul:
      return "MUL";
    case kOpRelu:
      return "RELU";
    case kOpReshape:
      return "RESHAPE";
    case kOpResizeBilinear:
      return "RESIZE_BILINEAR";
    case kOpCustom:
      return "CUSTOM";
    case kOpPad:
      return "PAD";
    case kOpTranspose:
      return "TRANSPOSE";
    case kOpMean:
      return "MEAN";
    case kOpSub:
      return "SUB";
    case kOpStridedSlice:
      return "STRIDED_SLICE";
    case kOpCast:
      return "CAST";
    case kOpPrelu:
      return "PRELU";
    case kOpArgMax:
      return "ARG_MAX";
    case kOpMinimum:
      return "MINIMUM";
    case kOpReduceMax:
      return "REDUCE_MAX";
    case kOpFloorMod:
      return "FLOOR_MOD";
    case kOpLeakyRelu:
      return "LEAKY_RELU";
    case kOpHardSwish:
      return "HARD_SWISH";
    default:
      return "BUILTIN_" + std::to_string(code);
  }
}

std::string TfliteModel::OpName(const OperatorDef& op) {
  return op.builtin_code == kOpCustom ? op.custom_code : BuiltinOpName(op.builtin_code);
}

std::unique_ptr<TfliteModel> TfliteModel::FromBytes(std::vector<uint8_t> bytes,
                                                    std::string* error) {
  std::unique_ptr<TfliteModel> model(new TfliteModel());
  model->bytes_ = std::move(bytes);
  if (!model->Parse(error)) return nullptr;
  return model;
}

std::unique_ptr<TfliteModel> TfliteModel::FromFile(const std::string& path, std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *error = "cannot open " + path;
    return nullptr;
  }
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
  return FromBytes(std::move(bytes), error);
}

bool TfliteModel::Parse(std::string* error) {
  const size_t size = bytes_.size();
  if (size < 8) {
    *error = "file shorter than 8 bytes";
    return false;
  }
  if (std::memcmp(bytes_.data() + 4, "TFL3", 4) != 0) {
    *error = "file identifier is not TFL3";
    return false;
  }
  Fb fb(bytes_.data(), size);
  const uint32_t root_off = fb.Read<uint32_t>(0);
  if (root_off < 8 || root_off >= size) {
    *error = "root table offset outside the file";
    return false;
  }
  const Fb::Table root = fb.TableAt(root_off);
  version_ = fb.Scalar<uint32_t>(root, 0, 0);
  description_ = fb.String(root, 3);

  // Operator codes.
  struct OpCode {
    int32_t code;
    std::string custom;
    int32_t version;
  };
  std::vector<OpCode> opcodes;
  for (const Fb::Table& oc : fb.TableVector(root, 1)) {
    const int8_t deprecated = fb.Scalar<int8_t>(oc, 0, 0);
    const int32_t builtin = fb.Scalar<int32_t>(oc, 3, 0);
    // Schema 3a: builtin_code added; readers take the larger of the two fields.
    opcodes.push_back({deprecated > builtin ? deprecated : builtin, fb.String(oc, 1),
                       fb.Scalar<int32_t>(oc, 2, 1)});
  }

  const std::vector<Fb::Table> buffers = fb.TableVector(root, 4);
  num_buffers_ = buffers.size();

  for (const Fb::Table& sg : fb.TableVector(root, 2)) {
    if (!fb.ok()) break;
    SubgraphDef sub;
    sub.name = fb.String(sg, 4);
    const std::vector<Fb::Table> tensors = fb.TableVector(sg, 0);
    for (size_t ti = 0; ti < tensors.size() && fb.ok(); ++ti) {
      const Fb::Table& t = tensors[ti];
      TensorDef td;
      td.shape = fb.ScalarVector<int32_t>(t, 0);
      const int8_t type = fb.Scalar<int8_t>(t, 1, 0);
      if (!ValidTensorType(type)) {
        fb.Fail("tensor " + std::to_string(ti) + " has invalid type " + std::to_string(type));
        break;
      }
      td.type = static_cast<TensorType>(type);
      td.buffer = fb.Scalar<uint32_t>(t, 2, 0);
      td.name = fb.String(t, 3);
      const Fb::Table q = fb.SubTable(t, 4);
      if (q.valid) {
        td.has_quantization = true;
        td.quantization.min = fb.ScalarVector<float>(q, 0);
        td.quantization.max = fb.ScalarVector<float>(q, 1);
        td.quantization.scale = fb.ScalarVector<float>(q, 2);
        td.quantization.zero_point = fb.ScalarVector<int64_t>(q, 3);
        td.quantization.quantized_dimension = fb.Scalar<int32_t>(q, 6, 0);
      }
      td.is_variable = fb.Scalar<uint8_t>(t, 5, 0) != 0;
      td.has_sparsity = fb.HasField(t, 6);
      if (td.buffer >= buffers.size()) {
        fb.Fail("tensor " + std::to_string(ti) + " buffer index " + std::to_string(td.buffer) +
                " out of range");
        break;
      }
      if (td.buffer != 0) {
        size_t start = 0;
        size_t count = 0;
        if (fb.Vector(buffers[td.buffer], 0, 1, &start, &count) && count > 0) {
          size_t elems = 1;
          for (int32_t d : td.shape) {
            if (d < 0 || (d > 0 && elems > SIZE_MAX / static_cast<size_t>(d))) {
              elems = SIZE_MAX;
              break;
            }
            elems *= static_cast<size_t>(d);
          }
          const size_t esize = TensorTypeSize(td.type);
          if (esize == 0 || elems == SIZE_MAX || elems * esize != count) {
            fb.Fail("tensor " + std::to_string(ti) + " (" + td.name + ") data size " +
                    std::to_string(count) + " does not match its shape and type");
            break;
          }
          td.data = fb.data() + start;
          td.data_size = count;
        }
      }
      sub.tensors.push_back(std::move(td));
    }
    sub.inputs = fb.ScalarVector<int32_t>(sg, 1);
    sub.outputs = fb.ScalarVector<int32_t>(sg, 2);
    const std::vector<Fb::Table> ops = fb.TableVector(sg, 3);
    for (size_t oi = 0; oi < ops.size() && fb.ok(); ++oi) {
      const Fb::Table& o = ops[oi];
      OperatorDef od;
      od.opcode_index = fb.Scalar<uint32_t>(o, 0, 0);
      if (od.opcode_index >= opcodes.size()) {
        fb.Fail("op " + std::to_string(oi) + " opcode index out of range");
        break;
      }
      const OpCode& code = opcodes[od.opcode_index];
      od.builtin_code = code.code;
      od.custom_code = code.custom;
      od.version = code.version;
      od.inputs = fb.ScalarVector<int32_t>(o, 1);
      od.outputs = fb.ScalarVector<int32_t>(o, 2);
      const uint8_t opt_type = fb.Scalar<uint8_t>(o, 3, 0);
      if (opt_type != kOptNone) {
        const Fb::Table opts = fb.SubTable(o, 4);
        if (!opts.valid) {
          fb.Fail("op " + std::to_string(oi) + " options type set without an options table");
          break;
        }
        ParseOptions(&fb, opts, opt_type, &od.options);
      }
      od.custom_options = fb.ScalarVector<uint8_t>(o, 5);
      od.custom_options_format = fb.Scalar<int8_t>(o, 6, 0);
      for (int32_t idx : od.inputs) {
        if (idx < -1 || idx >= static_cast<int32_t>(sub.tensors.size())) {
          fb.Fail("op " + std::to_string(oi) + " input index out of range");
        }
      }
      for (int32_t idx : od.outputs) {
        if (idx < 0 || idx >= static_cast<int32_t>(sub.tensors.size())) {
          fb.Fail("op " + std::to_string(oi) + " output index out of range");
        }
      }
      sub.ops.push_back(std::move(od));
    }
    for (int32_t idx : sub.inputs) {
      if (idx < 0 || idx >= static_cast<int32_t>(sub.tensors.size())) {
        fb.Fail("subgraph input index out of range");
      }
    }
    for (int32_t idx : sub.outputs) {
      if (idx < 0 || idx >= static_cast<int32_t>(sub.tensors.size())) {
        fb.Fail("subgraph output index out of range");
      }
    }
    subgraphs_.push_back(std::move(sub));
  }
  if (!fb.ok()) {
    *error = fb.error();
    return false;
  }
  if (subgraphs_.empty()) {
    *error = "model has no subgraph";
    return false;
  }
  return true;
}

std::string CanonicalGraphText(const TfliteModel& model) {
  const SubgraphDef& sg = model.subgraphs().front();
  std::string out = "model version=" + std::to_string(model.version()) +
                    " subgraphs=" + std::to_string(model.subgraphs().size()) + " name=" + sg.name +
                    "\n";
  for (size_t i = 0; i < sg.tensors.size(); ++i) {
    const TensorDef& t = sg.tensors[i];
    out += "tensor " + std::to_string(i) + " " + t.name + " " + TensorTypeName(t.type) + " [" +
           JoinInts(t.shape) + "] bytes=" + std::to_string(t.data_size) +
           " quant=" + std::to_string(t.quantization.scale.size()) +
           " var=" + (t.is_variable ? "1" : "0") + " sparse=" + (t.has_sparsity ? "1" : "0") +
           "\n";
  }
  for (size_t i = 0; i < sg.ops.size(); ++i) {
    const OperatorDef& op = sg.ops[i];
    out += "op " + std::to_string(i) + " " + TfliteModel::OpName(op) + " v" +
           std::to_string(op.version) + " in=[" + JoinInts(op.inputs) + "] out=[" +
           JoinInts(op.outputs) + "] opts={" + CanonicalOptions(op) +
           "} custom=" + Hex(op.custom_options) + "\n";
  }
  out += "inputs [" + JoinInts(sg.inputs) + "]\n";
  out += "outputs [" + JoinInts(sg.outputs) + "]\n";
  return out;
}

}  // namespace ml
}  // namespace slam_refact
