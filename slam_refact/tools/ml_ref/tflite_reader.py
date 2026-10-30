#!/usr/bin/env python3
"""TFLite model reader without the flatbuffers library (US-016, numpy reference side).

Reads the TFLite flatbuffer (schema: assets/references/xr_reference/official/tensorflow-fork/
tensorflow/lite/schema/schema.fbs) with struct only. The table accessor follows the scratchpad
inspector of the feasibility study (feas-C tflite_inspect.py) with bounds checks added. Custom op
options are FlexBuffers maps; flex_decode() reads the subset the models use.

This module is the numpy reference's own parser. The C++ engine parser
(slam_refact/engine/src/ml/tflite_model.cc) is written separately; the two are compared by the
op-case and model tests.
"""
import struct
from dataclasses import dataclass, field

import numpy as np

# TensorType (schema.fbs:34)
TENSOR_TYPES = {
    0: 'FLOAT32', 1: 'FLOAT16', 2: 'INT32', 3: 'UINT8', 4: 'INT64', 5: 'STRING', 6: 'BOOL',
    7: 'INT16', 8: 'COMPLEX64', 9: 'INT8', 10: 'FLOAT64', 11: 'COMPLEX128', 12: 'UINT64',
    13: 'RESOURCE', 14: 'VARIANT', 15: 'UINT32',
}
NUMPY_DTYPES = {
    'FLOAT32': np.float32, 'FLOAT16': np.float16, 'INT32': np.int32, 'UINT8': np.uint8,
    'INT64': np.int64, 'BOOL': np.bool_, 'INT16': np.int16, 'INT8': np.int8,
    'FLOAT64': np.float64,
}

# BuiltinOperator values used by the seven models (schema.fbs:227).
BUILTIN_OPS = {
    0: 'ADD', 2: 'CONCATENATION', 3: 'CONV_2D', 4: 'DEPTHWISE_CONV_2D', 6: 'DEQUANTIZE',
    14: 'LOGISTIC', 17: 'MAX_POOL_2D', 18: 'MUL', 19: 'RELU', 22: 'RESHAPE',
    23: 'RESIZE_BILINEAR', 32: 'CUSTOM', 34: 'PAD', 39: 'TRANSPOSE', 40: 'MEAN', 41: 'SUB',
    45: 'STRIDED_SLICE', 53: 'CAST', 54: 'PRELU', 56: 'ARG_MAX', 57: 'MINIMUM', 82: 'REDUCE_MAX',
    95: 'FLOOR_MOD', 98: 'LEAKY_RELU', 117: 'HARD_SWISH',
}
BUILTIN_CODES = {v: k for k, v in BUILTIN_OPS.items()}

# BuiltinOptions union member index (schema.fbs:383, 1-based; 0 = NONE).
OPTIONS_TYPES = {
    1: 'Conv2DOptions', 2: 'DepthwiseConv2DOptions', 5: 'Pool2DOptions',
    10: 'ConcatenationOptions', 11: 'AddOptions', 15: 'ResizeBilinearOptions',
    17: 'ReshapeOptions', 21: 'MulOptions', 22: 'PadOptions', 26: 'TransposeOptions',
    27: 'ReducerOptions', 28: 'SubOptions', 32: 'StridedSliceOptions', 37: 'CastOptions',
    38: 'DequantizeOptions', 39: 'MaximumMinimumOptions', 40: 'ArgMaxOptions',
    72: 'FloorModOptions', 75: 'LeakyReluOptions', 91: 'HardSwishOptions',
}
OPTIONS_CODES = {v: k for k, v in OPTIONS_TYPES.items()}

PADDING = {0: 'SAME', 1: 'VALID'}
ACTIVATIONS = {0: 'NONE', 1: 'RELU', 2: 'RELU_N1_TO_1', 3: 'RELU6', 4: 'TANH', 5: 'SIGN_BIT'}


class FormatError(ValueError):
  pass


class Table:
  """Flatbuffer table with bounds-checked field access."""

  def __init__(self, buf, pos):
    self.b = buf
    self.pos = pos
    self._need(pos, 4)
    self.vt = pos - struct.unpack_from('<i', buf, pos)[0]
    self._need(self.vt, 4)
    self.vtlen = struct.unpack_from('<H', buf, self.vt)[0]
    self._need(self.vt, self.vtlen)
    if self.vtlen < 4 or self.vtlen % 2:
      raise FormatError('bad vtable length %d at %d' % (self.vtlen, self.vt))

  def _need(self, pos, n):
    if pos < 0 or n < 0 or pos + n > len(self.b):
      raise FormatError('read [%d, +%d) outside buffer of %d bytes' % (pos, n, len(self.b)))

  def off(self, i):
    o = 4 + 2 * i
    if o >= self.vtlen:
      return 0
    return struct.unpack_from('<H', self.b, self.vt + o)[0]

  def scalar(self, i, fmt, default=0):
    o = self.off(i)
    if not o:
      return default
    self._need(self.pos + o, struct.calcsize(fmt))
    return struct.unpack_from(fmt, self.b, self.pos + o)[0]

  def ref(self, i):
    o = self.off(i)
    if not o:
      return None
    p = self.pos + o
    self._need(p, 4)
    return p + struct.unpack_from('<I', self.b, p)[0]

  def table(self, i):
    p = self.ref(i)
    return Table(self.b, p) if p is not None else None

  def string(self, i):
    p = self.ref(i)
    if p is None:
      return None
    self._need(p, 4)
    n = struct.unpack_from('<I', self.b, p)[0]
    self._need(p + 4, n)
    return self.b[p + 4:p + 4 + n].decode('utf-8', 'replace')

  def vec(self, i, elem_size):
    p = self.ref(i)
    if p is None:
      return None, 0
    self._need(p, 4)
    n = struct.unpack_from('<I', self.b, p)[0]
    self._need(p + 4, n * elem_size)
    return p + 4, n

  def vec_scalars(self, i, fmt):
    sz = struct.calcsize(fmt)
    p, n = self.vec(i, sz)
    if p is None:
      return []
    return list(struct.unpack_from('<%d%s' % (n, fmt.lstrip('<')), self.b, p))

  def vec_bytes(self, i):
    p, n = self.vec(i, 1)
    if p is None:
      return None
    return bytes(self.b[p:p + n])

  def vec_tables(self, i):
    p, n = self.vec(i, 4)
    if p is None:
      return []
    out = []
    for k in range(n):
      q = p + 4 * k
      out.append(Table(self.b, q + struct.unpack_from('<I', self.b, q)[0]))
    return out


# ---- FlexBuffers (custom op options) ------------------------------------------------------------
FBT_NULL, FBT_INT, FBT_UINT, FBT_FLOAT, FBT_KEY, FBT_STRING = 0, 1, 2, 3, 4, 5
FBT_INDIRECT_INT, FBT_INDIRECT_UINT, FBT_INDIRECT_FLOAT = 6, 7, 8
FBT_MAP, FBT_VECTOR = 9, 10
FBT_VECTOR_INT, FBT_VECTOR_KEY = 11, 14
FBT_VECTOR_INT2, FBT_VECTOR_FLOAT4 = 16, 24
FBT_BLOB, FBT_BOOL, FBT_VECTOR_BOOL = 25, 26, 36


def _flex_uint(b, p, w):
  if p < 0 or p + w > len(b):
    raise FormatError('flexbuffer read outside buffer')
  return int.from_bytes(b[p:p + w], 'little', signed=False)


def _flex_int(b, p, w):
  if p < 0 or p + w > len(b):
    raise FormatError('flexbuffer read outside buffer')
  return int.from_bytes(b[p:p + w], 'little', signed=True)


def _flex_float(b, p, w):
  if w == 4:
    return struct.unpack_from('<f', b, p)[0]
  if w == 8:
    return struct.unpack_from('<d', b, p)[0]
  raise FormatError('flexbuffer float width %d' % w)


def _flex_scalar(b, p, w, t):
  if t in (FBT_INT,):
    return _flex_int(b, p, w)
  if t in (FBT_UINT,):
    return _flex_uint(b, p, w)
  if t == FBT_BOOL:
    return _flex_uint(b, p, w) != 0
  if t == FBT_FLOAT:
    return _flex_float(b, p, w)
  if t == FBT_NULL:
    return None
  raise FormatError('not a scalar flexbuffer type %d' % t)


def _flex_value(b, p, parent_w, packed):
  t = packed >> 2
  w = 1 << (packed & 3)
  if t in (FBT_NULL, FBT_INT, FBT_UINT, FBT_FLOAT, FBT_BOOL):
    return _flex_scalar(b, p, parent_w, t)
  target = p - _flex_uint(b, p, parent_w)
  if t == FBT_INDIRECT_INT:
    return _flex_int(b, target, w)
  if t == FBT_INDIRECT_UINT:
    return _flex_uint(b, target, w)
  if t == FBT_INDIRECT_FLOAT:
    return _flex_float(b, target, w)
  if t == FBT_KEY:
    end = b.index(0, target)
    return b[target:end].decode('utf-8')
  if t in (FBT_STRING, FBT_BLOB):
    n = _flex_uint(b, target - w, w)
    raw = b[target:target + n]
    return raw.decode('utf-8') if t == FBT_STRING else bytes(raw)
  if t == FBT_MAP:
    n = _flex_uint(b, target - w, w)
    keys_at = target - 3 * w
    keys_vec = keys_at - _flex_uint(b, keys_at, w)
    keys_w = _flex_uint(b, target - 2 * w, w)
    out = {}
    for i in range(n):
      kp = keys_vec + i * keys_w
      ks = kp - _flex_uint(b, kp, keys_w)
      end = b.index(0, ks)
      key = b[ks:end].decode('utf-8')
      out[key] = _flex_value(b, target + i * w, w, b[target + n * w + i])
    return out
  if t == FBT_VECTOR:
    n = _flex_uint(b, target - w, w)
    return [_flex_value(b, target + i * w, w, b[target + n * w + i]) for i in range(n)]
  if FBT_VECTOR_INT <= t <= FBT_VECTOR_KEY or t == FBT_VECTOR_BOOL:
    n = _flex_uint(b, target - w, w)
    et = FBT_BOOL if t == FBT_VECTOR_BOOL else t - FBT_VECTOR_INT + FBT_INT
    if et == FBT_KEY:
      raise FormatError('key vectors are not used by the model options')
    return [_flex_scalar(b, target + i * w, w, et) for i in range(n)]
  if FBT_VECTOR_INT2 <= t <= FBT_VECTOR_FLOAT4:
    n = (t - FBT_VECTOR_INT2) // 3 + 2
    et = (t - FBT_VECTOR_INT2) % 3 + FBT_INT
    return [_flex_scalar(b, target + i * w, w, et) for i in range(n)]
  raise FormatError('unsupported flexbuffer type %d' % t)


def flex_decode(data):
  """Decodes a FlexBuffers root value (dict / list / scalar)."""
  b = bytes(data)
  if len(b) < 3:
    raise FormatError('flexbuffer too short')
  root_w = b[-1]
  packed = b[-2]
  return _flex_value(b, len(b) - 2 - root_w, root_w, packed)


# ---- Model ----------------------------------------------------------------------------------------
@dataclass
class TensorDef:
  index: int
  name: str
  shape: list
  dtype: str
  buffer: int
  data: object = None  # numpy array (constant) or None
  scale: list = field(default_factory=list)
  zero_point: list = field(default_factory=list)
  is_variable: bool = False
  has_sparsity: bool = False


@dataclass
class OpDef:
  index: int
  name: str            # builtin name or custom code
  builtin_code: int
  version: int
  inputs: list
  outputs: list
  options_type: int
  options: dict
  custom_options: bytes


@dataclass
class Model:
  path: str
  version: int
  description: str
  subgraph_name: str
  num_subgraphs: int
  tensors: list
  inputs: list
  outputs: list
  ops: list
  opcode_names: list


def _parse_options(op_name, ot, t):
  """Builtin option tables (field order from schema.fbs)."""
  if t is None:
    return {}
  name = OPTIONS_TYPES.get(ot)
  if name in ('Conv2DOptions',):
    return dict(padding=PADDING[t.scalar(0, '<b')], stride_w=t.scalar(1, '<i'),
                stride_h=t.scalar(2, '<i'), activation=ACTIVATIONS[t.scalar(3, '<b')],
                dilation_w=t.scalar(4, '<i', 1), dilation_h=t.scalar(5, '<i', 1))
  if name == 'DepthwiseConv2DOptions':
    return dict(padding=PADDING[t.scalar(0, '<b')], stride_w=t.scalar(1, '<i'),
                stride_h=t.scalar(2, '<i'), depth_multiplier=t.scalar(3, '<i'),
                activation=ACTIVATIONS[t.scalar(4, '<b')],
                dilation_w=t.scalar(5, '<i', 1), dilation_h=t.scalar(6, '<i', 1))
  if name == 'Pool2DOptions':
    return dict(padding=PADDING[t.scalar(0, '<b')], stride_w=t.scalar(1, '<i'),
                stride_h=t.scalar(2, '<i'), filter_w=t.scalar(3, '<i'),
                filter_h=t.scalar(4, '<i'), activation=ACTIVATIONS[t.scalar(5, '<b')])
  if name == 'ConcatenationOptions':
    return dict(axis=t.scalar(0, '<i'), activation=ACTIVATIONS[t.scalar(1, '<b')])
  if name in ('AddOptions', 'SubOptions', 'MulOptions'):
    return dict(activation=ACTIVATIONS[t.scalar(0, '<b')])
  if name == 'ResizeBilinearOptions':
    return dict(align_corners=bool(t.scalar(2, '<B')), half_pixel_centers=bool(t.scalar(3, '<B')))
  if name == 'ReshapeOptions':
    return dict(new_shape=t.vec_scalars(0, '<i') if t.ref(0) is not None else None)
  if name == 'ReducerOptions':
    return dict(keep_dims=bool(t.scalar(0, '<B')))
  if name == 'StridedSliceOptions':
    return dict(begin_mask=t.scalar(0, '<i'), end_mask=t.scalar(1, '<i'),
                ellipsis_mask=t.scalar(2, '<i'), new_axis_mask=t.scalar(3, '<i'),
                shrink_axis_mask=t.scalar(4, '<i'), offset=bool(t.scalar(5, '<B')))
  if name == 'CastOptions':
    return dict(in_type=TENSOR_TYPES[t.scalar(0, '<b')], out_type=TENSOR_TYPES[t.scalar(1, '<b')])
  if name == 'ArgMaxOptions':
    return dict(output_type=TENSOR_TYPES[t.scalar(0, '<b')])
  if name == 'LeakyReluOptions':
    return dict(alpha=t.scalar(0, '<f'))
  if name in ('PadOptions', 'TransposeOptions', 'DequantizeOptions', 'MaximumMinimumOptions',
              'FloorModOptions', 'HardSwishOptions'):
    return {}
  raise FormatError('op %s: unsupported builtin options type %d' % (op_name, ot))


def read_model(path_or_bytes):
  if isinstance(path_or_bytes, (bytes, bytearray)):
    b = bytes(path_or_bytes)
    path = '<bytes>'
  else:
    path = str(path_or_bytes)
    with open(path, 'rb') as f:
      b = f.read()
  if len(b) < 8:
    raise FormatError('file too short')
  if b[4:8] != b'TFL3':
    raise FormatError('file identifier %r != TFL3' % b[4:8])
  root = Table(b, struct.unpack_from('<I', b, 0)[0])
  opcode_names, opcode_codes, opcode_versions = [], [], []
  for oc in root.vec_tables(1):
    dep = oc.scalar(0, '<b')
    code = max(dep, oc.scalar(3, '<i'))
    custom = oc.string(1)
    opcode_codes.append(code)
    opcode_versions.append(oc.scalar(2, '<i', 1))
    if code == BUILTIN_CODES['CUSTOM']:
      opcode_names.append(custom)
    else:
      opcode_names.append(BUILTIN_OPS.get(code, 'BUILTIN_%d' % code))
  buffers = root.vec_tables(4)
  subgraphs = root.vec_tables(2)
  if not subgraphs:
    raise FormatError('no subgraph')
  sg = subgraphs[0]
  tensors = []
  for ti, t in enumerate(sg.vec_tables(0)):
    dtype = TENSOR_TYPES.get(t.scalar(1, '<b'), 'UNKNOWN')
    shape = t.vec_scalars(0, '<i')
    bi = t.scalar(2, '<I')
    data = None
    if bi >= len(buffers):
      raise FormatError('tensor %d buffer %d out of range' % (ti, bi))
    if bi:
      raw = buffers[bi].vec_bytes(0)
      if raw:
        np_t = NUMPY_DTYPES[dtype]
        count = int(np.prod(shape)) if shape else 1
        if len(raw) != count * np.dtype(np_t).itemsize:
          raise FormatError('tensor %d data %d bytes, shape %s %s' % (ti, len(raw), shape, dtype))
        data = np.frombuffer(raw, dtype=np.dtype(np_t).newbyteorder('<')).reshape(shape)
    q = t.table(4)
    scale, zp = [], []
    if q is not None:
      scale = q.vec_scalars(2, '<f')
      zp = q.vec_scalars(3, '<q')
    tensors.append(TensorDef(ti, t.string(3) or '', shape, dtype, bi, data, scale, zp,
                             bool(t.scalar(5, '<B')), t.ref(6) is not None))
  ops = []
  for oi, o in enumerate(sg.vec_tables(3)):
    idx = o.scalar(0, '<I')
    if idx >= len(opcode_names):
      raise FormatError('op %d opcode index %d out of range' % (oi, idx))
    name = opcode_names[idx]
    ot = o.scalar(3, '<B')
    options = _parse_options(name, ot, o.table(4) if ot else None)
    ops.append(OpDef(oi, name, opcode_codes[idx], opcode_versions[idx], o.vec_scalars(1, '<i'),
                     o.vec_scalars(2, '<i'), ot, options, o.vec_bytes(5) or b''))
  return Model(path, root.scalar(0, '<I'), root.string(3) or '', sg.string(4) or '',
               len(subgraphs), tensors, sg.vec_scalars(1, '<i'), sg.vec_scalars(2, '<i'), ops,
               opcode_names)
