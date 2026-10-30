#!/usr/bin/env python3
"""numpy reference interpreter for the TFLite subset of the seven reference runtime / MediaPipe models.

Independent of the C++ engine (slam_refact/engine/src/ml/). Float tensors are computed in float64
(weights and inputs are their float32 / fp16 values promoted to float64), integer tensors keep their
TFLite type. Semantics: TFLite reference kernels (tensorflow-fork 2.6
tensorflow/lite/kernels/internal/reference/) and the custom op sources
(reality/engine/deepnets/operations/*.cc, mediapipe/util/tflite/operations/transpose_conv_bias.cc).
TransformTensorBilinear computes its sample coordinates in float32 in the original order because its
out-of-bound test is discontinuous (docs/design/ml-runtime.md §4).
"""
import math

import numpy as np

from tflite_reader import flex_decode

FLOAT_TYPES = ('FLOAT32', 'FLOAT16', 'FLOAT64')
INT_DTYPES = {'INT32': np.int32, 'INT64': np.int64}


# float32 lowest(): the initial value of TFLite's MAX_POOL_2D and REDUCE_MAX (reference/pooling.h
# line 223, reduce.cc kMax). The reference keeps float64 values, so this is the float32 number
# widened.
FLOAT32_LOWEST = float(-np.finfo(np.float32).max)


class RefError(RuntimeError):
  pass


def _saturating_cast(v, dtype):
  """float -> integer: truncate, saturate at the type's range, NaN -> 0 (engine CAST)."""
  info = np.iinfo(dtype)
  v = np.trunc(np.asarray(v, dtype=np.float64))
  out = np.zeros(v.shape, dtype=dtype)
  finite = ~np.isnan(v)
  low = v <= float(info.min)
  high = v >= float(info.max) + 1.0
  inside = finite & ~low & ~high
  out[inside] = v[inside].astype(dtype)
  out[finite & low] = info.min
  out[finite & high] = info.max
  return out


def _arg_max_tflite(x, axis):
  """reference/arg_min_max.h: index of the first element that is strictly greater than every
  element before it (std::greater). NaN is never greater and nothing is greater than a NaN at
  index 0, unlike numpy.argmax which treats the first NaN as the maximum."""
  x = np.moveaxis(x, axis, 0)
  best = x[0].copy()
  idx = np.zeros(best.shape, dtype=np.int64)
  for i in range(1, x.shape[0]):
    gt = x[i] > best
    best = np.where(gt, x[i], best)
    idx = np.where(gt, i, idx)
  return idx


def _act(x, name):
  if name == 'NONE':
    return x
  if name == 'RELU':
    return np.maximum(x, 0.0)
  if name == 'RELU6':
    return np.minimum(np.maximum(x, 0.0), 6.0)
  if name == 'RELU_N1_TO_1':
    return np.minimum(np.maximum(x, -1.0), 1.0)
  raise RefError('unsupported fused activation %s' % name)


def _out_size(padding, in_size, k, stride, dil):
  eff = (k - 1) * dil + 1
  if padding == 'SAME':
    return (in_size + stride - 1) // stride
  return (in_size - eff + stride) // stride


def _pad_before(padding, in_size, out_size, k, stride, dil):
  if padding != 'SAME':
    return 0
  eff = (k - 1) * dil + 1
  total = max((out_size - 1) * stride + eff - in_size, 0)
  return total // 2


def conv2d(x, w, b, o):
  n, h, wd, c = x.shape
  oc, kh, kw, ic = w.shape
  if ic != c:
    raise RefError('conv channel mismatch %d vs %d' % (ic, c))
  sh, sw, dh, dw = o['stride_h'], o['stride_w'], o['dilation_h'], o['dilation_w']
  oh = _out_size(o['padding'], h, kh, sh, dh)
  ow = _out_size(o['padding'], wd, kw, sw, dw)
  pt = _pad_before(o['padding'], h, oh, kh, sh, dh)
  pl = _pad_before(o['padding'], wd, ow, kw, sw, dw)
  # Zero canvas large enough for every tap of every output position.
  need_h = (oh - 1) * sh + (kh - 1) * dh + 1
  need_w = (ow - 1) * sw + (kw - 1) * dw + 1
  canvas = np.zeros((n, max(need_h, pt + h), max(need_w, pl + wd), c), dtype=np.float64)
  canvas[:, pt:pt + h, pl:pl + wd, :] = x
  out = np.zeros((n, oh, ow, oc), dtype=np.float64)
  for ky in range(kh):
    for kx in range(kw):
      y0, x0 = ky * dh, kx * dw
      patch = canvas[:, y0:y0 + (oh - 1) * sh + 1:sh, x0:x0 + (ow - 1) * sw + 1:sw, :]
      out += np.tensordot(patch, w[:, ky, kx, :], axes=([3], [1]))
  if b is not None:
    out += b
  return _act(out, o['activation'])


def depthwise_conv2d(x, w, b, o):
  n, h, wd, c = x.shape
  _, kh, kw, oc = w.shape
  if oc % c:
    raise RefError('depthwise output channels %d not a multiple of %d' % (oc, c))
  dm = oc // c
  sh, sw, dh, dw = o['stride_h'], o['stride_w'], o['dilation_h'], o['dilation_w']
  oh = _out_size(o['padding'], h, kh, sh, dh)
  ow = _out_size(o['padding'], wd, kw, sw, dw)
  pt = _pad_before(o['padding'], h, oh, kh, sh, dh)
  pl = _pad_before(o['padding'], wd, ow, kw, sw, dw)
  xr = np.repeat(x, dm, axis=3)  # output channel c*dm + m reads input channel c
  need_h = (oh - 1) * sh + (kh - 1) * dh + 1
  need_w = (ow - 1) * sw + (kw - 1) * dw + 1
  canvas = np.zeros((n, max(need_h, pt + h), max(need_w, pl + wd), oc), dtype=np.float64)
  canvas[:, pt:pt + h, pl:pl + wd, :] = xr
  out = np.zeros((n, oh, ow, oc), dtype=np.float64)
  for ky in range(kh):
    for kx in range(kw):
      y0, x0 = ky * dh, kx * dw
      out += canvas[:, y0:y0 + (oh - 1) * sh + 1:sh, x0:x0 + (ow - 1) * sw + 1:sw, :] * w[0, ky, kx]
  if b is not None:
    out += b
  return _act(out, o['activation'])


def transpose_conv_bias(x, w, b, padding_same, sw, sh):
  """mediapipe transpose_conv_bias.cc (OHWI weights, padding = total // 2)."""
  n, h, wd, c = x.shape
  oc, kh, kw, ic = w.shape
  if ic != c:
    raise RefError('transpose conv channel mismatch')
  ph = max(0, kh - (h - 1) % sh - 1) if padding_same else 0
  pw = max(0, kw - (wd - 1) % sw - 1) if padding_same else 0
  oh = sh * (h - 1) + kh - ph
  ow = sw * (wd - 1) + kw - pw
  top, left = ph // 2, pw // 2
  canvas = np.zeros((n, sh * (h - 1) + kh, sw * (wd - 1) + kw, oc), dtype=np.float64)
  for ky in range(kh):
    for kx in range(kw):
      contrib = np.tensordot(x, w[:, ky, kx, :], axes=([3], [1]))  # n, h, w, oc
      canvas[:, ky:ky + (h - 1) * sh + 1:sh, kx:kx + (wd - 1) * sw + 1:sw, :] += contrib
  out = canvas[:, top:top + oh, left:left + ow, :] + b
  return out


def max_pool(x, o):
  n, h, wd, c = x.shape
  kh, kw, sh, sw = o['filter_h'], o['filter_w'], o['stride_h'], o['stride_w']
  oh = _out_size(o['padding'], h, kh, sh, 1)
  ow = _out_size(o['padding'], wd, kw, sw, 1)
  pt = _pad_before(o['padding'], h, oh, kh, sh, 1)
  pl = _pad_before(o['padding'], wd, ow, kw, sw, 1)
  need_h = (oh - 1) * sh + kh
  need_w = (ow - 1) * sw + kw
  # Padding positions hold -inf, which never exceeds the running maximum: the window is clipped to
  # the image like reference/pooling.h. The running maximum starts at float32 lowest() and a tap
  # replaces it only if it is strictly greater (std::max(max, v)): -inf elements and NaN never
  # win, so a window of only -inf / NaN gives lowest().
  canvas = np.full((n, max(need_h, pt + h), max(need_w, pl + wd), c), -np.inf)
  canvas[:, pt:pt + h, pl:pl + wd, :] = x
  out = np.full((n, oh, ow, c), FLOAT32_LOWEST)
  for ky in range(kh):
    for kx in range(kw):
      tap = canvas[:, ky:ky + (oh - 1) * sh + 1:sh, kx:kx + (ow - 1) * sw + 1:sw, :]
      out = np.where(out < tap, tap, out)
  return _act(out, o['activation'])


def resize_bilinear(x, oh, ow, align_corners, half_pixel):
  n, h, wd, c = x.shape

  def axis(out_size, in_size):
    scale = (in_size - 1) / (out_size - 1) if (align_corners and out_size > 1) else in_size / out_size
    v = np.arange(out_size, dtype=np.float64)
    s = (v + 0.5) * scale - 0.5 if half_pixel else v * scale
    lo = np.maximum(np.floor(s), 0).astype(np.int64)
    hi = np.minimum(np.ceil(s), in_size - 1).astype(np.int64)
    return s, lo, hi

  ys, y0, y1 = axis(oh, h)
  xs, x0, x1 = axis(ow, wd)
  dy = (ys - y0)[None, :, None, None]
  dx = (xs - x0)[None, None, :, None]
  q00 = x[:, y0][:, :, x0]
  q10 = x[:, y1][:, :, x0]
  q01 = x[:, y0][:, :, x1]
  q11 = x[:, y1][:, :, x1]
  return (q00 * (1 - dy) * (1 - dx) + q10 * dy * (1 - dx) + q01 * (1 - dy) * dx + q11 * dy * dx)


def strided_slice(x, begin, end, strides, o):
  for k in ('ellipsis_mask', 'new_axis_mask'):
    if o[k]:
      raise RefError('strided slice %s unsupported' % k)
  if o.get('offset'):
    raise RefError('strided slice offset unsupported')
  idx = []
  out_shape = []
  for a, size in enumerate(x.shape):
    st = strides[a]
    if st == 0:
      raise RefError('zero stride')
    start = begin[a]
    if o['begin_mask'] >> a & 1:
      start = -(1 << 31) if st > 0 else (1 << 31) - 1
    if start < 0:
      start += size
    start = min(max(start, 0), size) if st > 0 else min(max(start, -1), size - 1)
    if o['shrink_axis_mask'] >> a & 1:
      stop = start + 1
    else:
      stop = end[a]
      if o['end_mask'] >> a & 1:
        stop = (1 << 31) - 1 if st > 0 else -(1 << 31)
      if stop < 0:
        stop += size
      stop = min(max(stop, 0), size) if st > 0 else min(max(stop, -1), size - 1)
    sel = list(range(start, stop, st))
    idx.append(np.array(sel, dtype=np.int64))
    if not (o['shrink_axis_mask'] >> a & 1):
      out_shape.append(len(sel))
  out = x[np.ix_(*idx)]
  return out.reshape(out_shape)


def landmarks_to_transform_matrix(lm, p):
  """landmarks-to-transform-matrix.cc LandmarksToTransformMatrixV2 (float64 here)."""
  pts = lm.reshape(-1, 3)
  left = pts[p['left_rotation_idx']]
  right = pts[p['right_rotation_idx']]
  rotation = p.get('target_rotation_radians', 0.0) - math.atan2(right[1] - left[1], right[0] - left[0])
  subset = p['subset_idxs']
  # Pairs of the flat list; an odd trailing index becomes the pair (last, last). Parser
  # reconstructed from upstream TF delegates/gpu/common/mediapipe (not in the local tree; INFERRED).
  pairs = [(subset[2 * i], subset[2 * i + 1]) for i in range(len(subset) // 2)]
  if len(subset) % 2:
    pairs.append((subset[-1], subset[-1]))
  sel = []
  for a, b in pairs:
    sel.append(pts[a])
    sel.append(pts[b])
  sel = np.array(sel, dtype=np.float64)
  xy1 = np.stack([sel[:, 0], sel[:, 1], np.ones(len(sel))], axis=1)
  r = rotation
  rot = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
  rot_inv = np.array([[math.cos(-r), -math.sin(-r), 0], [math.sin(-r), math.cos(-r), 0], [0, 0, 1]])
  t = xy1 @ rot.T
  mx = t.max(axis=0)
  mn = t.min(axis=0)
  crop_w = mx[0] - mn[0]
  crop_h = mx[1] - mn[1]
  center = np.array([(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, 1.0])
  center = rot_inv @ center
  cx, cy = center[0], center[1]
  ow, oh = p['output_width'], p['output_height']
  shift = np.eye(4)
  shift[0, 3], shift[1, 3] = cx, cy
  rr = -rotation
  rot4 = np.eye(4)
  rot4[0, 0], rot4[0, 1], rot4[1, 0], rot4[1, 1] = math.cos(rr), -math.sin(rr), math.sin(rr), math.cos(rr)
  scale = np.eye(4)
  scale[0, 0] = p['scale_x'] * crop_w / ow
  scale[1, 1] = p['scale_y'] * crop_h / oh
  shift2 = np.eye(4)
  shift2[0, 3], shift2[1, 3] = -(ow / 2.0), -(oh / 2.0)
  return (shift @ rot4 @ scale @ shift2).reshape(1, 4, 4)


def transform_landmarks(lm, m, dims):
  """transform-landmarks.cc TransformLandmarksV2."""
  out = np.array(lm, dtype=np.float64, copy=True)
  flat_m = m.reshape(-1)
  pts = out.reshape(-1, out.shape[-1])
  x = pts[:, 0].copy()
  y = pts[:, 1].copy()
  pts[:, 0] = flat_m[0] * x + flat_m[1] * y + flat_m[3]
  pts[:, 1] = flat_m[4] * x + flat_m[5] * y + flat_m[7]
  del dims
  return out


def transform_tensor_bilinear(x, m, oh, ow):
  """transform-tensor-bilinear.cc TransformTensorBilinearV2 (coordinates in float32)."""
  f32 = np.float32
  _, ih, iw, c = x.shape
  fm = m.reshape(-1).astype(np.float32)
  xt = [f32(fm[0]), f32(fm[1]), f32(fm[2]), f32(fm[3])]
  yt = [f32(fm[4]), f32(fm[5]), f32(fm[6]), f32(fm[7])]
  # x_transform[3] += x_transform[0] * 0.5 + x_transform[1] * 0.5 - 0.5 (double arithmetic, float store)
  xt[3] = f32(float(xt[3]) + (float(xt[0]) * 0.5 + float(xt[1]) * 0.5 - 0.5))
  yt[3] = f32(float(yt[3]) + (float(yt[0]) * 0.5 + float(yt[1]) * 0.5 - 0.5))
  out = np.zeros((1, oh, ow, c), dtype=np.float64)
  img = x[0]
  for oy in range(oh):
    for ox in range(ow):
      cx, cy = f32(ox), f32(oy)
      # DotProduct: l.x*r.x + l.y*r.y + l.z*r.z + l.w*r.w, evaluated left to right in float.
      tx = f32(f32(f32(f32(xt[0] * cx) + f32(xt[1] * cy)) + f32(xt[2] * f32(0))) + f32(xt[3] * f32(1)))
      ty = f32(f32(f32(f32(yt[0] * cx) + f32(yt[1] * cy)) + f32(yt[2] * f32(0))) + f32(yt[3] * f32(1)))
      # Negated form so that a NaN coordinate counts as out of bound, like the engine (the
      # original converts the NaN to int, which is undefined).
      if not (tx >= 0.0 and tx <= iw - 1 and ty >= 0.0 and ty <= ih - 1):
        continue
      fx, fy = math.floor(float(tx)), math.floor(float(ty))

      def read(hh, ww):
        if hh < 0 or ww < 0 or hh >= ih or ww >= iw:
          return np.zeros(c)
        return img[hh, ww]

      q11, q21 = read(fy, fx), read(fy, fx + 1)
      q12, q22 = read(fy + 1, fx), read(fy + 1, fx + 1)
      rc = float(tx) - fx
      lc = float(ty) - fy
      upper = (1.0 - rc) * q11 + rc * q21
      lower = (1.0 - rc) * q12 + rc * q22
      out[0, oy, ox] = lc * lower + (1.0 - lc) * upper
  return out


class RefInterpreter:
  """Runs subgraph 0 of a tflite_reader.Model with numpy."""

  def __init__(self, model):
    self.m = model
    if model.num_subgraphs != 1:
      raise RefError('expected one subgraph, got %d' % model.num_subgraphs)

  def _const(self, t):
    if t.data is None:
      return None
    if t.dtype in FLOAT_TYPES:
      return t.data.astype(np.float64)
    return t.data.copy()

  def run(self, inputs):
    """inputs: list of arrays for model.inputs. Returns dict tensor index -> array."""
    m = self.m
    vals = {}
    for t in m.tensors:
      c = self._const(t)
      if c is not None:
        vals[t.index] = c
    if len(inputs) != len(m.inputs):
      raise RefError('expected %d inputs' % len(m.inputs))
    for ti, arr in zip(m.inputs, inputs):
      t = m.tensors[ti]
      a = np.asarray(arr)
      if list(a.shape) != list(t.shape):
        raise RefError('input %s shape %s != %s' % (t.name, a.shape, t.shape))
      vals[ti] = a.astype(np.float64) if t.dtype in FLOAT_TYPES else a.astype(INT_DTYPES[t.dtype])
    for op in m.ops:
      ins = [vals[i] if i >= 0 else None for i in op.inputs]
      with np.errstate(all='ignore'):  # inf / NaN inputs are part of the test cases
        outs = self._run_op(op, ins)
      for ti, v in zip(op.outputs, outs):
        t = m.tensors[ti]
        if list(v.shape) != list(t.shape):
          raise RefError('op %d %s output %s shape %s != declared %s' %
                         (op.index, op.name, t.name, list(v.shape), t.shape))
        if t.dtype in FLOAT_TYPES:
          v = np.asarray(v, dtype=np.float64)
        else:
          v = np.asarray(v).astype(INT_DTYPES[t.dtype])
        vals[ti] = v
    return vals

  def _out_t(self, op, k=0):
    return self.m.tensors[op.outputs[k]]

  def _run_op(self, op, ins):
    n = op.name
    o = op.options
    if n == 'CONV_2D':
      return [conv2d(ins[0], ins[1], ins[2] if len(ins) > 2 else None, o)]
    if n == 'DEPTHWISE_CONV_2D':
      return [depthwise_conv2d(ins[0], ins[1], ins[2] if len(ins) > 2 else None, o)]
    if n in ('ADD', 'SUB', 'MUL'):
      a, b = ins
      r = a + b if n == 'ADD' else (a - b if n == 'SUB' else a * b)
      return [_act(r, o['activation'])]
    if n == 'MINIMUM':
      # maximum_minimum.cc: el1 < el2 ? el1 : el2 (a NaN first operand loses, unlike np.minimum).
      return [np.where(ins[0] < ins[1], ins[0], ins[1])]
    if n == 'FLOOR_MOD':
      return [np.mod(ins[0], ins[1])]
    if n == 'RELU':
      # reference_ops.h:97-98: `const T lower = 0; const T clamped = val < lower ? lower : val;`
      # A NaN and a -0 pass unchanged; np.maximum(-0, 0) returned +0 (RM-1c, review RM-1b/RM-5 LOW).
      return [np.where(ins[0] < 0.0, 0.0, ins[0])]
    if n == 'PRELU':
      return [np.where(ins[0] >= 0, ins[0], ins[0] * ins[1])]
    if n == 'LEAKY_RELU':
      alpha = float(np.float32(o['alpha']))
      return [np.where(ins[0] > 0, ins[0], ins[0] * alpha)]
    if n == 'HARD_SWISH':
      x = ins[0]
      return [x * np.minimum(6.0, np.maximum(0.0, x + 3.0)) / 6.0]
    if n == 'LOGISTIC':
      return [1.0 / (1.0 + np.exp(-ins[0]))]
    if n == 'PAD':
      pads = ins[1].reshape(-1, 2)
      return [np.pad(ins[0], [(int(p[0]), int(p[1])) for p in pads])]
    if n == 'RESHAPE':
      if len(ins) > 1 and ins[1] is not None:
        shape = [int(v) for v in ins[1].reshape(-1)]
      else:
        shape = list(o['new_shape'])
      return [ins[0].reshape(shape)]
    if n == 'MAX_POOL_2D':
      return [max_pool(ins[0], o)]
    if n == 'CONCATENATION':
      axis = o['axis']
      return [_act(np.concatenate(ins, axis=axis), o['activation'])]
    if n == 'DEQUANTIZE':
      return [ins[0].astype(np.float64)]
    if n == 'STRIDED_SLICE':
      return [strided_slice(ins[0], [int(v) for v in ins[1]], [int(v) for v in ins[2]],
                            [int(v) for v in ins[3]], o)]
    if n == 'RESIZE_BILINEAR':
      oh, ow = [int(v) for v in ins[1]]
      return [resize_bilinear(ins[0], oh, ow, o['align_corners'], o['half_pixel_centers'])]
    if n in ('MEAN', 'REDUCE_MAX'):
      axes = tuple(int(a) % ins[0].ndim for a in np.array(ins[1]).reshape(-1))
      if n == 'MEAN':
        return [np.mean(ins[0], axis=axes, keepdims=o['keep_dims'])]
      # reduce.cc kMax: start at float32 lowest(), (in > current) ? in : current. fmax ignores NaN
      # and the initial value is what remains for an all -inf / NaN reduction.
      return [np.fmax.reduce(ins[0], axis=axes, keepdims=o['keep_dims'], initial=FLOAT32_LOWEST)]
    if n == 'ARG_MAX':
      axis = int(np.array(ins[1]).reshape(-1)[0]) % ins[0].ndim
      return [_arg_max_tflite(ins[0], axis)]
    if n == 'CAST':
      out_t = self._out_t(op)
      if out_t.dtype in FLOAT_TYPES:
        return [ins[0].astype(np.float64)]
      if ins[0].dtype.kind == 'f':
        return [_saturating_cast(ins[0], INT_DTYPES[out_t.dtype])]
      return [ins[0].astype(INT_DTYPES[out_t.dtype])]
    if n == 'TRANSPOSE':
      return [np.transpose(ins[0], [int(v) for v in ins[1]])]
    if n == 'Landmarks2TransformMatrix':
      return [landmarks_to_transform_matrix(ins[0], flex_decode(op.custom_options))]
    if n == 'TransformLandmarks':
      return [transform_landmarks(ins[0], ins[1], self._out_t(op).shape[2])]
    if n == 'TransformTensorBilinear':
      t = self._out_t(op)
      return [transform_tensor_bilinear(ins[0], ins[1], t.shape[1], t.shape[2])]
    if n == 'Convolution2DTransposeBias':
      raw = op.custom_options
      if len(raw) < 12:
        raise RefError('transpose conv options %d bytes' % len(raw))
      padding, sw, sh = np.frombuffer(raw[:12], dtype='<i4')
      if padding not in (1, 2):
        raise RefError('transpose conv padding %d' % padding)
      return [transpose_conv_bias(ins[0], ins[1], ins[2], padding == 1, int(sw), int(sh))]
    raise RefError('op %d: unsupported op %s' % (op.index, n))
