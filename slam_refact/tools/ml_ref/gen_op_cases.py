#!/usr/bin/env python3
"""Generates the single-op test cases of the C++ ML engine (tests/ml/data/op_cases).

Per case: <name>.tflite (written by tflite_writer.py), <name>.srt (graph inputs as 'in:<k>' and
float64 numpy reference outputs as 'out:<k>'), <name>.graph.txt (canonical graph text of the numpy
parser). cases.tsv lists name, tolerance class (E exact / F float, docs/design/ml-runtime.md §5.1)
and the op under test. Extra models: unsupported_op.tflite, bad_shape.tflite (rejection tests).

  python3 -I slam_refact/tools/ml_ref/gen_op_cases.py --out slam_refact/engine/tests/ml/data/op_cases
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from ml_ref import graph_text  # noqa: E402
from ref_ops import RefInterpreter  # noqa: E402
from tensor_bundle import write_bundle  # noqa: E402
from tflite_reader import read_model  # noqa: E402
from tflite_writer import build_model, flex_map  # noqa: E402

NP_DTYPES = {'FLOAT32': np.float32, 'FLOAT16': np.float16, 'INT32': np.int32, 'INT64': np.int64}


class Graph:
  def __init__(self):
    self.tensors = []
    self.ops = []
    self.inputs = []
    self.outputs = []
    self.feeds = []

  def _add(self, name, shape, dtype, data=None):
    self.tensors.append(dict(name=name, shape=list(shape), dtype=dtype, data=data))
    return len(self.tensors) - 1

  def input(self, name, value):
    value = np.asarray(value, dtype=np.float32)
    idx = self._add(name, value.shape, 'FLOAT32')
    self.inputs.append(idx)
    self.feeds.append(value)
    return idx

  def const(self, name, value, dtype='FLOAT32'):
    value = np.asarray(value, dtype=NP_DTYPES[dtype])
    return self._add(name, value.shape, dtype, value)

  def tensor(self, name, shape, dtype='FLOAT32'):
    return self._add(name, shape, dtype)

  def op(self, name, ins, outs, options=None, custom=b''):
    # The face-mesh custom ops are registered as version 2 (RegisterTransformLandmarksV2 etc.).
    version = 2 if name in ('TransformLandmarks', 'Landmarks2TransformMatrix',
                            'TransformTensorBilinear') else 1
    self.ops.append(dict(name=name, inputs=list(ins), outputs=list(outs), options=options or {},
                         custom_options=custom, version=version))

  def output(self, *idx):
    self.outputs.extend(idx)


CASES = []


def case(name, cls, op):
  def deco(fn):
    CASES.append((name, cls, op, fn))
    return fn
  return deco


def rnd(seed, shape, lo=-2.0, hi=2.0):
  return np.random.RandomState(seed).uniform(lo, hi, size=shape).astype(np.float32)


# ---- CONV_2D ---------------------------------------------------------------------------------------
def _conv(seed, xs, ws, padding, stride, act='NONE', dil=1, bias=True):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  w = g.const('w', rnd(seed + 1, ws, -1, 1))
  b = g.const('b', rnd(seed + 2, [ws[0]], -1, 1)) if bias else -1
  eff = lambda k: (k - 1) * dil + 1  # noqa: E731
  if padding == 'SAME':
    oh, ow = -(-xs[1] // stride), -(-xs[2] // stride)
  else:
    oh = (xs[1] - eff(ws[1]) + stride) // stride
    ow = (xs[2] - eff(ws[2]) + stride) // stride
  y = g.tensor('y', [xs[0], oh, ow, ws[0]])
  g.op('CONV_2D', [x, w, b], [y], dict(padding=padding, stride_w=stride, stride_h=stride,
                                       activation=act, dilation_w=dil, dilation_h=dil))
  g.output(y)
  return g


case('conv_same_s1_k3_relu', 'F', 'CONV_2D')(lambda: _conv(10, [1, 6, 7, 3], [4, 3, 3, 3], 'SAME', 1, 'RELU'))
case('conv_valid_s2_k3_batch2', 'F', 'CONV_2D')(lambda: _conv(20, [2, 7, 8, 3], [5, 3, 3, 3], 'VALID', 2))
case('conv_same_s2_k5_relu6', 'F', 'CONV_2D')(lambda: _conv(30, [1, 9, 10, 2], [3, 5, 5, 2], 'SAME', 2, 'RELU6'))
case('conv_1x1_valid', 'F', 'CONV_2D')(lambda: _conv(40, [1, 4, 5, 8], [6, 1, 1, 8], 'VALID', 1))
case('conv_1x1_same_relu_n1', 'F', 'CONV_2D')(lambda: _conv(50, [1, 4, 5, 8], [6, 1, 1, 8], 'SAME', 1, 'RELU_N1_TO_1'))
case('conv_same_dilated2', 'F', 'CONV_2D')(lambda: _conv(60, [1, 9, 9, 2], [3, 3, 3, 2], 'SAME', 1, dil=2))
case('conv_no_bias', 'F', 'CONV_2D')(lambda: _conv(70, [1, 5, 6, 3], [4, 3, 3, 3], 'SAME', 1, bias=False))
case('conv_valid_s2_k2', 'F', 'CONV_2D')(lambda: _conv(80, [1, 6, 6, 4], [7, 2, 2, 4], 'VALID', 2))
case('conv_valid_s3_k3_to_1x1', 'F', 'CONV_2D')(lambda: _conv(90, [1, 3, 3, 8], [20, 3, 3, 8], 'VALID', 3))


# ---- DEPTHWISE_CONV_2D -----------------------------------------------------------------------------
def _dw(seed, xs, k, mult, padding, stride, act='NONE', dil=1):
  g = Graph()
  oc = xs[3] * mult
  x = g.input('x', rnd(seed, xs))
  w = g.const('w', rnd(seed + 1, [1, k, k, oc], -1, 1))
  b = g.const('b', rnd(seed + 2, [oc], -1, 1))
  eff = (k - 1) * dil + 1
  if padding == 'SAME':
    oh, ow = -(-xs[1] // stride), -(-xs[2] // stride)
  else:
    oh, ow = (xs[1] - eff + stride) // stride, (xs[2] - eff + stride) // stride
  y = g.tensor('y', [xs[0], oh, ow, oc])
  g.op('DEPTHWISE_CONV_2D', [x, w, b], [y],
       dict(padding=padding, stride_w=stride, stride_h=stride, depth_multiplier=mult,
            activation=act, dilation_w=dil, dilation_h=dil))
  g.output(y)
  return g


case('dw_same_s1_k3', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw(100, [1, 6, 7, 5], 3, 1, 'SAME', 1))
case('dw_same_s2_k3_relu6', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw(110, [1, 7, 6, 4], 3, 1, 'SAME', 2, 'RELU6'))
case('dw_valid_s2_k5_relu', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw(120, [2, 11, 9, 4], 5, 1, 'VALID', 2, 'RELU'))
case('dw_mult2_same', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw(130, [1, 5, 5, 3], 3, 2, 'SAME', 1))
case('dw_dilated2_same', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw(140, [1, 8, 8, 3], 3, 1, 'SAME', 1, dil=2))
case('dw_same_s1_k5', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw(150, [1, 6, 5, 8], 5, 1, 'SAME', 1))


# ---- MAX_POOL_2D -------------------------------------------------------------------------------------
def _pool(seed, xs, k, stride, padding, act='NONE'):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  if padding == 'SAME':
    oh, ow = -(-xs[1] // stride), -(-xs[2] // stride)
  else:
    oh, ow = (xs[1] - k + stride) // stride, (xs[2] - k + stride) // stride
  y = g.tensor('y', [xs[0], oh, ow, xs[3]])
  g.op('MAX_POOL_2D', [x], [y], dict(padding=padding, stride_w=stride, stride_h=stride,
                                     filter_w=k, filter_h=k, activation=act))
  g.output(y)
  return g


case('maxpool_same_s2_k2_odd', 'E', 'MAX_POOL_2D')(lambda: _pool(200, [1, 7, 5, 3], 2, 2, 'SAME'))
case('maxpool_valid_s2_k2', 'E', 'MAX_POOL_2D')(lambda: _pool(210, [2, 6, 8, 4], 2, 2, 'VALID'))
case('maxpool_same_s1_k3_relu6', 'E', 'MAX_POOL_2D')(lambda: _pool(220, [1, 5, 6, 2], 3, 1, 'SAME', 'RELU6'))


# ---- binary / unary elementwise -----------------------------------------------------------------
def _binary(seed, name, a_shape, b_shape, out_shape, act=None, b_const=False, lo=-2.0, hi=2.0):
  g = Graph()
  a = g.input('a', rnd(seed, a_shape, lo, hi))
  if b_const:
    b = g.const('b', rnd(seed + 1, b_shape, lo, hi))
  else:
    b = g.input('b', rnd(seed + 1, b_shape, lo, hi))
  y = g.tensor('y', out_shape)
  g.op(name, [a, b], [y], dict(activation=act) if act is not None else {})
  g.output(y)
  return g


case('add_same_relu', 'F', 'ADD')(lambda: _binary(300, 'ADD', [1, 4, 5, 6], [1, 4, 5, 6], [1, 4, 5, 6], 'RELU'))
case('add_channel_broadcast', 'F', 'ADD')(lambda: _binary(310, 'ADD', [1, 4, 5, 6], [1, 1, 1, 6], [1, 4, 5, 6], 'NONE', True))
case('add_scalar_relu6', 'F', 'ADD')(lambda: _binary(320, 'ADD', [2, 3, 4], [], [2, 3, 4], 'RELU6', True, -8, 8))
case('sub_channel_broadcast', 'F', 'SUB')(lambda: _binary(330, 'SUB', [2, 6, 5, 3], [1, 1, 1, 3], [2, 6, 5, 3], 'NONE', True))
case('sub_scalar', 'F', 'SUB')(lambda: _binary(340, 'SUB', [1, 8, 4, 3], [], [1, 8, 4, 3], 'NONE', True))
case('mul_rank3_by_1', 'F', 'MUL')(lambda: _binary(350, 'MUL', [1, 30, 3], [1], [1, 30, 3], 'NONE', True))
case('mul_scalar', 'F', 'MUL')(lambda: _binary(360, 'MUL', [2, 3], [], [2, 3], 'NONE', True))
case('mul_two_way_broadcast', 'F', 'MUL')(lambda: _binary(370, 'MUL', [2, 1, 4], [1, 3, 1], [2, 3, 4], 'RELU'))
case('mul_matrix_same', 'F', 'MUL')(lambda: _binary(380, 'MUL', [1, 4, 4], [1, 4, 4], [1, 4, 4], 'NONE', True))
case('minimum_scalar', 'E', 'MINIMUM')(lambda: _binary(390, 'MINIMUM', [1, 1, 1, 16], [], [1, 1, 1, 16], None, True))
case('minimum_broadcast', 'E', 'MINIMUM')(lambda: _binary(395, 'MINIMUM', [2, 3, 4], [3, 1], [2, 3, 4]))
case('prelu_channel_alpha', 'F', 'PRELU')(lambda: _binary(400, 'PRELU', [1, 4, 4, 8], [1, 1, 8], [1, 4, 4, 8], None, True))


def _unary(seed, name, shape, lo, hi, options=None):
  g = Graph()
  x = g.input('x', rnd(seed, shape, lo, hi))
  y = g.tensor('y', shape)
  g.op(name, [x], [y], options or {})
  g.output(y)
  return g


case('relu', 'E', 'RELU')(lambda: _unary(410, 'RELU', [1, 5, 6, 4], -3, 3))
case('leaky_relu', 'F', 'LEAKY_RELU')(lambda: _unary(420, 'LEAKY_RELU', [2, 5, 4, 3], -3, 3, dict(alpha=0.01)))
case('hard_swish', 'F', 'HARD_SWISH')(lambda: _unary(430, 'HARD_SWISH', [1, 6, 6, 5], -6, 6))
case('logistic', 'F', 'LOGISTIC')(lambda: _unary(440, 'LOGISTIC', [1, 7, 5, 4], -20, 20))


# ---- PAD / RESHAPE / CONCATENATION / TRANSPOSE / STRIDED_SLICE -----------------------------------
def _pad(seed, xs, pads):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  p = g.const('pads', np.array(pads), 'INT32')
  y = g.tensor('y', [d + a + b for d, (a, b) in zip(xs, pads)])
  g.op('PAD', [x, p], [y])
  g.output(y)
  return g


case('pad_spatial', 'E', 'PAD')(lambda: _pad(500, [1, 5, 4, 3], [[0, 0], [1, 1], [1, 1], [0, 0]]))
case('pad_channels', 'E', 'PAD')(lambda: _pad(510, [1, 4, 4, 6], [[0, 0], [0, 0], [0, 0], [0, 4]]))
case('pad_asymmetric_batch2', 'E', 'PAD')(lambda: _pad(520, [2, 3, 4, 2], [[0, 0], [2, 1], [0, 3], [1, 0]]))


@case('reshape_options_infer', 'E', 'RESHAPE')
def _():
  g = Graph()
  x = g.input('x', rnd(530, [1, 4, 4, 6]))
  y = g.tensor('y', [1, 16, 6])
  g.op('RESHAPE', [x], [y], dict(new_shape=[1, -1, 6]))
  g.output(y)
  return g


@case('reshape_shape_tensor', 'E', 'RESHAPE')
def _():
  g = Graph()
  x = g.input('x', rnd(540, [2, 3, 5, 4]))
  s = g.const('shape', np.array([2, 3, 20]), 'INT32')
  y = g.tensor('y', [2, 3, 20])
  g.op('RESHAPE', [x, s], [y])
  g.output(y)
  return g


def _concat(seed, shapes, axis, out_shape):
  g = Graph()
  ins = [g.input('x%d' % i, rnd(seed + i, s)) for i, s in enumerate(shapes)]
  y = g.tensor('y', out_shape)
  g.op('CONCATENATION', ins, [y], dict(axis=axis, activation='NONE'))
  g.output(y)
  return g


case('concat_axis1_three', 'E', 'CONCATENATION')(lambda: _concat(550, [[1, 5, 1], [1, 3, 1], [1, 2, 1]], 1, [1, 10, 1]))
case('concat_axis_neg1', 'E', 'CONCATENATION')(lambda: _concat(560, [[1, 3, 3, 4], [1, 3, 3, 2]], -1, [1, 3, 3, 6]))
case('concat_axis2', 'E', 'CONCATENATION')(lambda: _concat(570, [[2, 3, 1], [2, 3, 1]], 2, [2, 3, 2]))


def _transpose(seed, xs, perm):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  p = g.const('perm', np.array(perm), 'INT32')
  y = g.tensor('y', [xs[i] for i in perm])
  g.op('TRANSPOSE', [x, p], [y])
  g.output(y)
  return g


case('transpose_0312', 'E', 'TRANSPOSE')(lambda: _transpose(580, [2, 4, 3, 3], [0, 3, 1, 2]))
case('transpose_201', 'E', 'TRANSPOSE')(lambda: _transpose(590, [3, 4, 5], [2, 0, 1]))


def _slice(seed, xs, begin, end, strides, out_shape, **masks):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  b = g.const('begin', np.array(begin), 'INT32')
  e = g.const('end', np.array(end), 'INT32')
  s = g.const('strides', np.array(strides), 'INT32')
  y = g.tensor('y', out_shape)
  g.op('STRIDED_SLICE', [x, b, e, s], [y], masks)
  g.output(y)
  return g


case('slice_identity', 'E', 'STRIDED_SLICE')(lambda: _slice(600, [1, 1, 1, 12], [0, 0, 0, 0], [1, 1, 1, 12], [1, 1, 1, 1], [1, 1, 1, 12]))
case('slice_negative_masks', 'E', 'STRIDED_SLICE')(lambda: _slice(610, [2, 6, 5], [0, -4, 1], [0, -1, 0], [1, 2, 1], [2, 2, 4], begin_mask=1, end_mask=5))
case('slice_reverse_shrink', 'E', 'STRIDED_SLICE')(lambda: _slice(620, [3, 7, 4], [1, 6, 0], [2, 0, 4], [1, -2, 1], [3, 4], shrink_axis_mask=1))


# ---- RESIZE_BILINEAR / MEAN / REDUCE_MAX / ARG_MAX ---------------------------------------------------
def _resize(seed, xs, size, ac, hp):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  s = g.const('size', np.array(size), 'INT32')
  y = g.tensor('y', [xs[0], size[0], size[1], xs[3]])
  g.op('RESIZE_BILINEAR', [x, s], [y], dict(align_corners=ac, half_pixel_centers=hp))
  g.output(y)
  return g


case('resize_align_corners_up', 'F', 'RESIZE_BILINEAR')(lambda: _resize(700, [2, 5, 4, 3], [10, 7], True, False))
case('resize_plain_up', 'F', 'RESIZE_BILINEAR')(lambda: _resize(710, [1, 4, 6, 2], [9, 11], False, False))
case('resize_half_pixel_up', 'F', 'RESIZE_BILINEAR')(lambda: _resize(720, [1, 4, 6, 2], [9, 11], False, True))
case('resize_half_pixel_down', 'F', 'RESIZE_BILINEAR')(lambda: _resize(730, [1, 9, 8, 3], [4, 3], False, True))


def _reduce(seed, name, xs, axes, keep, out_shape):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  a = g.const('axes', np.array(axes), 'INT32')
  y = g.tensor('y', out_shape)
  g.op(name, [x, a], [y], dict(keep_dims=keep))
  g.output(y)
  return g


case('mean_hw_keep', 'F', 'MEAN')(lambda: _reduce(740, 'MEAN', [1, 6, 5, 7], [1, 2], True, [1, 1, 1, 7]))
case('mean_last_nokeep', 'F', 'MEAN')(lambda: _reduce(750, 'MEAN', [2, 3, 9], [-1], False, [2, 3]))
case('reduce_max_axis2', 'E', 'REDUCE_MAX')(lambda: _reduce(760, 'REDUCE_MAX', [2, 3, 40], [2], False, [2, 3]))
case('reduce_max_hw_keep', 'E', 'REDUCE_MAX')(lambda: _reduce(770, 'REDUCE_MAX', [1, 4, 5, 3], [1, 2], True, [1, 1, 1, 3]))


def _argmax(seed, xs, axis, out_type, ties=False):
  g = Graph()
  v = rnd(seed, xs)
  if ties:
    v = np.round(v)  # many equal maxima: first index wins
  x = g.input('x', v)
  a = g.const('axis', np.array(axis), 'INT32')
  out_shape = [d for i, d in enumerate(xs) if i != axis % len(xs)]
  y = g.tensor('y', out_shape, out_type)
  g.op('ARG_MAX', [x, a], [y], dict(output_type=out_type))
  g.output(y)
  return g


case('argmax_axis2_int64', 'E', 'ARG_MAX')(lambda: _argmax(800, [2, 3, 50], 2, 'INT64'))
case('argmax_last_int32_ties', 'E', 'ARG_MAX')(lambda: _argmax(810, [4, 6], -1, 'INT32', ties=True))


# ---- CAST / FLOOR_MOD / DEQUANTIZE chains ---------------------------------------------------------
@case('cast_float_int64_float', 'E', 'CAST')
def _():
  g = Graph()
  x = g.input('x', rnd(820, [3, 5], -40, 40))
  i = g.tensor('i', [3, 5], 'INT64')
  f = g.tensor('f', [3, 5])
  g.op('CAST', [x], [i])
  g.op('CAST', [i], [f])
  g.output(i, f)
  return g


@case('cast_float_int32_float', 'E', 'CAST')
def _():
  g = Graph()
  x = g.input('x', rnd(825, [2, 7], -40, 40))
  i = g.tensor('i', [2, 7], 'INT32')
  f = g.tensor('f', [2, 7])
  g.op('CAST', [x], [i])
  g.op('CAST', [i], [f])
  g.output(i, f)
  return g


@case('floor_mod_int64', 'E', 'FLOOR_MOD')
def _():
  g = Graph()
  x = g.input('x', rnd(830, [4, 6], -60, 60))
  i = g.tensor('i', [4, 6], 'INT64')
  m = g.const('m', np.array(7), 'INT64')
  mn = g.const('mn', np.array(-5), 'INT64')
  r = g.tensor('r', [4, 6], 'INT64')
  rn = g.tensor('rn', [4, 6], 'INT64')
  g.op('CAST', [x], [i])
  g.op('FLOOR_MOD', [i, m], [r])
  g.op('FLOOR_MOD', [i, mn], [rn])
  g.output(r, rn)
  return g


@case('floor_mod_float', 'F', 'FLOOR_MOD')
def _():
  return _binary(840, 'FLOOR_MOD', [3, 4], [1, 4], [3, 4], None, True, -9, 9)


@case('ear_argmax_chain', 'E', 'ARG_MAX')
def _():
  # face-ear-model ops 150-163: transpose, reshape, argmax, cast, mul, cast, floor_mod, concat.
  g = Graph()
  x = g.input('heat', rnd(850, [2, 5, 4, 3]))
  perm = g.const('perm', np.array([0, 3, 1, 2]), 'INT32')
  t = g.tensor('t', [2, 3, 5, 4])
  g.op('TRANSPOSE', [x, perm], [t])
  shp = g.const('shape', np.array([2, 3, 20]), 'INT32')
  r = g.tensor('r', [2, 3, 20])
  g.op('RESHAPE', [t, shp], [r])
  ax = g.const('axis', np.array(2), 'INT32')
  am = g.tensor('am', [2, 3], 'INT64')
  g.op('ARG_MAX', [r, ax], [am], dict(output_type='INT64'))
  amf = g.tensor('amf', [2, 3])
  g.op('CAST', [am], [amf])
  inv_w = g.const('inv_w', np.array(np.float16(1.0 / 4.0)).astype(np.float32))
  q = g.tensor('q', [2, 3])
  g.op('MUL', [amf, inv_w], [q], dict(activation='NONE'))
  qi = g.tensor('qi', [2, 3], 'INT64')
  g.op('CAST', [q], [qi])
  w4 = g.const('w4', np.array(4), 'INT64')
  mod = g.tensor('mod', [2, 3], 'INT64')
  g.op('FLOOR_MOD', [am, w4], [mod])
  modf = g.tensor('modf', [2, 3])
  g.op('CAST', [mod], [modf])
  g.output(qi, modf)
  return g


@case('dequantize_fp16_add', 'E', 'DEQUANTIZE')
def _():
  g = Graph()
  vals = np.array([0.0, -0.0, 6e-8, -3.5e-6, 65504.0, -1.0, 0.333, 1e-4, 2.0, -7.25, 0.1, 1.5],
                  dtype=np.float16).reshape(1, 3, 4)
  w16 = g.const('w16', vals, 'FLOAT16')
  w = g.tensor('w', [1, 3, 4])
  g.op('DEQUANTIZE', [w16], [w])
  x = g.input('x', np.zeros((1, 3, 4), np.float32))
  y = g.tensor('y', [1, 3, 4])
  g.op('ADD', [x, w], [y], dict(activation='NONE'))
  g.output(y)
  return g


# ---- custom ops ---------------------------------------------------------------------------------
def _matrix(theta, sx, sy, tx, ty):
  c, s = np.cos(theta), np.sin(theta)
  return np.array([[[c * sx, -s * sy, 0, tx], [s * sx, c * sy, 0, ty], [0, 0, 1, 0], [0, 0, 0, 1]]],
                  dtype=np.float32)


@case('transform_landmarks_2d', 'F', 'TransformLandmarks')
def _():
  g = Graph()
  lm = g.input('lm', rnd(900, [1, 80, 2], 0, 16))
  m = g.input('m', _matrix(0.4, 1.3, 1.1, 20.0, -5.0))
  y = g.tensor('y', [1, 80, 2])
  g.op('TransformLandmarks', [lm, m], [y])
  g.output(y)
  return g


@case('transform_landmarks_3d', 'F', 'TransformLandmarks')
def _():
  g = Graph()
  lm = g.input('lm', rnd(910, [1, 10, 3], -3, 16))
  m = g.input('m', _matrix(-1.1, 0.7, 0.9, 3.0, 2.0))
  y = g.tensor('y', [1, 10, 3])
  g.op('TransformLandmarks', [lm, m], [y])
  g.output(y)
  return g


@case('landmarks_to_matrix_odd_subset', 'F', 'Landmarks2TransformMatrix')
def _():
  g = Graph()
  lm = g.input('lm', rnd(920, [1, 20, 3], 10, 180))
  y = g.tensor('y', [1, 4, 4])
  opts = flex_map({'left_rotation_idx': 2, 'right_rotation_idx': 7, 'output_height': 16,
                   'output_width': 12, 'scale_x': 1.5, 'scale_y': 1.25,
                   'target_rotation_radians': 0.3,
                   'subset_idxs': [2, 7, 3, 11, 19, 0, 5, 13, 17]})
  g.op('Landmarks2TransformMatrix', [lm], [y], custom=opts)
  g.output(y)
  return g


@case('landmarks_to_matrix_even_subset', 'F', 'Landmarks2TransformMatrix')
def _():
  g = Graph()
  lm = g.input('lm', rnd(925, [1, 30, 3], 10, 180))
  y = g.tensor('y', [1, 4, 4])
  opts = flex_map({'left_rotation_idx': 29, 'right_rotation_idx': 4, 'output_height': 16,
                   'output_width': 16, 'scale_x': 1.5, 'scale_y': 1.5,
                   'target_rotation_radians': 0.0,
                   'subset_idxs': [29, 4, 8, 9, 10, 21, 22, 1]})
  g.op('Landmarks2TransformMatrix', [lm], [y], custom=opts)
  g.output(y)
  return g


@case('transform_tensor_bilinear', 'F', 'TransformTensorBilinear')
def _():
  g = Graph()
  x = g.input('x', rnd(930, [1, 12, 10, 4]))
  # Samples cover the image and fall partly outside it (zero output).
  m = g.input('m', _matrix(0.35, 1.9, 2.1, -1.5, 0.5))
  y = g.tensor('y', [1, 6, 5, 4])
  g.op('TransformTensorBilinear', [x, m], [y],
       custom=flex_map({'output_height': 6, 'output_width': 5}))
  g.output(y)
  return g


def _tconv(seed, xs, ws, padding, stride, out_hw):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  w = g.const('w', rnd(seed + 1, ws, -1, 1))
  b = g.const('b', rnd(seed + 2, [ws[0]], -1, 1))
  y = g.tensor('y', [xs[0], out_hw[0], out_hw[1], ws[0]])
  raw = np.array([1 if padding == 'SAME' else 2, stride, stride], dtype='<i4').tobytes()
  g.op('Convolution2DTransposeBias', [x, w, b], [y], custom=raw)
  g.output(y)
  return g


case('tconv_same_s2_k2', 'F', 'Convolution2DTransposeBias')(lambda: _tconv(940, [1, 4, 5, 6], [3, 2, 2, 6], 'SAME', 2, [8, 9]))
case('tconv_same_s2_k3', 'F', 'Convolution2DTransposeBias')(lambda: _tconv(950, [1, 4, 4, 3], [5, 3, 3, 3], 'SAME', 2, [8, 8]))
case('tconv_valid_s2_k3_batch2', 'F', 'Convolution2DTransposeBias')(lambda: _tconv(960, [2, 3, 4, 2], [4, 3, 3, 2], 'VALID', 2, [7, 9]))


# ---- small chain (memory planning) --------------------------------------------------------------
@case('chain_conv_pool_concat', 'F', 'CONV_2D')
def _():
  g = Graph()
  x = g.input('x', rnd(970, [1, 8, 8, 3]))
  w1 = g.const('w1', rnd(971, [4, 3, 3, 3], -1, 1))
  b1 = g.const('b1', rnd(972, [4], -1, 1))
  c1 = g.tensor('c1', [1, 8, 8, 4])
  g.op('CONV_2D', [x, w1, b1], [c1], dict(padding='SAME', stride_w=1, stride_h=1, activation='RELU'))
  p1 = g.tensor('p1', [1, 4, 4, 4])
  g.op('MAX_POOL_2D', [c1], [p1], dict(padding='VALID', stride_w=2, stride_h=2, filter_w=2, filter_h=2))
  w2 = g.const('w2', rnd(973, [1, 3, 3, 4], -1, 1))
  b2 = g.const('b2', rnd(974, [4], -1, 1))
  d2 = g.tensor('d2', [1, 4, 4, 4])
  g.op('DEPTHWISE_CONV_2D', [p1, w2, b2], [d2],
       dict(padding='SAME', stride_w=1, stride_h=1, depth_multiplier=1))
  s = g.tensor('s', [1, 4, 4, 4])
  g.op('ADD', [d2, p1], [s], dict(activation='NONE'))
  cat = g.tensor('cat', [1, 4, 4, 8])
  g.op('CONCATENATION', [s, p1], [cat], dict(axis=3))
  r = g.tensor('r', [1, 16, 8])
  g.op('RESHAPE', [cat], [r], dict(new_shape=[1, -1, 8]))
  g.output(r, s)
  return g


# ---- RM-1 additions: non-square geometry, IEEE special values, zero-size tensors -----------------
# (appended after the original 72 cases so that their files stay byte-identical)
def _pair(v):
  return (v, v) if isinstance(v, int) else tuple(v)


def _out_len(padding, n, k, stride, dil):
  eff = (k - 1) * dil + 1
  return -(-n // stride) if padding == 'SAME' else (n - eff + stride) // stride


def _conv_hw(seed, xs, ws, padding, stride, dil=(1, 1), act='NONE'):
  """CONV_2D with independent height / width stride and dilation (stride, dil: (h, w))."""
  sh, sw = _pair(stride)
  dh, dw = _pair(dil)
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  w = g.const('w', rnd(seed + 1, ws, -1, 1))
  b = g.const('b', rnd(seed + 2, [ws[0]], -1, 1))
  oh = _out_len(padding, xs[1], ws[1], sh, dh)
  ow = _out_len(padding, xs[2], ws[2], sw, dw)
  y = g.tensor('y', [xs[0], oh, ow, ws[0]])
  g.op('CONV_2D', [x, w, b], [y], dict(padding=padding, stride_w=sw, stride_h=sh, activation=act,
                                       dilation_w=dw, dilation_h=dh))
  g.output(y)
  return g


case('conv_same_s1x2_k3x1_dil2x1', 'F', 'CONV_2D')(lambda: _conv_hw(1000, [1, 7, 9, 3], [4, 3, 1, 3], 'SAME', (1, 2), (2, 1)))
case('conv_valid_s2x1_k2x3', 'F', 'CONV_2D')(lambda: _conv_hw(1010, [1, 8, 7, 2], [5, 2, 3, 2], 'VALID', (2, 1)))
case('conv_same_s2x3_k5x3_relu6', 'F', 'CONV_2D')(lambda: _conv_hw(1020, [2, 10, 11, 2], [3, 5, 3, 2], 'SAME', (2, 3), act='RELU6'))
case('conv_valid_dil2x3_k2x2', 'F', 'CONV_2D')(lambda: _conv_hw(1030, [1, 9, 10, 2], [4, 2, 2, 2], 'VALID', (1, 1), (2, 3)))


def _dw_hw(seed, xs, kh, kw, mult, padding, stride, dil=(1, 1), act='NONE'):
  sh, sw = _pair(stride)
  dh, dw = _pair(dil)
  g = Graph()
  oc = xs[3] * mult
  x = g.input('x', rnd(seed, xs))
  w = g.const('w', rnd(seed + 1, [1, kh, kw, oc], -1, 1))
  b = g.const('b', rnd(seed + 2, [oc], -1, 1))
  oh = _out_len(padding, xs[1], kh, sh, dh)
  ow = _out_len(padding, xs[2], kw, sw, dw)
  y = g.tensor('y', [xs[0], oh, ow, oc])
  g.op('DEPTHWISE_CONV_2D', [x, w, b], [y],
       dict(padding=padding, stride_w=sw, stride_h=sh, depth_multiplier=mult, activation=act,
            dilation_w=dw, dilation_h=dh))
  g.output(y)
  return g


case('dw_same_s1x2_k1x3', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw_hw(1100, [1, 6, 9, 4], 1, 3, 1, 'SAME', (1, 2)))
case('dw_valid_s2x1_k3x2_dil1x2', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw_hw(1110, [1, 9, 8, 3], 3, 2, 1, 'VALID', (2, 1), (1, 2)))
case('dw_valid_mult3_s2', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw_hw(1120, [1, 7, 7, 2], 3, 3, 3, 'VALID', 2))
case('dw_same_mult2_s2x1_k2x3_relu', 'F', 'DEPTHWISE_CONV_2D')(lambda: _dw_hw(1130, [2, 5, 6, 2], 2, 3, 2, 'SAME', (2, 1), act='RELU'))


def _pool_hw(seed, xs, f, stride, padding, act='NONE'):
  fh, fw = _pair(f)
  sh, sw = _pair(stride)
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  y = g.tensor('y', [xs[0], _out_len(padding, xs[1], fh, sh, 1), _out_len(padding, xs[2], fw, sw, 1),
                     xs[3]])
  g.op('MAX_POOL_2D', [x], [y], dict(padding=padding, stride_w=sw, stride_h=sh, filter_w=fw,
                                     filter_h=fh, activation=act))
  g.output(y)
  return g


case('maxpool_same_s1x2_f3x1', 'E', 'MAX_POOL_2D')(lambda: _pool_hw(1200, [1, 6, 7, 3], (3, 1), (1, 2), 'SAME'))
case('maxpool_valid_s2x1_f2x3', 'E', 'MAX_POOL_2D')(lambda: _pool_hw(1210, [2, 7, 8, 2], (2, 3), (2, 1), 'VALID'))
case('maxpool_same_s3x2_f2x3_relu6', 'E', 'MAX_POOL_2D')(lambda: _pool_hw(1220, [1, 8, 9, 2], (2, 3), (3, 2), 'SAME', 'RELU6'))


def _tconv_hw(seed, xs, ws, padding, sw, sh):
  g = Graph()
  x = g.input('x', rnd(seed, xs))
  w = g.const('w', rnd(seed + 1, ws, -1, 1))
  b = g.const('b', rnd(seed + 2, [ws[0]], -1, 1))
  h, wd, kh, kw = xs[1], xs[2], ws[1], ws[2]
  ph = max(0, kh - (h - 1) % sh - 1) if padding == 'SAME' else 0
  pw = max(0, kw - (wd - 1) % sw - 1) if padding == 'SAME' else 0
  y = g.tensor('y', [xs[0], sh * (h - 1) + kh - ph, sw * (wd - 1) + kw - pw, ws[0]])
  raw = np.array([1 if padding == 'SAME' else 2, sw, sh], dtype='<i4').tobytes()
  g.op('Convolution2DTransposeBias', [x, w, b], [y], custom=raw)
  g.output(y)
  return g


case('tconv_same_sw2_sh1_k2x3', 'F', 'Convolution2DTransposeBias')(lambda: _tconv_hw(1300, [1, 4, 5, 3], [3, 2, 3, 3], 'SAME', 2, 1))
case('tconv_valid_sw1_sh3_k3x2', 'F', 'Convolution2DTransposeBias')(lambda: _tconv_hw(1310, [2, 3, 4, 2], [4, 3, 2, 2], 'VALID', 1, 3))
case('tconv_same_sw3_sh2_k3x3', 'F', 'Convolution2DTransposeBias')(lambda: _tconv_hw(1320, [1, 5, 4, 2], [2, 3, 3, 2], 'SAME', 3, 2))

case('resize_plain_down_legacy', 'F', 'RESIZE_BILINEAR')(lambda: _resize(1400, [1, 9, 8, 3], [4, 3], False, False))
case('resize_align_corners_down', 'F', 'RESIZE_BILINEAR')(lambda: _resize(1410, [1, 9, 8, 3], [4, 3], True, False))
case('resize_half_pixel_down_h_up_w', 'F', 'RESIZE_BILINEAR')(lambda: _resize(1420, [1, 8, 5, 2], [3, 9], False, True))
case('resize_plain_up_h_down_w', 'F', 'RESIZE_BILINEAR')(lambda: _resize(1430, [2, 3, 10, 2], [7, 4], False, False))


def _tr_bilinear_matrix(m, seed=1500):
  g = Graph()
  x = g.input('x', rnd(seed, [1, 12, 10, 4]))
  mt = g.input('m', m)
  y = g.tensor('y', [1, 6, 5, 4])
  g.op('TransformTensorBilinear', [x, mt], [y],
       custom=flex_map({'output_height': 6, 'output_width': 5}))
  g.output(y)
  return g


_nan_matrix = _matrix(0.35, 1.9, 2.1, -1.5, 0.5)
_nan_matrix[0, 0, 0] = np.nan
case('transform_tensor_bilinear_nan_matrix', 'F', 'TransformTensorBilinear')(lambda: _tr_bilinear_matrix(_nan_matrix))


# IEEE special values. Positions are flat indices into the input; values stay below 1e15 where two
# operands are multiplied and below 1e30 elsewhere so that the float64 reference and the float32
# engine agree on which results overflow (none do).
NAN, INF = float('nan'), float('inf')


def _special(seed, shape, put, lo=-2.0, hi=2.0):
  v = rnd(seed, shape, lo, hi).reshape(-1)
  for pos, val in put.items():
    v[pos % v.size] = val
  return v.reshape(shape)


def _binary_special(name, a, b, act=None, b_const=False):
  g = Graph()
  ai = g.input('a', a)
  bi = g.const('b', b) if b_const else g.input('b', b)
  y = g.tensor('y', list(np.broadcast_shapes(a.shape, b.shape)))
  g.op(name, [ai, bi], [y], dict(activation=act) if act is not None else {})
  g.output(y)
  return g


_sp = {0: NAN, 1: INF, 2: -INF, 3: 1e30, 4: -1e30, 5: 0.0, 6: -0.0, 7: INF, 8: -INF, 9: 1e30}
_sp_b = {0: 1.0, 1: INF, 2: INF, 3: 1e30, 4: 1e30, 5: INF, 6: NAN, 7: -INF, 8: -INF, 9: -1e30}
case('add_nonfinite', 'F', 'ADD')(lambda: _binary_special('ADD', _special(1600, [2, 3, 4], _sp), _special(1601, [2, 3, 4], _sp_b), 'NONE'))
case('sub_nonfinite_relu', 'F', 'SUB')(lambda: _binary_special('SUB', _special(1610, [2, 3, 4], _sp), _special(1611, [2, 3, 4], _sp_b), 'RELU'))
case('mul_nonfinite_relu6', 'F', 'MUL')(lambda: _binary_special('MUL', _special(1620, [2, 3, 4], {0: NAN, 1: INF, 2: -INF, 3: 1e15, 4: -1e15, 5: 0.0, 6: INF}), _special(1621, [2, 3, 4], {0: 0.0, 1: 0.0, 2: 2.0, 3: 1e15, 4: 1e15, 5: INF, 6: 0.0}), 'RELU6'))
case('add_large_values', 'F', 'ADD')(lambda: _binary_special('ADD', _special(1630, [1, 4, 6], {i: 1e30 * (1 if i % 2 else -1) for i in range(0, 24, 3)}), _special(1631, [1, 1, 6], {0: 1e30, 1: -1e30, 2: 3e37, 3: -3e37}), 'NONE'))
case('minimum_nan_tflite_semantics', 'E', 'MINIMUM')(lambda: _binary_special('MINIMUM', _special(1640, [2, 3, 4], {0: NAN, 1: 1.0, 2: NAN, 3: INF, 4: -INF}), _special(1641, [2, 3, 4], {0: 2.0, 1: NAN, 2: NAN, 3: -INF, 4: INF})))
case('prelu_nonfinite', 'F', 'PRELU')(lambda: _binary_special('PRELU', _special(1650, [1, 3, 4], {0: NAN, 1: INF, 2: -INF, 3: -1e30, 4: 0.0}), _special(1651, [1, 1, 4], {0: 0.25, 1: 0.0, 2: 0.5, 3: INF}), None, True))


def _unary_special(name, seed, shape, put, options=None, lo=-3.0, hi=3.0):
  g = Graph()
  x = g.input('x', _special(seed, shape, put, lo, hi))
  y = g.tensor('y', shape)
  g.op(name, [x], [y], options or {})
  g.output(y)
  return g


_usp = {0: NAN, 1: INF, 2: -INF, 3: 1e30, 4: -1e30, 5: 100.0, 6: -100.0, 7: 0.0, 8: -0.0}
case('relu_nonfinite', 'E', 'RELU')(lambda: _unary_special('RELU', 1700, [2, 3, 4], _usp))
case('leaky_relu_nonfinite', 'F', 'LEAKY_RELU')(lambda: _unary_special('LEAKY_RELU', 1710, [2, 3, 4], _usp, dict(alpha=0.01)))
case('hard_swish_nonfinite', 'F', 'HARD_SWISH')(lambda: _unary_special('HARD_SWISH', 1720, [2, 3, 4], _usp))
case('logistic_extreme', 'F', 'LOGISTIC')(lambda: _unary_special('LOGISTIC', 1730, [2, 3, 4], _usp))


def _conv_special(seed, dw):
  g = Graph()
  xs = [1, 5, 6, 3]
  x = g.input('x', _special(seed, xs, {0: NAN, 40: INF, 41: -INF, 70: 1e15, 71: -1e15, 100: INF, 101: INF}))
  if dw:
    w = g.const('w', rnd(seed + 1, [1, 3, 3, 3], -1, 1))
    b = g.const('b', rnd(seed + 2, [3], -1, 1))
    y = g.tensor('y', [1, 5, 6, 3])
    g.op('DEPTHWISE_CONV_2D', [x, w, b], [y], dict(padding='SAME', stride_w=1, stride_h=1, depth_multiplier=1, activation='NONE', dilation_w=1, dilation_h=1))
  else:
    w = g.const('w', rnd(seed + 1, [4, 3, 3, 3], -1, 1))
    b = g.const('b', rnd(seed + 2, [4], -1, 1))
    y = g.tensor('y', [1, 5, 6, 4])
    g.op('CONV_2D', [x, w, b], [y], dict(padding='SAME', stride_w=1, stride_h=1, activation='NONE', dilation_w=1, dilation_h=1))
  g.output(y)
  return g


case('conv_nonfinite', 'F', 'CONV_2D')(lambda: _conv_special(1800, False))
case('dw_nonfinite', 'F', 'DEPTHWISE_CONV_2D')(lambda: _conv_special(1810, True))


@case('maxpool_nonfinite_tflite_semantics', 'E', 'MAX_POOL_2D')
def _():
  # Windows that hold only -inf / NaN give float32 lowest() (TFLite reference); +inf wins.
  g = Graph()
  xs = [1, 6, 6, 2]
  v = rnd(1900, xs).copy()
  v[0, 0:2, 0:2, :] = -INF
  v[0, 2:4, 0:2, :] = NAN
  v[0, 4:6, 4:6, 0] = [[INF, -INF], [NAN, 1.0]]
  v[0, 0:2, 2:4, 1] = [[NAN, -INF], [-INF, NAN]]
  x = g.input('x', v)
  y = g.tensor('y', [1, 3, 3, 2])
  g.op('MAX_POOL_2D', [x], [y], dict(padding='VALID', stride_w=2, stride_h=2, filter_w=2, filter_h=2, activation='NONE'))
  y2 = g.tensor('y2', [1, 6, 6, 2])
  g.op('MAX_POOL_2D', [x], [y2], dict(padding='SAME', stride_w=1, stride_h=1, filter_w=3, filter_h=3, activation='NONE'))
  g.output(y, y2)
  return g


@case('reduce_max_nonfinite_tflite_semantics', 'E', 'REDUCE_MAX')
def _():
  g = Graph()
  v = rnd(1910, [2, 4, 3]).copy()
  v[0, :, 0] = -INF  # all -inf -> lowest()
  v[0, :, 1] = NAN   # all NaN -> lowest()
  v[0, :, 2] = [1.0, NAN, INF, 2.0]
  v[1, :, 0] = [NAN, -INF, -3.0, -INF]
  x = g.input('x', v)
  a = g.const('axes', np.array([1]), 'INT32')
  y = g.tensor('y', [2, 3])
  g.op('REDUCE_MAX', [x, a], [y], dict(keep_dims=False))
  g.output(y)
  return g


@case('argmax_nan_tflite_semantics', 'E', 'ARG_MAX')
def _():
  g = Graph()
  v = np.array([[NAN, 1.0, 2.0, 0.5],      # NaN first: nothing is greater -> 0
                [1.0, NAN, 3.0, 2.0],      # NaN in the middle is skipped -> 2
                [NAN, NAN, NAN, NAN],      # all NaN -> 0
                [-INF, -INF, -INF, -INF],  # ties -> 0
                [-INF, 5.0, INF, INF],     # first +inf -> 2
                [0.0, -0.0, 0.0, -0.0]], dtype=np.float32)  # equal -> 0
  x = g.input('x', v)
  a = g.const('axis', np.array(1), 'INT32')
  y = g.tensor('y', [6], 'INT64')
  g.op('ARG_MAX', [x, a], [y], dict(output_type='INT64'))
  g.output(y)
  return g


@case('mean_nonfinite', 'F', 'MEAN')
def _():
  g = Graph()
  v = rnd(1920, [2, 3, 5]).copy()
  v[0, 0, :] = [INF, 1.0, 2.0, 3.0, 4.0]
  v[0, 1, :] = [INF, -INF, 0.0, 0.0, 0.0]
  v[1, 0, :] = [NAN, 1.0, 1.0, 1.0, 1.0]
  v[1, 1, :] = [1e30, 1e30, -1e30, 1e30, 0.0]
  x = g.input('x', v)
  a = g.const('axes', np.array([-1]), 'INT32')
  y = g.tensor('y', [2, 3])
  g.op('MEAN', [x, a], [y], dict(keep_dims=False))
  g.output(y)
  return g


@case('resize_nonfinite', 'F', 'RESIZE_BILINEAR')
def _():
  g = Graph()
  x = g.input('x', _special(1930, [1, 4, 5, 2], {0: NAN, 7: INF, 20: -INF, 33: 1e30}))
  s = g.const('size', np.array([7, 9]), 'INT32')
  y = g.tensor('y', [1, 7, 9, 2])
  g.op('RESIZE_BILINEAR', [x, s], [y], dict(align_corners=False, half_pixel_centers=True))
  g.output(y)
  return g


@case('cast_nonfinite_saturates', 'E', 'CAST')
def _():
  g = Graph()
  vals = np.array([[NAN, INF, -INF, 1e10, -1e10, 2147483520.0, 2147483648.0, -2147483648.0],
                   [3e9, -3e9, 1e20, -1e20, 9.3e18, -9.3e18, 0.99, -0.99]], dtype=np.float32)
  x = g.input('x', vals)
  i32 = g.tensor('i32', [2, 8], 'INT32')
  i64 = g.tensor('i64', [2, 8], 'INT64')
  g.op('CAST', [x], [i32])
  g.op('CAST', [x], [i64])
  g.output(i32, i64)
  return g


# Zero-size tensors.
@case('slice_zero_length_then_concat', 'E', 'STRIDED_SLICE')
def _():
  g = Graph()
  x = g.input('x', rnd(2000, [1, 6]))
  b = g.const('begin', np.array([0, 3]), 'INT32')
  e = g.const('end', np.array([1, 3]), 'INT32')
  s = g.const('strides', np.array([1, 1]), 'INT32')
  empty = g.tensor('empty', [1, 0])
  g.op('STRIDED_SLICE', [x, b, e, s], [empty], dict(begin_mask=0, end_mask=0, ellipsis_mask=0, new_axis_mask=0, shrink_axis_mask=0))
  tail = g.input('tail', rnd(2001, [1, 2]))
  y = g.tensor('y', [1, 2])
  g.op('CONCATENATION', [empty, tail], [y], dict(axis=1, activation='NONE'))
  g.output(y, empty)
  return g


@case('add_zero_size_broadcast', 'F', 'ADD')
def _():
  g = Graph()
  a = g.input('a', np.zeros((2, 0, 3), np.float32))
  b = g.input('b', rnd(2010, [1, 1, 3]))
  y = g.tensor('y', [2, 0, 3])
  g.op('ADD', [a, b], [y], dict(activation='RELU'))
  g.output(y)
  return g


@case('pad_zero_size_axis', 'E', 'PAD')
def _():
  g = Graph()
  x = g.input('x', np.zeros((1, 0, 3), np.float32))
  p = g.const('pads', np.array([[0, 0], [1, 2], [0, 1]]), 'INT32')
  y = g.tensor('y', [1, 3, 4])
  g.op('PAD', [x, p], [y])
  g.output(y)
  return g


@case('reduce_max_empty_axis', 'E', 'REDUCE_MAX')
def _():
  g = Graph()
  x = g.input('x', np.zeros((2, 0, 3), np.float32))
  a = g.const('axes', np.array([1]), 'INT32')
  y = g.tensor('y', [2, 3])
  g.op('REDUCE_MAX', [x, a], [y], dict(keep_dims=False))
  g.output(y)
  return g


@case('mean_empty_axis', 'F', 'MEAN')
def _():
  g = Graph()
  x = g.input('x', np.zeros((2, 0, 3), np.float32))
  a = g.const('axes', np.array([1]), 'INT32')
  y = g.tensor('y', [2, 3])
  g.op('MEAN', [x, a], [y], dict(keep_dims=False))
  g.output(y)
  return g


@case('transpose_reshape_zero_size', 'E', 'TRANSPOSE')
def _():
  g = Graph()
  x = g.input('x', np.zeros((0, 4), np.float32))
  p = g.const('perm', np.array([1, 0]), 'INT32')
  t = g.tensor('t', [4, 0])
  g.op('TRANSPOSE', [x, p], [t])
  y = g.tensor('y', [2, 0, 2])
  g.op('RESHAPE', [t], [y], dict(new_shape=[2, 0, 2]))
  g.output(y)
  return g


# Run-time FLOOR_MOD by a data-dependent divisor (not in cases.tsv: the reference cannot take a zero
# divisor; tests/ml/ml_ops_test.cc feeds the inputs itself).
def _floor_mod_dynamic():
  g = Graph()
  a = g.input('a', np.array([[7.0, -7.0, -2147483648.0, 5.0]], np.float32))
  b = g.input('b', np.array([[3.0, 3.0, -1.0, 2.0]], np.float32))
  ai = g.tensor('ai', [1, 4], 'INT32')
  bi = g.tensor('bi', [1, 4], 'INT32')
  y = g.tensor('y', [1, 4], 'INT32')
  g.op('CAST', [a], [ai])
  g.op('CAST', [b], [bi])
  g.op('FLOOR_MOD', [ai, bi], [y])
  g.output(y)
  return g


def _resize_wide_model(in_h, in_w, out_h, out_w, align_corners, half_pixel, size_dtype='INT64'):
  """Declared-only RESIZE_BILINEAR: consistent input / size / output (the file stays tiny)."""
  g = Graph()
  x = g._add('x', [1, in_h, in_w, 1], 'FLOAT32')
  g.inputs.append(x)
  g.feeds.append(np.zeros(1, np.float32))
  s = g.const('size', np.array([out_h, out_w], np.int64 if size_dtype == 'INT64' else np.int32),
              size_dtype)
  y = g.tensor('y', [1, out_h, out_w, 1])
  g.op('RESIZE_BILINEAR', [x, s], [y], dict(align_corners=align_corners, half_pixel_centers=half_pixel))
  g.output(y)
  return g


def _shared_buffer_model(n_tensors, n_elems):
  g = Graph()
  x = g.input('x', rnd(3040, [1, 4]))
  y = g.tensor('y', [1, 4])
  g.op('RELU', [x], [y])
  g.output(y)
  first = g.const('shared0', np.zeros(n_elems, np.float16), 'FLOAT16')
  for k in range(1, n_tensors):
    g.tensors.append(dict(name='shared%d' % k, shape=[n_elems], dtype='FLOAT16', data=None,
                          share_buffer_with=first))
  return g


def declared_only(op, ins, out_shape, options=None, custom=b''):
  """A graph whose tensors are only declared (no data), so the file stays tiny."""
  g = Graph()
  idx = []
  for k, shape in enumerate(ins):
    t = g._add('in%d' % k, shape, 'FLOAT32')
    g.inputs.append(t)
    g.feeds.append(np.zeros(1, np.float32))
    idx.append(t)
  y = g.tensor('y', out_shape)
  g.op(op, idx, [y], options or {}, custom)
  g.output(y)
  return g


def conv_model(xs, ws, oshape, **opts):
  g = Graph()
  x = g._add('x', xs, 'FLOAT32')
  g.inputs.append(x)
  w = g.const('w', np.zeros(ws, np.float32))
  y = g.tensor('y', oshape)
  o = dict(padding='SAME', stride_w=1, stride_h=1, activation='NONE', dilation_w=1, dilation_h=1)
  o.update(opts)
  g.op('CONV_2D', [x, w, -1], [y], o)
  g.output(y)
  return g


# Models that Interpreter::Create (or the parser) must reject although they are well-formed
# flatbuffers: name -> (expected substring of the error, builder). Review M4.
def _reject_models():
  def pad_model(pads, in_shape, out_shape, dtype='INT64'):
    g = Graph()
    x = g.input('x', rnd(3000, in_shape))
    p = g.const('pads', np.array(pads, dtype=np.int64 if dtype == 'INT64' else np.int32), dtype)
    y = g.tensor('y', out_shape)
    g.op('PAD', [x, p], [y])
    g.output(y)
    return g

  def slice_model(begin, end, strides, out_shape=(1, 0)):
    g = Graph()
    x = g.input('x', rnd(3010, [1, 6]))
    b = g.const('b', np.array(begin, np.int64), 'INT64')
    e = g.const('e', np.array(end, np.int64), 'INT64')
    s = g.const('s', np.array(strides, np.int64), 'INT64')
    y = g.tensor('y', list(out_shape))
    g.op('STRIDED_SLICE', [x, b, e, s], [y], dict(begin_mask=0, end_mask=0, ellipsis_mask=0, new_axis_mask=0, shrink_axis_mask=0))
    g.output(y)
    return g

  def resize_model(size):
    g = Graph()
    x = g.input('x', rnd(3020, [1, 2, 2, 1]))
    s = g.const('size', np.array(size, np.int64), 'INT64')
    y = g.tensor('y', [1, 2, 2, 1])
    g.op('RESIZE_BILINEAR', [x, s], [y], dict(align_corners=False, half_pixel_centers=False))
    g.output(y)
    return g

  def reshape_model(shape):
    g = Graph()
    x = g.input('x', rnd(3030, [4]))
    s = g.const('shape', np.array(shape, np.int64), 'INT64')
    y = g.tensor('y', [4])
    g.op('RESHAPE', [x, s], [y])
    g.output(y)
    return g

  tconv_raw = np.array([1, 2 ** 30, 1], dtype='<i4').tobytes()
  # Reviewer's reproductions first (scratchpad crafted/make_models.py).
  return [
      ('reject_pad_int64_wraps_to_small', 'padding out of the int32 range',
       lambda: pad_model([[0, 0], [2 ** 31, 2 ** 31 + 1]], [1, 4], [1, 5])),
      ('reject_pad_int64_huge', 'padding out of the int32 range',
       lambda: pad_model([[0, 0], [0, 2 ** 40]], [1, 4], [1, 5])),
      ('reject_pad_negative', 'negative padding',
       lambda: pad_model([[0, 0], [-1, 0]], [1, 4], [1, 3], 'INT32')),
      ('reject_slice_stride_2pow32_is_zero_as_int32', 'out of the int32 range',
       lambda: slice_model([0, 0], [1, 6], [1, 2 ** 32])),
      ('reject_slice_begin_2pow32', 'out of the int32 range',
       lambda: slice_model([0, 2 ** 32 + 1], [1, 6], [1, 1])),
      ('reject_tensor_count_2pow32_wraps_on_wasm32', 'has more than',
       lambda: declared_only('RELU', [[65536, 65536]], [65536, 65536])),
      ('reject_tensor_count_9266_after_wrap', 'has more than',
       lambda: declared_only('RELU', [[1, 46341, 46341, 2]], [1, 46341, 46341, 2])),
      ('reject_reshape_target_overflows', 'target shape has too many elements',
       lambda: reshape_model([65536, 65536, 65536])),
      ('reject_conv_dilation_2pow30', 'out of range',
       lambda: conv_model([1, 4, 4, 1], [1, 3, 3, 1], [1, 4, 4, 1], dilation_w=2 ** 30, dilation_h=2 ** 30)),
      ('reject_conv_stride_2pow30', 'out of range',
       lambda: conv_model([1, 4, 4, 1], [1, 3, 3, 1], [1, 1, 1, 1], stride_w=2 ** 30, stride_h=2 ** 30)),
      ('reject_conv_im2col_too_large', 'im2col buffer too large',
       lambda: conv_model([1, 4096, 4096, 1], [1, 5, 5, 1], [1, 4096, 4096, 1])),
      ('reject_maxpool_window_2pow30', 'out of range',
       lambda: declared_only('MAX_POOL_2D', [[1, 4, 4, 1]], [1, 1, 1, 1],
                             dict(padding='SAME', stride_w=1, stride_h=1, filter_w=2 ** 30, filter_h=1, activation='NONE'))),
      ('reject_tconv_stride_2pow30', 'stride out of range',
       lambda: declared_only('Convolution2DTransposeBias', [[1, 4, 4, 1], [1, 3, 3, 1], [1]], [1, 4, 4, 1], None, tconv_raw)),
      ('reject_resize_size_int32_max', 'size out of range',
       lambda: resize_model([2 ** 31 - 1, 2])),
      ('reject_transform_tensor_bilinear_height_zero', 'invalid output size',
       lambda: declared_only('TransformTensorBilinear', [[1, 4, 4, 1], [4, 4]], [1, 0, 4, 1], None,
                             flex_map({'output_height': 0, 'output_width': 4}))),
      ('reject_arena_exceeds_limit', 'exceeds the limit',
       lambda: declared_only('ADD', [[1, 2 ** 25], [1, 2 ** 25]], [1, 2 ** 25], dict(activation='NONE'))),
      # RM-1b. Reviewer MAJOR-1 reproduction: float rounding gave x0 == in_w (an out-of-bounds read
      # in the old ResizeBilinear). 2^25 outputs are 128 MiB, more than the total memory limit allows,
      # so this exact model is now refused at Create; the same defect at sizes that fit is exercised
      # by the family_* models (kernel clamp).
      ('reject_resize_legacy_2pow25_outputs', 'exceeds the limit',
       lambda: _resize_wide_model(1, 1, 1, 2 ** 25, False, False)),
      # RM-1b, review MINOR-2: an arena (96 MiB) and an im2col buffer (100 MiB) that were limited
      # separately, 256 MiB each, now share one budget.
      ('reject_total_memory_arena_plus_im2col', 'exceeds the limit',
       lambda: conv_model([1, 2048, 1024, 4], [8, 3, 1, 4], [1, 2048, 1024, 8])),
      # RM-1b, review MINOR-2: 1,100 tensors reference one 64 KiB buffer; each is decoded to its own
      # float32 copy (1,100 x 128 KiB = 140 MB) from a 0.13 MB file.
      ('reject_shared_buffer_constants_total_memory', 'exceeds the limit',
       lambda: _shared_buffer_model(1100, 32768)),
  ]


# Structure-aware variations (RM-1b, review MAJOR-1 "classes the fuzzer misses"): models whose
# declared shapes, constants and options are changed TOGETHER, which a byte flip never does. Each row:
# name, builder, lo, hi. The C++ test creates every model; one that is accepted must run and every
# output must be finite and inside [lo, hi] (inputs are filled with values in [0.25, 0.75]); a
# rejected one must carry an error message. Under ASan an out-of-bounds access in Invoke is a failure.
# The resize sizes are the smallest ones where the float product x * scale rounds up to the input
# size (found by exhaustive float32 search); before the clamp each of them read past the input.
def _family_models():
  fam = []
  conv_range = (-1e-6, 1e-6)
  img = (0.25 - 1e-5, 0.75 + 1e-5)
  # RESIZE_BILINEAR: legacy mode is the one affected; half-pixel and align_corners are controls.
  for name, dims, ac, hp, dt in [
      # (in_h, in_w, out_h, out_w)
      ('family_resize_w_in1_out13103685', (1, 1, 1, 13103685), False, False, 'INT64'),
      ('family_resize_w_in1_out13103685_int32', (1, 1, 1, 13103685), False, False, 'INT32'),
      ('family_resize_h_in3_out10935533', (3, 1, 10935533, 1), False, False, 'INT64'),
      ('family_resize_w_in5_out9718272', (1, 5, 1, 9718272), False, False, 'INT64'),
      ('family_resize_w_in7_out12056526', (1, 7, 1, 12056526), False, False, 'INT64'),
      ('family_resize_w_in1_out13103685_half_pixel', (1, 1, 1, 13103685), False, True, 'INT64'),
      ('family_resize_w_in1_out13103685_align_corners', (1, 1, 1, 13103685), True, False, 'INT64'),
      ('family_resize_w_in1_out2pow24', (1, 1, 1, 2 ** 24), False, False, 'INT64'),
      ('family_resize_w_in2_out3', (1, 2, 1, 3), False, False, 'INT64'),
      ('family_resize_big_input_small_output', (1, 8388611, 1, 5), False, False, 'INT64'),
      ('family_resize_big_input_half_pixel', (1, 8388611, 1, 7), False, True, 'INT64'),
  ]:
    fam.append((name, (lambda a=dims + (ac, hp, dt): _resize_wide_model(*a)), img[0], img[1]))
  # PAD: output near the memory limit (accepted: 64 MiB; refused: 128 MiB), and a wide one.
  def pad_wide(out_w):
    g = Graph()
    x = g._add('x', [1, 4], 'FLOAT32')
    g.inputs.append(x)
    g.feeds.append(np.zeros(1, np.float32))
    p = g.const('pads', np.array([[0, 0], [0, out_w - 4]], np.int32), 'INT32')
    y = g.tensor('y', [1, out_w])
    g.op('PAD', [x, p], [y])
    g.output(y)
    return g
  fam.append(('family_pad_out2pow24', lambda: pad_wide(2 ** 24), 0.0, 0.75 + 1e-5))
  fam.append(('family_pad_out2pow25', lambda: pad_wide(2 ** 25), 0.0, 0.75 + 1e-5))
  # CONV_2D: arena + im2col together just below / above the shared memory budget.
  fam.append(('family_conv_im2col_98mib', lambda: conv_model([1, 2048, 1024, 2], [4, 3, 1, 2], [1, 2048, 1024, 4]),
              conv_range[0], conv_range[1]))
  fam.append(('family_conv_im2col_49mib', lambda: conv_model([1, 1024, 512, 4], [8, 3, 1, 4], [1, 1024, 512, 8]),
              conv_range[0], conv_range[1]))
  fam.append(('family_conv_im2col_over_budget', lambda: conv_model([1, 2048, 1024, 4], [8, 3, 1, 4], [1, 2048, 1024, 8]),
              conv_range[0], conv_range[1]))
  fam.append(('family_conv_stride3_valid_large', lambda: conv_model([1, 3001, 2001, 3], [6, 5, 5, 3], [1, 1000, 666, 6],
                                                                    padding='VALID', stride_w=3, stride_h=3),
              conv_range[0], conv_range[1]))
  # TransformTensorBilinear: output size and declared output shape together.
  def ttb(h, w):
    return declared_only('TransformTensorBilinear', [[1, 64, 64, 1], [4, 4]], [1, h, w, 1], None,
                         flex_map({'output_height': h, 'output_width': w}))
  fam.append(('family_ttb_4096x4096', lambda: ttb(4096, 4096), 0.0, 0.75 + 1e-5))
  fam.append(('family_ttb_8192x8192', lambda: ttb(8192, 8192), 0.0, 0.75 + 1e-5))
  fam.append(('family_ttb_1x2pow24', lambda: ttb(1, 2 ** 24), 0.0, 0.75 + 1e-5))
  # Convolution2DTransposeBias: stride and the computed output size together.
  def tconv(stride):
    out = stride * (8 - 1) + 5
    raw = np.array([1, stride, stride], dtype='<i4').tobytes()
    return declared_only('Convolution2DTransposeBias', [[1, 8, 8, 2], [3, 5, 5, 2], [3]], [1, out, out, 3], None, raw)
  fam.append(('family_tconv_stride300', lambda: tconv(300), -1e4, 1e4))
  fam.append(('family_tconv_stride500', lambda: tconv(500), -1e4, 1e4))
  return fam


def write_case(out_dir, name, g):
  model_bytes = build_model(g.tensors, g.inputs, g.outputs, g.ops)
  m = read_model(model_bytes)
  vals = RefInterpreter(m).run(g.feeds)
  tensors = [('in:%d' % k, v) for k, v in enumerate(g.feeds)]
  for k, oi in enumerate(m.outputs):
    v = vals[oi]
    tensors.append(('out:%d' % k, v.astype(np.float64) if v.dtype.kind == 'f' else v))
  with open(os.path.join(out_dir, name + '.tflite'), 'wb') as f:
    f.write(model_bytes)
  write_bundle(os.path.join(out_dir, name + '.srt'), tensors)
  with open(os.path.join(out_dir, name + '.graph.txt'), 'w') as f:
    f.write(graph_text(m))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--out', required=True)
  args = ap.parse_args()
  os.makedirs(args.out, exist_ok=True)
  rows = []
  for name, cls, op, fn in CASES:
    write_case(args.out, name, fn())
    rows.append('%s\t%s\t%s' % (name, cls, op))
  with open(os.path.join(args.out, 'cases.tsv'), 'w') as f:
    f.write('# name\tclass\top  (gen_op_cases.py; classes: docs/design/ml-runtime.md 5.1)\n')
    f.write('\n'.join(rows) + '\n')
  # Rejection models: an unsupported builtin (SOFTMAX = 25), an unsupported custom op, and a
  # wrong declared output shape.
  for fname, opname in (('unsupported_builtin.tflite', 'BUILTIN_25'),
                        ('unsupported_custom.tflite', 'NotARealOp')):
    g = Graph()
    x = g.input('x', rnd(990, [1, 4]))
    y = g.tensor('y', [1, 4])
    g.op(opname, [x], [y])
    g.output(y)
    with open(os.path.join(args.out, fname), 'wb') as f:
      f.write(build_model(g.tensors, g.inputs, g.outputs, g.ops))
  g = Graph()
  x = g.input('x', rnd(991, [1, 4, 4, 2]))
  y = g.tensor('y', [1, 3, 3, 2])  # PAD by 1 gives [1, 6, 6, 2]
  p = g.const('pads', np.array([[0, 0], [1, 1], [1, 1], [0, 0]]), 'INT32')
  g.op('PAD', [x, p], [y])
  g.output(y)
  with open(os.path.join(args.out, 'bad_shape.tflite'), 'wb') as f:
    f.write(build_model(g.tensors, g.inputs, g.outputs, g.ops))
  with open(os.path.join(args.out, 'floor_mod_dynamic.tflite'), 'wb') as f:
    g = _floor_mod_dynamic()
    f.write(build_model(g.tensors, g.inputs, g.outputs, g.ops))
  rejects = _reject_models()
  with open(os.path.join(args.out, 'rejects.tsv'), 'w') as f:
    f.write('# model\texpected substring of the error  (gen_op_cases.py; review M4)\n')
    for name, expect, build in rejects:
      g = build()
      with open(os.path.join(args.out, name + '.tflite'), 'wb') as m:
        m.write(build_model(g.tensors, g.inputs, g.outputs, g.ops))
      f.write('%s\t%s\n' % (name, expect))
  family = _family_models()
  with open(os.path.join(args.out, 'family.tsv'), 'w') as f:
    f.write('# name\tlo\thi  (gen_op_cases.py; RM-1b structure-aware models: accepted ones must run with every output in [lo, hi])\n')
    for name, build, lo, hi in family:
      g = build()
      with open(os.path.join(args.out, name + '.tflite'), 'wb') as m:
        m.write(build_model(g.tensors, g.inputs, g.outputs, g.ops))
      f.write('%s\t%r\t%r\n' % (name, lo, hi))
  print('%d cases, %d rejection models and %d family models written to %s' % (len(CASES), len(rejects), len(family), args.out))


if __name__ == '__main__':
  main()
