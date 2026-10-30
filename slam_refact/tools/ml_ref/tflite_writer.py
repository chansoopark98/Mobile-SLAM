#!/usr/bin/env python3
"""Minimal TFLite flatbuffer writer for single-op test models (no flatbuffers library).

Objects are laid out front to back: a table's vtable is written first, then the table, then its
children, so every uoffset points forward (flatbuffers requires uoffsets to be unsigned) and the
vtable soffset is positive. Field indices follow schema.fbs (see tflite_reader.py).
"""
import struct

import numpy as np

from tflite_reader import BUILTIN_CODES, OPTIONS_CODES

TYPE_CODES = {'FLOAT32': 0, 'FLOAT16': 1, 'INT32': 2, 'INT64': 4}
PADDING_CODES = {'SAME': 0, 'VALID': 1}
ACT_CODES = {'NONE': 0, 'RELU': 1, 'RELU_N1_TO_1': 2, 'RELU6': 3}

# Field kinds: scalars ('b','B','h','i','I','f','q'), 'str', 'table', 'tables', 'vec:<fmt>', 'bytes'.


class Table:
  def __init__(self, fields):
    self.fields = fields  # list of (field index, kind, value)


class _Writer:
  def __init__(self):
    self.b = bytearray()

  def align(self, n):
    while len(self.b) % n:
      self.b.append(0)

  def put(self, fmt, *vals):
    pos = len(self.b)
    self.b += struct.pack('<' + fmt, *vals)
    return pos

  def patch_uoffset(self, at, target):
    if target <= at:
      raise ValueError('uoffset must point forward')
    struct.pack_into('<I', self.b, at, target - at)

  def write_vector(self, kind, value):
    if kind == 'bytes':
      data = bytes(value)
      # Buffer data is force_align: 16; align the payload (after the u32 length) to 16.
      while (len(self.b) + 4) % 16:
        self.b.append(0)
      pos = self.put('I', len(data))
      self.b += data
      return pos
    fmt = kind.split(':', 1)[1]
    size = struct.calcsize('<' + fmt)
    while (len(self.b) + 4) % max(size, 4):
      self.b.append(0)
    pos = self.put('I', len(value))
    if value:
      self.b += struct.pack('<%d%s' % (len(value), fmt), *value)
    return pos

  def write_string(self, s):
    data = s.encode('utf-8')
    self.align(4)
    pos = self.put('I', len(data))
    self.b += data + b'\0'
    return pos

  def write_tables(self, tables):
    self.align(4)
    pos = self.put('I', len(tables))
    slots = []
    for _ in tables:
      slots.append(self.put('I', 0))
    for slot, t in zip(slots, tables):
      self.patch_uoffset(slot, self.write_table(t))
    return pos

  def write_table(self, table):
    fields = sorted(table.fields, key=lambda f: f[0])
    # Inline layout after the 4-byte soffset: scalars then 4-byte reference slots.
    layout = []
    off = 4
    for idx, kind, value in fields:
      if kind in ('str', 'table', 'tables', 'bytes') or kind.startswith('vec:'):
        size = 4
      else:
        size = struct.calcsize('<' + kind)
      while off % size:
        off += 1
      layout.append((idx, kind, value, off, size))
      off += size
    table_size = off
    while table_size % 4:
      table_size += 1
    nslots = (fields[-1][0] + 1) if fields else 0
    vt = [0] * nslots
    for idx, _, _, o, _ in layout:
      vt[idx] = o
    self.align(2)
    vt_pos = self.put('HH', 4 + 2 * nslots, table_size)
    for v in vt:
      self.put('H', v)
    self.align(8)
    t_pos = len(self.b)
    self.b += bytes(table_size)
    struct.pack_into('<i', self.b, t_pos, t_pos - vt_pos)
    refs = []
    for idx, kind, value, o, size in layout:
      at = t_pos + o
      if size == 4 and (kind in ('str', 'table', 'tables', 'bytes') or kind.startswith('vec:')):
        refs.append((at, kind, value))
      else:
        struct.pack_into('<' + kind, self.b, at, value)
    for at, kind, value in refs:
      if kind == 'str':
        child = self.write_string(value)
      elif kind == 'table':
        child = self.write_table(value)
      elif kind == 'tables':
        child = self.write_tables(value)
      else:
        child = self.write_vector(kind, value)
      self.patch_uoffset(at, child)
    return t_pos


def builtin_options(op_name, o):
  """(union type, Table) for the builtin options of op_name."""
  def act(k='activation'):
    return ACT_CODES[o.get(k, 'NONE')]
  if op_name == 'CONV_2D':
    return OPTIONS_CODES['Conv2DOptions'], Table([
        (0, 'b', PADDING_CODES[o['padding']]), (1, 'i', o['stride_w']), (2, 'i', o['stride_h']),
        (3, 'b', act()), (4, 'i', o.get('dilation_w', 1)), (5, 'i', o.get('dilation_h', 1))])
  if op_name == 'DEPTHWISE_CONV_2D':
    return OPTIONS_CODES['DepthwiseConv2DOptions'], Table([
        (0, 'b', PADDING_CODES[o['padding']]), (1, 'i', o['stride_w']), (2, 'i', o['stride_h']),
        (3, 'i', o.get('depth_multiplier', 1)), (4, 'b', act()),
        (5, 'i', o.get('dilation_w', 1)), (6, 'i', o.get('dilation_h', 1))])
  if op_name == 'MAX_POOL_2D':
    return OPTIONS_CODES['Pool2DOptions'], Table([
        (0, 'b', PADDING_CODES[o['padding']]), (1, 'i', o['stride_w']), (2, 'i', o['stride_h']),
        (3, 'i', o['filter_w']), (4, 'i', o['filter_h']), (5, 'b', act())])
  if op_name == 'CONCATENATION':
    return OPTIONS_CODES['ConcatenationOptions'], Table([(0, 'i', o['axis']), (1, 'b', act())])
  if op_name in ('ADD', 'SUB', 'MUL'):
    name = {'ADD': 'AddOptions', 'SUB': 'SubOptions', 'MUL': 'MulOptions'}[op_name]
    return OPTIONS_CODES[name], Table([(0, 'b', act())])
  if op_name == 'RESIZE_BILINEAR':
    return OPTIONS_CODES['ResizeBilinearOptions'], Table([
        (2, 'B', int(o['align_corners'])), (3, 'B', int(o['half_pixel_centers']))])
  if op_name == 'RESHAPE':
    if o.get('new_shape') is None:
      return OPTIONS_CODES['ReshapeOptions'], Table([])
    return OPTIONS_CODES['ReshapeOptions'], Table([(0, 'vec:i', list(o['new_shape']))])
  if op_name in ('MEAN', 'REDUCE_MAX'):
    return OPTIONS_CODES['ReducerOptions'], Table([(0, 'B', int(o['keep_dims']))])
  if op_name == 'STRIDED_SLICE':
    return OPTIONS_CODES['StridedSliceOptions'], Table([
        (0, 'i', o.get('begin_mask', 0)), (1, 'i', o.get('end_mask', 0)),
        (2, 'i', o.get('ellipsis_mask', 0)), (3, 'i', o.get('new_axis_mask', 0)),
        (4, 'i', o.get('shrink_axis_mask', 0))])
  if op_name == 'ARG_MAX':
    return OPTIONS_CODES['ArgMaxOptions'], Table([(0, 'b', TYPE_CODES[o['output_type']])])
  if op_name == 'LEAKY_RELU':
    return OPTIONS_CODES['LeakyReluOptions'], Table([(0, 'f', o['alpha'])])
  if op_name == 'PAD':
    return OPTIONS_CODES['PadOptions'], Table([])
  if op_name == 'TRANSPOSE':
    return OPTIONS_CODES['TransposeOptions'], Table([])
  if op_name == 'DEQUANTIZE':
    return OPTIONS_CODES['DequantizeOptions'], Table([])
  if op_name == 'MINIMUM':
    return OPTIONS_CODES['MaximumMinimumOptions'], Table([])
  if op_name == 'FLOOR_MOD':
    return OPTIONS_CODES['FloorModOptions'], Table([])
  if op_name == 'HARD_SWISH':
    return OPTIONS_CODES['HardSwishOptions'], Table([])
  return 0, None  # RELU, PRELU, LOGISTIC, CAST (TOCO models omit options), custom ops


def build_model(tensors, inputs, outputs, ops, description='slam_refact ml op case'):
  """tensors: list of dict(name, shape, dtype, data=None (numpy) ).
  ops: list of dict(name, inputs, outputs, options={}, custom_options=b'' (custom ops),
  version=1 (operator code version)).
  Returns the .tflite bytes."""
  opcode_keys = []
  versions = {}
  for op in ops:
    key = op['name']
    if key not in opcode_keys:
      opcode_keys.append(key)
      versions[key] = op.get('version', 1)
  opcodes = []
  for key in opcode_keys:
    if key in BUILTIN_CODES or key.startswith('BUILTIN_'):
      code = BUILTIN_CODES[key] if key in BUILTIN_CODES else int(key[len('BUILTIN_'):])
      opcodes.append(Table([(0, 'b', min(code, 127)), (2, 'i', versions[key]), (3, 'i', code)]))
    else:
      code = BUILTIN_CODES['CUSTOM']
      opcodes.append(Table([(0, 'b', code), (1, 'str', key), (2, 'i', versions[key]),
                            (3, 'i', code)]))
  buffers = [Table([])]
  tensor_tables = []
  tensor_buffer = []
  for t in tensors:
    fields = [(0, 'vec:i', list(t['shape'])), (1, 'b', TYPE_CODES[t['dtype']]), (3, 'str', t['name'])]
    if t.get('share_buffer_with') is not None:
      # Several tensors referencing one flatbuffer buffer (hardening tests: the file stays small
      # while the decoded constants are large).
      fields.append((2, 'I', tensor_buffer[t['share_buffer_with']]))
      tensor_buffer.append(tensor_buffer[t['share_buffer_with']])
    elif t.get('data') is not None:
      raw = np.ascontiguousarray(t['data']).astype(np.dtype(t['data'].dtype).newbyteorder('<')).tobytes()
      buffers.append(Table([(0, 'bytes', raw)]))
      fields.append((2, 'I', len(buffers) - 1))
      tensor_buffer.append(len(buffers) - 1)
    else:
      fields.append((2, 'I', 0))
      tensor_buffer.append(0)
    tensor_tables.append(Table(fields))
  op_tables = []
  for op in ops:
    fields = [(0, 'I', opcode_keys.index(op['name'])), (1, 'vec:i', list(op['inputs'])),
              (2, 'vec:i', list(op['outputs']))]
    if op['name'].startswith('BUILTIN_'):
      pass  # raw builtin code without options (rejection tests)
    elif op['name'] in BUILTIN_CODES:
      ot, opts = builtin_options(op['name'], op.get('options', {}))
      if opts is not None:
        fields += [(3, 'B', ot), (4, 'table', opts)]
    else:
      fields += [(5, 'bytes', op.get('custom_options', b'')), (6, 'b', 0)]
    op_tables.append(Table(fields))
  subgraph = Table([(0, 'tables', tensor_tables), (1, 'vec:i', list(inputs)),
                    (2, 'vec:i', list(outputs)), (3, 'tables', op_tables), (4, 'str', 'main')])
  model = Table([(0, 'I', 3), (1, 'tables', opcodes), (2, 'tables', [subgraph]),
                 (3, 'str', description), (4, 'tables', buffers)])
  w = _Writer()
  w.put('I', 0)
  w.b += b'TFL3'
  root = w.write_table(model)
  struct.pack_into('<I', w.b, 0, root)
  return bytes(w.b)


# ---- FlexBuffers map writer (custom op options) -----------------------------------------------
def flex_map(entries):
  """Encodes {key: int | float | list[int]} as a FlexBuffers map with 4-byte widths.

  Layout: key strings, then int vectors, then [keys typed vector], then the map (values + types).
  Values: int -> FBT_INT (packed type 1<<2|2), float -> FBT_FLOAT (3<<2|2), list[int] -> offset to
  FBT_VECTOR_INT (11<<2|2)."""
  w = 4
  b = bytearray()
  keys = sorted(entries)  # FlexBuffers maps are sorted by key
  key_pos = {}
  for k in keys:
    key_pos[k] = len(b)
    b += k.encode('utf-8') + b'\0'
  vec_pos = {}
  for k in keys:
    v = entries[k]
    if isinstance(v, (list, tuple)):
      while len(b) % w:
        b.append(0)
      b += struct.pack('<I', len(v))
      vec_pos[k] = len(b)
      b += struct.pack('<%di' % len(v), *v)
  while len(b) % w:
    b.append(0)
  b += struct.pack('<I', len(keys))
  keys_vec = len(b)
  for k in keys:
    b += struct.pack('<I', len(b) - key_pos[k])
  # Map prefix: keys offset, keys byte width, size; then values; then types.
  prefix_keys_at = len(b)
  b += struct.pack('<I', 0)
  b += struct.pack('<I', w)
  b += struct.pack('<I', len(keys))
  map_pos = len(b)
  struct.pack_into('<I', b, prefix_keys_at, prefix_keys_at - keys_vec)
  types = []
  for k in keys:
    v = entries[k]
    if isinstance(v, (list, tuple)):
      b += struct.pack('<I', len(b) - vec_pos[k])
      types.append((11 << 2) | 2)
    elif isinstance(v, float):
      b += struct.pack('<f', v)
      types.append((3 << 2) | 2)
    else:
      b += struct.pack('<i', v)
      types.append((1 << 2) | 2)
  b += bytes(types)
  # Root: offset to the map, packed type (map, width 4), root byte width.
  root_at = len(b)
  b += struct.pack('<I', root_at - map_pos)
  b += bytes([(9 << 2) | 2, 4])
  return bytes(b)
