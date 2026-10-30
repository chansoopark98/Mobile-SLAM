#!/usr/bin/env python3
"""Tensor bundle (.srt) files shared by the numpy reference and the C++ ML tests.

Layout (little endian): magic b'SRMLT001', u32 count, then per tensor: u32 name length, name
bytes (utf-8), u8 type (0 float32, 2 int32, 4 int64, 10 float64; TFLite TensorType codes),
u32 rank, i32 dims[rank], raw data. Mirrors slam_refact/engine/src/ml/tensor_bundle.cc.
Reference outputs are stored as float64 (the numpy reference precision).
"""
import struct

import numpy as np

MAGIC = b'SRMLT001'
TYPE_CODES = {np.dtype(np.float32): 0, np.dtype(np.int32): 2, np.dtype(np.int64): 4,
              np.dtype(np.float64): 10}
CODE_TYPES = {v: k for k, v in TYPE_CODES.items()}


def write_bundle(path, tensors):
  """tensors: list of (name, numpy array) with dtype float32 / int32 / int64."""
  out = bytearray(MAGIC)
  out += struct.pack('<I', len(tensors))
  for name, arr in tensors:
    a = np.ascontiguousarray(arr)
    if a.dtype not in TYPE_CODES:
      raise ValueError('%s: unsupported dtype %s' % (name, a.dtype))
    nb = name.encode('utf-8')
    out += struct.pack('<I', len(nb)) + nb
    out += struct.pack('<BI', TYPE_CODES[a.dtype], a.ndim)
    out += struct.pack('<%di' % a.ndim, *a.shape)
    out += a.astype(a.dtype.newbyteorder('<'), copy=False).tobytes()
  with open(path, 'wb') as f:
    f.write(bytes(out))


def read_bundle(path):
  with open(path, 'rb') as f:
    b = f.read()
  if b[:8] != MAGIC:
    raise ValueError('%s: bad magic' % path)
  p = 8
  (n,) = struct.unpack_from('<I', b, p)
  p += 4
  out = []
  for _ in range(n):
    (ln,) = struct.unpack_from('<I', b, p)
    p += 4
    name = b[p:p + ln].decode('utf-8')
    p += ln
    code, rank = struct.unpack_from('<BI', b, p)
    p += 5
    dims = list(struct.unpack_from('<%di' % rank, b, p))
    p += 4 * rank
    dt = CODE_TYPES[code]
    count = int(np.prod(dims)) if dims else 1
    arr = np.frombuffer(b, dtype=dt.newbyteorder('<'), count=count, offset=p).reshape(dims)
    p += count * dt.itemsize
    out.append((name, arr.astype(dt)))
  if p != len(b):
    raise ValueError('%s: %d trailing bytes' % (path, len(b) - p))
  return out
