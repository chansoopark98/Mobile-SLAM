"""Minimal WebAssembly binary parser and per-function feature extractor (stdlib only).

Function indices are absolute wasm indices: imported functions occupy 0..num_imports-1,
defined functions follow (num_imports..num_imports+num_defined-1). This matches the
`// funcN` comments of wasm-decompile and `func[N]` of wasm-objdump.
"""

import collections
import hashlib
import struct


class Reader:
  __slots__ = ("b", "p")

  def __init__(self, b, p=0):
    self.b = b
    self.p = p

  def u8(self):
    v = self.b[self.p]
    self.p += 1
    return v

  def uleb(self):
    b = self.b
    p = self.p
    result = 0
    shift = 0
    while True:
      x = b[p]
      p += 1
      result |= (x & 0x7F) << shift
      if x < 0x80:
        break
      shift += 7
    self.p = p
    return result

  def sleb(self, bits=64):
    b = self.b
    p = self.p
    result = 0
    shift = 0
    while True:
      x = b[p]
      p += 1
      result |= (x & 0x7F) << shift
      shift += 7
      if x < 0x80:
        if x & 0x40:
          result -= 1 << shift
        break
    self.p = p
    return result

  def name(self):
    n = self.uleb()
    s = self.b[self.p:self.p + n].decode("utf-8", "replace")
    self.p += n
    return s

  def bytes(self, n):
    s = self.b[self.p:self.p + n]
    self.p += n
    return s


def _init_expr(r):
  """Parse a constant init expression; return int value for i32/i64.const else None."""
  val = None
  while True:
    op = r.u8()
    if op == 0x0B:
      return val
    if op == 0x41 or op == 0x42:
      val = r.sleb()
    elif op == 0x23:
      r.uleb()
    elif op == 0x43:
      r.bytes(4)
    elif op == 0x44:
      r.bytes(8)
    elif op == 0xD2:
      r.uleb()
    elif op == 0xD0:
      r.u8()
    else:
      raise ValueError("unsupported init expr op 0x%x" % op)


class Module:
  def __init__(self, path):
    with open(path, "rb") as f:
      self.raw = f.read()
    self.sha256 = hashlib.sha256(self.raw).hexdigest()
    self.types = []
    self.imports = []  # (module, field, kind, typeidx)
    self.func_types = []  # absolute index -> typeidx
    self.num_imported_funcs = 0
    self.exports = {}  # func index -> [names]
    self.elem = {}  # table index -> func index
    self.data_segments = []  # (mem offset, bytes)
    self.bodies = []  # (absolute index, body_start_file_offset, body_end, code_start)
    self.custom_sections = []
    self.globals = []
    self._parse()

  def _parse(self):
    b = self.raw
    assert b[:4] == b"\0asm", "not wasm"
    r = Reader(b, 8)
    defined_types = []
    while r.p < len(b):
      sid = r.u8()
      size = r.uleb()
      start = r.p
      end = start + size
      sr = Reader(b, start)
      if sid == 0:
        self.custom_sections.append(sr.name())
      elif sid == 1:
        for _ in range(sr.uleb()):
          form = sr.u8()
          assert form == 0x60
          params = [sr.u8() for _ in range(sr.uleb())]
          results = [sr.u8() for _ in range(sr.uleb())]
          self.types.append((tuple(params), tuple(results)))
      elif sid == 2:
        for _ in range(sr.uleb()):
          mod = sr.name()
          field = sr.name()
          kind = sr.u8()
          if kind == 0:
            t = sr.uleb()
            self.imports.append((mod, field, kind, t))
            self.func_types.append(t)
          elif kind == 1:
            sr.u8()
            flags = sr.uleb()
            sr.uleb()
            if flags & 1:
              sr.uleb()
            self.imports.append((mod, field, kind, None))
          elif kind == 2:
            flags = sr.uleb()
            sr.uleb()
            if flags & 1:
              sr.uleb()
            self.imports.append((mod, field, kind, None))
          elif kind == 3:
            sr.u8()
            sr.u8()
            self.imports.append((mod, field, kind, None))
          else:
            sr.u8()
            sr.uleb()
            self.imports.append((mod, field, kind, None))
        self.num_imported_funcs = len(self.func_types)
      elif sid == 3:
        for _ in range(sr.uleb()):
          defined_types.append(sr.uleb())
      elif sid == 6:
        for _ in range(sr.uleb()):
          vt = sr.u8()
          mut = sr.u8()
          self.globals.append((vt, mut, _init_expr(sr)))
      elif sid == 7:
        for _ in range(sr.uleb()):
          nm = sr.name()
          kind = sr.u8()
          idx = sr.uleb()
          if kind == 0:
            self.exports.setdefault(idx, []).append(nm)
      elif sid == 9:
        for _ in range(sr.uleb()):
          flags = sr.uleb()
          if flags == 0:
            off = _init_expr(sr)
            n = sr.uleb()
            for i in range(n):
              self.elem[off + i] = sr.uleb()
          elif flags == 2:
            sr.uleb()
            off = _init_expr(sr)
            sr.u8()
            n = sr.uleb()
            for i in range(n):
              self.elem[off + i] = sr.uleb()
          else:
            raise ValueError("unsupported elem flags %d" % flags)
      elif sid == 10:
        n = sr.uleb()
        for i in range(n):
          bsize = sr.uleb()
          bstart = sr.p
          bend = bstart + bsize
          lr = Reader(b, bstart)
          for _ in range(lr.uleb()):
            lr.uleb()
            lr.u8()
          self.bodies.append((self.num_imported_funcs + i, bstart, bend, lr.p))
          sr.p = bend
      elif sid == 11:
        for _ in range(sr.uleb()):
          flags = sr.uleb()
          if flags == 0:
            off = _init_expr(sr)
          elif flags == 2:
            sr.uleb()
            off = _init_expr(sr)
          else:
            off = None
          n = sr.uleb()
          self.data_segments.append((off, b[sr.p:sr.p + n]))
          sr.p += n
      r.p = end
    self.func_types.extend(defined_types)

  @property
  def num_funcs(self):
    return len(self.func_types)

  def import_name(self, idx):
    imp = [i for i in self.imports if i[2] == 0][idx]
    return "%s.%s" % (imp[0], imp[1])

  def mem_image(self):
    """Return (base, bytearray) covering all active data segments."""
    lo = min(o for o, _ in self.data_segments if o is not None)
    hi = max(o + len(d) for o, d in self.data_segments if o is not None)
    img = bytearray(hi - lo)
    for o, d in self.data_segments:
      if o is not None:
        img[o - lo:o - lo + len(d)] = d
    return lo, img


# Opcode immediate kinds for the single-byte opcode space.
_MEMARG = set(range(0x28, 0x3F))
_IDX = {0x0C, 0x0D, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x07, 0x08, 0x09, 0x18, 0xD2}
_BLOCK = {0x02, 0x03, 0x04, 0x06}
_LOADS = set(range(0x28, 0x36))
_FLOAT_OPS = set(range(0x5B, 0x67)) | set(range(0x8B, 0xA7)) | set(range(0xB2, 0xC0))
_SIMD_MEMARG = set(range(0, 12)) | {92, 93}
_SIMD_LANE = set(range(21, 35))
_SIMD_MEMARG_LANE = set(range(84, 92))


def decode_function(raw, code_start, end, want_seq=True):
  """Decode one function body. Returns a feature dict."""
  r = Reader(raw, code_start)
  calls = collections.Counter()
  indirect = collections.Counter()
  addrs = collections.Counter()  # candidate static addresses (i32.const, const+memarg offset)
  load_offsets = collections.Counter()
  ops = collections.Counter()
  seq = [] if want_seq else None
  n = 0
  simd = 0
  flt = 0
  prev_const = None
  while r.p < end:
    op = r.u8()
    n += 1
    cur_const = None
    if want_seq:
      seq.append(op)
    if op == 0x41:
      v = r.sleb()
      cur_const = v & 0xFFFFFFFF
      addrs[cur_const] += 1
    elif op == 0x10:
      calls[r.uleb()] += 1
    elif op in _MEMARG:
      r.uleb()
      off = r.uleb()
      if prev_const is not None and off:
        addrs[(prev_const + off) & 0xFFFFFFFF] += 1
      if op in _LOADS and off:
        load_offsets[off] += 1
    elif op in _IDX:
      r.uleb()
    elif op in _BLOCK:
      t = r.b[r.p]
      if t == 0x40 or t >= 0x6F:
        r.p += 1
      else:
        r.sleb()
    elif op == 0x11 or op == 0x13:
      t = r.uleb()
      r.uleb()
      indirect[t] += 1
    elif op == 0x12:
      calls[r.uleb()] += 1
    elif op == 0x42:
      r.sleb()
    elif op == 0x43:
      r.p += 4
      flt += 1
    elif op == 0x44:
      r.p += 8
      flt += 1
    elif op == 0x0E:
      for _ in range(r.uleb() + 1):
        r.uleb()
    elif op == 0x3F or op == 0x40:
      r.p += 1
    elif op == 0x1C:
      for _ in range(r.uleb()):
        r.u8()
    elif op == 0xD0:
      r.u8()
    elif op == 0xFC:
      sub = r.uleb()
      if want_seq:
        seq.append(0x100 + sub)
      if sub == 8:
        r.uleb()
        r.p += 1
      elif sub == 9 or sub == 13 or sub in (15, 16, 17):
        r.uleb()
      elif sub == 10:
        r.p += 2
      elif sub == 11:
        r.p += 1
      elif sub == 12 or sub == 14:
        r.uleb()
        r.uleb()
      ops[0xFC00 + sub] += 1
    elif op == 0xFD:
      sub = r.uleb()
      simd += 1
      if want_seq:
        seq.append(0x200 + (sub & 0x1FF))
      if sub in _SIMD_MEMARG:
        r.uleb()
        off = r.uleb()
        if off:
          load_offsets[off] += 1
          if prev_const is not None:
            addrs[(prev_const + off) & 0xFFFFFFFF] += 1
      elif sub == 12 or sub == 13:
        r.p += 16
      elif sub in _SIMD_LANE:
        r.p += 1
      elif sub in _SIMD_MEMARG_LANE:
        r.uleb()
        r.uleb()
        r.p += 1
    elif op == 0xFE:
      sub = r.uleb()
      if sub == 3:
        r.p += 1
      else:
        r.uleb()
        r.uleb()
    elif op in _FLOAT_OPS:
      flt += 1
    ops[op] += 1
    prev_const = cur_const
  if r.p != end:
    raise ValueError("decode overran body end at 0x%x (end 0x%x)" % (r.p, end))
  out = {
      "n_instr": n,
      "calls": calls,
      "indirect": indirect,
      "addrs": addrs,
      "load_offsets": load_offsets,
      "simd": simd,
      "float": flt,
      "ops": ops,
  }
  if want_seq:
    out["seq_hash"] = hashlib.sha1(bytes(x & 0xFF for x in seq) + bytes(x >> 8 for x in seq)).hexdigest()[:16]
  return out


def extract_features(mod):
  feats = {}
  for idx, bstart, bend, cstart in mod.bodies:
    f = decode_function(mod.raw, cstart, bend)
    f["file_offset"] = bstart
    f["size"] = bend - bstart
    feats[idx] = f
  return feats
