"""Static evidence extraction for xr-slam.wasm: strings, string xrefs, RTTI/vtables, call graph."""

import collections
import os
import re
import struct
import subprocess

import wasm_parse

STRING_RE = re.compile(rb"[\x09\x0a\x0d\x20-\x7e]{3,}\x00")
MANGLED_TYPE_RE = re.compile(r"^(N\d|\d|St|S[a-z]|Z)[0-9A-Za-z_]*$")


class Evidence:
  def __init__(self, wasm_path):
    self.mod = wasm_parse.Module(wasm_path)
    self.feats = wasm_parse.extract_features(self.mod)
    self.base, self.img = self.mod.mem_image()
    self._strings()
    self._xrefs()
    self._rtti()
    self._callgraph()

  # ---------------------------------------------------------------- strings
  def _strings(self):
    self.strings = {}
    raw = bytes(self.img)
    for mt in STRING_RE.finditer(raw):
      self.strings[self.base + mt.start()] = mt.group()[:-1].decode("ascii")
    self._sorted_addrs = sorted(self.strings)

  def string_at(self, addr):
    """Return (start, text) if addr points inside a known string, else None."""
    import bisect
    i = bisect.bisect_right(self._sorted_addrs, addr) - 1
    if i < 0:
      return None
    s = self._sorted_addrs[i]
    if addr < s + len(self.strings[s]):
      return s, self.strings[s]
    return None

  def _xrefs(self):
    """func -> list of (string start, text, exact) referenced by i32.const or const+offset."""
    self.str_refs = {}
    self.str_users = collections.defaultdict(set)
    self.data_refs = {}  # func -> set of non-string static addresses (for vtable / typeinfo refs)
    for idx, f in self.feats.items():
      refs = {}
      other = set()
      for a in f["load_offsets"]:
        if a in self.strings:
          refs[a] = (a, self.strings[a], True)
      for a in f["addrs"]:
        if a in self.strings:
          refs[a] = (a, self.strings[a], True)
        else:
          hit = self.string_at(a)
          if hit:
            refs.setdefault(hit[0], (hit[0], hit[1], False))
          elif self.base <= a < self.base + len(self.img):
            other.add(a)
      self.str_refs[idx] = sorted(refs.values())
      self.data_refs[idx] = other
      for s, _, exact in refs.values():
        if exact:
          self.str_users[s].add(idx)

  # ---------------------------------------------------------------- RTTI
  def word(self, addr):
    return struct.unpack_from("<I", self.img, addr - self.base)[0]

  def sword(self, addr):
    return struct.unpack_from("<i", self.img, addr - self.base)[0]

  def _rtti(self):
    words = collections.defaultdict(list)
    n = len(self.img) - 3
    for i in range(0, n, 4):
      w = struct.unpack_from("<I", self.img, i)[0]
      words[w].append(self.base + i)
    self._words = words
    names = {a: s for a, s in self.strings.items() if MANGLED_TYPE_RE.match(s) and len(s) >= 3}
    self.typeinfo = {}  # typeinfo addr -> mangled type name
    for a, s in names.items():
      for w in words.get(a, ()):
        self.typeinfo[w - 4] = s
    self.vtables = {}  # vtable address point -> (type name, offset_to_top, [func])
    for t, nm in self.typeinfo.items():
      for r in words.get(t, ()):
        ott = self.sword(r - 4)
        if not -65536 < ott <= 0:
          continue
        slots = []
        p = r + 4
        end = self.base + len(self.img)
        while p < end:
          w = self.word(p)
          if w and w in self.mod.elem:
            slots.append(self.mod.elem[w])
            p += 4
          else:
            break
        if slots:
          self.vtables[r + 4] = (nm, ott, slots)
    self.demangled = demangle_types(sorted(set(self.typeinfo.values())))
    # function -> classes via vtable slot membership
    self.vt_slot_classes = collections.defaultdict(set)
    for vt, (nm, _, slots) in self.vtables.items():
      for f in slots:
        self.vt_slot_classes[f].add(nm)
    # function -> classes whose vtable (address point) or typeinfo it references
    self.vt_ref_classes = collections.defaultdict(set)
    self.ti_ref_classes = collections.defaultdict(set)
    for idx, f in self.feats.items():
      for a in f["addrs"]:
        if a in self.vtables:
          self.vt_ref_classes[idx].add(self.vtables[a][0])
        if a in self.typeinfo:
          self.ti_ref_classes[idx].add(self.typeinfo[a])

  # ---------------------------------------------------------------- call graph
  def _callgraph(self):
    self.callees = {i: set(f["calls"]) for i, f in self.feats.items()}
    self.callers = collections.defaultdict(set)
    for c, cs in self.callees.items():
      for k in cs:
        self.callers[k].add(c)
    # Indirect: functions whose table slot is referenced only through vtables of class X.
    self.table_funcs = set(self.mod.elem.values())


def demangle_types(names):
  """Demangle typeinfo names with c++filt (prefix _ZTS)."""
  if not names:
    return {}
  try:
    out = subprocess.run(["c++filt"], input="\n".join("_ZTS" + n for n in names), capture_output=True,
                         text=True, check=True).stdout.splitlines()
  except (OSError, subprocess.CalledProcessError):
    return {n: n for n in names}
  res = {}
  for n, d in zip(names, out):
    res[n] = d[len("typeinfo name for "):] if d.startswith("typeinfo name for ") else d
  return res


def public_source_literals(root, subdirs=("c8", "reality")):
  """Collect string literals from public reference runtime C++ sources: literal -> first file path."""
  lit_re = re.compile(r'"((?:[^"\\\n]|\\.){3,})"')
  out = {}
  for sd in subdirs:
    for dp, _, fns in os.walk(os.path.join(root, sd)):
      for fn in fns:
        if not fn.endswith((".cc", ".h", ".cpp", ".hpp", ".c")):
          continue
        p = os.path.join(dp, fn)
        try:
          with open(p, encoding="utf-8", errors="replace") as fh:
            txt = fh.read()
        except OSError:
          continue
        rel = os.path.relpath(p, root)
        for m in lit_re.finditer(txt):
          out.setdefault(m.group(1), rel)
  return out


def public_source_class_names(root, subdirs=("c8", "reality")):
  """Names declared as class/struct in public sources -> first file path."""
  decl = re.compile(r"\b(?:class|struct)\s+(?:alignas\([^)]*\)\s+)?([A-Z][A-Za-z0-9_]*)\b")
  out = {}
  for sd in subdirs:
    for dp, _, fns in os.walk(os.path.join(root, sd)):
      for fn in fns:
        if not fn.endswith((".cc", ".h", ".cpp", ".hpp")):
          continue
        p = os.path.join(dp, fn)
        try:
          with open(p, encoding="utf-8", errors="replace") as fh:
            txt = fh.read()
        except OSError:
          continue
        rel = os.path.relpath(p, root)
        for m in decl.finditer(txt):
          out.setdefault(m.group(1), rel)
  return out
