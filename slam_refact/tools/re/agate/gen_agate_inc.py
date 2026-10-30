"""Generate the engine's agate dictionary / PCA .inc files from the original xr-slam.wasm (US-025 LC-2b).

Pipeline (stdlib only, reads only, no network, no git-lfs):
  1. locate the two Cap'n Proto blobs in the wasm data segments (extract_agate_dict.py, LC-2a);
  2. check size and sha256 against the git-lfs pointer files (oid/size) -- refuse on any mismatch;
  3. parse the flat-array messages by hand (segment table, struct/list/far pointers; no capnp library):
       DescriptorDictionary { descriptorSize @0 :UInt32; data @1 :Data }
       BinaryPcaData { basisVectorSize @0 :UInt32; basis @1 :List(UInt64); translationLut @2 :List(Float32);
                       translationIdx @3 :List(UInt8); whitening @4 :List(Float32) }
     (reality/engine/features/api/descriptors.capnp);
  4. write two C++ include files with the decoded arrays. Floats are written as IEEE-754 bit patterns
     (uint32), so no decimal round trip can change a value.

Usage (cwd = repo root):
  python3 -B -I slam_refact/tools/re/agate/gen_agate_inc.py \
      --wasm build/slam-refact-20261015/inputs/xr-slam.wasm \
      --pointer-dir assets/references/xr_reference/official/archive/reality/engine/features/dictionary \
      --expect-wasm-sha256 780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422 \
      --out-dir slam_refact/engine/src/map_builder
  (--check: regenerate in memory and compare with the files in --out-dir; exit 1 on any difference.)

The output is deterministic (no timestamps, no absolute paths).
"""

import argparse
import hashlib
import importlib.util
import os
import struct
import sys

DICT_NAME = "agate-keyframe-dictionary-64-n0.capnpbin"
PCA_NAME = "agate-keyframe-binary-pca-32d.capnpbin"
DICT_INC = "agate_dictionary64_n0.inc"
PCA_INC = "agate_pca32.inc"

# Shape facts the engine code depends on (checked, not assumed).
DESCRIPTOR_BYTES = 32            # OrbFeature: 32 bytes
DICT_ENTRIES = 64
PCA_BASIS_VECTOR_BITS = 16384    # 64 entries * 256 bits
PCA_BASIS_WORDS = 8192           # 32 output dimensions * 256 uint64 blocks
PCA_DIMS = 32


def _load_extract_module():
  here = os.path.dirname(os.path.abspath(__file__))
  spec = importlib.util.spec_from_file_location("extract_agate_dict", os.path.join(here, "extract_agate_dict.py"))
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


class CapnpMessage:
  """Minimal reader for a flat-array Cap'n Proto message (single and double far pointers supported)."""

  def __init__(self, blob):
    nseg = struct.unpack_from("<I", blob, 0)[0] + 1
    sizes = struct.unpack_from("<%dI" % nseg, blob, 4)
    header = 4 + 4 * nseg
    header += header % 8
    self.segs = []
    off = header
    for words in sizes:
      self.segs.append(blob[off:off + 8 * words])
      off += 8 * words
    if off != len(blob):
      raise ValueError("capnp message: segment table covers %d bytes, blob has %d" % (off, len(blob)))

  def word(self, seg, idx):
    return struct.unpack_from("<Q", self.segs[seg], 8 * idx)[0]

  def resolve(self, seg, idx):
    """Follow the pointer stored at (seg, idx) -> (kind, seg, content_word_index, a, b, ptr_word).

    kind 'struct': a = data words, b = pointer count.  kind 'list': a = element size code, b = count.
    kind 'null' for a zero pointer.
    """
    w = self.word(seg, idx)
    if w == 0:
      return ("null", seg, 0, 0, 0)
    t = w & 3
    if t == 2:  # far pointer
      double = (w >> 2) & 1
      pad = (w >> 3) & 0x1FFFFFFF
      pseg = w >> 32
      if not double:
        return self._resolve_near(pseg, pad, self.word(pseg, pad))
      # double far: pad[0] = far pointer to the content, pad[1] = tag word (offset field is 0)
      first = self.word(pseg, pad)
      if first & 3 != 2 or (first >> 2) & 1:
        raise ValueError("double-far landing pad does not start with a single far pointer")
      cseg = first >> 32
      cidx = (first >> 3) & 0x1FFFFFFF
      tag = self.word(pseg, pad + 1)
      return self._decode(tag, cseg, cidx)
    return self._resolve_near(seg, idx, w)

  def _resolve_near(self, seg, idx, w):
    off = (w >> 2) & 0x3FFFFFFF
    if off & 0x20000000:
      off -= 0x40000000
    return self._decode(w, seg, idx + 1 + off)

  @staticmethod
  def _decode(w, seg, content_idx):
    t = w & 3
    if t == 0:
      return ("struct", seg, content_idx, (w >> 32) & 0xFFFF, (w >> 48) & 0xFFFF)
    if t == 1:
      return ("list", seg, content_idx, (w >> 32) & 7, w >> 35)
    raise ValueError("unsupported capnp pointer type %d" % t)

  def struct_root(self):
    kind, seg, idx, dwords, nptrs = self.resolve(0, 0)
    if kind != "struct":
      raise ValueError("root is not a struct")
    return seg, idx, dwords, nptrs

  def struct_ptr(self, root, n):
    seg, idx, dwords, nptrs = root
    if n >= nptrs:
      return ("null", seg, 0, 0, 0)
    return self.resolve(seg, idx + dwords + n)

  def list_bytes(self, ptr, size_code, elem_bytes):
    kind, seg, idx, code, count = ptr
    if kind == "null":
      return b""
    if kind != "list" or code != size_code:
      raise ValueError("list pointer %s with element code %s, expected %d" % (kind, code, size_code))
    return bytes(self.segs[seg][8 * idx:8 * idx + count * elem_bytes])

  def struct_u32(self, root, byte_off):
    seg, idx, dwords, _ = root
    if byte_off + 4 > 8 * dwords:
      return 0
    return struct.unpack_from("<I", self.segs[seg], 8 * idx + byte_off)[0]


def parse_dictionary(blob):
  m = CapnpMessage(blob)
  root = m.struct_root()
  desc_size = m.struct_u32(root, 0)
  data = m.list_bytes(m.struct_ptr(root, 0), 2, 1)
  return desc_size, data


def parse_pca(blob):
  m = CapnpMessage(blob)
  root = m.struct_root()
  basis_vector_size = m.struct_u32(root, 0)
  basis = struct.unpack("<%dQ" % (len(b := m.list_bytes(m.struct_ptr(root, 0), 5, 8)) // 8), b)
  lut = struct.unpack("<%dI" % (len(b := m.list_bytes(m.struct_ptr(root, 1), 4, 4)) // 4), b)
  idx = m.list_bytes(m.struct_ptr(root, 2), 2, 1)
  whitening = struct.unpack("<%dI" % (len(b := m.list_bytes(m.struct_ptr(root, 3), 4, 4)) // 4), b)
  return basis_vector_size, basis, lut, idx, whitening


def _fmt_rows(items, per_row, fmt):
  rows = []
  for i in range(0, len(items), per_row):
    rows.append("    " + ", ".join(fmt % v for v in items[i:i + per_row]) + ",")
  return "\n".join(rows)


def _header(title, provenance):
  lines = ["// GENERATED by slam_refact/tools/re/agate/gen_agate_inc.py -- do not edit.",
           "// %s" % title]
  lines += ["// %s" % p for p in provenance]
  return "\n".join(lines) + "\n"


def render_dictionary_inc(desc_size, data, prov):
  n = len(data) // desc_size
  out = _header("Agate dictionary-64-n0 (DescriptorDictionary): %d entries x %d bytes." % (n, desc_size), prov)
  out += "constexpr uint32_t kAgateDictionaryDescriptorSize = %d;\n" % desc_size
  out += "constexpr uint32_t kAgateDictionaryFileEntries = %d;\n" % n
  out += "constexpr uint8_t kAgateDictionary64N0[%d] = {\n%s\n};\n" % (len(data), _fmt_rows(list(data), 16, "0x%02x"))
  return out


def render_pca_inc(bvs, basis, lut, idx, whitening, prov):
  out = _header("Agate BinaryPca 32d (BinaryPcaData): basisVectorSize=%d bits, %d basis words, %d LUT entries, "
                "%d whitening factors. Floats are IEEE-754 bit patterns." % (bvs, len(basis), len(lut), len(whitening)),
                prov)
  out += "constexpr uint32_t kAgatePcaBasisVectorSize = %d;\n" % bvs
  out += "constexpr uint32_t kAgatePcaTranslationLutSize = %d;\n" % len(lut)
  out += "constexpr uint32_t kAgatePcaWhiteningSize = %d;\n" % len(whitening)
  out += "constexpr uint64_t kAgatePcaBasis[%d] = {\n%s\n};\n" % (len(basis), _fmt_rows(list(basis), 4, "0x%016xull"))
  out += "constexpr uint32_t kAgatePcaTranslationLutBits[%d] = {\n%s\n};\n" % (len(lut), _fmt_rows(list(lut), 8, "0x%08x"))
  out += "constexpr uint8_t kAgatePcaTranslationIdx[%d] = {\n%s\n};\n" % (len(idx), _fmt_rows(list(idx), 16, "0x%02x"))
  out += "constexpr uint32_t kAgatePcaWhiteningBits[%d] = {\n%s\n};\n" % (len(whitening), _fmt_rows(list(whitening), 8, "0x%08x"))
  return out


def build(wasm_path, pointer_dir, expect_wasm_sha):
  ex = _load_extract_module()
  with open(wasm_path, "rb") as f:
    wasm = f.read()
  wasm_sha = hashlib.sha256(wasm).hexdigest()
  if expect_wasm_sha and wasm_sha != expect_wasm_sha:
    raise SystemExit("wasm sha256 %s != expected %s" % (wasm_sha, expect_wasm_sha))
  segs = ex.parse_data_segments(wasm)
  pointers = ex.read_pointers(pointer_dir)
  blobs = {}
  for name in (DICT_NAME, PCA_NAME):
    ptr = pointers.get(name)
    if ptr is None:
      raise SystemExit("no git-lfs pointer for %s in %s" % (name, pointer_dir))
    cands = [c for c in ex.find_blob(wasm, segs, ptr["size"], ptr["oid"]) if c["sha_match"]]
    if not cands:
      raise SystemExit("%s: no sha256-matching blob in the wasm data segments" % name)
    c = cands[0]
    blob = wasm[c["wasm_file_offset"]:c["wasm_file_offset"] + c["size"]]
    if hashlib.sha256(blob).hexdigest() != ptr["oid"] or len(blob) != ptr["size"]:
      raise SystemExit("%s: blob does not match the pointer oid/size" % name)
    blobs[name] = (blob, c, ptr)

  def prov(name):
    blob, c, ptr = blobs[name]
    return ["Source: %s (%d bytes), sha256 %s (= git-lfs pointer oid)." % (name, ptr["size"], ptr["oid"]),
            "Found in xr-slam.wasm sha256 %s at wasm file offset %d, linear memory 0x%x." % (
                wasm_sha, c["wasm_file_offset"], c["linear_memory_addr"])]

  dsize, ddata = parse_dictionary(blobs[DICT_NAME][0])
  if dsize != DESCRIPTOR_BYTES or len(ddata) != DESCRIPTOR_BYTES * DICT_ENTRIES:
    raise SystemExit("dictionary shape: descriptorSize=%d data=%d bytes" % (dsize, len(ddata)))
  bvs, basis, lut, idx, whitening = parse_pca(blobs[PCA_NAME][0])
  if (bvs != PCA_BASIS_VECTOR_BITS or len(basis) != PCA_BASIS_WORDS or len(idx) != PCA_BASIS_VECTOR_BITS
      or len(whitening) != PCA_DIMS or not 0 < len(lut) <= 256):
    raise SystemExit("pca shape: bvs=%d basis=%d idx=%d whitening=%d lut=%d" % (
        bvs, len(basis), len(idx), len(whitening), len(lut)))
  if max(idx) >= len(lut):
    raise SystemExit("pca translationIdx references LUT entry %d >= %d" % (max(idx), len(lut)))
  return {
      DICT_INC: render_dictionary_inc(dsize, ddata, prov(DICT_NAME)),
      PCA_INC: render_pca_inc(bvs, basis, lut, idx, whitening, prov(PCA_NAME)),
  }


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--pointer-dir", required=True)
  ap.add_argument("--out-dir", required=True)
  ap.add_argument("--expect-wasm-sha256")
  ap.add_argument("--check", action="store_true", help="compare with the files in --out-dir instead of writing")
  a = ap.parse_args(argv)
  files = build(a.wasm, a.pointer_dir, a.expect_wasm_sha256)
  rc = 0
  for name, text in sorted(files.items()):
    path = os.path.join(a.out_dir, name)
    data = text.encode("ascii")
    if a.check:
      same = os.path.exists(path) and open(path, "rb").read() == data
      print("%s %s" % ("OK  " if same else "DIFF", path))
      rc |= 0 if same else 1
    else:
      os.makedirs(a.out_dir, exist_ok=True)
      with open(path, "wb") as f:
        f.write(data)
      print("wrote %s (%d bytes, sha256 %s)" % (path, len(data), hashlib.sha256(data).hexdigest()))
  return rc


if __name__ == "__main__":
  sys.exit(main())
