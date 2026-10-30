"""Tests for gen_agate_inc.py (capnp reader on synthetic messages; the committed .inc files against the pinned wasm).

  python3 -B -I slam_refact/tools/re/agate/test_gen_agate_inc.py -v      (from the repo root)
The last test regenerates both .inc files from the pinned original wasm and compares them byte for byte with the
committed files; it is skipped when the wasm / pointer directory are not present.
"""
import pathlib
import struct
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True
import gen_agate_inc as g  # noqa: E402

ROOT = HERE.parents[3]
REAL_WASM = ROOT / "build/slam-refact-20261015/inputs/xr-slam.wasm"
REAL_PTRS = ROOT / "assets/references/xr_reference/official/archive/reality/engine/features/dictionary"
REAL_WASM_SHA = "780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422"
INC_DIR = ROOT / "slam_refact/engine/src/map_builder"


def struct_ptr(offset, dwords, nptrs):
  return struct.pack("<Q", ((offset & 0x3FFFFFFF) << 2) | (dwords << 32) | (nptrs << 48))


def list_ptr(offset, code, count):
  return struct.pack("<Q", 1 | ((offset & 0x3FFFFFFF) << 2) | (code << 32) | (count << 35))


def far_ptr(seg, word, double=False):
  return struct.pack("<Q", 2 | (int(double) << 2) | (word << 3) | (seg << 32))


def message(*segments):
  head = struct.pack("<I", len(segments) - 1) + b"".join(struct.pack("<I", len(s) // 8) for s in segments)
  if len(head) % 8:
    head += b"\0" * (8 - len(head) % 8)
  return head + b"".join(segments)


class CapnpReader(unittest.TestCase):
  def test_single_segment_dictionary(self):
    data = bytes(range(64))
    seg = struct_ptr(0, 1, 1) + struct.pack("<II", 32, 0) + list_ptr(0, 2, len(data)) + data
    size, got = g.parse_dictionary(message(seg))
    self.assertEqual(size, 32)
    self.assertEqual(got, data)

  def test_far_pointer_to_other_segment(self):
    data = bytes(range(32))
    seg0 = far_ptr(1, 0)                                      # root: far pointer to the landing pad in segment 1
    seg1 = struct_ptr(0, 1, 1) + struct.pack("<II", 32, 0) + far_ptr(2, 0)   # struct at pad+1, its ptr -> segment 2
    seg2 = list_ptr(0, 2, len(data)) + data                   # landing pad (list tag) then content
    size, got = g.parse_dictionary(message(seg0, seg1, seg2))
    self.assertEqual(size, 32)
    self.assertEqual(got, data)

  def test_double_far_pointer(self):
    data = bytes(range(16))
    seg0 = far_ptr(1, 0, double=True)                          # root: double far, pad at segment 1 word 0
    seg1 = far_ptr(2, 0) + struct_ptr(0, 1, 1)                 # pad: far -> content in segment 2, tag = struct(1,1)
    seg2 = struct.pack("<II", 32, 0) + list_ptr(0, 2, len(data)) + data
    size, got = g.parse_dictionary(message(seg0, seg1, seg2))
    self.assertEqual(size, 32)
    self.assertEqual(got, data)

  def test_segment_table_must_cover_blob(self):
    with self.assertRaises(ValueError):
      g.CapnpMessage(message(struct_ptr(0, 0, 0)) + b"\0" * 8)

  def test_wrong_list_element_code_is_refused(self):
    seg = struct_ptr(0, 1, 1) + struct.pack("<II", 32, 0) + list_ptr(0, 4, 4) + b"\0" * 16
    with self.assertRaises(ValueError):
      g.parse_dictionary(message(seg))


@unittest.skipUnless(REAL_WASM.is_file() and REAL_PTRS.is_dir(), "pinned original wasm / pointer dir not present")
class RealWasm(unittest.TestCase):
  def test_committed_inc_files_equal_regeneration(self):
    files = g.build(str(REAL_WASM), str(REAL_PTRS), REAL_WASM_SHA)
    self.assertEqual(sorted(files), [g.DICT_INC, g.PCA_INC])
    for name, text in files.items():
      self.assertEqual((INC_DIR / name).read_bytes(), text.encode("ascii"), name)

  def test_wrong_wasm_pin_is_refused(self):
    with self.assertRaises(SystemExit):
      g.build(str(REAL_WASM), str(REAL_PTRS), "0" * 64)


if __name__ == "__main__":
  unittest.main()
