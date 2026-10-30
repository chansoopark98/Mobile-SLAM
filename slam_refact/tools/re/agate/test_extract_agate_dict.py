"""Tests for extract_agate_dict.py.

  python3 -B -I slam_refact/tools/re/agate/test_extract_agate_dict.py -v      (from the repo root)
Synthetic wasm + pointer files (no network, no repo data). The last test runs on the real pinned wasm only if
build/slam-refact-20261015/inputs/xr-slam.wasm and the pointer directory exist (otherwise skipped).
"""
import hashlib
import json
import os
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True
import extract_agate_dict as x  # noqa: E402

ROOT = HERE.parents[3]
REAL_WASM = ROOT / "build/slam-refact-20261015/inputs/xr-slam.wasm"
REAL_PTRS = ROOT / "assets/references/xr_reference/official/archive/reality/engine/features/dictionary"
REAL_WASM_SHA = "780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422"


def uleb(v):
  out = bytearray()
  while True:
    b = v & 0x7F
    v >>= 7
    if v:
      out.append(b | 0x80)
    else:
      out.append(b)
      return bytes(out)


def sleb(v):
  out = bytearray()
  while True:
    b = v & 0x7F
    v >>= 7
    if (v == 0 and not b & 0x40) or (v == -1 and b & 0x40):
      out.append(b)
      return bytes(out)
    out.append(b | 0x80)


def capnp_message(seg_words):
  """Flat-array message with segments of the given word counts (zero-filled apart from a marker)."""
  hdr = (len(seg_words) - 1).to_bytes(4, "little") + b"".join(w.to_bytes(4, "little") for w in seg_words)
  if len(hdr) % 8:
    hdr += b"\0" * (8 - len(hdr) % 8)
  body = b"".join(bytes([(i * 7 + 3) & 0xFF]) * (8 * w) for i, w in enumerate(seg_words))
  return hdr + body


def make_wasm(segments):
  """segments: [(addr, bytes)] -> wasm with only a data section (section order is irrelevant to the parser)."""
  payload = uleb(len(segments))
  for addr, data in segments:
    payload += uleb(0) + b"\x41" + sleb(addr) + b"\x0b" + uleb(len(data)) + data
  return b"\0asm\x01\0\0\0" + b"\x0b" + uleb(len(payload)) + payload


def pointer_text(blob):
  return "version https://git-lfs.github.com/spec/v1\noid sha256:%s\nsize %d\n" % (hashlib.sha256(blob).hexdigest(), len(blob))


class SyntheticTests(unittest.TestCase):

  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.dir = pathlib.Path(self.tmp.name)
    self.ptrs = self.dir / "ptrs"
    self.ptrs.mkdir()
    # 3-segment message like the real BinaryPcaData one (an old single-segment-header search would miss it)
    self.multi = capnp_message([5, 3, 9])
    self.single = capnp_message([4])
    self.pad = b"\x11" * 24
    self.wasm = make_wasm([(1024, self.pad + self.single + self.pad + self.multi + self.pad)])
    (self.dir / "a.wasm").write_bytes(self.wasm)
    (self.ptrs / "multi.capnpbin").write_text(pointer_text(self.multi))
    (self.ptrs / "single.capnpbin").write_text(pointer_text(self.single))
    (self.ptrs / "notapointer.capnpbin").write_bytes(b"\0" * 64)

  def tearDown(self):
    self.tmp.cleanup()

  def test_data_segment_parse_and_offsets(self):
    segs = x.parse_data_segments(self.wasm)
    self.assertEqual(len(segs), 1)
    self.assertEqual(segs[0]["addr"], 1024)
    self.assertEqual(self.wasm[segs[0]["file_offset"]:segs[0]["file_offset"] + segs[0]["size"]][:24], self.pad)

  def test_multi_segment_found_and_sha_match(self):
    rep = x.run(str(self.dir / "a.wasm"), str(self.ptrs), str(self.dir / "out"), ["multi.capnpbin", "single.capnpbin"])
    self.assertTrue(rep["required_all_found_sha_match"])
    m = rep["entries"]["multi.capnpbin"]["matches"][0]
    self.assertEqual(m["linear_memory_addr"], 1024 + 24 + len(self.single) + 24)
    self.assertEqual(m["segment_table_words"], [5, 3, 9])
    self.assertEqual((self.dir / "out" / "multi.capnpbin").read_bytes(), self.multi)
    self.assertNotIn("notapointer.capnpbin", rep["entries"])

  def test_one_byte_difference_is_not_a_match(self):
    bad = bytearray(self.wasm)
    i = bad.find(self.multi) + len(self.multi) - 1
    bad[i] ^= 1
    (self.dir / "bad.wasm").write_bytes(bytes(bad))
    rep = x.run(str(self.dir / "bad.wasm"), str(self.ptrs), str(self.dir / "out2"), ["multi.capnpbin"])
    self.assertFalse(rep["required_all_found_sha_match"])
    e = rep["entries"]["multi.capnpbin"]
    self.assertEqual(e["status"], "SIZE_ONLY_NO_SHA_MATCH")
    self.assertFalse((self.dir / "out2" / "multi.capnpbin").exists())

  def test_absent_blob(self):
    other = capnp_message([6, 6])
    (self.ptrs / "other.capnpbin").write_text(pointer_text(other))
    rep = x.run(str(self.dir / "a.wasm"), str(self.ptrs), None, ["other.capnpbin"])
    self.assertEqual(rep["entries"]["other.capnpbin"]["status"], "NOT_IN_WASM")
    self.assertFalse(rep["required_all_found_sha_match"])

  def test_main_exit_codes_and_wasm_pin(self):
    base = ["--wasm", str(self.dir / "a.wasm"), "--pointer-dir", str(self.ptrs), "--require", "multi.capnpbin"]
    self.assertEqual(x.main(base + ["--report", str(self.dir / "r.json")]), 0)
    self.assertTrue(json.loads((self.dir / "r.json").read_text())["required_all_found_sha_match"])
    self.assertEqual(x.main(base + ["--expect-wasm-sha256", "0" * 64]), 2)


@unittest.skipUnless(REAL_WASM.is_file() and REAL_PTRS.is_dir(), "pinned original wasm / pointer dir not present")
class RealWasmTests(unittest.TestCase):

  def test_pca_32d_present_and_byte_exact(self):
    rep = x.run(str(REAL_WASM), str(REAL_PTRS), None, x.DEFAULT_REQUIRE)
    self.assertEqual(rep["wasm_sha256"], REAL_WASM_SHA)
    e = rep["entries"]["agate-keyframe-binary-pca-32d.capnpbin"]
    self.assertEqual(e["status"], "FOUND_SHA_MATCH")
    self.assertEqual(e["pointer_size"], 82248)
    self.assertEqual(e["matches"][0]["wasm_file_offset"], 3714951)
    self.assertEqual(e["matches"][0]["linear_memory_addr"], 0x663B0)
    self.assertEqual(e["matches"][0]["segment_table_words"], [0x25, 0x801, 0x2001])
    # the multi-PCA (328,080 B) is NOT embedded in this wasm
    self.assertEqual(rep["entries"]["agate-keyframe-binary-multi-pca-32d.capnpbin"]["status"], "NOT_IN_WASM")


if __name__ == "__main__":
  unittest.main()
