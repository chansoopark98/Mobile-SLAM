"""Extract the agate dictionary blobs (Cap'n Proto messages) from the original xr-slam.wasm data segment
and verify them byte-exactly against the git-lfs pointer files (sha256 oid + size).

The blobs are `embedded_data` rodata in the closed wasm: flat-array Cap'n Proto messages
(segment table + segments). The BinaryPcaData message of agate-keyframe-binary-pca-32d is a THREE-segment
message (table `02 00 00 00 | 25 00 00 00 | 01 08 00 00 | 01 20 00 00`), so searching for a single-segment
header finds nothing. This tool therefore looks for any well-formed flat-array header whose total byte length
equals a pointer's size, and then compares sha256.

Usage (cwd = repo root):
  python3 -B -I slam_refact/tools/re/agate/extract_agate_dict.py \
      --wasm build/slam-refact-20261015/inputs/xr-slam.wasm \
      --pointer-dir assets/references/xr_reference/official/archive/reality/engine/features/dictionary \
      --out-dir build/slam-refact-us025-20261024/lc2a/blobs \
      --report build/slam-refact-us025-20261024/lc2a/extract-report.json

Exit code 0 when every blob named by --require (default: agate-keyframe-binary-pca-32d.capnpbin) was found
with sha256 and size equal to its pointer; 1 otherwise. Blobs are written only when --out-dir is given and only
for sha-verified matches. Reads only; no network; does not touch the repo tree.
"""

import argparse
import hashlib
import json
import os
import re
import sys

DEFAULT_REQUIRE = ["agate-keyframe-binary-pca-32d.capnpbin"]


def _leb_u(buf, p):
  r = 0
  s = 0
  while True:
    b = buf[p]
    p += 1
    r |= (b & 0x7F) << s
    s += 7
    if not b & 0x80:
      return r, p


def _leb_s(buf, p):
  r = 0
  s = 0
  while True:
    b = buf[p]
    p += 1
    r |= (b & 0x7F) << s
    s += 7
    if not b & 0x80:
      if b & 0x40:
        r -= 1 << s
      return r, p


def parse_data_segments(wasm):
  """Return [{index, flag, addr (linear memory, None if passive), file_offset, size}] of the data section."""
  if wasm[:4] != b"\0asm":
    raise ValueError("not a wasm file")
  p = 8
  while p < len(wasm):
    sid = wasm[p]
    p += 1
    size, p = _leb_u(wasm, p)
    if sid != 11:
      p += size
      continue
    end = p + size
    n, q = _leb_u(wasm, p)
    segs = []
    for i in range(n):
      flag, q = _leb_u(wasm, q)
      addr = None
      if flag == 2:
        _, q = _leb_u(wasm, q)  # memory index
      if flag in (0, 2):
        if wasm[q] != 0x41:  # only i32.const offsets are expected for this module
          raise ValueError("data segment %d: non i32.const offset expr 0x%02x" % (i, wasm[q]))
        q += 1
        addr, q = _leb_s(wasm, q)
        if wasm[q] != 0x0B:
          raise ValueError("data segment %d: offset expr not terminated" % i)
        q += 1
      length, q = _leb_u(wasm, q)
      segs.append({"index": i, "flag": flag, "addr": addr, "file_offset": q, "size": length})
      q += length
    if q != end:
      raise ValueError("data section length mismatch: parsed to %d, section ends %d" % (q, end))
    return segs
  raise ValueError("no data section")


def read_pointers(pointer_dir):
  out = {}
  for name in sorted(os.listdir(pointer_dir)):
    if not name.endswith(".capnpbin"):
      continue
    with open(os.path.join(pointer_dir, name), "rb") as f:
      text = f.read(4096).decode("ascii", "replace")
    m_oid = re.search(r"^oid sha256:([0-9a-f]{64})$", text, re.M)
    m_size = re.search(r"^size (\d+)$", text, re.M)
    if text.startswith("version https://git-lfs.github.com/spec/v1") and m_oid and m_size:
      out[name] = {"oid": m_oid.group(1), "size": int(m_size.group(1))}
  return out


def flat_message_length(buf, off):
  """Total byte length of a flat-array Cap'n Proto message starting at off, or None if the header is not plausible."""
  if off + 8 > len(buf):
    return None
  nseg = int.from_bytes(buf[off:off + 4], "little") + 1
  if nseg > 16:
    return None
  header = 4 + 4 * nseg
  header += header % 8
  if off + header > len(buf):
    return None
  words = 0
  for i in range(nseg):
    words += int.from_bytes(buf[off + 4 + 4 * i:off + 8 + 4 * i], "little")
  total = header + 8 * words
  if words == 0 or off + total > len(buf):
    return None
  return total


def find_blob(wasm, segs, size, oid):
  """Scan each data segment (4-byte steps) for a message of exactly `size` bytes; return all candidates."""
  cands = []
  for sg in segs:
    base = sg["file_offset"]
    seg = wasm[base:base + sg["size"]]
    for off in range(0, len(seg) - size + 1, 4):
      if flat_message_length(seg, off) != size:
        continue
      blob = seg[off:off + size]
      h = hashlib.sha256(blob).hexdigest()
      cands.append({
          "data_segment": sg["index"],
          "wasm_file_offset": base + off,
          "linear_memory_addr": None if sg["addr"] is None else sg["addr"] + off,
          "size": size,
          "sha256": h,
          "sha_match": h == oid,
          "segment_table_words": [
              int.from_bytes(blob[4 + 4 * i:8 + 4 * i], "little")
              for i in range(int.from_bytes(blob[0:4], "little") + 1)
          ],
      })
  return cands


def run(wasm_path, pointer_dir, out_dir, require):
  with open(wasm_path, "rb") as f:
    wasm = f.read()
  wasm_sha = hashlib.sha256(wasm).hexdigest()
  segs = parse_data_segments(wasm)
  pointers = read_pointers(pointer_dir)
  entries = {}
  for name, ptr in pointers.items():
    cands = find_blob(wasm, segs, ptr["size"], ptr["oid"])
    ok = [c for c in cands if c["sha_match"]]
    entries[name] = {
        "pointer_oid": ptr["oid"],
        "pointer_size": ptr["size"],
        "status": "FOUND_SHA_MATCH" if ok else ("SIZE_ONLY_NO_SHA_MATCH" if cands else "NOT_IN_WASM"),
        "matches": ok,
        "size_matching_candidates_without_sha_match": [c for c in cands if not c["sha_match"]],
    }
    if ok and out_dir:
      os.makedirs(out_dir, exist_ok=True)
      c = ok[0]
      with open(os.path.join(out_dir, name), "wb") as f:
        f.write(wasm[c["wasm_file_offset"]:c["wasm_file_offset"] + c["size"]])
  req_ok = all(entries.get(n, {}).get("status") == "FOUND_SHA_MATCH" for n in require)
  return {
      "wasm": wasm_path,
      "wasm_sha256": wasm_sha,
      "wasm_size": len(wasm),
      "data_segments": [{k: s[k] for k in ("index", "flag", "addr", "file_offset", "size")} for s in segs],
      "pointer_dir": pointer_dir,
      "required": require,
      "required_all_found_sha_match": req_ok,
      "entries": entries,
  }


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--pointer-dir", required=True)
  ap.add_argument("--out-dir", help="write sha-verified blobs here (build/ only; never the repo tree)")
  ap.add_argument("--report", help="write the JSON report here")
  ap.add_argument("--require", nargs="+", default=DEFAULT_REQUIRE)
  ap.add_argument("--expect-wasm-sha256", help="refuse to run on a different wasm")
  a = ap.parse_args(argv)
  rep = run(a.wasm, a.pointer_dir, a.out_dir, a.require)
  if a.expect_wasm_sha256 and rep["wasm_sha256"] != a.expect_wasm_sha256:
    print("wasm sha256 %s != expected %s" % (rep["wasm_sha256"], a.expect_wasm_sha256), file=sys.stderr)
    return 2
  if a.report:
    os.makedirs(os.path.dirname(os.path.abspath(a.report)), exist_ok=True)
    with open(a.report, "w") as f:
      json.dump(rep, f, indent=2, sort_keys=True)
      f.write("\n")
  for name, e in rep["entries"].items():
    where = ""
    if e["matches"]:
      m = e["matches"][0]
      where = " wasm_off=%d addr=0x%x segs=%s" % (m["wasm_file_offset"], m["linear_memory_addr"], m["segment_table_words"])
    print("%-52s %-24s size=%d%s" % (name, e["status"], e["pointer_size"], where))
  print("required %s: %s" % (",".join(a.require), "OK" if rep["required_all_found_sha_match"] else "MISSING"))
  return 0 if rep["required_all_found_sha_match"] else 1


if __name__ == "__main__":
  sys.exit(main())
