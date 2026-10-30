"""Copy the small agate golden vectors (US-025 LC-2b) from the lc-g dump into engine/tests/data/agate/.

Sources (original xr-slam.wasm, produced by the LC-G worker, read only):
  G-A  <golden>/G-A/<run>/{keyframes.jsonl,orb.bin,SHA256SUMS}: every keyframe created in a real run, with the
       keyframe's ORB descriptor store (kf+0x2c) and the 32 B the original wrote to WorldMap+0x184 (func 1263).
  G-B  <golden>/G-B/{cases.jsonl,blobs.bin}: func 1263 called alone; only the 20 synthetic cases are copied (the
       187 real cases are the G-A keyframes again).

Outputs (compact, no JSON parsing in the C++ test):
  ga-<run>.tsv   k  orb_n  orb_off  orb_sha256  agate_hex        (one line per keyframe; '#' comment lines)
  ga-<run>.bin   the run's orb.bin, unchanged
  gb-synthetic.tsv  name  n  off  agate_hex
  gb-synthetic.bin  the synthetic inputs, concatenated (offsets in the tsv)

Usage (cwd = repo root):
  python3 -B -I slam_refact/tools/re/agate/gen_agate_test_data.py \
      --golden build/slam-refact-pkgh-20261022/us025/golden --out slam_refact/engine/tests/data/agate
Refuses to run without <golden>/G-A/READY unless --allow-unready is given (marker of the dump worker).
Every source file is checked against the run's SHA256SUMS; every orb slice against its sha256.
"""

import argparse
import hashlib
import json
import os
import sys


def sha256(b):
  return hashlib.sha256(b).hexdigest()


def read(path):
  with open(path, "rb") as f:
    return f.read()


def check_sums(run_dir):
  sums = {}
  for line in read(os.path.join(run_dir, "SHA256SUMS")).decode().splitlines():
    digest, name = line.split(None, 1)
    sums[name.strip()] = digest
  for name, digest in sums.items():
    if sha256(read(os.path.join(run_dir, name))) != digest:
      raise SystemExit("%s: %s does not match SHA256SUMS" % (run_dir, name))


def convert_ga(golden, out):
  root = os.path.join(golden, "G-A")
  runs = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
  if not runs:
    raise SystemExit("no G-A runs in %s" % root)
  total = 0
  for run in runs:
    run_dir = os.path.join(root, run)
    check_sums(run_dir)
    orb = read(os.path.join(run_dir, "orb.bin"))
    rows = [json.loads(l) for l in read(os.path.join(run_dir, "keyframes.jsonl")).decode().splitlines() if l.strip()]
    lines = ["# US-025 LC-2b G-A %s: k\torb_n\torb_off\torb_sha256\tagate_hex (32 B written by the original func 1263)" % run]
    for r in rows:
      if r["orb_len"] != 32 * r["orb_n"]:
        raise SystemExit("%s k=%d: orb_len %d != 32*orb_n" % (run, r["k"], r["orb_len"]))
      if sha256(orb[r["orb_off"]:r["orb_off"] + r["orb_len"]]) != r["orb_sha256"]:
        raise SystemExit("%s k=%d: orb slice sha256 mismatch" % (run, r["k"]))
      if len(r["agate_hex"]) != 64:
        raise SystemExit("%s k=%d: agate_hex length" % (run, r["k"]))
      lines.append("%d\t%d\t%d\t%s\t%s" % (r["k"], r["orb_n"], r["orb_off"], r["orb_sha256"], r["agate_hex"]))
    with open(os.path.join(out, "ga-%s.tsv" % run), "w") as f:
      f.write("\n".join(lines) + "\n")
    with open(os.path.join(out, "ga-%s.bin" % run), "wb") as f:
      f.write(orb)
    total += len(rows)
    print("G-A %s: %d keyframes, %d bytes of ORB" % (run, len(rows), len(orb)))
  return total


def convert_gb(golden, out):
  root = os.path.join(golden, "G-B")
  blobs = read(os.path.join(root, "blobs.bin"))
  rows = [json.loads(l) for l in read(os.path.join(root, "cases.jsonl")).decode().splitlines() if l.strip()]
  data = bytearray()
  lines = ["# US-025 LC-2b G-B synthetic: name\tn\toff\tagate_hex (func 1263 called alone on the original)"]
  count = 0
  for r in rows:
    if r["kind"] != "synthetic":
      continue
    inp = r["input"]
    chunk = blobs[inp["off"]:inp["off"] + inp["len"]]
    if sha256(chunk) != inp["sha256"] or len(chunk) != 32 * r["n"]:
      raise SystemExit("G-B %s: input slice check failed" % r["name"])
    outp = r["output"]
    if blobs[outp["off"]:outp["off"] + 32].hex() != r["outputHex"]:
      raise SystemExit("G-B %s: output blob != outputHex" % r["name"])
    if not r.get("repeatSame", False) or not r.get("outputWritten", False):
      raise SystemExit("G-B %s: dump flags unexpected" % r["name"])
    lines.append("%s\t%d\t%d\t%s" % (r["name"], r["n"], len(data), r["outputHex"]))
    data += chunk
    count += 1
  with open(os.path.join(out, "gb-synthetic.tsv"), "w") as f:
    f.write("\n".join(lines) + "\n")
  with open(os.path.join(out, "gb-synthetic.bin"), "wb") as f:
    f.write(bytes(data))
  print("G-B synthetic: %d cases, %d bytes" % (count, len(data)))
  return count


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument("--golden", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--allow-unready", action="store_true")
  a = ap.parse_args(argv)
  if not os.path.exists(os.path.join(a.golden, "G-A", "READY")) and not a.allow_unready:
    raise SystemExit("%s/G-A/READY is missing (the dump is not final); use --allow-unready only for a trial" % a.golden)
  os.makedirs(a.out, exist_ok=True)
  convert_ga(a.golden, a.out)
  convert_gb(a.golden, a.out)
  return 0


if __name__ == "__main__":
  sys.exit(main())
