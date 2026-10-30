"""Map-building cadence from original reference runtime reference traces (lane S3 parity baseline).

Reads build/slam-refact-20261015/reference/runs/<scenario>/<run>/trace.jsonl and reports, per run,
in PROCESSED-frame units (fresh callbacks only; dropped input frames are skipped like the engine):
  - first translation (position leaves the configured origin) and first public world points,
  - the gap between them (frames and seconds),
  - "burst" frames where public world points grew by >= --burst points (keyframe proxy),
  - burst interval histogram and the share of intervals equal to 5 or 6 (keyframeCooldown = 5).

Usage (cwd = repo root):
  python3 -I slam_refact/tools/re/spec_map_builder/cadence.py \
      --runs build/slam-refact-20261015/reference/runs --burst 10 \
      --out build/slam-refact-20261015/spec/map-builder/cadence.json
"""

import argparse
import collections
import glob
import json
import os


def analyze(path, burst):
  rows = [json.loads(line) for line in open(path)]
  fresh = [r for r in rows if r["fresh"]]
  origin = fresh[0]["p"]
  k_first_move = next((k for k, r in enumerate(fresh) if r["p"] != origin), None)
  k_first_wp = next((k for k, r in enumerate(fresh) if r["worldPoints"] > 0), None)
  bursts = [k for k, r in enumerate(fresh) if r["wpAdded"] >= burst]
  intervals = [b - a for a, b in zip(bursts, bursts[1:])]
  hist = collections.Counter(intervals)
  # Bursts after map init only (first 200 processed frames after first world points).
  early = [k for k in bursts if k_first_wp is not None and k_first_wp <= k < k_first_wp + 200]
  early_iv = [b - a for a, b in zip(early, early[1:])]
  return {
    "fresh_frames": len(fresh),
    "first_translation": None if k_first_move is None else {
      "processed_k": k_first_move, "input_i": fresh[k_first_move]["i"], "t": fresh[k_first_move]["t"]},
    "first_world_points": None if k_first_wp is None else {
      "processed_k": k_first_wp, "input_i": fresh[k_first_wp]["i"], "t": fresh[k_first_wp]["t"],
      "count": fresh[k_first_wp]["worldPoints"]},
    "init_gap_frames": None if None in (k_first_move, k_first_wp) else k_first_wp - k_first_move,
    "init_gap_s": None if None in (k_first_move, k_first_wp) else
      round(fresh[k_first_wp]["t"] - fresh[k_first_move]["t"], 4),
    "first_bursts_input_i": [fresh[k]["i"] for k in bursts[:8]],
    "first_bursts_added": [fresh[k]["wpAdded"] for k in bursts[:8]],
    "n_bursts": len(bursts),
    "interval_hist": dict(sorted(hist.items())),
    "share_interval_1": round(hist.get(1, 0) / max(1, len(intervals)), 4),
    "share_interval_5_6": round((hist.get(5, 0) + hist.get(6, 0)) / max(1, len(intervals)), 4),
    "early_intervals": early_iv[:20],
  }


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--runs", required=True)
  ap.add_argument("--burst", type=int, default=10)
  ap.add_argument("--out", required=True)
  a = ap.parse_args()
  res = {}
  for path in sorted(glob.glob(os.path.join(a.runs, "*", "r[0-9]*", "trace.jsonl"))):
    key = os.path.relpath(os.path.dirname(path), a.runs)
    res[key] = analyze(path, a.burst)
  json.dump({"burst_threshold": a.burst, "runs": res}, open(a.out, "w"), indent=1)
  for k, v in res.items():
    print("%-22s move=%s wp=%s gap=%s frames/%ss bursts=%d iv5-6=%.2f iv1=%.2f early=%s" % (
      k, v["first_translation"] and v["first_translation"]["input_i"],
      v["first_world_points"] and v["first_world_points"]["input_i"], v["init_gap_frames"], v["init_gap_s"],
      v["n_bursts"], v["share_interval_5_6"], v["share_interval_1"], v["early_intervals"][:10]))


if __name__ == "__main__":
  main()
