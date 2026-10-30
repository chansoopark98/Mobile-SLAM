#!/usr/bin/env python3
"""Summaries over function-index.json for 01-binary-map.md: counts, module entry tables,
call graph from c8EmAsm_processStagedFrame, ranked unknowns.

Usage: python3 -I report.py --index function-index.json --wasm xr-slam.wasm [--root 508] --out report.json
"""

import argparse
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import wasm_parse  # noqa: E402

W8 = ("8w-closed-slam", "8w-public", "8w-other")
SLAM_MODULES = ("VioTracker", "MapTracker", "MapBuilder", "WorldMaps", "FrameFrameTracker", "MetricScaleEstimator",
                "Alignment", "EpipolarMatcher", "GlobalMatcher", "PosePnP", "Bundle", "Gr8", "Descriptor",
                "FramePoint", "FeatureManager", "Executor")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--index", required=True)
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--root", type=int, default=508)
  ap.add_argument("--out", required=True)
  args = ap.parse_args()

  with open(args.index) as fh:
    idx = json.load(fh)
  fs = {e["index"]: e for e in idx["functions"]}
  mod = wasm_parse.Module(args.wasm)
  callees = {}
  for i, bs, be, cs in mod.bodies:
    callees[i] = set(wasm_parse.decode_function(mod.raw, cs, be, want_seq=False)["calls"])
  callers = collections.defaultdict(set)
  for c, ks in callees.items():
    for k in ks:
      callers[k].add(c)

  # BFS from root over all direct edges.
  dist = {args.root: 0}
  parent = {}
  dq = collections.deque([args.root])
  while dq:
    u = dq.popleft()
    for v in sorted(callees.get(u, ())):
      if v not in dist:
        dist[v] = dist[u] + 1
        parent[v] = u
        dq.append(v)

  def path(v):
    p = [v]
    while p[-1] != args.root and p[-1] in parent:
      p.append(parent[p[-1]])
    return p[::-1] if p[-1] == args.root else None

  reach_cls = collections.Counter(fs[i]["class"] for i in dist)
  reach_bytes = collections.Counter()
  for i in dist:
    reach_bytes[fs[i]["class"]] += fs[i]["size"]

  # Module entries: functions of module M (8w classes) with a caller outside M, or with no direct callers.
  entries = collections.defaultdict(list)
  for i, e in fs.items():
    m = e.get("module")
    if not m or e["class"] not in W8:
      continue
    cs = callers.get(i, set())
    outside = sorted(c for c in cs if fs[c].get("module") != m)
    if outside or not cs:
      entries[(e["class"], m)].append({
          "index": i, "size": e["size"], "label": e["label"], "callers_outside": outside[:6],
          "n_callers": len(cs), "reach_depth": dist.get(i), "path": path(i) if i in dist else None,
          "ghidra_c": e.get("ghidra_c"), "dcmp_lines": e.get("dcmp_lines"), "evidence": e["evidence"][:3]})
  for k in entries:
    entries[k].sort(key=lambda x: -x["size"])

  # Module-level call graph among 8w modules.
  medges = collections.Counter()
  for c, ks in callees.items():
    mc = fs[c].get("module") if fs[c]["class"] in W8 else None
    if not mc:
      continue
    for k in ks:
      mk = fs[k].get("module") if fs[k]["class"] in W8 else None
      if mk and mk != mc:
        medges[(mc, mk)] += 1

  # Big-function skeleton reachable from root (8w, >= 4 KB) with their parent chain.
  skeleton = []
  for i in sorted(dist, key=lambda x: (dist[x], x)):
    e = fs[i]
    if e["class"] in W8 and e["size"] >= 4000:
      skeleton.append({"index": i, "depth": dist[i], "parent": parent.get(i), "size": e["size"], "class": e["class"],
                       "module": e.get("module")})

  # Unknowns ranked: reachable from root, called by closed SLAM code, size.
  unk = []
  for i, e in fs.items():
    if e["class"] != "unknown":
      continue
    cs = callers.get(i, set())
    closed_callers = sorted(c for c in cs if fs[c]["class"] == "8w-closed-slam")
    score = (len(closed_callers) * 4 + len(cs)) * (1 + e["size"] / 500.0) * (2 if i in dist else 1)
    unk.append({"index": i, "size": e["size"], "n_callers": len(cs), "closed_callers": closed_callers[:8],
                "closed_caller_modules": sorted({fs[c].get("module") or "?" for c in closed_callers}),
                "reachable_from_root": i in dist, "score": round(score, 1), "evidence": e["evidence"],
                "ghidra_c": e.get("ghidra_c"), "dcmp_lines": e.get("dcmp_lines")})
  unk.sort(key=lambda x: -x["score"])

  out = {
      "root": args.root,
      "reachable_functions": len(dist),
      "reachable_by_class": dict(reach_cls),
      "reachable_bytes_by_class": dict(reach_bytes),
      "module_entries": {"%s|%s" % k: v for k, v in sorted(entries.items())},
      "module_edges": {"%s->%s" % k: v for k, v in sorted(medges.items(), key=lambda x: -x[1])},
      "skeleton": skeleton,
      "unknowns_ranked": unk[:40],
      "unknowns_total": len(unk),
  }
  with open(args.out, "w") as fh:
    json.dump(out, fh, indent=1)
  print(json.dumps({k: out[k] for k in ("reachable_functions", "reachable_by_class", "unknowns_total")}, indent=1))


if __name__ == "__main__":
  main()
