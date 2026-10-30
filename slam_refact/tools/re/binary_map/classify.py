#!/usr/bin/env python3
"""Classify every function of xr-slam.wasm and write function-index.json.

Usage (from repo root):
  python3 -I slam_refact/tools/re/binary_map/classify.py \
      --wasm build/slam-refact-20261015/inputs/xr-slam.wasm \
      --dcmp build/slam-refact-20261015/re/wabt/xr-slam.dcmp \
      --public-src assets/references/xr_reference/official/xr_reference \
      --ghidra-export build/slam-refact-20261015/re/ghidra/export \
      --out build/slam-refact-20261015/re/function-index.json

Python 3 stdlib only (plus `c++filt` for demangling RTTI names).
"""

import argparse
import collections
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import evidence as ev_mod  # noqa: E402
import rules  # noqa: E402

LIB_CLASSES = ("emscripten-runtime", "libc/libc++", "eigen", "ceres", "glog-mini/other-3p")
W8_CLASSES = ("8w-closed-slam", "8w-other", "8w-public")
GENERIC_STRING_RE = re.compile(r"^[\s%a-zA-Z0-9_.,:;=<>!+\-*/()\[\]{}|&^~'\"]{0,7}$")


class Vote:
  __slots__ = ("cls", "module", "method", "evidence", "strength")

  def __init__(self, cls, module, method, evidence, strength):
    self.cls = cls
    self.module = module
    self.method = method
    self.evidence = evidence
    self.strength = strength


def parse_dcmp_ranges(path):
  """Return {func index: (first line, last line)} for wasm-decompile output (1-based lines)."""
  hdr = re.compile(r"^(?:export |import )?function .*// func(\d+)\s*$")
  starts = []
  n = 0
  with open(path, encoding="utf-8", errors="replace") as fh:
    for n, line in enumerate(fh, 1):
      if line.startswith(("function ", "export function ", "import function ")):
        m = hdr.match(line.rstrip("\n"))
        if m:
          starts.append((int(m.group(1)), n))
  ranges = {}
  for i, (idx, ln) in enumerate(starts):
    end = starts[i + 1][1] - 1 if i + 1 < len(starts) else n
    ranges[idx] = (ln, end)
  return ranges


def load_ghidra(export_dir, ev):
  """Map Ghidra functions.jsonl records to wasm function indices by entry address.

  Ghidra's WASM loader places imported function k at 0x7f000000 + 4*k and a defined function at
  0x80000000 + (file offset of its body, i.e. the byte after the body-size LEB). Names are
  cross-checked: `unnamed_function_<N>` must have N == wasm index, named ones must be the export
  (or import field) name of that index.
  """
  path = os.path.join(export_dir, "functions.jsonl")
  if not os.path.exists(path):
    return {}, {"status": "BLOCKED", "reason": "functions.jsonl not present at classification time"}
  by_offset = {f["file_offset"]: idx for idx, f in ev.feats.items()}
  imports = [i for i in ev.mod.imports if i[2] == 0]
  n_imp = len(imports)
  by_idx = {}
  stats = collections.Counter()
  bad = []
  with open(path, encoding="utf-8", errors="replace") as fh:
    for line in fh:
      line = line.strip()
      if not line:
        continue
      try:
        r = json.loads(line)
      except json.JSONDecodeError:
        stats["bad_json"] += 1
        continue
      stats["records"] += 1
      try:
        entry = int(r.get("entry", "").split(":")[-1], 16)
      except ValueError:
        stats["bad_entry"] += 1
        continue
      if 0x7F000000 <= entry < 0x80000000 and (entry - 0x7F000000) % 4 == 0:
        idx = (entry - 0x7F000000) // 4
        if idx >= n_imp:
          stats["import_out_of_range"] += 1
          continue
        expect = {imports[idx][1]}
      elif entry >= 0x80000000 and entry - 0x80000000 in by_offset:
        idx = by_offset[entry - 0x80000000]
        expect = set(ev.mod.exports.get(idx, [])) | {"unnamed_function_%d" % idx}
      else:
        stats["unmapped_entry"] += 1
        bad.append(r.get("name"))
        continue
      nm = r.get("name", "")
      if nm in expect or nm.split("::")[-1] in expect:
        stats["name_agrees"] += 1
      else:
        stats["name_differs"] += 1
        if len(bad) < 20:
          bad.append("%s@%d" % (nm, idx))
      by_idx[idx] = r
  info = {
      "status": "VERIFIED" if by_idx else "BLOCKED",
      "mapped_functions": len(by_idx),
      "defined_mapped": sum(1 for i in by_idx if i >= n_imp),
      "decompiled_ok": sum(1 for r in by_idx.values() if r.get("decompiled")),
      "stats": dict(stats),
      "examples_name_mismatch_or_unmapped": bad[:20],
      "rule": "import k -> 0x7f000000+4k; defined -> 0x80000000+body_file_offset; unnamed_function_N == wasm index N",
  }
  return by_idx, info


def string_votes(ev, idx, pub_literals):
  votes = []
  refs = ev.str_refs.get(idx, [])
  cfg_by_class = collections.defaultdict(list)
  for addr, text, exact in refs:
    if not exact:
      continue
    m = re.match(r"^([A-Z][A-Za-z0-9]+)\.([a-z][A-Za-z0-9]+)$", text)
    if m:
      cfg_by_class[m.group(1)].append(text)
      continue
    if text in rules.CLOSED_SCOPES:
      votes.append(Vote("8w-closed-slam", rules.CLOSED_SCOPES[text], "scope-timer-string",
                        "scope:%s" % text, 3))
      continue
    if text in rules.OTHER_SCOPES:
      votes.append(Vote("8w-other", rules.OTHER_SCOPES[text], "scope-timer-string", "scope:%s" % text, 3))
      continue
    if text in rules.PUBLIC_SCOPE_MODULES:
      votes.append(Vote("8w-public", rules.PUBLIC_SCOPE_MODULES[text], "scope-timer-string",
                        "scope:%s (public src %s)" % (text, pub_literals.get(text, "?")),
                        3 if text in rules.EXECUTOR_SCOPES else 2))
      continue
    if text in rules.PUBLIC_OTHER_SCOPES:
      votes.append(Vote("8w-other", rules.PUBLIC_OTHER_SCOPES[text], "scope-timer-string",
                        "scope:%s (public src %s)" % (text, pub_literals.get(text, "?")), 2))
      continue
    if rules.VPS_STATIC_INIT_RE.match(text):
      votes.append(Vote("8w-other", "VPS", "vps-endpoint-string", "str:%s" % text[:60], 2))
      continue
    pm = re.match(r"^\[([A-Za-z0-9_:\-\. ]+)(?:@[^\]]*)?\]", text)
    if pm and pm.group(1) in rules.LOG_PREFIX:
      cls, mod = rules.LOG_PREFIX[pm.group(1)]
      votes.append(Vote(cls, mod, "log-prefix-string", "log:%s" % text[:60],
                        2 if pm.group(1) in rules.ACCESSOR_LOG_PREFIXES else 3))
      continue
    hit = False
    for msg, mod in rules.CLOSED_MESSAGES:
      if text.startswith(msg):
        votes.append(Vote("8w-closed-slam", mod, "closed-message-string", "msg:%s" % text[:60], 3))
        hit = True
        break
    if hit:
      continue
    for rx, cls, meth in rules.LIB_STRING_RULES:
      if rx.search(text):
        votes.append(Vote(cls, None, meth, "str:%s" % text[:70], 3))
        hit = True
        break
    if hit:
      continue
    mm = re.match(r"^N?S?t?(?:3__2|NSt3__2)?.*?(5ceres|2c8|7c8flann|5capnp|2kj)", text)
    if text.startswith(("ZN", "NSt3__2", "NSt3__", "ZNK")) and mm:
      ns = mm.group(1)
      cls = {"5ceres": "ceres", "2c8": "8w-public", "7c8flann": "glog-mini/other-3p",
             "5capnp": "glog-mini/other-3p", "2kj": "glog-mini/other-3p"}[ns]
      mod = None
      if "2c817VpsRequestManager" in text or "2c813VpsMapManager" in text or "Vps" in text or "Wayspot" in text:
        cls, mod = "8w-other", "VPS"
      elif "writePixelsToJpg" in text:
        cls, mod = "8w-other", "Render"
      votes.append(Vote(cls, mod, "std-function-target-typename", "tname:%s" % text[:70], 2))
      continue
    if len(text) >= 10 and not GENERIC_STRING_RE.match(text) and text in pub_literals:
      votes.append(Vote("8w-public", None, "public-source-literal",
                        "pubstr:%s (%s)" % (text[:50], pub_literals[text]), 1))
  for cname, keys in cfg_by_class.items():
    strength = 3 if len(keys) >= 2 else 1
    ev_s = "cfg:%s (%d keys, e.g. %s)" % (cname, len(keys), keys[0])
    if cname in rules.CLOSED_SLAM_CONFIG:
      votes.append(Vote("8w-closed-slam", rules.CLOSED_SLAM_CONFIG[cname], "config-key-string", ev_s, strength))
    elif cname in rules.PUBLIC_SLAM_CONFIG:
      votes.append(Vote("8w-public", rules.PUBLIC_SLAM_CONFIG[cname], "config-key-string", ev_s,
                        min(strength, 2) if strength == 3 else 1))
    elif cname in rules.OTHER_CONFIG:
      votes.append(Vote("8w-other", rules.OTHER_CONFIG[cname], "config-key-string", ev_s, strength))
  return votes


def rtti_votes(ev, idx):
  votes = []
  slot = ev.vt_slot_classes.get(idx)
  if slot:
    mapped = [(rules.rtti_class(ev.demangled.get(t, t)), ev.demangled.get(t, t)) for t in slot]
    mapped = [(c, d) for c, d in mapped if c[0]]
    if mapped:
      libs = [(c, d) for c, d in mapped if c[0] in LIB_CLASSES]
      pubs = [(c, d) for c, d in mapped if c[0] == "8w-public"]
      pick = libs or pubs or mapped  # shared (ICF/inherited) slot -> base/library owner
      (cls, mod), d = sorted(pick, key=lambda x: x[1])[0]
      namespaces = {x[1].split("::")[0] for x in mapped}
      # A slot shared by many unrelated hierarchies is a generic body (e.g. __cxa_pure_virtual,
      # identical-code-folded trivial dtor): weak evidence only.
      strength = 1 if len(namespaces) >= 3 else 3
      votes.append(Vote(cls, mod, "rtti-vtable-slot",
                        "vtslot:%s%s" % (d[:80], " (+%d)" % (len(slot) - 1) if len(slot) > 1 else ""), strength))
  for kind, table in (("vtref", ev.vt_ref_classes), ("tiref", ev.ti_ref_classes)):
    ref = table.get(idx)
    if not ref:
      continue
    for t in sorted(ref):
      d = ev.demangled.get(t, t)
      cls, mod = rules.rtti_class(d)
      if cls:
        votes.append(Vote(cls, mod, "rtti-%s" % kind, "%s:%s" % (kind, d[:80]), 1))
  return votes


def decide(votes):
  """Resolve direct votes to (class, module, label, deciding votes)."""
  if not votes:
    return None
  top = [v for v in votes if v.strength >= 4]
  if top:
    v = top[0]
    return v.cls, v.module, "VERIFIED", [v]
  best = {}
  for v in votes:
    if v.cls not in best or v.strength > best[v.cls][0]:
      best[v.cls] = (v.strength, v)
  w8 = [c for c in W8_CLASSES if c in best and best[c][0] >= 2]
  if w8:
    # strongest 8w identity wins; ties -> closed > other > public (closed code inlines public
    # helpers, not the reverse).
    top_s = max(best[c][0] for c in w8)
    cls = next(c for c in W8_CLASSES if c in w8 and best[c][0] == top_s)
  else:
    lib = sorted(((best[c][0], sum(1 for v in votes if v.cls == c), c) for c in best if c in LIB_CLASSES),
                 reverse=True)
    if lib and lib[0][0] >= 2:
      cls = lib[0][2]
    else:
      cands = sorted(((best[c][0], sum(1 for v in votes if v.cls == c), c) for c in best), reverse=True)
      cls = cands[0][2]
  dec = [v for v in votes if v.cls == cls]
  mods = collections.Counter()
  for v in dec:
    if v.module:
      mods[v.module] += v.strength * 10 + 1
  module = mods.most_common(1)[0][0] if mods else None
  strength = max(v.strength for v in dec)
  # VERIFIED = a decoded reference inside this function body (import/export name, specific string,
  # RTTI vtable slot) supports the class. Weak hints (public-literal match, vtable-pointer store)
  # and every propagation step stay INFERRED.
  label = "VERIFIED" if strength >= 2 else "INFERRED"
  return cls, module, label, dec


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--dcmp", required=True)
  ap.add_argument("--public-src", required=True)
  ap.add_argument("--ghidra-export", required=True)
  ap.add_argument("--libmatch", default=None, help="optional libmatch.json from libmatch.py")
  ap.add_argument("--ghidra-extra", default=None,
                  help="optional dir with functions.jsonl/c from GhidraDecompileSome.java (long timeout)")
  ap.add_argument("--out", required=True)
  ap.add_argument("--summary", default=None)
  args = ap.parse_args()

  ev = ev_mod.Evidence(args.wasm)
  mod = ev.mod
  pub_literals = ev_mod.public_source_literals(args.public_src, subdirs=("c8", "reality"))
  dcmp = parse_dcmp_ranges(args.dcmp)
  ghidra, ghidra_info = load_ghidra(args.ghidra_export, ev)
  libmatch = {}
  if args.libmatch and os.path.exists(args.libmatch):
    with open(args.libmatch) as fh:
      libmatch = {int(k): v for k, v in json.load(fh).get("matches", {}).items()}

  n_imp = mod.num_imported_funcs
  all_idx = list(range(mod.num_funcs))
  rec = {}

  # ---- 1. direct evidence ------------------------------------------------------------
  imports = [i for i in mod.imports if i[2] == 0]
  for idx in all_idx:
    votes = []
    if idx < n_imp:
      m, f, _, _ = imports[idx]
      votes.append(Vote("emscripten-runtime", None, "import", "import:%s.%s" % (m, f), 4))
    for nm in mod.exports.get(idx, []):
      cls, mdl = rules.export_class(nm)
      if cls:
        votes.append(Vote(cls, mdl, "export-name", "export:%s" % nm, 4))
    if idx >= n_imp:
      votes += string_votes(ev, idx, pub_literals)
      votes += rtti_votes(ev, idx)
      if idx in rules.PUBLIC_HELPERS:
        nm, mdl = rules.PUBLIC_HELPERS[idx]
        votes.append(Vote("8w-public", mdl, "structural-decompile-identity", "struct:%s" % nm,
                          2 if mdl == "Params" else 1))
      if idx in rules.STRUCTURAL_NAMES:
        nm, c = rules.STRUCTURAL_NAMES[idx]
        votes.append(Vote(c, None, "structural-decompile-identity", "struct:%s" % nm, 1))
      if idx in libmatch and libmatch[idx]["n_symbols"] == 1 and libmatch[idx]["ambiguous_targets"] <= 2:
        lm = libmatch[idx]
        votes.append(Vote(lm["class"], None, "library-opcode-hash", "libmatch:%s" % lm["symbol"][:80], 2))
    d = decide(votes)
    r = {"votes": votes, "cls": None, "module": None, "label": None, "methods": [], "evidence": []}
    if d:
      cls, mdl, label, dec = d
      r.update(cls=cls, module=mdl, label=label)
      r["methods"] = list(dict.fromkeys(v.method for v in sorted(dec, key=lambda v: -v.strength)))
      r["evidence"] = [v.evidence for v in sorted(dec, key=lambda v: -v.strength)][:6]
    rec[idx] = r

  seeded = {i for i in all_idx if rec[i]["cls"]}

  # ---- 2. link-order regions ------------------------------------------------------------
  region_of = {}
  for lo, hi, rcls, desc in rules.LINK_REGIONS:
    for i in range(lo, hi + 1):
      region_of[i] = (lo, hi, rcls, desc)
  region_stats = []
  for lo, hi, rcls, desc in rules.LINK_REGIONS:
    seeds = collections.Counter(rec[i]["cls"] for i in range(lo, hi + 1)
                                if i in seeded and rec[i]["label"] == "VERIFIED")
    region_stats.append({"range": [lo, hi], "default_class": rcls, "desc": desc,
                         "verified_seed_classes": dict(seeds)})
  in_app = lambda i: i in region_of and region_of[i][2] is None  # noqa: E731
  for idx in all_idx:
    if idx < n_imp or idx not in region_of:
      continue
    lo, hi, rcls, desc = region_of[idx]
    r = rec[idx]
    if rcls is None:
      w8_rtti = [v for v in r["votes"] if v.cls in W8_CLASSES and v.method in ("rtti-vtref", "rtti-tiref")]
      if r["cls"] in LIB_CLASSES and w8_rtti and r["cls"] != "emscripten-runtime":
        # app object that constructs/uses c8 polymorphic types and inlines library code (LTO).
        v = w8_rtti[0]
        r.update(cls=v.cls, module=v.module, label="INFERRED")
        r["methods"] = ["app-region-rtti-override"] + r["methods"]
        r["evidence"] = [v.evidence] + r["evidence"][:4] + [
            "note: library evidence inlined into app object (link region %d-%d)" % (lo, hi)]
        continue
      if r["cls"] in LIB_CLASSES and r["label"] == "VERIFIED" and r["cls"] != "emscripten-runtime":
        # library strings inside an application object: kept, but flagged (possible LTO inlining).
        r["label"] = "INFERRED"
        r["evidence"] = r["evidence"] + ["note: library evidence inside app link region %d-%d" % (lo, hi)]
      continue
    weak = r["cls"] is None or max((v.strength for v in r["votes"]), default=0) <= 1
    if weak:
      r.update(cls=rcls, module=None, label="INFERRED")
      r["methods"] = ["link-order-region"]
      r["evidence"] = ["region:%d-%d %s" % (lo, hi, desc.split(":")[0])]

  # ---- 3. call-graph propagation inside app regions --------------------------------------
  callers = ev.callers
  callees = ev.callees
  hubs = {i for i in all_idx if rec[i]["cls"] == "emscripten-runtime"}

  def module_ok(cls, mdl):
    if mdl is None:
      return True
    if cls == "8w-other":
      return mdl in rules.OTHER_MODULES
    if cls in W8_CLASSES:
      return mdl not in rules.OTHER_MODULES
    return False

  def callers_rule():
    changed = 0
    for idx in all_idx:
      r = rec[idx]
      if r["cls"] or not in_app(idx):
        continue
      cs = callers.get(idx, set()) - {idx} - hubs
      if not cs:
        continue
      ccls = [rec[c]["cls"] for c in cs]
      if any(c is None for c in ccls):
        continue
      s = set(ccls)
      if len(s) == 1:
        cls = s.pop()
      elif s <= set(W8_CLASSES):
        if len(cs) >= 10:
          continue  # high fan-in helper shared by several 8w components: leave unknown
        cls = "8w-public" if "8w-public" in s else "8w-closed-slam"
      else:
        continue
      mods = {rec[c]["module"] for c in cs}
      mdl = mods.pop() if len(mods) == 1 else None
      r.update(cls=cls, module=mdl if module_ok(cls, mdl) else None, label="INFERRED")
      r["methods"] = ["callgraph-callers"]
      r["evidence"] = ["callers:%s all %s" % (sorted(cs)[:8], "/".join(sorted(s)))]
      changed += 1
    return changed

  def thunk_rule():
    changed = 0
    for idx in all_idx:
      r = rec[idx]
      if r["cls"] or not in_app(idx):
        continue
      f = ev.feats[idx]
      cs = callees.get(idx, set())
      if f["size"] <= 40 and len(cs) == 1:
        c = next(iter(cs))
        lib_fwd = rec[c]["cls"] == "libc/libc++" and f["size"] <= 16
        if rec[c]["cls"] and (lib_fwd or (len(callers.get(c, ())) <= 8 and c not in hubs)):
          r.update(cls=rec[c]["cls"], module=rec[c]["module"], label="INFERRED")
          r["methods"] = ["thunk-callee"]
          r["evidence"] = ["thunk:%dB body calls only %d (%s)" % (f["size"], c, rec[c]["cls"])]
          changed += 1
    return changed

  def neighbor_rule(max_gap):
    """Link-order fill inside app regions: unknown run bounded by the same class within max_gap."""
    changed = 0
    known = [i for i in range(n_imp, mod.num_funcs) if rec[i]["cls"]]
    for a, b in zip(known, known[1:]):
      if b - a <= 1 or b - a - 1 > max_gap or not (in_app(a) and in_app(b)):
        continue
      if region_of[a][0] != region_of[b][0]:
        continue
      ca, cb = rec[a]["cls"], rec[b]["cls"]
      if ca != cb:
        continue
      ma, mb = rec[a]["module"], rec[b]["module"]
      for idx in range(a + 1, b):
        r = rec[idx]
        cs = callers.get(idx, set()) - {idx} - hubs
        known = [rec[c] for c in cs if rec[c]["cls"]]
        if known and not any(k["cls"] == ca for k in known):
          continue  # callers contradict the neighbours (shared helper placed in this object)
        if len(cs) >= 10 and len({k["cls"] for k in known}) >= 2:
          continue  # high fan-in helper used by several components (LTO places it at first use)
        mdl = ma if ma == mb else None
        if mdl and known and not all(k["module"] == mdl for k in known):
          mdl = None
        r.update(cls=ca, module=mdl, label="INFERRED")
        r["methods"] = ["link-order-neighbors"]
        r["evidence"] = ["layout:between %d and %d (%s)" % (a, b, ca)]
        changed += 1
    return changed

  def fixpoint():
    total = [0, 0]
    for _ in range(30):
      c1 = callers_rule()
      c2 = thunk_rule()
      total[0] += c1
      total[1] += c2
      if c1 + c2 == 0:
        break
    return total

  def static_init_rule():
    """Tiny elem-table thunks right after a static initializer (called by __wasm_call_ctors) are
    its atexit destructors: same class as the initializer."""
    changed = 0
    ctors = sorted(c for c in callees.get(259, ()) if rec[c]["cls"] and in_app(c))
    for s0 in ctors:
      j = s0 + 1
      while j < mod.num_funcs and rec[j]["cls"] is None and ev.feats[j]["size"] <= 32 \
          and j in ev.table_funcs and not (callers.get(j, set()) - {j}):
        r = rec[j]
        r.update(cls=rec[s0]["cls"], module=rec[s0]["module"], label="INFERRED")
        r["methods"] = ["static-init-dtor-group"]
        r["evidence"] = ["static-init: tiny atexit thunk after initializer %d (%s)" % (s0, rec[s0]["cls"])]
        changed += 1
        j += 1
    return changed

  c0 = static_init_rule()
  rounds = [fixpoint()]
  c3 = neighbor_rule(4)
  rounds.append(fixpoint())
  c4 = neighbor_rule(16)
  rounds.append(fixpoint())

  # ---- 4. module propagation inside 8w code (callers-unanimous on module) ---------------
  for _ in range(30):
    changed = 0
    for idx in all_idx:
      r = rec[idx]
      if r["module"] or r["cls"] not in W8_CLASSES:
        continue
      cs = callers.get(idx, set()) - {idx} - hubs
      if not cs:
        continue
      mods = {rec[c]["module"] for c in cs}
      if len(mods) == 1 and None not in mods:
        mdl = mods.pop()
        if not module_ok(r["cls"], mdl):
          continue
        r["module"] = mdl
        r["methods"] = r["methods"] + ["module-callers"]
        r["evidence"] = r["evidence"] + ["module: all callers %s in %s" % (sorted(cs)[:6], mdl)]
        changed += 1
    if not changed:
      break

  # ---- 4b. module by exclusive reach: 8w function reachable (via 8w callees, not crossing
  # functions owned by another module) from exactly one module.
  reach = collections.defaultdict(set)
  owned = {i: rec[i]["module"] for i in all_idx if rec[i]["module"] and rec[i]["cls"] in W8_CLASSES}
  for mdl in set(owned.values()):
    stack = [i for i, m in owned.items() if m == mdl]
    seen = set(stack)
    while stack:
      u = stack.pop()
      for v in callees.get(u, ()):
        if v in seen or rec[v]["cls"] not in W8_CLASSES:
          continue
        if v in owned and owned[v] != mdl:
          continue
        seen.add(v)
        stack.append(v)
        reach[v].add(mdl)
  n_reach = 0
  for idx, mods in reach.items():
    r = rec[idx]
    if r["module"] is None and len(mods) == 1:
      mdl = next(iter(mods))
      if module_ok(r["cls"], mdl):
        r["module"] = mdl
        r["methods"] = r["methods"] + ["module-exclusive-reach"]
        r["evidence"] = r["evidence"] + ["module: reachable only from %s" % mdl]
        n_reach += 1

  # ---- 4c. annotate remaining unknowns with what is known about them.
  for idx in all_idx:
    r = rec[idx]
    if r["cls"]:
      continue
    f = ev.feats[idx]
    cs = callers.get(idx, set()) - {idx}
    ccls = collections.Counter(rec[c]["cls"] or "unknown" for c in cs)
    r["evidence"] = ["callers %d by class %s; callees %d; %d instr, %d float, %d simd%s" % (
        len(cs), dict(ccls), len(callees.get(idx, ())), f["n_instr"], f["float"], f["simd"],
        "; in elem table (indirect target)" if idx in ev.table_funcs else "")]

  # ---- 5. output ------------------------------------------------------------------------
  ghidra_dir = os.path.join(args.ghidra_export, "c")
  extra = {}
  if args.ghidra_extra and os.path.exists(os.path.join(args.ghidra_extra, "functions.jsonl")):
    with open(os.path.join(args.ghidra_extra, "functions.jsonl")) as fh:
      for line in fh:
        try:
          x = json.loads(line)
        except json.JSONDecodeError:
          continue
        if x.get("decompiled") and x.get("file"):
          extra[int(x["index"])] = os.path.join(args.ghidra_extra, "c", x["file"])
  ghidra_info["extra_long_timeout_decompiles"] = sorted(extra)
  out = []
  for idx in all_idx:
    r = rec[idx]
    f = ev.feats.get(idx)
    entry = {"index": idx}
    if idx < n_imp:
      m, fl, _, _ = imports[idx]
      entry["import"] = "%s.%s" % (m, fl)
    else:
      entry["wasm_name"] = "func%d" % idx
    if idx in mod.exports:
      entry["export"] = mod.exports[idx]
    entry["size"] = f["size"] if f else 0
    if f:
      entry["file_offset"] = f["file_offset"]
      entry["n_callers"] = len(callers.get(idx, ()))
      entry["n_callees"] = len(callees.get(idx, ()))
    entry["class"] = r["cls"] or "unknown"
    entry["module"] = r["module"]
    subs = sorted({v.module for v in r["votes"] if v.module and v.module != r["module"]})
    if subs:
      entry["submodules_seen"] = subs
    entry["label"] = r["label"] or "UNVERIFIED"
    entry["classified_by"] = r["methods"]
    entry["evidence"] = r["evidence"]
    g = ghidra.get(idx)
    entry["ghidra_name"] = g.get("name") if g else None
    entry["ghidra_entry"] = g.get("entry") if g else None
    entry["ghidra_c"] = os.path.join(ghidra_dir, g["file"]) if g and g.get("file") else None
    if entry["ghidra_c"] is None and idx in extra:
      entry["ghidra_c"] = extra[idx]
    entry["dcmp_lines"] = list(dcmp[idx]) if idx in dcmp else None
    out.append(entry)

  counts = collections.Counter(e["class"] for e in out)
  defined = [e for e in out if e["index"] >= n_imp]
  dcounts = collections.Counter(e["class"] for e in defined)
  bytes_by_class = collections.Counter()
  for e in defined:
    bytes_by_class[e["class"]] += e["size"]
  mcounts = collections.Counter((e["class"], e["module"]) for e in defined if e["module"])
  meth = collections.Counter(m for e in defined for m in e["classified_by"][:1])
  labels = collections.Counter(e["label"] for e in defined)
  summary = {
      "wasm_sha256": mod.sha256,
      "imported_functions": n_imp,
      "defined_functions": len(defined),
      "seeded_by_direct_evidence": len([i for i in seeded if i >= n_imp]),
      "class_counts_defined": dict(dcounts),
      "class_counts_all": dict(counts),
      "class_bytes_defined": dict(bytes_by_class),
      "module_counts": {"%s|%s" % k: v for k, v in sorted(mcounts.items(), key=lambda x: (-x[1], x[0]))},
      "primary_method_counts": dict(meth),
      "label_counts": dict(labels),
      "propagation_rounds_callers_thunk": rounds,
      "neighbor_fill": [c3, c4],
      "static_init_dtor_group": c0,
      "module_exclusive_reach": n_reach,
      "link_regions": region_stats,
      "ghidra": ghidra_info,
      "strings": len(ev.strings),
      "typeinfo": len(ev.typeinfo),
      "vtables": len(ev.vtables),
      "public_literals": len(pub_literals),
  }
  os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
  with open(args.out, "w") as fh:
    json.dump({"schema": "slam-refact/function-index/v1", "summary": summary, "functions": out}, fh,
              indent=None, separators=(",", ":"))
  if args.summary:
    with open(args.summary, "w") as fh:
      json.dump(summary, fh, indent=1)
  print(json.dumps(summary, indent=1))


if __name__ == "__main__":
  main()
