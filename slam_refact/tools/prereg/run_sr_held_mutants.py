#!/usr/bin/env python3
"""Mutation check of sr_held: every check of the shared device must be what rejects its cases.

For every mutant of sr_held_mutants.json (one switched-off check, or one altered constant) a COPY of the C++ header
(engine/tests/support/sr_held.h) and of the node module (tools/prereg/sr_held.mjs) is made, the mutation is applied to the copy,
and the conformance test of that language is run on it. The mutant is KILLED when every target case of the mutant deviates
from its expectation (FAIL, crash, or not run). The sources in the repository are never modified.

    python3 -I run_sr_held_mutants.py --out DIR [--lang cpp|js|both] [--only MUTANT_ID] [--cases FILE] [--mutants FILE]

Writes DIR/mutants.log (one line per mutant and language), DIR/mutants-summary.json, and the work copies under DIR/work/.
Builds one mutant at a time (g++ -O1, single process). Exit code 0 when the baseline passes, every mutant is valid and killed,
and every case that expects a refusal is failed by at least one mutant (except the documented defense-in-depth cases).
"""
import argparse
import json
import pathlib
import re
import shutil
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ENGINE_SUPPORT = HERE.parent.parent / "engine" / "tests" / "support"
ENGINE_COMMON = HERE.parent.parent / "engine" / "tests" / "common"

# Cases that two independent checks refuse (a switched-off single check leaves the same refusal code). They are refused, and the
# conformance test proves it, but no single-check mutant can attribute the refusal to one line. Listed so that the coverage
# report cannot silently grow.
DEFENSE_IN_DEPTH = {
    "manifest-malformed": "truncated manifest: the parser refuses it, and the partial object lacks the required fields",
    "manifest-bom": "a BOM: the parser refuses it, and the BOM byte is also not a JSON value",
}


def skip_string(line, i):
    q = line[i]
    i += 1
    while i < len(line):
        if line[i] == "\\":
            i += 2
            continue
        if line[i] == q:
            return i
        i += 1
    raise ValueError("unterminated literal in: " + line)


def if_false(line, match):
    """`if (COND) action` -> `if ((COND) && false) action`, for the if that owns the text `match`."""
    p = line.index(match)
    q = line.rfind("if (", 0, p)
    if q < 0:
        raise ValueError("no `if (` before the match in: " + line.strip())
    i = q + 3
    depth = 0
    while i < len(line):
        c = line[i]
        if c in "\"'`":
            i = skip_string(line, i)
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                break
        i += 1
    cond = line[q + 4:i]
    return line[:q] + "if ((" + cond + ") && false)" + line[i + 1:]


def apply_edits(text, edits, lang):
    for e in edits:
        if "if_false" in e:
            m = e["if_false"]
            m = m if isinstance(m, str) else m.get(lang)
            if m is None:
                continue
            lines = text.split("\n")
            hits = [n for n, l in enumerate(lines) if m in l]
            if len(hits) != 1:
                raise ValueError("%s: match %r occurs on %d lines" % (lang, m, len(hits)))
            lines[hits[0]] = if_false(lines[hits[0]], m)
            text = "\n".join(lines)
        else:
            pair = e["replace"].get(lang)
            if pair is None:
                continue
            if text.count(pair[0]) != 1:
                raise ValueError("%s: find %r occurs %d times" % (lang, pair[0], text.count(pair[0])))
            text = text.replace(pair[0], pair[1])
    return text


def edits_apply_to(edits, lang):
    for e in edits:
        if "if_false" in e:
            m = e["if_false"]
            if isinstance(m, str) or lang in m:
                return True
        elif lang in e["replace"]:
            return True
    return False


def run_cpp(workdir, header_text, cases, all_ids=(), timeout=300):
    support = workdir / "support"
    support.mkdir(parents=True, exist_ok=True)
    (workdir / "common").mkdir(exist_ok=True)
    shutil.copy(ENGINE_COMMON / "sha256_hex.h", workdir / "common" / "sha256_hex.h")
    shutil.copy(ENGINE_SUPPORT / "sr_held_conformance.cc", support / "sr_held_conformance.cc")
    (support / "sr_held.h").write_text(header_text)
    exe = workdir / "runner"
    cc = subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-o", str(exe), str(support / "sr_held_conformance.cc")],
                        capture_output=True, text=True, errors="replace", timeout=timeout)
    if cc.returncode != 0:
        return {"valid": False, "why": "compile error: " + cc.stderr.strip().splitlines()[0] if cc.stderr.strip() else "compile error"}
    env = {"PATH": "/usr/bin:/bin", "SR_HELD_TMP": str(workdir / "tmp")}
    r = subprocess.run([str(exe), str(cases)], capture_output=True, text=True, errors="replace", env=env, timeout=timeout)
    status, running = {}, None
    for line in r.stdout.splitlines():
        if line.startswith("RUN "):
            running = line[4:]
        elif line.startswith("CASE "):
            parts = line.split(" ", 3)
            status[parts[1]] = parts[2]
            running = None
    crashed = r.returncode not in (0, 1)
    if crashed:
        # a crash ends the process: run every case that did not get a result on its own, so that each case is judged by itself
        for cid in all_ids:
            if cid in status:
                continue
            one = subprocess.run([str(exe), str(cases)], capture_output=True, text=True, errors="replace", env=dict(env, SR_HELD_ONLY=cid), timeout=timeout)
            got = [ln.split(" ", 3) for ln in one.stdout.splitlines() if ln.startswith("CASE ")]
            status[cid] = got[0][2] if got else "CRASH"
    return {"valid": bool(status) or r.returncode == 1, "status": status, "crashed_in": running if crashed else None,
            "returncode": r.returncode, "stdout_tail": r.stdout[-300:]}


def run_js(workdir, module_text, cases, all_ids=(), timeout=300):
    workdir.mkdir(parents=True, exist_ok=True)
    shutil.copy(HERE / "sr_held_conformance.test.mjs", workdir / "sr_held_conformance.test.mjs")
    (workdir / "sr_held.mjs").write_text(module_text)
    chk = subprocess.run(["node", "--check", str(workdir / "sr_held.mjs")], capture_output=True, text=True, errors="replace")
    if chk.returncode != 0:
        return {"valid": False, "why": "syntax error: " + (chk.stderr.strip().splitlines() or ["?"])[-1]}
    env = {"PATH": "/usr/bin:/bin:" + str(pathlib.Path(shutil.which("node")).parent), "SR_HELD_CASES": str(cases), "SR_HELD_TMP": str(workdir / "tmp")}
    r = subprocess.run(["node", "--test", "--test-reporter=tap", str(workdir / "sr_held_conformance.test.mjs")], capture_output=True, text=True, errors="replace", env=env, timeout=timeout)
    status = {}
    for line in r.stdout.splitlines():
        m = re.match(r"^(not )?ok \d+ - (.*)$", line)
        if m:
            status[m.group(2)] = "FAIL" if m.group(1) else "PASS"
    return {"valid": bool(status), "status": status, "crashed_in": None, "returncode": r.returncode, "stdout_tail": r.stdout[-300:]}


def deviates(status, target):
    if target.endswith("*"):
        ids = [k for k in status if k.startswith(target[:-1])]
        return any(status[k] != "PASS" for k in ids) if ids else True
    return status.get(target, "NOT_RUN") != "PASS"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--lang", default="both", choices=("cpp", "js", "both"))
    ap.add_argument("--only")
    ap.add_argument("--cases", default=str(HERE / "sr_held_cases.json"))
    ap.add_argument("--mutants", default=str(HERE / "sr_held_mutants.json"))
    a = ap.parse_args()
    out = pathlib.Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    cases = pathlib.Path(a.cases).resolve()
    table = json.loads(cases.read_text())
    mutants = json.loads(pathlib.Path(a.mutants).read_text())["mutants"]
    langs = ["cpp", "js"] if a.lang == "both" else [a.lang]
    sources = {"cpp": (ENGINE_SUPPORT / "sr_held.h").read_text(), "js": (HERE / "sr_held.mjs").read_text()}
    runners = {"cpp": run_cpp, "js": run_js}
    reject_cases = {c["id"] for c in table["cases"] if any(not s["expect"]["ok"] for s in c["steps"])}
    log = open(out / "mutants.log", "w")

    def say(msg):
        print(msg)
        log.write(msg + "\n")
        log.flush()

    summary = {"baseline": {}, "mutants": [], "uncovered": {}, "reject_cases": len(reject_cases)}
    bad = False
    all_ids = {}
    for lang in langs:
        base = runners[lang](out / "work" / "baseline" / lang, sources[lang], cases)
        all_ids[lang] = list((base.get("status") or {}).keys())
        failing = [k for k, v in (base.get("status") or {}).items() if v != "PASS"]
        ok = base["valid"] and not failing and base.get("returncode") == 0
        say("BASELINE %s %s: %d cases, %d not passing%s" % (lang, "OK" if ok else "BAD", len(base.get("status") or {}), len(failing), "" if ok else " " + str(failing[:5] or base)))
        summary["baseline"][lang] = {"ok": ok, "cases": len(base.get("status") or {})}
        bad = bad or not ok
    killed_by = {lang: {} for lang in langs}
    for m in mutants:
        if a.only and m["id"] != a.only:
            continue
        for lang in langs:
            if not edits_apply_to(m["edits"], lang):
                continue
            wd = out / "work" / "mutants" / m["id"] / lang
            try:
                text = apply_edits(sources[lang], m["edits"], lang)
            except ValueError as e:
                say("MUTANT %-26s %-3s INVALID (cannot apply: %s)" % (m["id"], lang, e))
                summary["mutants"].append({"id": m["id"], "lang": lang, "result": "INVALID", "why": str(e)})
                bad = True
                continue
            res = runners[lang](wd, text, cases, all_ids[lang])
            if not res["valid"]:
                say("MUTANT %-26s %-3s INVALID (%s)" % (m["id"], lang, res.get("why", "no result")))
                summary["mutants"].append({"id": m["id"], "lang": lang, "result": "INVALID", "why": res.get("why", "no result")})
                bad = True
                continue
            status = res["status"]
            survived = [t for t in m["targets"] if not deviates(status, t)]
            failing = sorted(k for k, v in status.items() if v != "PASS")
            if res.get("crashed_in"):
                failing.append(res["crashed_in"])
            for k in failing:
                killed_by[lang].setdefault(k, []).append(m["id"])
            verdict = "KILLED" if not survived else "SURVIVED"
            say("MUTANT %-26s %-3s %-8s targets=%d/%d failing=%d%s%s" % (
                m["id"], lang, verdict, len(m["targets"]) - len(survived), len(m["targets"]), len(failing),
                " crash-in=" + res["crashed_in"] if res.get("crashed_in") else "", "" if not survived else " SURVIVING-TARGETS=" + ",".join(survived)))
            summary["mutants"].append({"id": m["id"], "lang": lang, "result": verdict, "targets": m["targets"], "survivors": survived,
                                       "failing": failing[:40], "crashed_in": res.get("crashed_in")})
            bad = bad or bool(survived)
    if not a.only:
        for lang in langs:
            uncovered = sorted(c for c in reject_cases if c not in killed_by[lang] and c not in DEFENSE_IN_DEPTH)
            documented = sorted(c for c in reject_cases if c not in killed_by[lang] and c in DEFENSE_IN_DEPTH)
            say("COVERAGE %s: %d refusing cases, %d failed by at least one mutant, %d defense-in-depth (documented), %d uncovered %s" % (
                lang, len(reject_cases), len([c for c in reject_cases if c in killed_by[lang]]), len(documented), len(uncovered), uncovered))
            summary["uncovered"][lang] = {"uncovered": uncovered, "defense_in_depth": documented}
            bad = bad or bool(uncovered)
        total = [x for x in summary["mutants"]]
        killed = [x for x in total if x["result"] == "KILLED"]
        say("TOTAL mutants=%d killed=%d survived=%d invalid=%d" % (len(total), len(killed), len([x for x in total if x["result"] == "SURVIVED"]), len([x for x in total if x["result"] == "INVALID"])))
        for lang in langs:
            t = [x for x in total if x["lang"] == lang]
            say("TOTAL %s: mutants=%d killed=%d" % (lang, len(t), len([x for x in t if x["result"] == "KILLED"])))
    (out / "mutants-summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    say("RESULT " + ("FAIL" if bad else "OK"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
