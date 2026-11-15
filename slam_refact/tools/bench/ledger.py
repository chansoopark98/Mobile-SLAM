"""Append-only result ledger (bench-ledger.jsonl) and fixture x arm summary table. Report-only.

There is no API to rewrite or delete a line: the file is only ever opened in append mode. A line missing any
required field is rejected before anything is written.
"""
import argparse
import fcntl
import json
import os
import sys

REQUIRED = ("ts", "head_sha", "head_dirty", "fixture", "arm", "profile_env", "engine_sha256", "bundle_sha256",
            "scorer_sha256", "env_label", "gt_kind", "metrics", "kind")
ARMS = ("original", "port-default", "vins")  # plus "candidate:<name>"
ENV_LABELS = ("desktop-replay", "headless", "device")
GT_KINDS = ("independent", "reference", "synthetic")
KINDS = ("bench", "screening")
VERDICT = "report-only"


def validate(entry):
    missing = [k for k in REQUIRED if k not in entry or entry[k] is None or entry[k] == ""]
    if missing:
        raise ValueError("ledger line rejected, missing fields: " + ",".join(missing))
    arm = entry["arm"]
    if not (arm in ARMS or (arm.startswith("candidate:") and len(arm) > len("candidate:"))):
        raise ValueError("bad arm %r" % arm)
    for key, allowed in (("env_label", ENV_LABELS), ("gt_kind", GT_KINDS), ("kind", KINDS)):
        if entry[key] not in allowed:
            raise ValueError("bad %s %r" % (key, entry[key]))
    if not isinstance(entry["metrics"], dict):
        raise ValueError("metrics must be an object")


def append(path, entry):
    validate(entry)
    line = json.dumps(entry, sort_keys=True, separators=(",", ":")) + "\n"
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:  # append only
        fcntl.flock(f, fcntl.LOCK_EX)
        f.write(line)
        f.flush()
        os.fsync(f.fileno())
        fcntl.flock(f, fcntl.LOCK_UN)


def read(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(x) for x in f if x.strip()]


def _fmt(v, nd=4):
    return "-" if v is None else ("%.*f" % (nd, v))


def summary_markdown(entries):
    """One table per (env_label, gt_kind): rows never merge different environment labels or GT kinds.
    INVALID rows are listed separately and are not part of any comparison column."""
    groups = {}
    for e in entries:
        groups.setdefault((e["env_label"], e["gt_kind"]), []).append(e)
    out = ["# xr-bench summary (%s)" % VERDICT, ""]
    for (env, gt), rows in sorted(groups.items()):
        ref = " (ARCore reference pose, not independent GT)" if gt == "reference" else ""
        out += ["## env=%s, GT=%s%s" % (env, gt, ref), "",
                "| fixture | arm | kind | ATE pos m | ATE rot deg | RPE1s pos m | coverage | NORMAL ratio | resets | verdict |",
                "|---|---|---|---|---|---|---|---|---|---|"]
        invalid = []
        for e in sorted(rows, key=lambda r: (r["fixture"], r["arm"], r["ts"])):
            acc = e["metrics"].get("accuracy", {})
            if not acc.get("valid"):
                invalid.append(e)
                continue
            rob = e["metrics"].get("robustness", {})
            out.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                e["fixture"], e["arm"], e["kind"], _fmt(acc["ate_pos_rmse_m"]), _fmt(acc["ate_rot_rmse_deg"], 3),
                _fmt((acc.get("rpe_1s") or {}).get("pos_rmse_m")), _fmt(acc["coverage"], 3),
                _fmt(rob.get("normal_ratio"), 3), rob.get("reset_count", "-"), VERDICT))
        out.append("")
        for e in invalid:
            out.append("- INVALID (excluded from comparison): %s / %s: %s" % (
                e["fixture"], e["arm"], e["metrics"].get("accuracy", {}).get("invalid_reason")))
        out.append("")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="bench ledger summary (report-only)")
    ap.add_argument("summary", nargs="?")
    ap.add_argument("--ledger", required=True)
    a = ap.parse_args(argv)
    print(summary_markdown(read(a.ledger)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
