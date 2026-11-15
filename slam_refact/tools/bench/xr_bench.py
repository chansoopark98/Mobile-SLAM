#!/usr/bin/env python3
"""xr-bench: report-only runner/scorer for slam_refact traces (B-1). Never changes engine behavior or defaults.

  xr_bench.py score --arm A --env E --fixture NAME --trace trace.jsonl [--ledger L ...]
  xr_bench.py run   --arm port-default --env E --fixture NAME --bundle-dir B --out-dir O [--ledger L ...]

Fixture locations come from --fixture-dir / --bundle-dir or from SLAM_REFACT_BENCH_FIXTURE_ROOT (a directory that
contains slam-refact-20261015/reference/fixtures/<NAME>). Every output row carries verdict=report-only; nothing
here has a pass threshold. The `original` and `vins` arms are only scored from existing traces, never run here.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ledger  # noqa: E402
import metrics  # noqa: E402

ROOT_ENV = "SLAM_REFACT_BENCH_FIXTURE_ROOT"
REPLAY_ENV = "SLAM_REFACT_REPLAY"
FIXTURE_REL = "slam-refact-20261015/reference/fixtures"
D_GL_CV = np.diag([1.0, -1.0, -1.0])
STAGES = ("frontend", "mapTracker", "mapBuilder", "metricScale", "glue")
ORIGIN = [0, 2, 0]
VINS_NOTE = "vins is a control arm and does not represent the mobile configuration (DEF-37)"
REF_NOTE = "ARCore reference pose (reference VIO), not independent GT"


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def scorer_sha256():
    d = os.path.dirname(os.path.abspath(__file__))
    h = hashlib.sha256()
    for n in ("metrics.py", "xr_bench.py"):
        h.update(sha256_file(os.path.join(d, n)).encode())
    return h.hexdigest()


def bundle_sha256(bundle_dir):
    h = hashlib.sha256()
    for n in ("sensors.tsv", "frames.tsv", "engine-config.tsv"):
        h.update(sha256_file(os.path.join(bundle_dir, n)).encode())
    return h.hexdigest()


def git_state():
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        sha = subprocess.check_output(["git", "-C", here, "rev-parse", "HEAD"], text=True).strip()
        dirty = bool(subprocess.check_output(["git", "-C", here, "status", "--porcelain"], text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        return "unknown", True
    return sha, dirty


def load_trace(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(x) for x in f if x.strip()]


def load_fixture(fixture_dir):
    with open(os.path.join(fixture_dir, "manifest.json"), encoding="utf-8") as f:
        man = json.load(f)
    gt = np.loadtxt(os.path.join(fixture_dir, "groundtruth-body.csv"), delimiter=",")
    return man, {"t": gt[:, 0], "p": gt[:, 1:4],
                 "R": np.array([metrics.quat_xyzw_to_matrix(r[4:8]) for r in gt])}


def infer_gt_kind(fixture):
    if fixture.startswith("flip6-arcore"):
        return "reference"
    if fixture.startswith("tum"):
        return "independent"
    raise SystemExit("cannot infer GT kind for %s; pass --gt-kind" % fixture)


def _quat_wxyz_to_matrix(q):
    w, x, y, z = q
    return metrics.quat_xyzw_to_matrix(np.array([x, y, z, w], float))


def is_accepted(r):
    """A pose counts toward accuracy/coverage only if fresh, NORMAL and not a repeated/predicted publication."""
    return (r.get("fresh") is True and r.get("status") == "NORMAL" and not r.get("srRepeated")
            and r.get("p") is not None and r.get("q_wxyz") is not None)


def trace_to_body(rows, man):
    r_bc = np.array(man["R_BFromCam"], float)
    t_ic = np.array(man["t_ic"], float)
    t, rr, pp = [], [], []
    for r in rows:
        if not is_accepted(r):
            continue
        q = np.array(r["q_wxyz"], float)
        if not np.all(np.isfinite(q)) or np.linalg.norm(q) == 0:
            continue  # zero/non-finite quaternion: counted as a rejected pose by the scorer below
        r_cv = _quat_wxyz_to_matrix(q) @ D_GL_CV
        r_wb = r_cv @ r_bc.T
        t.append(r["t"])
        rr.append(r_wb)
        pp.append(np.array(r["p"], float) - r_wb @ t_ic)
    return {"t": np.array(t, float), "R": np.array(rr).reshape(-1, 3, 3), "p": np.array(pp).reshape(-1, 3)}


def robustness(rows, meta):
    n = len(rows)
    status = {}
    for r in rows:
        status[r.get("status")] = status.get(r.get("status"), 0) + 1
    fresh = [r for r in rows if r.get("fresh") is True]
    first_normal = next((r for r in fresh if r.get("status") == "NORMAL"), None)
    first_move = next((r for r in fresh if r.get("status") == "NORMAL" and r.get("p") != ORIGIN), None)
    t0 = rows[0]["t"] if rows else None
    resets = [r.get("srMapReset") for r in rows]
    return {
        "frames": n,
        "status_counts": dict(sorted(status.items(), key=lambda kv: str(kv[0]))),
        "normal_ratio": (status.get("NORMAL", 0) / n) if n else None,
        "limited_ratio": (status.get("LIMITED", 0) / n) if n else None,
        "lost_ratio": ((n - status.get("NORMAL", 0) - status.get("LIMITED", 0)) / n) if n else None,
        "lost_definition": "status not in {NORMAL, LIMITED} (NOT_AVAILABLE/UNSPECIFIED)",
        "reset_count": None if any(x is None for x in resets) else int(sum(1 for x in resets if x)),
        "stale_not_fresh_poses": int(sum(1 for r in rows if r.get("fresh") is not True)),
        "repeated_poses": int(sum(1 for r in rows if r.get("srRepeated"))),
        "predicted_poses": 0,
        "predicted_note": "the engine trace has no predicted-pose marker; none are counted as accepted",
        "excluded_from_success_denominator": "stale/repeated/LIMITED/lost poses are never counted as accepted",
        "time_to_first_normal_s": None if first_normal is None else first_normal["t"] - t0,
        "time_to_first_translation_s": None if first_move is None else first_move["t"] - t0,
        "processed_frames": len(fresh),
        "dropped_frames": (meta or {}).get("droppedFrames"),
        "accepted_poses": int(sum(1 for r in rows if is_accepted(r))),
    }


def _pcts(v):
    if not v:
        return None
    a = np.array(v, float)
    return {"n": len(v), "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)), "convention": "numpy linear"}


def timing(rows):
    """Wall-clock stage timing from srStageMs. Not deterministic run to run, hence kept out of the row digest."""
    first_move = next((r["i"] for r in rows if r.get("fresh") is True and r.get("status") == "NORMAL"
                       and r.get("p") != ORIGIN), None)
    out = {"pre_init_post_init_split": "init = first fresh NORMAL row whose p differs from the origin",
           "init_row_index": first_move, "pose_age_ms": None,
           "pose_age_note": "unavailable in desktop replay traces (publish/latency keys are null)"}
    for phase, pred in (("pre_init", lambda r: first_move is None or r["i"] < first_move),
                        ("post_init", lambda r: first_move is not None and r["i"] >= first_move)):
        sel = [r for r in rows if pred(r) and isinstance(r.get("srStageMs"), dict)]
        ph = {s: _pcts([r["srStageMs"][s] for r in sel if r["srStageMs"].get(s) is not None]) for s in STAGES}
        ph["total"] = _pcts([sum(r["srStageMs"].get(s) or 0.0 for s in STAGES) for r in sel])
        out[phase] = ph
    return out


def build_row(args, rows, man, gt, meta):
    est = trace_to_body(rows, man)
    acc = metrics.score_trajectory(est, gt, expected_frames=len(rows), max_dt=args.max_dt)
    gt_kind = args.gt_kind or infer_gt_kind(args.fixture)
    core = {"fixture": args.fixture, "arm": args.arm, "env_label": args.env, "gt_kind": gt_kind,
            "gt_note": REF_NOTE if gt_kind == "reference" else None,
            "arm_note": VINS_NOTE if args.arm == "vins" else None,
            "verdict": ledger.VERDICT, "comparable": bool(acc["valid"]),
            "accuracy": acc, "robustness": robustness(rows, meta)}
    return core, timing(rows)


def canonical(o):
    return json.dumps(o, sort_keys=True, separators=(",", ":"))


def emit(args, core, tim, engine_sha, bundle_sha):
    row = dict(core)
    row["row_sha256"] = hashlib.sha256(canonical(core).encode()).hexdigest()
    row["timing"] = tim
    text = canonical(row)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
    print(text)
    if args.ledger:
        sha, dirty = git_state()
        import datetime
        ledger.append(args.ledger, {
            "ts": datetime.datetime.now(datetime.timezone.utc).isoformat(), "head_sha": sha, "head_dirty": dirty,
            "fixture": core["fixture"], "arm": core["arm"], "profile_env": args.profile_env or "",
            "engine_sha256": engine_sha or "", "bundle_sha256": bundle_sha or "", "scorer_sha256": scorer_sha256(),
            "env_label": core["env_label"], "gt_kind": core["gt_kind"], "kind": args.kind,
            "metrics": {"accuracy": core["accuracy"], "robustness": core["robustness"], "timing": tim,
                        "row_sha256": row["row_sha256"], "verdict": ledger.VERDICT}})
    return 0 if core["accuracy"]["valid"] else 3  # 3 = INVALID (reported, never a pass)


def resolve(args):
    root = os.environ.get(ROOT_ENV)
    fixture_dir = args.fixture_dir or (root and os.path.join(root, FIXTURE_REL, args.fixture))
    if not fixture_dir:
        raise SystemExit("set %s or pass --fixture-dir" % ROOT_ENV)
    return fixture_dir


def cmd_score(args):
    rows = load_trace(args.trace)
    man, gt = load_fixture(resolve(args))
    meta_path = os.path.join(os.path.dirname(args.trace), "meta.json")
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else None
    core, tim = build_row(args, rows, man, gt, meta)
    bsha = bundle_sha256(args.bundle_dir) if args.bundle_dir else None
    return emit(args, core, tim, args.engine_sha256, bsha)


def cmd_run(args):
    if args.arm not in ("port-default",) and not args.arm.startswith("candidate:"):
        raise SystemExit("run supports port-default and candidate:<name>; score existing traces for other arms")
    binary = args.replay_bin or os.environ.get(REPLAY_ENV)
    if not binary or not os.path.isfile(binary):
        raise SystemExit("replay binary: pass --replay-bin or set %s" % REPLAY_ENV)
    if os.path.exists(args.out_dir):
        raise SystemExit("refusing to reuse existing out dir %s (the runner requires a new directory)" % args.out_dir)
    os.makedirs(os.path.dirname(os.path.abspath(args.out_dir)), exist_ok=True)
    log_path = os.path.abspath(args.out_dir) + ".replay.log"
    with open(log_path, "w") as log:
        rc = subprocess.call([binary, "--bundle", args.bundle_dir, "--out", args.out_dir], stdout=log, stderr=log)
    if rc != 0:
        raise SystemExit("replay exited %d (see %s)" % (rc, os.path.basename(log_path)))
    args.trace = os.path.join(args.out_dir, "trace.jsonl")
    args.engine_sha256 = sha256_file(binary)
    return cmd_score(args)


def parser():
    ap = argparse.ArgumentParser(description="xr-bench (report-only)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("score", "run"):
        p = sub.add_parser(name)
        p.add_argument("--arm", required=True)
        p.add_argument("--env", required=True, choices=ledger.ENV_LABELS)
        p.add_argument("--fixture", required=True)
        p.add_argument("--fixture-dir")
        p.add_argument("--gt-kind", choices=ledger.GT_KINDS)
        p.add_argument("--bundle-dir")
        p.add_argument("--max-dt", type=float, default=0.006)
        p.add_argument("--profile-env", default="")
        p.add_argument("--kind", default="bench", choices=ledger.KINDS)
        p.add_argument("--ledger")
        p.add_argument("--out")
        if name == "score":
            p.add_argument("--trace", required=True)
            p.add_argument("--engine-sha256")
        else:
            p.add_argument("--out-dir", required=True)
            p.add_argument("--replay-bin")
            p.set_defaults(bundle_dir=None)
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    arm = args.arm
    if not (arm in ledger.ARMS or arm.startswith("candidate:")):
        raise SystemExit("bad arm %r" % arm)
    if args.cmd == "run" and not args.bundle_dir:
        raise SystemExit("--bundle-dir required")
    return cmd_run(args) if args.cmd == "run" else cmd_score(args)


if __name__ == "__main__":
    sys.exit(main())
