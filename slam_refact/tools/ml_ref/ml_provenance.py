#!/usr/bin/env python3
"""Records which sources and which binaries a set of ML-lane test results came from (C-3).

Review r2 of the ML lane closure (docs/design/reviews/opus-xhigh-r2-ml-lane-closure.md, C-3) found
that the closure evidence named no source sha list and no binary sha, so "this result = this code"
could not be shown. This script runs the lane's ctest selection in an existing build directory and
writes, next to the log, everything needed to tie the result to a source tree and to the binaries:

  provenance.json        git HEAD, tree hash of slam_refact/engine at HEAD, dirty state, hash of
                         the uncommitted diff, CMake source dir, ninja up-to-date check, ctest
                         command and exit code, hashes of the other files, tool versions
  worktree.diff          `git diff HEAD` of the lane paths (empty when the tree is clean)
  source-sha256.txt      sha256 of every file under the lane source and test directories
  binary-sha256.txt      sha256 of every test executable ctest runs (before and after the run)
  ninja-check.txt        `ninja` for the lane targets run just before ctest: "no work to do" means
                         the binaries are up to date with the sources hashed above (ninja's own
                         mtime/command check). `ninja -n` is not usable: with CONFIGURE_DEPENDS
                         globs it always reports a pending CMake re-run because it cannot execute
                         the glob re-check step
  ctest-unit.log         full output of `ctest -L unit -R <regex>`
  ctest-mlmodel.log      full output of `ctest --no-tests=error -L mlmodel` (face models, ML e2e)

It never modifies the source tree. It runs `ninja` on the lane targets only to ask whether they are
up to date; if they were stale, ninja rebuilds them (the log shows that, ninja_no_work_to_do is then
false and the exit code is 1). ctest writes Testing/Temporary inside the build directory. Python 3
standard library only.

  ml_provenance.py --repo-root <worktree root> --build-dir <existing build dir> --out-dir <dir>
                   [--lock-file <flock file>] [--extra-binary <path> ...]
                   [--ctest-regex 'Ml|Face|Sky|Hand|Ear|Layers'] [--jobs 2]

Exit code: 0 when ctest passed and ninja reported no work to do, 1 otherwise (the files are still
written, so a failing or stale run is recorded, not hidden).
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys

LANE_DIRS = [
    "slam_refact/engine/src/ml",
    "slam_refact/engine/src/face",
    "slam_refact/engine/src/sky",
    "slam_refact/engine/src/hand",
    "slam_refact/engine/tests/ml",
    "slam_refact/engine/tests/face",
    "slam_refact/engine/tests/sky",
    "slam_refact/engine/tests/hand",
]
NINJA_TARGETS = [
    "sr_tests_ml_ops", "sr_tests_ml_models", "sr_tests_face", "sr_tests_face_models",
    "sr_tests_sky", "sr_tests_sky_model", "sr_tests_sky_gate", "sr_tests_hand",
    "sr_tests_hand_model", "sr_tests_hand_gate", "sr_ml_harden",
]


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run(cmd, cwd=None, env=None):
    p = subprocess.run(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, errors="replace")
    return p.returncode, p.stdout


def locked(cmd, lock_file):
    return ["flock", lock_file] + cmd if lock_file else cmd


def list_files(repo_root):
    files = []
    for d in LANE_DIRS:
        for dirpath, _, names in os.walk(os.path.join(repo_root, d)):
            for n in names:
                files.append(os.path.relpath(os.path.join(dirpath, n), repo_root))
    return sorted(files)


def ctest_executables(build_dir, selection):
    rc, out = run(["ctest", "-N", "--show-only=json-v1"] + selection, cwd=build_dir)
    exes = set()
    if rc == 0:
        start = out.find("{")
        data = json.loads(out[start:])
        for t in data.get("tests", []):
            cmd = t.get("command") or []
            if cmd and os.path.isabs(cmd[0]) and os.path.basename(cmd[0]).startswith("sr_"):
                exes.add(cmd[0])
    return sorted(exes)


def hash_binaries(paths):
    return {p: (sha256_file(p) if os.path.exists(p) else None) for p in paths}


def write_lines(path, lines):
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--repo-root", required=True)
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--lock-file", default=None)
    ap.add_argument("--extra-binary", action="append", default=[])
    ap.add_argument("--ctest-regex", default="Ml|Face|Sky|Hand|Ear|Layers")
    ap.add_argument("--jobs", type=int, default=2)
    a = ap.parse_args()
    repo = os.path.abspath(a.repo_root)
    build = os.path.abspath(a.build_dir)
    out = os.path.abspath(a.out_dir)
    os.makedirs(out, exist_ok=True)

    prov = {"repo_root": repo, "build_dir": build}
    prov["git_head"] = run(["git", "rev-parse", "HEAD"], cwd=repo)[1].strip()
    prov["git_branch"] = run(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=repo)[1].strip()
    prov["git_tree_slam_refact_engine_at_head"] = run(
        ["git", "rev-parse", "HEAD:slam_refact/engine"], cwd=repo)[1].strip()
    status = run(["git", "status", "--porcelain", "--"] + LANE_DIRS, cwd=repo)[1]
    prov["lane_paths_dirty"] = bool(status.strip())
    prov["lane_paths_git_status_porcelain"] = status.splitlines()
    diff = run(["git", "diff", "HEAD", "--"] + LANE_DIRS, cwd=repo)[1]
    with open(os.path.join(out, "worktree.diff"), "w") as f:
        f.write(diff)
    prov["worktree_diff_sha256"] = hashlib.sha256(diff.encode("utf-8", "replace")).hexdigest()

    files = list_files(repo)
    lines = ["%s  %s" % (sha256_file(os.path.join(repo, p)), p) for p in files]
    write_lines(os.path.join(out, "source-sha256.txt"), lines)
    prov["source_file_count"] = len(files)
    prov["source_sha256_list_sha256"] = sha256_file(os.path.join(out, "source-sha256.txt"))

    cache = os.path.join(build, "CMakeCache.txt")
    with open(cache) as f:
        for line in f:
            if line.startswith("CMAKE_HOME_DIRECTORY:"):
                prov["cmake_home_directory"] = line.split("=", 1)[1].strip()
            if line.startswith("CMAKE_BUILD_TYPE:"):
                prov["cmake_build_type"] = line.split("=", 1)[1].strip()
    prov["cmake_home_matches_repo"] = (
        prov.get("cmake_home_directory") == os.path.join(repo, "slam_refact", "engine"))

    rc_n, out_n = run(locked(["ninja", "-j", str(a.jobs), "-C", build] + NINJA_TARGETS,
                             a.lock_file))
    with open(os.path.join(out, "ninja-check.txt"), "w") as f:
        f.write(out_n)
    prov["ninja_check_rc"] = rc_n
    prov["ninja_no_work_to_do"] = rc_n == 0 and "ninja: no work to do" in out_n

    selections = {
        "unit": ["-L", "unit", "-R", a.ctest_regex],
        "mlmodel": ["-L", "mlmodel", "--no-tests=error"],
    }
    exes = set(os.path.abspath(p) for p in a.extra_binary)
    for sel in selections.values():
        exes |= set(ctest_executables(build, sel))
    exes = sorted(exes)
    before = hash_binaries(exes)
    prov["ctest_runs"] = {}
    rc_c = 0
    summary = []
    for name, sel in selections.items():
        cmd = ["ctest"] + sel + ["--parallel", str(a.jobs), "--output-on-failure"]
        rc, text = run(cmd, cwd=build)
        with open(os.path.join(out, "ctest-%s.log" % name), "w") as f:
            f.write(text)
        lines_ = [l for l in text.splitlines() if "tests passed" in l or "Total Test time" in l]
        prov["ctest_runs"][name] = {"command": "cd %s && %s" % (build, " ".join(cmd)),
                                    "exit_code": rc, "summary": lines_}
        rc_c = rc_c or rc
        summary += ["%s: %s" % (name, l) for l in lines_]
    after = hash_binaries(exes)
    write_lines(os.path.join(out, "binary-sha256.txt"),
                ["%s  %s  (before ctest)" % (before[p], p) for p in exes] +
                ["%s  %s  (after ctest)" % (after[p], p) for p in exes])
    prov["ctest_exit_code"] = rc_c
    prov["ctest_summary"] = summary
    prov["binaries_sha256"] = before
    prov["binaries_unchanged_during_ctest"] = before == after
    prov["tool_versions"] = {
        "cmake": run(["cmake", "--version"])[1].splitlines()[0],
        "ninja": run(["ninja", "--version"])[1].strip(),
        "c++": run(["c++", "--version"])[1].splitlines()[0],
    }
    prov["limits"] = [
        "A binary sha does not by itself prove which source built it; the chain is: CMake source "
        "dir == repo (cmake_home_matches_repo), ninja reports no work to do (ninja_no_work_to_do),"
        " and source-sha256.txt + git_head + worktree.diff identify the sources at that moment.",
        "Only the lane directories are hashed file by file; the rest of slam_refact/engine is "
        "identified by git_tree_slam_refact_engine_at_head (valid only when it is not modified).",
        "Desktop x86-64 native build; says nothing about WASM or devices.",
    ]
    with open(os.path.join(out, "provenance.json"), "w") as f:
        json.dump(prov, f, indent=2, sort_keys=True)
        f.write("\n")

    ok = rc_c == 0 and prov["ninja_no_work_to_do"] and prov["cmake_home_matches_repo"]
    print("git_head=%s dirty=%s" % (prov["git_head"], prov["lane_paths_dirty"]))
    print("ninja_no_work_to_do=%s cmake_home_matches_repo=%s ctest_rc=%d" %
          (prov["ninja_no_work_to_do"], prov["cmake_home_matches_repo"], rc_c))
    print("\n".join(prov["ctest_summary"]))
    print("written to", out)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
