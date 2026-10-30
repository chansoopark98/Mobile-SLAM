#!/usr/bin/env python3
"""Generates sr_held_cases.json, the case table that BOTH conformance tests read
(engine/tests/support/sr_held_conformance.cc and tools/prereg/sr_held_conformance.test.mjs).

The expected values are computed here, by a third implementation: the canonical manifest form is the
verify_manifest.py snippet of preregistration-protocol.md section 6 (json.dumps(sort_keys, separators, ensure_ascii=False)),
the instants come from datetime, the hashes from hashlib. Neither sr_held.h nor sr_held.mjs is imported or consulted.

    python3 -I gen_sr_held_cases.py [--out sr_held_cases.json]

The output is deterministic (no clock, no random). Everything is test data: gate "demo-gate", a fake review, fake binaries.
"""
import base64
import copy
import datetime
import hashlib
import json
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
NOW = "2026-10-21T00:00:00+09:00"
GATE = "demo-gate"
SEED = "18446744073709551615"
SEED_TEXT = (SEED + " 2026-10-20T13:51:51+09:00\n").encode()

sha = lambda b: hashlib.sha256(b).hexdigest()

# ---- reference canonical form: the code of the section 6 snippet --------------------------------------------------------
PIN_BLANKED = ("review", "first_real_run_at", "held_run")
canon_text = lambda mm: json.dumps({k: v for k, v in mm.items() if k not in PIN_BLANKED} if isinstance(mm, dict) else mm,
                                   sort_keys=True, separators=(",", ":"), ensure_ascii=False)
canon = lambda mm: hashlib.sha256(canon_text(mm).encode()).hexdigest()


def tree_sha(files):  # {relative path: bytes}
    lines = "".join("%s\t%s\n" % (p, sha(files[p])) for p in sorted(files, key=lambda x: x.encode()))
    return sha(lines.encode())


def enc(b):  # a file body as JSON: text when it is valid UTF-8 without NUL, else base64
    try:
        t = b.decode("utf-8")
        if "\x00" not in t:
            return {"text": t}
    except UnicodeDecodeError:
        pass
    return {"b64": base64.b64encode(b).decode()}


TOOL = bytes(range(256)) * 3
WASM = b"\x00asm\x01\x00\x00\x00" + bytes(range(255, 0, -1)) * 2
DATA = {"a.bin": bytes(range(200)), "sub/b.bin": b"registered gate data b\n"}
EXTRA = b"an executable the manifest does not register\n"
GATE_MD = "# demo-gate v1\n\ngate text\n".encode()
REVIEW_MD = "# review of demo-gate.v1\n\nAPPROVE\n".encode()
DEFAULT_REQUEST = {
    "prereg_dir": "prereg", "gate_id": GATE, "version": 1, "kind": "main", "env": "", "pins": ["bin/tool.bin", "bin/engine.wasm"],
    "data_dir": "data/main", "want_seed": False, "now": NOW,
}
RAW = lambda tok: "@@RAW:%s@@" % tok


def build(*, created="2026-10-20T10:00:00+09:00", review_at="2026-10-20T11:00:00+09:00", run_at=None, kinds=("main",),
          seed=False, seed_body=None, seed_file=True, pin_seed=False, dev_seeds=(), sealed_files=(),
          gate_field=None, review_field=None, designation_field=None, designation_at=None,
          pre=None, post_manifest=None, designation_edit=None, designation_bytes=None, post_designation=None, post_files=None,
          gate_md=GATE_MD, review_md=REVIEW_MD, marker=False, drop=(), request=None, extra_files=None):
    """One consistent registration: manifest, designation, review, gate file, pins, data. Hooks:
    pre              text edit of the manifest BEFORE the designation is made (a consistent registration)
    designation_edit dict edit of the designation before its sha256 is recorded in the manifest
    post_*           edits AFTER the registration (tampering)"""
    files = {}
    files["prereg/%s.v1.md" % GATE] = gate_md
    files["prereg/%s.v1.review.md" % GATE] = review_md
    files["bin/tool.bin"] = TOOL
    files["bin/engine.wasm"] = WASM
    for rel, body in DATA.items():
        files["data/main/" + rel] = body
    seed_bytes = SEED_TEXT if seed_body is None else seed_body
    if seed and seed_file:
        files["prereg/%s.v1.coordinator-seed.txt" % GATE] = seed_bytes
    held = {
        "kinds": list(kinds),
        "pins": {"tool.bin": sha(TOOL), "engine.wasm": sha(WASM)},
        "inputs": {"main": tree_sha(DATA)},
        "seed": bool(seed),
        "dev_seeds": list(dev_seeds),
        "sealed_file_sha256": list(sealed_files),
    }
    manifest = {
        "gate_id": GATE,
        "version": 1,
        "supersedes": None,
        "file": gate_field or "%s.v1.md" % GATE,
        "sha256": sha(gate_md),
        "created_at": created,
        "tolerance": {"value": RAW("0.00001"), "unit": "m", "exact": RAW("1.0"), "big": RAW("12345678901234567890123"),
                      "neg_zero": RAW("-0"),
                      "edge": [RAW(t) for t in ("1e22", "0.1", "1E-7", "123456789.125", "1e16", "1.5e300", "-0.0", "5e-324",
                                                "100.0", "0.0001", "0.00001234", "1e15", "2.5E+3")]},
        "note": "한글 \"quote\" \\ é \u0001 \u007f \U0001F600",
        "～": 1,
        "\U0001F600": 2,
        "review_fixes": {"at": "2099-01-01T00:00:00+09:00", "file": "decoy.md", "sha256": "0" * 64},
        "held": held,
        "review": {"PLACEHOLDER": True},
        "first_real_run_at": None,
    }

    def dump(m):
        t = json.dumps(m, indent=1, ensure_ascii=False)
        return re.sub(r'"@@RAW:([^@]+)@@"', lambda g: g.group(1), t) + "\n"

    text = dump(manifest)
    if pre:
        text = pre(text)
    m_for_canon = json.loads(text)
    pin_manifest = canon(m_for_canon)
    designation = {
        "notes": {"gate": "decoy-gate.v9", "first_real_run_permitted": False, "pinned": {"gate_md_sha256": "f" * 64}},
        "gate": "%s.v1" % GATE,
        "first_real_run_permitted": True,
        "pinned": {"gate_md_sha256": sha(gate_md), "review_file_sha256": sha(review_md), "manifest_before_review_sha256": pin_manifest},
        "designated_by": "coordinator (test)",
    }
    if pin_seed:
        designation["pinned"]["seed_file_sha256"] = sha(seed_bytes)
    if designation_edit:
        designation_edit(designation)
    dbytes = designation_bytes if designation_bytes is not None else (json.dumps(designation, indent=1, ensure_ascii=False) + "\n").encode()
    dname = designation_field or "%s.v1.review-designation.json" % GATE
    dloc = designation_at or ("prereg/" + dname)
    dsha = sha(dbytes)
    review = {
        "reviewer": "독립 리뷰어 (coordinator 지정, test)",
        "at": review_at,
        "file": review_field or "%s.v1.review.md" % GATE,
        "sha256": sha(review_md),
        "designation_file": dname,
        "designation_file_sha256": dsha,
    }
    if review_at is None:
        review["at"] = None
    # the final manifest: the review object and first_real_run_at filled in place, every other byte of `text` kept
    review_json = json.dumps(review, indent=1, ensure_ascii=False).replace("\n", "\n ")
    ftext, n = re.subn(r'"review":\s*\{\s*"PLACEHOLDER":\s*true\s*\}', lambda g: '"review": ' + review_json, text)
    assert n == 1, "review placeholder"
    if run_at is not None:
        ftext, n = re.subn(r'"first_real_run_at":\s*null', '"first_real_run_at": "%s"' % run_at, ftext)
        assert n == 1, "first_real_run_at"
    final = json.loads(ftext)
    if post_manifest:
        ftext = post_manifest(ftext)
    files["prereg/%s.v1.manifest.json" % GATE] = ftext.encode()
    files[dloc] = post_designation(dbytes) if post_designation else dbytes
    if marker:
        files["prereg/%s.v1.first-run.main.marker" % GATE] = b"started\n"
    if post_files:
        post_files(files)
    for d in drop:
        files.pop(d, None)
    for k, v in (extra_files or {}).items():
        files[k] = v
    req = copy.deepcopy(DEFAULT_REQUEST)
    req["env"] = dsha
    req["want_seed"] = bool(seed)
    if request:
        req.update(request)
    return {"files": files, "request": req, "dsha": dsha, "pin_manifest": pin_manifest, "manifest": final}


BASES = {
    "plain": build(),
    "seeded": build(seed=True),
    "twokinds": build(kinds=("main", "alt")),
    "audited": build(run_at="2026-10-20T11:30:00+09:00", marker=True),
}


def case_files(base_name, fx):
    """Overrides of the base: path -> {text|b64} or null (removed)."""
    base = BASES[base_name]["files"]
    out = {}
    for p, b in fx["files"].items():
        if base.get(p) != b:
            out[p] = enc(b)
    for p in base:
        if p not in fx["files"]:
            out[p] = None
    return out


def step(op, expect, **request):
    return {"op": op, "request": request, "expect": expect}


OK = lambda **kw: dict({"ok": True, "code": ""}, **kw)
NO = lambda code, **kw: dict({"ok": False, "code": code}, **kw)

CASES = []


def add(cid, doc, fx, steps, base="plain", symlinks=None):
    b = BASES[base]
    over = {}
    for k, v in fx["request"].items():  # request differences against the base become overrides of every step
        if b["request"].get(k) != v:
            over[k] = v
    steps = copy.deepcopy(steps)
    for s in steps:
        s["request"] = dict(over, **s["request"])
    c = {"id": cid, "doc": doc, "base": base, "files": case_files(base, fx), "steps": steps}
    if symlinks:
        c["symlinks"] = symlinks
    CASES.append(c)


def plain(cid, doc, steps, **kw):
    add(cid, doc, build(**kw.pop("fx", {})), steps, **kw)


def rep(old, new, count=1):
    def f(t):
        assert t.count(old) >= 1, old
        return t.replace(old, new, count)
    return f


# ---- accepted -----------------------------------------------------------------------------------------------------------
add("ok-release", "a complete, consistent registration is released and claims its marker (nested decoy keys, canon probes)",
    build(), [step("release", OK())])
add("ok-check-then-release", "CheckRelease has no side effect; the same registration is then released once",
    build(), [step("check", OK()), step("release", OK())])
add("ok-input-moved", "the data tree sha256 does not depend on the directory name",
    build(extra_files={"elsewhere/renamed/" + k: v for k, v in DATA.items()}), [step("release", OK(), data_dir="elsewhere/renamed")])
add("ok-time-mixed-offsets", "instants, not text: created 10:00+09:00 (=01:00Z), review 09:00Z is later although it sorts earlier as text",
    build(review_at="2026-10-20T09:00:00Z"), [step("release", OK())])
add("ok-time-fraction", "fractions of a second are compared",
    build(review_at="2026-10-20T10:00:00.5+09:00"), [step("release", OK())])
add("ok-twokinds", "one marker per kind: main and alt are each released once, main is not released twice",
    build(kinds=("main", "alt")),
    [step("release", OK()), step("release", OK(), kind="alt", data_dir=""), step("release", NO("MARKER_EXISTS"))], base="twokinds")
add("ok-seed", "the seed is handed out by HeldGate only, with the file's annotation ignored; CheckRelease never returns it",
    build(seed=True),
    [step("check", OK(), want_seed=True), step("release", OK(seed=SEED, no_leak=[]), want_seed=True)], base="seeded")
add("ok-seed-pinned", "designation pinned.seed_file_sha256 equal to the seed file",
    build(seed=True, pin_seed=True), [step("release", OK(seed=SEED))], base="seeded")
add("ok-audit", "post-run audit: created_at < review.at < first_real_run_at and the marker exists",
    build(run_at="2026-10-20T11:30:00+09:00", marker=True), [step("audit", OK())], base="audited")

# ---- the one-run marker --------------------------------------------------------------------------------------------------
add("rerun-refused", "a second start of the same kind is refused: the marker was created with O_EXCL and is never removed",
    build(), [step("release", OK()), step("release", NO("MARKER_EXISTS")), step("check", OK())])
add("marker-preexisting", "a marker that exists before the first start refuses the run",
    build(marker=True), [step("release", NO("MARKER_EXISTS"))])
add("marker-symlink-preexisting", "a dangling symlink at the marker path is not a free path (O_EXCL)",
    build(), [step("release", NO("MARKER_EXISTS"))], symlinks={"prereg/%s.v1.first-run.main.marker" % GATE: "no-such-target"})
add("kind-unregistered", "a kind that is not in held.kinds cannot be claimed",
    build(), [step("release", NO("KIND_UNREGISTERED"), kind="other")])

# ---- arguments and env ---------------------------------------------------------------------------------------------------
fx = build()
add("args-gate-id-path", "gate_id with a slash", fx, [step("release", NO("ARGS"), gate_id="x/../demo-gate")])
add("args-kind-path", "kind with a slash", fx, [step("release", NO("ARGS"), kind="main/../x")])
add("args-version-zero", "version 0", fx, [step("release", NO("ARGS"), version=0)])
add("args-prereg-empty", "prereg_dir empty", fx, [step("release", NO("ARGS"), prereg_dir="")])
add("args-bad-now", "now without a time zone", fx, [step("release", NO("ARGS"), now="2026-10-21T00:00:00")])
add("env-missing", "no environment value", fx, [step("release", NO("ENV_MISSING"), env="")])
add("env-wrong", "an environment value that is not the designation sha256", fx, [step("release", NO("ENV_MISMATCH"), env="0" * 64)])
add("env-missing-wins", "check order: a missing env is reported before a tampered manifest",
    build(post_manifest=rep("0.00001", "0.0001")),
    [step("release", NO("ENV_MISSING"), env="")])

# ---- the manifest --------------------------------------------------------------------------------------------------------
add("manifest-missing", "no manifest", build(drop=("prereg/%s.v1.manifest.json" % GATE,)), [step("release", NO("MANIFEST_UNREADABLE"))])
add("manifest-malformed", "truncated manifest", build(post_manifest=lambda t: t[: len(t) // 2]), [step("release", NO("MANIFEST_INVALID"))])
add("manifest-not-object", "manifest is an array", build(post_manifest=lambda t: "[1]\n"), [step("release", NO("MANIFEST_INVALID"))])
add("manifest-bom", "manifest starts with a byte order mark", build(post_manifest=lambda t: "﻿" + t), [step("release", NO("MANIFEST_INVALID"))])
add("manifest-duplicate-key", "a second first_real_run_at (a naive reader takes the last or the first value)",
    build(post_manifest=rep('"first_real_run_at": null', '"first_real_run_at": null,\n "first_real_run_at": "2026-10-20T12:00:00+09:00"')),
    [step("release", NO("MANIFEST_INVALID"))])
add("manifest-duplicate-nested-key", "a duplicate key inside held.pins",
    build(post_manifest=rep('"tool.bin": "', '"tool.bin": "' + "1" * 64 + '",\n  "tool.bin": "')), [step("release", NO("MANIFEST_INVALID"))])
add("manifest-identity-id", "manifest of another gate", build(pre=rep('"gate_id": "demo-gate"', '"gate_id": "other-gate"')),
    [step("release", NO("MANIFEST_IDENTITY"))])
add("manifest-identity-version", "manifest of another version", build(pre=rep('"version": 1', '"version": 2')),
    [step("release", NO("MANIFEST_IDENTITY"))])
add("manifest-version-string", "version as a string is a shape error", build(pre=rep('"version": 1', '"version": "1"')),
    [step("release", NO("MANIFEST_INVALID"))])
add("manifest-number-no-canon", "a number with no canonical form (1e999 is not finite)", build(post_manifest=rep("0.00001", "1e999")),
    [step("release", NO("MANIFEST_INVALID"))])
add("manifest-held-missing", "no held object", build(pre=lambda t: json.dumps({k: v for k, v in json.loads(t).items() if k != "held"}, indent=1, ensure_ascii=False)),
    [step("release", NO("MANIFEST_INVALID"))])
add("manifest-pin-bad-sha", "held.pins entry that is not a sha256", build(pre=rep(sha(TOOL), "x" * 64)), [step("release", NO("MANIFEST_INVALID"))])
add("manifest-input-kind-unknown", "held.inputs names a kind that held.kinds does not",
    build(pre=rep('"inputs": {\n   "main"', '"inputs": {\n   "ghost"')), [step("release", NO("MANIFEST_INVALID"))])
add("manifest-seed-not-bool", "held.seed as a string", build(pre=rep('"seed": false', '"seed": "false"')), [step("release", NO("MANIFEST_INVALID"))])

# ---- file names ----------------------------------------------------------------------------------------------------------
sub_gate = "prereg/sub/%s.v1.md" % GATE
add("name-gate-file-path", "manifest file with a path", build(gate_field="sub/%s.v1.md" % GATE, extra_files={sub_gate: GATE_MD}),
    [step("release", NO("NAME_UNSAFE"))])
add("name-gate-file-other", "manifest file is not <gate>.v<N>.md", build(gate_field="other.md", extra_files={"prereg/other.md": GATE_MD}),
    [step("release", NO("NAME_UNSAFE"))])
add("name-review-file-path", "review.file with a path", build(review_field="../prereg/%s.v1.review.md" % GATE),
    [step("release", NO("NAME_UNSAFE"))])
add("name-designation-path", "review.designation_file with a path (the file exists there and its sha256 matches)",
    build(designation_field="sub/d.json", designation_at="prereg/sub/d.json"), [step("release", NO("NAME_UNSAFE"))])
add("name-designation-dotdot", "review.designation_file is ..", build(designation_field="..", designation_at="prereg/dd.json"),
    [step("release", NO("NAME_UNSAFE"))])
add("name-designation-backslash", "review.designation_file with a backslash", build(designation_field="sub\\d.json", designation_at="prereg/sub\\d.json"),
    [step("release", NO("NAME_UNSAFE"))])
add("name-designation-empty-review", "review.reviewer empty", build(post_manifest=rep("독립 리뷰어 (coordinator 지정, test)", "")),
    [step("release", NO("MANIFEST_INVALID"))])

# ---- time order ----------------------------------------------------------------------------------------------------------
add("time-review-null", "review.at is null: the review is not recorded", build(review_at=None), [step("release", NO("REVIEW_MISSING"))])
add("time-review-before-created", "review.at before created_at", build(review_at="2026-10-20T09:00:00+09:00"), [step("release", NO("ORDER"))])
add("time-review-equals-created", "review.at equal to created_at (strict order)", build(review_at="2026-10-20T10:00:00+09:00"), [step("release", NO("ORDER"))])
add("time-review-future", "review.at after now", build(review_at="2026-10-22T00:00:00+09:00"), [step("release", NO("ORDER"))])
add("time-order-offset-trap", "review.at sorts after created_at as text but is earlier as an instant",
    build(created="2026-10-20T10:00:00Z", review_at="2026-10-20T10:30:00+09:00"), [step("release", NO("ORDER"))])
add("time-fraction-order", "same second, fractions decide: review.at .25 is before created_at .5",
    build(created="2026-10-20T10:00:00.5+09:00", review_at="2026-10-20T10:00:00.25+09:00"), [step("release", NO("ORDER"))])
add("time-no-timezone", "created_at without a time zone", build(created="2026-10-20T10:00:00"), [step("release", NO("MANIFEST_INVALID"))])
add("time-bad-date", "created_at 30 February", build(created="2026-02-30T10:00:00+09:00"), [step("release", NO("MANIFEST_INVALID"))])
add("time-review-bad-format", "review.at with a space instead of T", build(review_at="2026-10-20 11:00:00+09:00"), [step("release", NO("MANIFEST_INVALID"))])
add("time-review-not-string", "review.at as a number", build(post_manifest=rep('"at": "2026-10-20T11:00:00+09:00"', '"at": 1760925600')),
    [step("release", NO("MANIFEST_INVALID"))])
add("already-run", "first_real_run_at is set: this registration was run (the designation does not pin that field)",
    build(post_manifest=rep('"first_real_run_at": null', '"first_real_run_at": "2026-10-20T11:30:00+09:00"')), [step("release", NO("ALREADY_RUN"))])
add("audit-not-run", "audit of a manifest that records no run", build(), [step("audit", NO("AUDIT_NOT_RUN"))])
add("audit-run-before-review", "first_real_run_at before review.at", build(run_at="2026-10-20T10:30:00+09:00", marker=True),
    [step("audit", NO("ORDER"))], base="audited")
add("audit-run-equals-review", "first_real_run_at equal to review.at", build(run_at="2026-10-20T11:00:00+09:00", marker=True),
    [step("audit", NO("ORDER"))], base="audited")
add("audit-run-bad-format", "first_real_run_at that is not an ISO instant", build(run_at="yesterday", marker=True), [step("audit", NO("MANIFEST_INVALID"))], base="audited")
add("audit-marker-missing", "a recorded run without a marker", build(run_at="2026-10-20T11:30:00+09:00"),
    [step("audit", NO("MARKER_MISSING"))], base="audited")

# ---- gate, review and designation files -------------------------------------------------------------------------------------
add("gate-file-missing", "gate file removed", build(drop=("prereg/%s.v1.md" % GATE,)), [step("release", NO("GATE_FILE_MISSING"))])
add("gate-file-tampered", "gate file edited after the manifest recorded it",
    build(post_files=lambda f: f.__setitem__("prereg/%s.v1.md" % GATE, GATE_MD + b"edited after registration\n")), [step("release", NO("GATE_SHA_MISMATCH"))])
add("review-file-missing", "review file removed", build(drop=("prereg/%s.v1.review.md" % GATE,)), [step("release", NO("REVIEW_FILE_MISSING"))])
add("review-file-tampered", "review file edited after the manifest recorded it",
    build(post_files=lambda f: f.__setitem__("prereg/%s.v1.review.md" % GATE, REVIEW_MD + b"REJECT\n")), [step("release", NO("REVIEW_FILE_SHA"))])
fx_missing = build(drop=("prereg/%s.v1.review-designation.json" % GATE,))
add("designation-missing", "designation file removed", fx_missing, [step("release", NO("DESIGNATION_MISSING"))])
add("designation-tampered", "designation file edited after the manifest recorded its sha256 (permitted flipped back)",
    build(post_designation=lambda b: b + b"\n"), [step("release", NO("DESIGNATION_SHA"))])
add("designation-malformed", "designation that is not JSON (registered consistently)", build(designation_bytes=b"{not json"),
    [step("release", NO("DESIGNATION_INVALID"))])
add("designation-duplicate-key", "designation with a duplicate first_real_run_permitted",
    build(designation_bytes=b'{\n "gate": "demo-gate.v1",\n "first_real_run_permitted": false,\n "first_real_run_permitted": true,\n "pinned": {}\n}\n'),
    [step("release", NO("DESIGNATION_INVALID"))])
add("designation-not-object", "designation is an array", build(designation_bytes=b"[]\n"), [step("release", NO("DESIGNATION_INVALID"))])
add("designation-no-pinned", "designation without pinned", build(designation_edit=lambda d: d.pop("pinned")), [step("release", NO("DESIGNATION_INVALID"))])
add("designation-pinned-incomplete", "designation pinned without review_file_sha256",
    build(designation_edit=lambda d: d["pinned"].pop("review_file_sha256")), [step("release", NO("DESIGNATION_INVALID"))])
add("designation-gate-other-version", "designation names demo-gate.v2", build(designation_edit=lambda d: d.update(gate="demo-gate.v2")),
    [step("release", NO("DESIGNATION_GATE"))])
add("designation-gate-prefix", "designation names demo-gate.v10 (a prefix of nothing, a longer name)",
    build(designation_edit=lambda d: d.update(gate="demo-gate.v10")), [step("release", NO("DESIGNATION_GATE"))])
add("designation-gate-first-key-trap", "the top-level gate is wrong while a nested decoy names the right gate first",
    build(designation_edit=lambda d: (d["notes"].update(gate="demo-gate.v1"), d.update(gate="other-gate.v1"))),
    [step("release", NO("DESIGNATION_GATE"))])
add("designation-not-permitted", "first_real_run_permitted false", build(designation_edit=lambda d: d.update(first_real_run_permitted=False)),
    [step("release", NO("DESIGNATION_NOT_PERMITTED"))])
add("designation-permitted-string", "first_real_run_permitted as the string true", build(designation_edit=lambda d: d.update(first_real_run_permitted="true")),
    [step("release", NO("DESIGNATION_NOT_PERMITTED"))])
add("designation-permitted-trap", "top-level false while a nested decoy says true",
    build(designation_edit=lambda d: (d["notes"].update(first_real_run_permitted=True), d.update(first_real_run_permitted=False))),
    [step("release", NO("DESIGNATION_NOT_PERMITTED"))])
add("designation-pin-gate", "pinned.gate_md_sha256 is another file", build(designation_edit=lambda d: d["pinned"].update(gate_md_sha256="1" * 64)),
    [step("release", NO("DESIGNATION_PIN"))])
add("designation-pin-review", "pinned.review_file_sha256 is another file", build(designation_edit=lambda d: d["pinned"].update(review_file_sha256="2" * 64)),
    [step("release", NO("DESIGNATION_PIN"))])
add("designation-pin-manifest", "pinned.manifest_before_review_sha256 is another manifest",
    build(designation_edit=lambda d: d["pinned"].update(manifest_before_review_sha256="3" * 64)), [step("release", NO("DESIGNATION_PIN"))])

# ---- a manifest edited after the designation (protocol 6, bypass B1) ---------------------------------------------------------
add("tamper-tolerance", "tolerance edited after the designation (the file hashes still match)",
    build(post_manifest=rep("0.00001", "0.0001")), [step("release", NO("DESIGNATION_PIN"))])
tampered_tool = TOOL + b"patched"
add("tamper-registered-pin", "the registered pin replaced by the sha256 of a patched binary (the pin check alone would pass)",
    build(post_manifest=rep(sha(TOOL), sha(tampered_tool)), post_files=lambda f: f.__setitem__("bin/tool.bin", tampered_tool)),
    [step("release", NO("DESIGNATION_PIN"))])
add("tamper-registered-inputs", "the registered tree sha256 replaced by the one of modified data",
    build(post_manifest=rep(tree_sha(DATA), tree_sha(dict(DATA, **{"a.bin": b"x"}))),
          post_files=lambda f: f.__setitem__("data/main/a.bin", b"x")), [step("release", NO("DESIGNATION_PIN"))])
add("tamper-gate-and-manifest", "gate file and the manifest's sha256 edited together",
    build(post_files=lambda f: f.__setitem__("prereg/%s.v1.md" % GATE, GATE_MD + b"x"),
          post_manifest=rep(sha(GATE_MD), sha(GATE_MD + b"x"))), [step("release", NO("DESIGNATION_PIN"))])
add("review-block-is-not-pinned", "the review object is outside the pinned form (section 6): a changed reviewer name does not break the pin",
    build(post_manifest=rep("독립 리뷰어", "다른 리뷰어")), [step("release", OK())])

# ---- pins ----------------------------------------------------------------------------------------------------------------
add("pin-sha-mismatch", "a presented binary differs from the registered sha256",
    build(post_files=lambda f: f.__setitem__("bin/engine.wasm", WASM + b"!")), [step("release", NO("PIN_MISMATCH"))])
add("pin-unregistered", "a presented file the manifest does not register", build(extra_files={"bin/extra.bin": EXTRA}),
    [step("release", NO("PIN_UNREGISTERED"), pins=["bin/tool.bin", "bin/engine.wasm", "bin/extra.bin"])])
add("pin-not-presented", "a registered pin the request does not present", build(),
    [step("release", NO("PIN_NOT_PRESENTED"), pins=["bin/tool.bin"])])
add("pin-unreadable", "a presented path that does not exist", build(),
    [step("release", NO("PIN_UNREADABLE"), pins=["bin/gone/tool.bin", "bin/engine.wasm"])])
add("pin-duplicate", "two presented files with the same base name", build(extra_files={"bin/other/tool.bin": TOOL}),
    [step("release", NO("PIN_DUPLICATE"), pins=["bin/tool.bin", "bin/other/tool.bin", "bin/engine.wasm"])])

# ---- input data ----------------------------------------------------------------------------------------------------------
add("input-file-changed", "one data file changed", build(post_files=lambda f: f.__setitem__("data/main/a.bin", b"changed")),
    [step("release", NO("INPUT_MISMATCH"))])
add("input-extra-file", "an extra file in the data directory", build(extra_files={"data/main/extra.bin": b"x"}), [step("release", NO("INPUT_MISMATCH"))])
add("input-missing-file", "a data file removed", build(drop=("data/main/sub/b.bin",)), [step("release", NO("INPUT_MISMATCH"))])
add("input-symlink", "a symlink in the data directory", build(), [step("release", NO("INPUT_NONREGULAR"))], symlinks={"data/main/link.bin": "a.bin"})
add("input-directory-missing", "no data directory", build(drop=tuple("data/main/" + k for k in DATA)), [step("release", NO("INPUT_UNREADABLE"))])
add("input-not-supplied", "the manifest registers data for the kind but the request supplies none", build(), [step("release", NO("INPUT_NOT_SUPPLIED"), data_dir="")])
add("input-unregistered", "a data directory for a kind that registers none", build(kinds=("main", "alt")),
    [step("release", NO("INPUT_UNREGISTERED"), kind="alt")], base="twokinds")

# ---- the coordinator seed ------------------------------------------------------------------------------------------------
add("seed-policy-not-wanted", "the manifest registers a seed, the request does not take it", build(seed=True), [step("release", NO("SEED_POLICY"), want_seed=False)], base="seeded")
add("seed-policy-not-registered", "the request takes a seed the manifest does not register", build(), [step("release", NO("SEED_POLICY"), want_seed=True)])
add("seed-missing", "the registered seed file is absent", build(seed=True, seed_file=False), [step("release", NO("SEED_MISSING"))], base="seeded")
add("seed-invalid-text", "the seed file does not start with a decimal integer", build(seed=True, seed_body=b"abc 2026\n"), [step("release", NO("SEED_INVALID"))], base="seeded")
add("seed-invalid-leading-zero", "seed with a leading zero", build(seed=True, seed_body=b"007\n"), [step("release", NO("SEED_INVALID"))], base="seeded")
add("seed-invalid-too-large", "seed above 2^64-1", build(seed=True, seed_body=b"18446744073709551616\n"), [step("release", NO("SEED_INVALID"))], base="seeded")
add("seed-invalid-empty", "empty seed file", build(seed=True, seed_body=b""), [step("release", NO("SEED_INVALID"))], base="seeded")
add("seed-forbidden", "the seed equals a seed the executor ran before the review", build(seed=True, dev_seeds=(SEED, "777")),
    [step("release", NO("SEED_FORBIDDEN"))], base="seeded")
add("seed-pin-mismatch", "designation pinned.seed_file_sha256 differs from the seed file",
    build(seed=True, pin_seed=True, designation_edit=lambda d: d["pinned"].update(seed_file_sha256="4" * 64)),
    [step("release", NO("SEED_PIN", no_leak=[SEED]))], base="seeded")
add("seed-not-leaked-on-pin-refusal", "a refusal before the seed is read does not carry the seed anywhere",
    build(seed=True, post_files=lambda f: f.__setitem__("bin/engine.wasm", WASM + b"!")),
    [step("release", NO("PIN_MISMATCH", no_leak=[SEED]))], base="seeded")
add("seed-not-leaked-on-rerun", "a refused second start does not carry the seed",
    build(seed=True), [step("release", OK(seed=SEED)), step("release", NO("MARKER_EXISTS", no_leak=[SEED]))], base="seeded")
add("seed-not-leaked-by-check", "CheckRelease validates the seed but neither returns nor logs it",
    build(seed=True), [step("check", OK(seed="", no_leak=[SEED]))], base="seeded")
add("seed-bad-leaves-no-marker", "a bad seed does not burn the run: the refusal comes before the marker (every refusal leaves the markers unchanged)",
    build(seed=True, seed_body=b"abc\n"), [step("release", NO("SEED_INVALID"))], base="seeded")

# ---- report mode refuses registered data (by content) ---------------------------------------------------------------------------
sealed_probe = b"a sealed replay list\n"
copy_files = {"report/renamed/" + k: v for k, v in DATA.items()}
add("sealed-original", "the registered data directory itself", build(), [step("sealed", NO("SEALED_DATA"), data_dir="data/main")])
add("sealed-renamed-copy", "a copy under another directory name has the same content, so it is refused",
    build(extra_files=copy_files), [step("sealed", NO("SEALED_DATA"), data_dir="report/renamed")])
add("sealed-other-data", "other data is not registered data", build(extra_files={"report/dev/x.bin": b"development data\n"}),
    [step("sealed", OK(), data_dir="report/dev")])
add("sealed-file-sha", "a directory holding a file with a registered sealed sha256",
    build(sealed_files=(sha(sealed_probe),), extra_files={"report/mixed/replay.txt": sealed_probe, "report/mixed/other.bin": b"o"}),
    [step("sealed", NO("SEALED_DATA"), data_dir="report/mixed")])
add("sealed-manifest-missing", "no manifest: fail closed", build(drop=("prereg/%s.v1.manifest.json" % GATE,), extra_files={"report/dev/x.bin": b"d"}),
    [step("sealed", NO("SEALED_DATA"), data_dir="report/dev")])
add("sealed-manifest-malformed", "unparsable manifest: fail closed", build(post_manifest=lambda t: "{", extra_files={"report/dev/x.bin": b"d"}),
    [step("sealed", NO("SEALED_DATA"), data_dir="report/dev")])
add("sealed-unhashable", "a symlink in the directory: it cannot be told from registered data, fail closed",
    build(extra_files={"report/dev/x.bin": b"d"}), [step("sealed", NO("SEALED_DATA"), data_dir="report/dev")], symlinks={"report/dev/l.bin": "x.bin"})
add("sealed-manifest-no-held", "a manifest without held: fail closed", build(post_manifest=lambda t: "{}", extra_files={"report/dev/x.bin": b"d"}),
    [step("sealed", NO("SEALED_DATA"), data_dir="report/dev")])
add("sealed-args", "no data_dir", build(), [step("sealed", NO("ARGS"), data_dir="")])


# ---- canonical form and parser strictness -----------------------------------------------------------------------------------------
def canon_case(name, text, *, raw=None, err=False):
    body = text.encode() if raw is None else raw
    c = {"name": name, "input": enc(body)}
    if err:
        c["expect_text"] = "ERR"
    else:
        c["expect_text"] = canon_text(json.loads(text))
        c["expect_sha256"] = sha(c["expect_text"].encode())
    return c


CANON = [
    canon_case("ordering-nesting", '{"b":1,"a":{"d":1,"c":[3,2,{"z":null,"y":true}]}}'),
    canon_case("blanked-top-level-only", '{"review":{"a":1},"x":{"review":1,"held_run":2},"first_real_run_at":null,"held_run":1,"k":[{"review":1}]}'),
    canon_case("number-forms", '{"v":[1e22,0.1,1E-7,123456789.125,1e16,1.5e300,-0.0,5e-324,100.0,0.0001,0.00001234,1e15,2.5E+3,-0,0,1.0,0.5e1,1e0,12345678901234567890123,1.7976931348623157e308,2.2250738585072014e-308,9007199254740993.0,0.30000000000000004,1e-5,1e-4,123456789012345680.0]}'),
    canon_case("string-escapes", '{"s":"quote\\" back\\\\ slash\\/ tab\\t nl\\n cr\\r bs\\b ff\\f nul\\u0000 one\\u0001 us\\u001f del\\u007f é 한글 \\u00e9 \\ud83d\\ude00 😀"}'),
    canon_case("key-order-codepoint", '{"\\uff5e":1,"\\ud83d\\ude00":2,"a":3,"\\u00e9":4,"Z":5,"":6}'),
    canon_case("empty-containers", '{"a":{"b":{"c":[[],{}, [[]]]}}, "e":{}, "f":[]}'),
    canon_case("whitespace", '  \n\t {"a" : 1 }  \r\n'),
    canon_case("dup-key", '{"a":1,"a":2}', err=True),
    canon_case("dup-key-nested", '{"a":{"b":1,"b":1}}', err=True),
    canon_case("trailing-comma-object", '{"a":1,}', err=True),
    canon_case("trailing-comma-array", '[1,]', err=True),
    canon_case("leading-zero", '{"a":01}', err=True),
    canon_case("nan", '{"a":NaN}', err=True),
    canon_case("infinity", '{"a":Infinity}', err=True),
    canon_case("exponent-overflow", '{"a":1e999}', err=True),
    canon_case("lone-minus", '{"a":-}', err=True),
    canon_case("trailing-dot", '{"a":1.}', err=True),
    canon_case("leading-dot", '{"a":.5}', err=True),
    canon_case("plus-sign", '{"a":+1}', err=True),
    canon_case("single-quote", '{\'a\':1}', err=True),
    canon_case("trailing-garbage", '{"a":1} x', err=True),
    canon_case("lone-high-surrogate", '{"a":"\\ud800"}', err=True),
    canon_case("lone-low-surrogate", '{"a":"\\udc00"}', err=True),
    canon_case("high-surrogate-then-bmp", '{"a":"\\ud800\\u0041"}', err=True),
    canon_case("bad-escape", '{"a":"\\x41"}', err=True),
    canon_case("raw-tab-in-string", '{"a":"tab\there"}', err=True),
    canon_case("bom", '﻿{"a":1}', err=True),
    canon_case("empty-input", '', err=True),
    canon_case("unterminated", '{"a":1', err=True),
    canon_case("bad-literal", '{"a":tru}', err=True),
    canon_case("bad-hex-escape", '{"a":"\\u12G4"}', err=True),
    canon_case("utf8-invalid-byte", '', raw=b'{"a":"\xff"}', err=True),
    canon_case("utf8-overlong", '', raw=b'{"a":"\xc0\x80"}', err=True),
    canon_case("utf8-surrogate", '', raw=b'{"a":"\xed\xa0\x80"}', err=True),
    canon_case("utf8-above-max", '', raw=b'{"a":"\xf4\x90\x80\x80"}', err=True),
    canon_case("utf8-truncated", '', raw=b'{"a":"\xe2\x82"}', err=True),
    canon_case("deep-nesting-70", '[' * 70 + ']' * 70, err=True),
]
# depth 64 is accepted (65 nested arrays are depth 64 after the object), 66+ is not
CANON.append(canon_case("deep-nesting-64", '[' * 64 + ']' * 64))

# the canonical sha256 of the real manifest of every base (what --print-pin prints) must be what the designation pins
CANON_MANIFESTS = []
for name, b in BASES.items():
    text = b["files"]["prereg/%s.v1.manifest.json" % GATE].decode()
    CANON_MANIFESTS.append({"base": name, "file": "prereg/%s.v1.manifest.json" % GATE, "expect_sha256": b["pin_manifest"]})


def inst(text):
    try:
        t = datetime.datetime.fromisoformat(text)
    except ValueError:
        return None
    if t.tzinfo is None or "T" not in text:
        return None
    return t


def instant_case(text, valid=True):
    if not valid:
        return {"text": text, "valid": False}
    t = datetime.datetime.fromisoformat(text)
    assert t.tzinfo is not None
    d = t - datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)
    frac = text.split(".")[1] if "." in text else ""
    digits = "".join(ch for ch in frac[: next((i for i, c in enumerate(frac) if not c.isdigit()), len(frac))])
    return {"text": text, "valid": True, "sec": d.days * 86400 + d.seconds, "ns": int(digits.ljust(9, "0")[:9]) if digits else 0}


INSTANTS = [instant_case(t) for t in (
    "1970-01-01T00:00:00Z", "1970-01-01T00:00:00+00:00", "2026-10-20T11:15:21+09:00", "2026-10-20T01:00:00-05:30",
    "2024-02-29T23:59:59Z", "2000-02-29T00:00:00Z", "1999-12-31T23:59:59.999999999Z", "2026-10-20T10:00:00.5+09:00",
    "2026-10-20T10:00:00.123456+09:00", "0001-01-01T00:00:00Z", "9999-12-31T23:59:59+23:59", "1969-12-31T23:59:59Z",
    "1900-03-01T00:00:00Z")]
INSTANTS += [instant_case(t, valid=False) for t in (
    "2026-10-20T11:15:21", "2026-10-20 11:15:21+09:00", "2026-10-20T11:15:21+0900", "2026-10-20T11:15:21+09", "2026-02-30T00:00:00Z",
    "2023-02-29T00:00:00Z", "1900-02-29T00:00:00Z", "2026-13-01T00:00:00Z", "2026-00-10T00:00:00Z", "2026-10-00T00:00:00Z",
    "2026-10-20T24:00:00Z", "2026-10-20T23:60:00Z", "2026-10-20T23:59:60Z", "2026-10-20T11:15:21+24:00", "2026-10-20T11:15:21+09:60",
    "2026-10-20T11:15:21.Z", "2026-10-20T11:15:21.1234567890Z", "2026-10-20T11:15:21z", "26-10-20T11:15:21Z", "0000-01-01T00:00:00Z",
    "2026-10-20T11:15:21Z ", " 2026-10-20T11:15:21Z", "", "now", "2026-10-20T11:15:21,5Z")]

PROBE = dict(DATA)
PROBE.update({"a.b": b"dot before slash", "a/b.bin": b"slash after dot", "z.bin": b"z", "한.bin": b"\xed\x95\x9c", "d/e/f/g.bin": b"deep", "A.bin": b"upper", "empty.bin": b""})
TREE_PROBE = {"files": {k: enc(v) for k, v in PROBE.items()}, "expect_sha256": tree_sha(PROBE)}

out = {
    "format": "sr_held_cases.v1",
    "generator": "tools/prereg/gen_sr_held_cases.py (expected values from the section 6 snippet, hashlib and datetime; no sr_held code)",
    "now": NOW,
    "gate": GATE,
    "bases": {n: {"files": {p: enc(b) for p, b in v["files"].items()}, "request": v["request"]} for n, v in BASES.items()},
    "canon_cases": CANON,
    "canon_manifests": CANON_MANIFESTS,
    "instant_cases": INSTANTS,
    "tree_probe": TREE_PROBE,
    "cases": CASES,
}

if __name__ == "__main__":
    dest = HERE / "sr_held_cases.json"
    if "--out" in sys.argv:
        dest = pathlib.Path(sys.argv[sys.argv.index("--out") + 1])
    ids = [c["id"] for c in CASES]
    assert len(ids) == len(set(ids)), "duplicate case ids"
    dest.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print("wrote %s: %d cases, %d canon cases, %d instant cases" % (dest, len(CASES), len(CANON), len(INSTANTS)))
