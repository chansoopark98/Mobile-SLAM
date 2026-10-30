// Conformance test of sr_held.mjs against the shared case table sr_held_cases.json (made by gen_sr_held_cases.py from
// independent references). The C++ twin is slam_refact/engine/tests/support/sr_held_conformance.cc; both read the same
// file and must make the same decision for every case.
//
//   node --test slam_refact/tools/prereg/sr_held_conformance.test.mjs
//
// Environment (all optional):
//   SR_HELD_CASES       the case table (default: sr_held_cases.json next to this file)
//   SR_HELD_TMP         directory for the per-case work roots (default: the system temp dir); roots are kept
//   SR_HELD_DECISIONS   file that receives one "DECISION <id>#<n> <op> <OK|CODE> seed=<yes|no>" line per step (the C++ test
//                       writes the same lines; tools/prereg/run_sr_held_conformance.sh diffs the two)
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test, { after } from 'node:test';
import { fileURLToPath } from 'node:url';

import * as held from './sr_held.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const casesPath = process.env.SR_HELD_CASES ?? path.join(here, 'sr_held_cases.json');
const table = JSON.parse(fs.readFileSync(casesPath, 'utf8'));
const tmpBase = process.env.SR_HELD_TMP ?? os.tmpdir();
fs.mkdirSync(tmpBase, { recursive: true });
const runDir = fs.mkdtempSync(path.join(tmpBase, 'sr-held-node-'));
const decisions = [];

const body = (spec) => (spec.text !== undefined ? Buffer.from(spec.text, 'utf8') : Buffer.from(spec.b64, 'base64'));

function materialize(root, baseName, overrides, symlinks) {
  const files = { ...table.bases[baseName].files };
  for (const [p, spec] of Object.entries(overrides ?? {})) {
    if (spec === null) delete files[p];
    else files[p] = spec;
  }
  for (const [p, spec] of Object.entries(files)) {
    const dest = path.join(root, p);
    fs.mkdirSync(path.dirname(dest), { recursive: true });
    fs.writeFileSync(dest, body(spec));
  }
  for (const [p, target] of Object.entries(symlinks ?? {})) {
    const dest = path.join(root, p);
    fs.mkdirSync(path.dirname(dest), { recursive: true });
    fs.symlinkSync(target, dest);
  }
}

const markers = (root) => {
  try {
    return fs.readdirSync(path.join(root, 'prereg')).filter((n) => n.endsWith('.marker')).sort();
  } catch {
    return [];
  }
};

function toRequest(root, r) {
  const abs = (p) => (p === '' ? '' : path.join(root, p));
  return {
    preregDir: abs(r.prereg_dir), gateId: r.gate_id, version: r.version, kind: r.kind, env: r.env,
    pins: (r.pins ?? []).map(abs), dataDir: abs(r.data_dir ?? ''), wantSeed: r.want_seed === true, now: r.now,
  };
}

const run = {
  release: held.heldGate,
  check: held.checkRelease,
  audit: held.audit,
  sealed: held.refuseSealed,
};

for (const c of table.cases) {
  test(c.id, () => {
    const root = fs.mkdtempSync(path.join(runDir, `${c.id}-`));
    materialize(root, c.base, c.files, c.symlinks);
    c.steps.forEach((s, n) => {
      const before = markers(root);
      const rq = toRequest(root, { ...table.bases[c.base].request, ...s.request });
      const o = run[s.op](rq);
      const label = `${c.id}#${n} ${s.op}`;
      decisions.push(`DECISION ${c.id}#${n} ${s.op} ${o.ok ? 'OK' : o.code} seed=${o.seed === '' ? 'no' : 'yes'}`);
      assert.equal(o.ok, s.expect.ok, `${label}: ok (code ${o.code}: ${o.why})`);
      assert.equal(o.code, s.expect.code, `${label}: code (${o.why})`);
      assert.equal(o.seed, s.expect.seed ?? '', `${label}: seed`);
      const after = markers(root);
      if (s.op === 'release' && o.ok) {
        const created = after.filter((m) => !before.includes(m));
        assert.deepEqual(created, [held.markerName({ gateId: rq.gateId, version: rq.version, kind: rq.kind })], `${label}: exactly the marker of this kind is created`);
        const markerPath = path.join(rq.preregDir, created[0]);
        const doc = JSON.parse(fs.readFileSync(markerPath, 'utf8'));
        assert.equal(doc.gate, `${rq.gateId}.v${rq.version}`);
        assert.equal(doc.kind, rq.kind);
        assert.equal(doc.designation_sha256, o.designationSha256);
        assert.equal(fs.statSync(markerPath).mode & 0o777, 0o444, `${label}: marker mode`);
        assert.equal(o.marker, markerPath);
      } else {
        assert.deepEqual(after, before, `${label}: a step that does not release must leave the markers unchanged`);
        assert.equal(o.marker, '', `${label}: no marker path`);
      }
      const text = [o.why, o.code, ...o.log].join('\n');
      for (const secret of s.expect.no_leak ?? []) assert.ok(!text.includes(secret), `${label}: the seed leaked into the reason or the log`);
      if (s.op !== 'release') assert.equal(o.seed, '', `${label}: only a release hands out the seed`);
    });
  });
}

table.canon_cases.forEach((c) => {
  test(`canon-${c.name}`, () => {
    const parsed = held.parseJson(body(c.input));
    let text = 'ERR';
    if (parsed.err === undefined) text = held.canonManifestText(parsed.json) ?? 'ERR';
    decisions.push(`DECISION canon-${c.name} canon ${text === 'ERR' ? 'ERR' : 'OK'} seed=no`);
    assert.equal(text, c.expect_text);
    if (c.expect_sha256 !== undefined) assert.equal(held.sha256(Buffer.from(text, 'utf8')), c.expect_sha256);
  });
});

for (const m of table.canon_manifests) {
  test(`canon-manifest-${m.base}`, () => {
    const parsed = held.parseJson(body(table.bases[m.base].files[m.file]));
    assert.equal(parsed.err, undefined);
    assert.equal(held.canonManifestSha(parsed.json), m.expect_sha256);
  });
}

table.instant_cases.forEach((c, n) => {
  test(`instant-${n}`, () => {
    const t = held.parseInstant(c.text);
    decisions.push(`DECISION instant-${n} instant ${t === null ? 'INVALID' : 'OK'} seed=no`);
    if (!c.valid) {
      assert.equal(t, null, `"${c.text}" must be refused`);
    } else {
      assert.notEqual(t, null, `"${c.text}" must be accepted`);
      assert.equal(t.sec, c.sec);
      assert.equal(t.ns, c.ns);
    }
  });
});

test('tree-probe', () => {
  const dir = fs.mkdtempSync(path.join(runDir, 'tree-'));
  for (const [rel, spec] of Object.entries(table.tree_probe.files)) {
    fs.mkdirSync(path.dirname(path.join(dir, rel)), { recursive: true });
    fs.writeFileSync(path.join(dir, rel), body(spec));
  }
  assert.equal(held.treeSha(dir).sha, table.tree_probe.expect_sha256);
});

test('cli-print-pin-and-check', () => {
  const root = fs.mkdtempSync(path.join(runDir, 'cli-'));
  materialize(root, 'seeded', {}, {});
  const manifest = path.join(root, 'prereg', `${table.gate}.v1.manifest.json`);
  const cli = path.join(here, 'sr_held.mjs');
  const pin = execFileSync(process.execPath, [cli, 'print-pin', manifest], { encoding: 'utf8' }).trim();
  assert.equal(pin, JSON.parse(body(table.bases.seeded.files[`prereg/${table.gate}.v1.review-designation.json`]).toString()).pinned.manifest_before_review_sha256);
  const req = table.bases.seeded.request;
  const out = execFileSync(process.execPath, [cli, 'check', '--prereg', path.join(root, 'prereg'), '--gate', table.gate, '--version', '1',
    '--env-var', 'SR_HELD_TEST_ENV', '--pin', path.join(root, 'bin/tool.bin'), '--pin', path.join(root, 'bin/engine.wasm'),
    '--data', path.join(root, 'data/main'), '--seed'], { encoding: 'utf8', env: { ...process.env, SR_HELD_TEST_ENV: req.env } });
  assert.match(out, /RELEASED \(check only: no marker created\)/);
  assert.ok(!out.includes('18446744073709551615'), 'the check CLI must not print the seed');
  assert.deepEqual(markers(root), []);
});

after(() => {
  if (process.env.SR_HELD_DECISIONS) fs.writeFileSync(process.env.SR_HELD_DECISIONS, `${decisions.sort().join('\n')}\n`);
});
