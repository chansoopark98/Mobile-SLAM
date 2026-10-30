import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import test from 'node:test';
import { verifyXrTreeIntegrity } from './xr-tree-integrity.mjs';

const RULE = 'sha256 over "served path\\0sha256(content)\\n", sorted, excluding slam-refact-xr/deploy.json';
const sha = (text) => crypto.createHash('sha256').update(text).digest('hex');
const original = [
  { path: 'slam-refact-xr.html', bytes: 1, sha256: sha('a') },
  { path: 'slam-refact-xr/engine/xr.js', bytes: 1, sha256: sha('b') },
];
const current = [original[0], { ...original[1], bytes: 2, sha256: sha('bc') }];
const digest = (rows) => sha([...rows].sort((a, b) => a.path.localeCompare(b.path)).map((r) => `${r.path}\0${r.sha256}\n`).join(''));
const clone = (v) => structuredClone(v);
const deploy = () => ({ schema: 'slam-refact-xr-deploy-v1', bundle_digest_rule: RULE,
  bundle_sha256: digest(original), files: clone(original) });
const source = () => ({ schema: 'slam-refact-runtime-integrity-v1', record_kind: 'source-artifact-index',
  bundle_digest_rule: RULE, bundle_sha256: digest(current), files: clone(current) });
const currentIndex = () => ({ ...deploy(), record_kind: 'source-artifact-index', bundle_sha256: digest(current), files: clone(current) });
const verify = (record = source(), actual = current, receipt = currentIndex()) => verifyXrTreeIntegrity({ files: actual, deploy: receipt, currentIntegrity: record });

test('public runtime metadata verifies exact current bytes without private deployment records', () => {
  const receipt = currentIndex(), before = clone(receipt);
  const result = verify(source(), current, receipt);
  assert.equal(result.metadataKind, 'runtime-integrity');
  assert.equal(result.bundleSha256, digest(current));
  assert.equal(result.files, 2);
  assert.deepEqual(receipt, before);
});

test('fresh staging uses its ordinary deployment metadata', () => {
  const result = verifyXrTreeIntegrity({ files: original, deploy: deploy() });
  assert.equal(result.metadataKind, 'deployment-receipt');
  assert.equal(result.bundleSha256, digest(original));
  assert.throws(() => verifyXrTreeIntegrity({ files: current, deploy: deploy() }), /sha256|bytes|digest/);
});

test('content changes and incorrect size or digest cannot pass current metadata', () => {
  const content = clone(current); content[1].sha256 = sha('bd');
  assert.throws(() => verify(source(), content), /sha256/);
  const size = source(); size.files[1].bytes++;
  assert.throws(() => verify(size), /bytes/);
  const digestMismatch = source(); digestMismatch.bundle_sha256 = sha('different');
  assert.throws(() => verify(digestMismatch), /digest/);
});

test('metadata must contain the exact content-file set, with no missing, extra or duplicate row', () => {
  const missing = source(); missing.files.pop();
  assert.throws(() => verify(missing), /file set/);
  const extra = source(); extra.files.push({ path: 'slam-refact-xr/extra.js', bytes: 1, sha256: sha('x') });
  assert.throws(() => verify(extra), /file set/);
  const duplicate = source(); duplicate.files.push(clone(duplicate.files[0]));
  assert.throws(() => verify(duplicate), /duplicate/);
  const receipt = currentIndex(); receipt.files.pop();
  assert.throws(() => verify(source(), current, receipt), /file set/);
});

test('malformed schema, hashes, sizes and file lists are rejected without fallback', () => {
  for (const value of [null, [], {}, { ...source(), schema: 'slam-refact-runtime-integrity-v2' },
    { ...source(), files: null }, { ...source(), files: [] }, { ...source(), record_kind: 'deployment' },
    { ...source(), bundle_digest_rule: 'other' }]) {
    assert.throws(() => verify(value), /integrity|schema|files|deployment|digest rule|file set/);
  }
  for (const change of [{ bytes: '2' }, { bytes: -1 }, { bytes: NaN }, { sha256: 'bad' }]) {
    const record = source(); Object.assign(record.files[1], change);
    assert.throws(() => verify(record), /bytes|sha256/);
  }
});

test('metadata paths cannot select outside files or the deployment receipt itself', () => {
  for (const name of ['../outside', 'web/../outside', '/slam-refact-xr.html', 'slam-refact-xr/../outside',
    'slam-refact-xr//engine/xr.js', 'slam-refact-xr/engine\\xr.js', 'slam-refact-xr/deploy.json']) {
    const record = source(); record.files[1].path = name;
    assert.throws(() => verify(record), /path/);
  }
  const indexMismatch = currentIndex(); indexMismatch.bundle_sha256 = sha('not the current artifact index');
  assert.throws(() => verify(source(), current, indexMismatch), /digest/);
});

test('ordinary deployment metadata retains per-file size, hash, set and digest checks', () => {
  for (const mutate of [
    (m) => { m.files[0].sha256 = sha('wrong'); },
    (m) => { m.files[0].bytes++; },
    (m) => { m.files.pop(); },
    (m) => { m.bundle_sha256 = sha('wrong'); },
    (m) => { m.schema = 'other'; },
  ]) {
    const receipt = deploy(); mutate(receipt);
    assert.throws(() => verifyXrTreeIntegrity({ files: original, deploy: receipt }), /sha256|bytes|file set|digest|schema/);
  }
});
