import assert from 'node:assert/strict';
import {readFileSync, writeFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {dirname, join} from 'node:path';
const base = dirname(fileURLToPath(import.meta.url));
// Decimal strings prevent the JSON Number parser from losing precision first.
const exact = ['9007199254740992', '9007199254740993'].map(BigInt);
const asNumbers = exact.map(Number);
assert.equal(exact[1] - exact[0], 1n);
assert.equal(asNumbers[1] - asNumbers[0], 0);
const localOrigin = exact[0];
assert.equal(Number(exact[1] - localOrigin), 1);
assert.equal(Number(exact[0] - localOrigin), 0);
// A fabricated one-ns mismatch must remain a mismatch, never be paired by Number equality.
assert.notEqual(exact[0], exact[1]);
assert.equal(asNumbers[0], asNumbers[1]);
const result = {
  schema: 'vuforia-public-timestamp-number-probe-v1',
  evidence_scope: 'javascript_integer_representation_only',
  vuforia_runtime_calls: 0,
  node_version: process.version,
  input_nanoseconds: exact.map(String),
  number_difference_ns: asNumbers[1] - asNumbers[0],
  bigint_difference_ns: String(exact[1] - exact[0]),
  rebased_number_difference_ns: Number(exact[1] - localOrigin),
  assertions_passed: 6,
  fixture: JSON.parse(readFileSync(join(base, 'contract-fixture.json'), 'utf8')).schema,
  limits: 'Fabricated large-ns boundary. Not a measured phone clock or Vuforia timestamp bug.'
};
writeFileSync(join(process.env.VUFORIA_EVIDENCE_DIR || base, 'timestamp-results.json'), JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result));
