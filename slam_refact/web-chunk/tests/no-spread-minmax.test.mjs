// FX-C2a: a spread into Math.max/Math.min throws RangeError on large arrays and returns -/+Infinity on empty ones.
// Statically forbid the pattern in slam_refact/web-chunk and slam_refact/web-smoke (this file is excluded: it names the pattern).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const roots = [path.resolve(here, '..'), path.resolve(here, '../../web-smoke')];
const SKIP_DIRS = new Set(['node_modules', '.git']);
const EXT = new Set(['.js', '.mjs', '.cjs', '.html']);
const PATTERN = /Math\.(max|min)\(\s*\.\.\./;

function* walk(dir) {
  for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
    if (e.isDirectory()) { if (!SKIP_DIRS.has(e.name)) yield* walk(path.join(dir, e.name)); }
    else if (EXT.has(path.extname(e.name))) yield path.join(dir, e.name);
  }
}

test('FX-C2a: no Math.max/min(...spread) in web-chunk or web-smoke', () => {
  const self = fileURLToPath(import.meta.url);
  const hits = [];
  let scanned = 0;
  for (const root of roots) for (const f of walk(root)) {
    if (f === self) continue;
    scanned++;
    if (PATTERN.test(fs.readFileSync(f, 'utf8'))) hits.push(path.relative(path.resolve(here, '../..'), f));
  }
  assert.ok(scanned > 20, `scanned only ${scanned} files`);
  assert.deepEqual(hits, []);
});

test('FX-C2a: the pattern regexp matches spread forms, including across a newline', () => {
  assert.ok(PATTERN.test('Math.max(...a)'));
  assert.ok(PATTERN.test('Math.min(\n  ...a)'));
  assert.ok(!PATTERN.test('Math.max(a, b)'));
});
