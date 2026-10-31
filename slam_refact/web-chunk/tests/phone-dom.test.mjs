// The phone page's HTML and the scripts that bind to it: every id the scripts look up exists, and the two anchors deploy_xr.py needs are intact.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const dir = path.join(path.dirname(fileURLToPath(import.meta.url)), '../samples/phone');
const html = fs.readFileSync(path.join(dir, 'index.html'), 'utf8');
const ids = new Set([...html.matchAll(/\sid="([^"]+)"/g)].map((m) => m[1]));

test('every id that app.js and hud.js look up exists in index.html', () => {
  for (const file of ['app.js', 'hud.js']) {
    const src = fs.readFileSync(path.join(dir, file), 'utf8');
    const wanted = [...src.matchAll(/(?:\$|getElementById)\('([A-Za-z][\w-]*)'\)/g)].map((m) => m[1]);
    assert.ok(wanted.length > 10, `${file}: looked ids up`);
    for (const id of wanted) assert.ok(ids.has(id), `${file} looks up #${id}, not in index.html`);
  }
});

test('ids are unique and the controls the metrics/recorder path needs are still there', () => {
  assert.equal([...html.matchAll(/\sid="([^"]+)"/g)].length, ids.size, 'duplicate id');
  for (const id of ['b-metrics', 'b-recenter', 'b-clear', 'b-stop', 'b-restart', 'f-json', 'f-image', 'b-add-target', 'demo-targets', 'target-list', 'camerafeed']) assert.ok(ids.has(id), id);
});

test('deploy_xr.py anchors: one meta charset, the exact start-note tag, no <base>', () => {
  assert.equal(html.split('<meta charset="utf-8">').length - 1, 1);
  assert.equal(html.split('<p id="start-note" class="note"></p>').length - 1, 1);
  assert.ok(!html.includes('<base'));
});

test('camera-resolution.js is still the first script, before xr.js can be loaded', () => {
  assert.ok(html.indexOf('camera-resolution.js') > 0 && html.indexOf('camera-resolution.js') < html.indexOf('app.js'));
});
