import fs from 'node:fs/promises';
import path from 'node:path';
import assert from 'node:assert/strict';
import {stripTypeScriptTypes} from 'node:module';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';

// Execute pinned upstream modules after type erasure, without editing the checkout.
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../..');
const upstream = path.join(root, 'assets/references/8thwall/official/8thwall');
const out = path.resolve(root, process.argv[2] || 'build/8thwall-deep-20261014/source-contracts');
if (!out.startsWith(path.join(root, 'build') + path.sep)) throw new Error('Output must be inside local build/');
await fs.mkdir(out, {recursive: false});
const sha = data => createHash('sha256').update(data).digest('hex');
const report = {
  startedAt: new Date().toISOString(), node: process.version,
  head: execFileSync('git', ['-C', upstream, 'rev-parse', 'HEAD'], {encoding: 'utf8'}).trim(),
  method: 'Node stripTypeScriptTypes in strip mode; original source snapshots and generated JS retained',
  modules: [], cases: [],
  limits: ['Isolated source contracts, not production bundle parity or SLAM accuracy',
    'No upstream npm/Bazel full suite or typecheck performed'],
};
async function load(name) {
  const relative = `reality/app/xr/js/src/${name}.ts`;
  const bytes = await fs.readFile(path.join(upstream, relative));
  const js = stripTypeScriptTypes(bytes.toString(), {mode: 'strip'});
  await fs.writeFile(path.join(out, name + '.ts'), bytes);
  await fs.writeFile(path.join(out, name + '.mjs'), js);
  report.modules.push({path: relative, sha256: sha(bytes), executableSha256: sha(js)});
  return import('data:text/javascript;base64,' + Buffer.from(js).toString('base64'));
}
function check(name, fn) {
  try { const observation = fn(); report.cases.push({name, passed: true, observation}); }
  catch (e) { report.cases.push({name, passed: false, error: String(e)}); }
}
try {
  const pixels = await load('pixel-transforms');
  const {parsePreloadChunks} = await load('preload-chunks-format');
  const {createCallbackManager} = await load('callback-manager');
  const {createContinuousLifecycle} = await load('lifecycle');
  const {SetManager} = await load('set-manager');
  check('crop chooses covering scale rather than fitting scale', () => {
    assert.equal(pixels.scaleFactorSameAspectRatioCrop(640, 480, 320, 320), 2 / 3);
    return {scale: 2 / 3, scaledWidth: 640 * 2 / 3, scaledHeight: 320};
  });
  const rgba = ids => Uint8Array.from(ids.flatMap(x => [x, x, x, 255]));
  const ids = array => Array.from(array).filter((_, i) => i % 4 === 0);
  check('2x3 image clockwise and counterclockwise pixel identity', () => {
    const source = rgba([1, 2, 3, 4, 5, 6]);
    const cw = new Uint8Array(24), ccw = new Uint8Array(24);
    pixels.toImageArrayRotateCW90(source, 2, 3, 3, 2, cw);
    pixels.toImageArrayRotateCCW90(source, 2, 3, 3, 2, ccw);
    assert.deepEqual(ids(cw), [5, 3, 1, 6, 4, 2]);
    assert.deepEqual(ids(ccw), [2, 4, 6, 1, 3, 5]);
    return {cw: ids(cw), ccw: ids(ccw)};
  });
  check('letterbox writes center but caller owns padding initialization', () => {
    const dst = new Uint8Array(4 * 4 * 4).fill(77);
    pixels.toImageArrayLetterbox(rgba([1, 2, 3, 4]), 2, 2, 4, 4, dst);
    assert.deepEqual(ids(dst), [77, 77, 77, 77, 77, 1, 2, 77, 77, 3, 4, 77, 77, 77, 77, 77]);
    return {redChannel: ids(dst), implication: 'Padding is not zeroed by this helper'};
  });
  check('color normalization maps 0 and 255 to 0 and 1', () => {
    const arr = new Float32Array([0, 127.5, 255]); pixels.normalizeColors(arr);
    assert.deepEqual(Array.from(arr), [0, 0.5, 1]); return Array.from(arr);
  });
  check('preload parsing retains URL colon and removes empty segments', () => {
    const actual = parsePreloadChunks(' slam, face: https://local.test/xr-face.js, , sky: ');
    assert.deepEqual(actual, [{name: 'slam', url: null}, {name: 'face', url: 'https://local.test/xr-face.js'}, {name: 'sky', url: null}]);
    assert.deepEqual(parsePreloadChunks(''), []); return actual;
  });
  check('removed module disables callback even in an earlier callback snapshot', () => {
    const manager = createCallbackManager(), calls = [];
    manager.addModule({name: 'a', onUpdate: () => calls.push('a')});
    manager.addModule({name: 'b', onUpdate: () => calls.push('b')});
    const snapshot = manager.getCallbacks('onUpdate'); manager.removeModule('a');
    snapshot.forEach(fn => fn()); assert.deepEqual(calls, ['b']);
    assert.deepEqual(manager.getNamedCallbacks('onUpdate').map(x => x.name), ['b']);
    return {calls};
  });
  check('wrapping applies to frame callback but not lifecycle callback', () => {
    const manager = createCallbackManager(), calls = [];
    manager.addModule({name: 'a', onUpdate: () => calls.push('update'), onStart: () => calls.push('start')},
      fn => (...args) => { calls.push('wrapped'); return fn(...args); });
    manager.getCallbacks('onStart')[0](); manager.getCallbacks('onUpdate')[0]();
    assert.deepEqual(calls, ['start', 'wrapped', 'update']); return {calls};
  });
  check('modules added during pause attach on resume and detach once', () => {
    const modules = ['a', 'b'], events = [];
    const lifecycle = createContinuousLifecycle(
      predicate => modules.filter(predicate).forEach(x => events.push('start:' + x)),
      predicate => modules.filter(predicate).forEach(x => events.push('stop:' + x)));
    lifecycle.add('a'); lifecycle.attach(); lifecycle.pause(); lifecycle.add('b');
    assert.deepEqual(events, ['start:a']); lifecycle.resume(); lifecycle.remove('a'); lifecycle.detach();
    assert.deepEqual(events, ['start:a', 'start:b', 'stop:a', 'stop:b']); return {events};
  });
  check('dynamic resource set refresh and copy isolation', () => {
    const manager = new SetManager(['base']); let resource = 'first';
    manager.add('dynamic', () => [resource]);
    assert.deepEqual([...manager.get()], ['base', 'first']); resource = 'second';
    const copy = manager.getCopy(); copy.add('foreign');
    assert.deepEqual([...manager.get()], ['base', 'second']); manager.remove('dynamic');
    assert.deepEqual([...manager.get()], ['base']); return {final: [...manager.get()]};
  });
} catch (e) { report.error = String(e); }
report.passed = !report.error && report.cases.length === 9 && report.cases.every(x => x.passed);
report.finishedAt = new Date().toISOString();
await fs.copyFile(fileURLToPath(import.meta.url), path.join(out, 'source-contracts.mjs'));
await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({passed: report.passed, cases: report.cases, error: report.error}, null, 2));
if (!report.passed) process.exitCode = 1;
