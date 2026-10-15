import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const argv = process.argv.slice(2);
if (argv.length !== 4 || argv[0] !== '--input' || argv[2] !== '--output') throw Error('Usage: --input target.wasm --output NEW_DIRECTORY');
const input = path.resolve(argv[1]);
const outputPath = path.resolve(argv[3]);
const resolvedParent = fs.realpathSync(path.dirname(outputPath));
if (resolvedParent === '/mnt/backup/SLAM' || resolvedParent.startsWith('/mnt/backup/SLAM/')) throw Error('Canonical dataset is read-only');
if (fs.existsSync(outputPath)) throw Error('Output already exists; preserving evidence');
const out = pathToFileURL(outputPath + '/');
const original = fs.readFileSync(input);
const hash = b => crypto.createHash('sha256').update(b).digest('hex');
const leb = n => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
if (hash(original) !== '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422') throw Error('Input differs from pinned standalone module');
fs.mkdirSync(outputPath);
fs.copyFileSync(new URL(import.meta.url), new URL('binary-residual-probe.mjs', out));
const read = (b, pos) => { let n = 0, shift = 0, v; do { v = b[pos++]; n |= (v & 127) << shift; shift += 7; } while (v & 128); return [n, pos]; };
const exports = [['probeCameraMovement', 1589], ['probeRotationTarget', 1607], ['probeZeroTarget', 1604]];
const parts = [original.subarray(0, 8)];
let pos = 8, codeBefore, codeAfter;
while (pos < original.length) {
  const start = pos, id = original[pos++];
  const [length, p] = read(original, pos);
  pos = p + length;
  let payload = original.subarray(p, pos);
  if (id === 10) codeBefore = hash(payload);
  if (id === 7) {
    const [count, rest] = read(payload, 0);
    const extra = exports.map(([name, index]) => Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(index)]));
    payload = Buffer.concat([leb(count + exports.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else parts.push(original.subarray(start, pos));
  if (id === 10) codeAfter = hash(payload);
}
const patched = Buffer.concat(parts);
fs.writeFileSync(new URL('analytical-export-copy.wasm', out), patched);
const compiled = new WebAssembly.Module(patched);
const stubs = {}, invokedImports = [];
for (const imp of WebAssembly.Module.imports(compiled)) {
  if (imp.kind !== 'function') throw Error('Unexpected nonfunction import ' + imp.name);
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...args) => { invokedImports.push({name: imp.name, args}); throw Error('Unexpected imported call ' + imp.name); };
}
const instance = new WebAssembly.Instance(compiled, stubs);
const view = new DataView(instance.exports.memory.buffer);
const object = 2 ** 21, functor = object + 256, parameters = object + 512, camera = object + 768;
const residual = object + 1024, jacobianArray = object + 1280, jacobian = object + 1536;
const inputCamera = [1, 2, 3, 4, 5, 6], reference = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
const results = [];
for (const [name] of exports) {
  for (const withJacobian of [false, true]) {
    new Uint8Array(instance.exports.memory.buffer, object, 2048).fill(0);
    view.setUint32(object + 20, functor, true);
    view.setUint32(parameters, camera, true);
    inputCamera.forEach((v, i) => view.setFloat64(camera + 8 * i, v, true));
    reference.forEach((v, i) => view.setFloat64(functor + 8 * i, v, true));
    const rows = name === 'probeCameraMovement' ? 6 : 3;
    let expected, expectedJacobian;
    if (name === 'probeCameraMovement') {
      view.setFloat64(functor + 48, 0.2, true); view.setFloat64(functor + 56, 2, true);
      expected = [1.8, 3.6, 5.4, 0.72, 0.9, 1.08];
      expectedJacobian = Array.from({length: 36}, (_, n) => Math.floor(n / 6) === n % 6 ? (n % 6 < 3 ? 2 : 0.2) : 0);
    } else if (name === 'probeRotationTarget') {
      expected = [0.9, 1.8, 2.7];
      expectedJacobian = Array.from({length: 18}, (_, n) => Math.floor(n / 6) === n % 6 ? 1 : 0);
    } else {
      view.setInt32(functor, 3, true);
      expected = [4, 5, 6];
      expectedJacobian = Array.from({length: 18}, (_, n) => Math.floor(n / 6) + 3 === n % 6 ? 1 : 0);
    }
    view.setUint32(jacobianArray, jacobian, true);
    const returned = instance.exports[name](object, parameters, residual, withJacobian ? jacobianArray : 0);
    const actual = Array.from({length: rows}, (_, i) => view.getFloat64(residual + i * 8, true));
    const actualJacobian = withJacobian ? Array.from({length: rows * 6}, (_, i) => view.getFloat64(jacobian + i * 8, true)) : null;
    const pass = returned === 1 && actual.every((v, i) => Math.abs(v - expected[i]) < 1e-12) && (!withJacobian || actualJacobian.every((v, i) => v === expectedJacobian[i]));
    results.push({name, withJacobian, returned, expected, actual, expectedJacobian: withJacobian ? expectedJacobian : null, actualJacobian, pass});
  }
}
const result = {mutation: {section_id: 7, operation: 'append function exports', exports}, node: process.version, original_sha256: hash(original), analytical_copy_sha256: hash(patched), original_code_section_sha256: codeBefore, analytical_code_section_sha256: codeAfter, code_bytes_unchanged: codeBefore === codeAfter, script_sha256: hash(fs.readFileSync(new URL(import.meta.url))), import_stubs: WebAssembly.Module.imports(compiled).length, invokedImports, results, pass: results.every(r => r.pass) && !invokedImports.length, limits: ['Only export section augmented in isolated analytical copy; SDK original untouched', 'Constructed object memory and imported function traps bypass engine initialization', 'These six residual/Jacobian component calls do not prove their dispatch in camera tracking or accuracy']};
fs.writeFileSync(new URL('result.json', out), JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result, null, 2));
if (!result.pass) process.exitCode = 1;
