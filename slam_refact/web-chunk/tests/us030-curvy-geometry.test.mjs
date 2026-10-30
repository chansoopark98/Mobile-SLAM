// node --test slam_refact/web-chunk/tests/us030-curvy-geometry.test.mjs
// US-030 (full-parity PRD): the capnp CurvyGeometry of the engine (nine fields) -> the five fields of the original JS imagefound /
// imageupdated / imagelost / imagescanning payloads (height, radiusTop, radiusBottom, arcLengthRadians, arcStartRadians).
// Spec docs/spec/07-xr-slam-chunk.md section 11.
//   SLAM_REFACT_CHUNK_DIR=<dir with the chunk files> (default slam_refact/web-chunk; tests/run-us030-mutants.mjs points it at modified
//   copies under build/ to show that these tests fail on them)
// The expected mapping is not typed into this file: it is read from the original sources (the public tracking-controller.ts and the
// deployed xr-slam.js), and the field names of the engine side from reality.capnp and wasm/bindings.cc.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const PUB = path.join(repo, 'assets/references/xr_reference/official/xr_reference/reality');
const read = (...p) => fs.readFileSync(path.join(...p), 'utf8');
const { createImageTargets, curvyGeometryToJs } = await import(pathToFileURL(path.join(chunkDir, 'image-targets.js')));
const { dispatchImageEvents } = await import(pathToFileURL(path.join(chunkDir, 'image-events.js')));

// ---- the original, as text ----
const lcFirst = (s) => s[0].toLowerCase() + s.slice(1);
// [[jsKey, capnpField], ...] in source order from `jsKey: <var>.getXxx(),` lines
const pairsFrom = (text) => [...text.matchAll(/(\w+)\s*:\s*\w+\.get(\w+)\(\)/g)].map((m) => [m[1], lcFirst(m[2])]);
const tsText = read(PUB, 'app/xr/js/src/tracking-controller.ts');
const tsBlock = tsText.slice(tsText.indexOf('itMetadata.geometry = {\n          height:'), tsText.indexOf('} else {', tsText.indexOf('itMetadata.geometry = {\n          height:')));
const sdkText = read(repo, 'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js');
const sdkAt = sdkText.indexOf('A.geometry={height:');
const sdkBlock = sdkText.slice(sdkAt, sdkText.indexOf('}', sdkAt));
const ORIGINAL_PAIRS = pairsFrom(tsBlock);

const capnpText = read(PUB, 'engine/api/reality.capnp');
const capnpStruct = capnpText.slice(capnpText.indexOf('struct CurvyGeometry {'), capnpText.indexOf('struct ImageTargetMetadata'));
const CAPNP_FIELDS = [...capnpStruct.matchAll(/^\s*(\w+) @(\d+): Float32;/gm)].sort((a, b) => a[2] - b[2]).map((m) => m[1]);
const bindingsText = read(repo, 'slam_refact/engine/wasm/bindings.cc');
const bindingKeys = [...bindingsText.slice(bindingsText.indexOf('const CurvyGeometryOut& c = d.curvy_geometry;')).matchAll(/g\.set\("(\w+)"/g)].map((m) => m[1]).slice(0, 9);

test('US030-S1: the mapping read from the public source and from the deployed chunk is the same five pairs', () => {
  assert.equal(ORIGINAL_PAIRS.length, 5, tsBlock);
  assert.deepEqual(pairsFrom(sdkBlock), ORIGINAL_PAIRS, 'tracking-controller.ts and the deployed xr-slam.js must agree');
  assert.deepEqual(ORIGINAL_PAIRS, [['height', 'height'], ['radiusTop', 'topRadius'], ['radiusBottom', 'bottomRadius'],
    ['arcLengthRadians', 'arcLengthRadians'], ['arcStartRadians', 'arcStartRadians']], 'the pairs, spelled out once more for the reader');
});

test('US030-S2: the engine side field names are the nine capnp fields, and the five sources are among them', () => {
  assert.equal(CAPNP_FIELDS.length, 9);
  assert.deepEqual(bindingKeys.slice().sort(), CAPNP_FIELDS.slice().sort(), 'wasm/bindings.cc curvyGeometry keys = reality.capnp CurvyGeometry fields');
  for (const [, field] of ORIGINAL_PAIRS) assert.ok(CAPNP_FIELDS.includes(field), `${field} is a capnp CurvyGeometry field`);
  const dropped = CAPNP_FIELDS.filter((f) => !ORIGINAL_PAIRS.some(([, field]) => field === f));
  assert.deepEqual(dropped, ['curvyCircumferenceTop', 'curvyCircumferenceBottom', 'curvySideLength', 'targetCircumferenceTop'],
    'the four fields the client provides are not forwarded by the original');
});

// distinct values per capnp field: a swapped or misnamed field cannot give the right number by accident
const probe = () => Object.fromEntries(CAPNP_FIELDS.map((f, i) => [f, 100 + 7 * i + 0.25]));

test('US030-M1: curvyGeometryToJs applies exactly the original mapping (names, values, order, nothing else)', () => {
  const g = probe();
  const out = curvyGeometryToJs(g);
  assert.deepEqual(Object.keys(out), ORIGINAL_PAIRS.map(([js]) => js), 'the five keys, in the original order');
  for (const [js, field] of ORIGINAL_PAIRS) assert.equal(out[js], g[field], `${js} <- ${field}`);
  for (const f of ['topRadius', 'bottomRadius', 'curvyCircumferenceTop', 'curvyCircumferenceBottom', 'curvySideLength', 'targetCircumferenceTop']) {
    assert.ok(!(f in out), `${f} is not a key of the JS geometry`);
  }
  assert.notEqual(out.radiusTop, out.radiusBottom, 'the probe separates top and bottom');
  // the input is not changed
  assert.deepEqual(g, probe());
});

// ---- the chunk wiring (image-targets.js) with a fake engine glue ----
const W = 120, H = 160;
const props = () => ({ left: 0, top: 0, width: W, height: H, isRotated: false, originalWidth: W, originalHeight: H,
  cylinderCircumferenceTop: 100, cylinderCircumferenceBottom: 100, cylinderSideLength: 50, targetCircumferenceTop: 50 });
const target = (name, type) => ({ type, name, imagePath: `image-targets/${name}.png`, metadata: JSON.stringify({ tag: name }), userMetadataIsJson: true,
  properties: props() });
const engineImage = (name, type, x, curvyGeometry) => ({ name, id: -1, type, position: { x, y: 0.5, z: -1 }, rotation: { x: 0, y: 0, z: 0, w: 1 },
  widthInMeters: 0.3, heightInMeters: 0.4, scale: 0.4, trackingStatus: 0, ...(curvyGeometry ? { curvyGeometry } : {}) });
const CYL = { ...probe(), topRadius: 0.5, bottomRadius: 0.5, height: 1.5, arcLengthRadians: 0.7, arcStartRadians: -0.35 };
const CONE = { ...probe(), topRadius: 0.8, bottomRadius: 0.4, height: 1.25, arcLengthRadians: 1.1, arcStartRadians: 0.2 };

async function session(targets) {
  const events = [];
  const framework = { dispatchEvent: (n, d) => events.push([n, d]) };
  const diag = {};
  const glueCalls = [];
  const glue = { setImageTargets: (list) => glueCalls.push(['setImageTargets', list.length]), unloadImageTargets: () => {} };
  const it = createImageTargets({
    options: { loadLuminance: async () => ({ width: W, height: H, luminance: new Uint8Array(W * H) }), nextFrame: async () => {} },
    diag, pushEntry: (arr, v) => arr.push(v), getComputeCtx: () => null, getGlue: () => glue,
  });
  diag.warnings = [];
  it.configure(targets, { cameraProcessingDisabled: false, appResourcesLoaded: true, framework });
  await diag.imageTargets.whenIdle();
  return { it, events, glueCalls };
}

test('US030-W1: a curved target gets no flat geometry; the answers carry the five curved fields; flat targets are unchanged', async () => {
  const s = await session([target('tFlat', 'PLANAR'), target('tCyl', 'CYLINDER'), target('tCone', 'CONICAL')]);
  const scanning = s.events.find(([n]) => n === 'imagescanning')[1].imageTargets;
  assert.deepEqual(scanning.map((t) => [t.name, t.type]), [['tFlat', 'FLAT'], ['tCyl', 'CYLINDRICAL'], ['tCone', 'CONICAL']]);
  assert.deepEqual(scanning[0].geometry, { scaledWidth: W / H, scaledHeight: 1 }, 'flat: scaledWidth / scaledHeight as before');
  assert.equal('geometry' in scanning[1], false, 'curved, not detected yet: no geometry (the engine reports it with a detection)');
  assert.equal('geometry' in scanning[2], false);

  const images = s.it.build([engineImage('tFlat', 'FLAT', 0), engineImage('tCyl', 'CYLINDRICAL', 1, CYL), engineImage('tCone', 'CONICAL', 2, CONE)]);
  assert.equal(images.length, 3);
  const base = ['name', 'metadata', 'position', 'rotation', 'scale', 'type', 'properties'];
  assert.deepEqual(Object.keys(images[0]), [...base, 'scaledWidth', 'scaledHeight']);
  const curvedKeys = [...base, 'height', 'radiusTop', 'radiusBottom', 'arcLengthRadians', 'arcStartRadians'];
  assert.deepEqual(Object.keys(images[1]), curvedKeys);
  assert.deepEqual(Object.keys(images[2]), curvedKeys);
  assert.deepEqual([images[1].type, images[2].type], ['CYLINDRICAL', 'CONICAL']);
  for (const [img, g] of [[images[1], CYL], [images[2], CONE]]) {
    assert.deepEqual([img.height, img.radiusTop, img.radiusBottom, img.arcLengthRadians, img.arcStartRadians],
      [g.height, g.topRadius, g.bottomRadius, g.arcLengthRadians, g.arcStartRadians], img.name);
  }
  assert.ok(images[2].radiusTop > images[2].radiusBottom, 'conical: top larger than bottom, the right way round');
  // the geometry the first detection revealed is the target's geometry from then on (the original sets it once, at load)
  assert.deepEqual(scanning[1].geometry, curvyGeometryToJs(CYL));
  const again = s.it.build([engineImage('tCyl', 'CYLINDRICAL', 5, { ...CYL, height: 99 })]);
  assert.equal(again[0].height, CYL.height, 'a later report does not change the stored geometry');
});

test('US030-W2: the imagefound / imageupdated / imagelost diff over curved targets: found once, no update from the geometry, lost once', async () => {
  const s = await session([target('tCyl', 'CYLINDER')]);
  const map = {}; const seen = [];
  const frame = (x) => dispatchImageEvents(map, s.it.build(x === null ? [] : [engineImage('tCyl', 'CYLINDRICAL', x, CYL)]), (n, d) => seen.push([n, d]));
  frame(1); frame(1); frame(1); frame(2); frame(null); frame(null);
  assert.deepEqual(seen.map(([n]) => n), ['imagefound', 'imageupdated', 'imagelost']);
  assert.equal(seen[0][1].radiusTop, CYL.topRadius);
  assert.equal(seen[0][1].arcStartRadians, CYL.arcStartRadians);
  assert.equal(seen[1][1].position.x, 2, 'the update is the position change');
  assert.equal(seen[2][1].radiusBottom, CYL.bottomRadius, 'imagelost carries the last found payload (the original copies it)');
  assert.equal(seen[2][1].height, CYL.height);
});

test('US030-W3: a curved target whose engine detection has no curvyGeometry gets no invented geometry', async () => {
  const s = await session([target('tCyl', 'CYLINDER')]);
  const [img] = s.it.build([engineImage('tCyl', 'CYLINDRICAL', 0)]);
  assert.deepEqual(Object.keys(img), ['name', 'metadata', 'position', 'rotation', 'scale', 'type', 'properties']);
});
