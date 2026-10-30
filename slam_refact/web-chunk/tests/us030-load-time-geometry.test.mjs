// node --test slam_refact/web-chunk/tests/us030-load-time-geometry.test.mjs
// US-030 / D-1: with an engine that has getImageTargetGeometry (docs/design/request-engine-image-target-geometry.md) the five JS
// fields are set at load time, in the original's order (load -> geometry -> imagescanning), not at the first detection.
// A fake engine glue stands in for the engine; us030-curvy-geometry.test.mjs covers the engines without the call.
import assert from 'node:assert/strict';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const { createImageTargets, curvyGeometryToJs } = await import(pathToFileURL(path.join(chunkDir, 'image-targets.js')));

const W = 120, H = 160;
const CYL = { curvyCircumferenceTop: 1, curvyCircumferenceBottom: 2, curvySideLength: 3, targetCircumferenceTop: 4,
  height: 1.5, topRadius: 0.5, bottomRadius: 0.45, arcLengthRadians: 0.7, arcStartRadians: -0.35 };
const props = () => ({ left: 0, top: 0, width: W, height: H, isRotated: false, originalWidth: W, originalHeight: H,
  cylinderCircumferenceTop: 100, cylinderCircumferenceBottom: 100, cylinderSideLength: 50, targetCircumferenceTop: 50 });
const target = (name, type) => ({ type, name, imagePath: `image-targets/${name}.png`, metadata: JSON.stringify({ tag: name }),
  userMetadataIsJson: true, properties: props() });

async function session(targets, glueExtra) {
  const events = [];
  const framework = { dispatchEvent: (n, d) => events.push([n, d]) };
  const diag = { warnings: [] };
  const asked = [];
  const glue = { setImageTargets: () => {}, unloadImageTargets: () => {}, ...glueExtra(asked) };
  const it = createImageTargets({
    options: { loadLuminance: async () => ({ width: W, height: H, luminance: new Uint8Array(W * H) }), nextFrame: async () => {} },
    diag, pushEntry: (arr, v) => arr.push(v), getComputeCtx: () => null, getGlue: () => glue,
  });
  it.configure(targets, { cameraProcessingDisabled: false, appResourcesLoaded: true, framework });
  await diag.imageTargets.whenIdle();
  return { it, events, asked };
}

test('US030-L1: imagescanning already carries the five curved fields; flat targets keep scaledWidth/scaledHeight; the engine is asked by name', async () => {
  const s = await session([target('tFlat', 'PLANAR'), target('tCyl', 'CYLINDER')],
    (asked) => ({ getImageTargetGeometry: (n) => { asked.push(n); return n === 'tCyl' ? CYL : undefined; } }));
  const scanning = s.events.find(([n]) => n === 'imagescanning')[1].imageTargets;
  assert.deepEqual(scanning[1].geometry, curvyGeometryToJs(CYL));
  assert.deepEqual(Object.keys(scanning[1].geometry), ['height', 'radiusTop', 'radiusBottom', 'arcLengthRadians', 'arcStartRadians']);
  assert.deepEqual(scanning[0].geometry, { scaledWidth: W / H, scaledHeight: 1 });
  assert.deepEqual(s.asked, ['tCyl'], 'only the curved target is asked');
  const [img] = s.it.build([{ name: 'tCyl', type: 'CYLINDRICAL', position: { x: 0, y: 0, z: -1 }, rotation: { x: 0, y: 0, z: 0, w: 1 }, scale: 1 }]);
  assert.equal(img.radiusTop, CYL.topRadius, 'a detection without curvyGeometry still carries the load-time fields');
  assert.equal(img.radiusBottom, CYL.bottomRadius);
});

test('US030-L2: an engine that answers undefined (or has no such call) leaves the geometry to the first detection', async () => {
  for (const extra of [() => ({ getImageTargetGeometry: () => undefined }), () => ({})]) {
    const s = await session([target('tCyl', 'CYLINDER')], extra);
    const scanning = s.events.find(([n]) => n === 'imagescanning')[1].imageTargets;
    assert.equal('geometry' in scanning[0], false, 'nothing invented');
  }
});
