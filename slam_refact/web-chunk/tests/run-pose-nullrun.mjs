// N-pose null-run of the US-010 sample smoke (preregistration us010-samples.v2.md section 2): the pose comparison of S7-S9
// (samples/common/probe.js poseErrors) applied to camera objects that were set from known poses with exactly the calls XR8 uses
// (three.js module `m`: projectionMatrix.elements[i] = K[i], setRotationFromQuaternion(q), position.set; A-Frame xraframe: object3D.position.set,
// setRotationFromQuaternion, projection elements copy), for 10,000 seeded random poses, on three.js r178 (the three.module.min.js the sample
// uses) and on A-Frame's own THREE. The largest error is the measurement resolution R; the registered tolerance is 1e-6 and must be >= 3 R.
// A sensitivity part perturbs the object by known amounts to show the comparison reports them (1e-6 position, 1e-6 rad, 1e-6 projection).
//   node slam_refact/web-chunk/tests/run-pose-nullrun.mjs --label <label> [--evidence-root <dir under build/>]      -> <evidence root>/nullrun-pose/<label>.json (default: the v3 root build/.../us010-20261020)
import fs from 'node:fs/promises';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';
import { startSampleServer } from '../samples/serve.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values } = parseArgs({ options: { label: { type: 'string' }, 'evidence-root': { type: 'string', default: 'build/slam-refact-full-20261019/chunk/us010-20261020' } } });
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required');
const evidence = path.resolve(repo, values['evidence-root']);
if (!evidence.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--evidence-root must be under build/');
const outDir = path.join(evidence, 'nullrun-pose');
await fs.mkdir(outDir, { recursive: true });
const outFile = path.join(outDir, `${values.label}.json`);
await fs.writeFile(outFile, '', { flag: 'wx' });

const page_html = `<!doctype html><html><head><meta charset="utf-8"><script src="/aframe/aframe-v1.5.0.min.js"></script><script src="/samples/common/probe.js"></script></head>
<body><script type="module">
import * as THREE3 from '/three/three.module.min.js';
const probe = window.XrSampleProbe;
const rng = (a) => () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
function run(T, name) {
  const r = rng(20261020);
  const cam = new T.PerspectiveCamera();
  const parent = new T.Object3D(); const obj = new T.Object3D(); parent.add(obj);
  let maxPos = 0, maxRot = 0, maxProj = 0, n = 10000;
  const sens = { pos: [], rot: [], proj: [] };
  for (let i = 0; i < n; i++) {
    const pos = { x: (r() - .5) * 20, y: (r() - .5) * 20, z: (r() - .5) * 20 };
    let q = [r() - .5, r() - .5, r() - .5, r() - .5]; const l = Math.hypot(...q); q = q.map((v) => v / l);
    const rot = { w: q[0], x: q[1], y: q[2], z: q[3] };
    const K = Array.from({ length: 16 }, () => (r() - .5) * 6);
    // what the three.js module does with the camera, and what xraframe does with the entity and the camera
    for (let g = 0; g < 16; g++) cam.projectionMatrix.elements[g] = K[g];
    cam.setRotationFromQuaternion(rot); cam.position.set(pos.x, pos.y, pos.z);
    obj.setRotationFromQuaternion(rot); obj.position.set(pos.x, pos.y, pos.z);
    for (const [pose, proj] of [[cam, cam], [obj, cam]]) {
      const e = probe.poseErrors(pose, proj, pos, rot, K);
      maxPos = Math.max(maxPos, e.posErr); maxRot = Math.max(maxRot, e.rotErr); maxProj = Math.max(maxProj, e.projErr);
    }
    if (i < 200) {
      // sensitivity: known perturbations must be reported (to within the double resolution of the sum)
      const p0 = cam.position.x; cam.position.x = p0 + 1e-6; sens.pos.push(probe.poseErrors(cam, cam, pos, rot, K).posErr); cam.position.x = p0;
      const q0 = cam.quaternion.clone(); const d = 1e-6 / 2; const qa = { x: q0.x, y: q0.y, z: q0.z, w: q0.w };
      const c = Math.cos(d), s = Math.sin(d);  // rotate by 1e-6 rad about the local x axis: q * (c, s, 0, 0)
      cam.quaternion.set(q0.w * s + q0.x * c, q0.y * c + q0.z * s, q0.z * c - q0.y * s, q0.w * c - q0.x * s);
      sens.rot.push(probe.poseErrors(cam, cam, pos, rot, K).rotErr); cam.quaternion.set(qa.x, qa.y, qa.z, qa.w);
      const k0 = cam.projectionMatrix.elements[5]; cam.projectionMatrix.elements[5] = k0 + 1e-6; sens.proj.push(probe.poseErrors(cam, cam, pos, rot, K).projErr); cam.projectionMatrix.elements[5] = k0;
    }
  }
  const rev = T.REVISION;
  const mm = (a) => ({ min: Math.min(...a), max: Math.max(...a) });
  return { name, revision: rev, poses: n, maxPosErr: maxPos, maxRotErr: maxRot, maxProjErr: maxProj, sensitivity: { pos1e6: mm(sens.pos), rot1e6: mm(sens.rot), proj1e6: mm(sens.proj) } };
}
window.__nullrun = [run(THREE3, 'three.module.min.js (sample three.js)'), run(AFRAME.THREE, 'AFRAME.THREE (' + AFRAME.version + ')')];
window.__nullrunDone = true;
</script></body></html>`;

const srv = await startSampleServer({ extra: async (p) => p === '/__nullrun/index.html' ? { body: Buffer.from(page_html), type: 'text/html' } : null });
let browser, result;
try {
  browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args: ['--enable-unsafe-swiftshader'] });
  const page = await browser.newPage();
  const errors = [];
  page.on('pageerror', (e) => errors.push(e.message));
  await page.route('**/*', (route) => (route.request().url().startsWith(srv.origin + '/') ? route.continue() : route.abort('blockedbyclient')));
  await page.goto(srv.origin + '/__nullrun/index.html?probe=1');
  await page.waitForFunction(() => window.__nullrunDone === true, null, { timeout: 60000 });
  const runs = await page.evaluate(() => window.__nullrun);
  const R = Math.max(...runs.flatMap((r) => [r.maxPosErr, r.maxRotErr, r.maxProjErr]));
  result = { schema: 'us010-pose-nullrun-v1', label: values.label, at: new Date().toISOString(), browser: browser.version(), runs, pageErrors: errors,
    resolutionR: R, tolerance: 1e-6, threeTimesRule: 3 * R <= 1e-6, tolerance_over_R: R > 0 ? 1e-6 / R : null,
    note: 'R is the largest error of the comparison on objects set from known poses; the floor of the IEEE double representation of the compared values is ~2.2e-16 relative (about 4e-15 at |value| 20), so R = 0 is reported as the representation floor, not as zero' };
} finally {
  if (browser) await browser.close();
  await srv.close();
}
await fs.writeFile(outFile, JSON.stringify(result, null, 1) + '\n');
console.log(JSON.stringify(result, null, 1));
process.exitCode = result.threeTimesRule && result.pageErrors.length === 0 ? 0 : 1;
