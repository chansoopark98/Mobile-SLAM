// The recorded input of the US-010 sample smoke: the first N frames of a scenario of slam_refact/tools/re/runtime/scenarios.json plus its
// synthetic sensor packets, in the form slam_refact/web-chunk/tests/sample-input.js plays them (same rules as run-chunk-ab.mjs loadDataset:
// flip6 fixtures only here). Read-only; reads the dataset directory of the scenario.
import fs from 'node:fs/promises';
import { createHash } from 'node:crypto';
import path from 'node:path';
import { parsePGM } from '../../../scripts/dev/replay-android-input.mjs';
import { buildSensorPackets } from '../../../scripts/dev/references/xr_reference/sensor_adapter.mjs';

const root = path.resolve(import.meta.dirname, '../../..');
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex');

export async function loadFixture(scenarioName = 'flip6-s2', maxFrames = 300) {
  const scenarios = JSON.parse(await fs.readFile(path.join(root, 'slam_refact/tools/re/runtime/scenarios.json'), 'utf8'));
  const scenario = scenarios[scenarioName];
  if (!scenario || scenario.kind !== 'flip6') throw new Error('flip6 scenarios only');
  const dir = path.resolve(root, scenario.dataset);
  const bytes = await fs.readFile(path.join(dir, 'record.json'));
  const record = JSON.parse(bytes);
  if (record.schema !== 'mobile-slam-android-input-v1') throw new Error('unexpected record schema');
  const rec = record.recording;
  const firstFrame = rec.frames[0].timestamp;
  const t0 = Math.max(...rec.imu.filter((r) => r[0] <= firstFrame).map((r) => r[0]));
  if (!Number.isFinite(t0)) throw new Error('no IMU sample precedes the first frame');
  const lastFrame = rec.frames.at(-1).timestamp;
  const all = rec.frames.map((f, i) => ({ index: i, t: +(f.timestamp - t0).toFixed(9), file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256 }));
  const frames = maxFrames > 0 ? all.slice(0, maxFrames) : all;
  const imuAll = rec.imu.filter((r) => r[0] >= t0 && r[0] <= lastFrame + 0.1).map((r) => [+(r[0] - t0).toFixed(9), ...r.slice(1)]);
  const imu = imuAll.filter((r) => r[0] <= frames.at(-1).t + 0.05);
  const sensors = buildSensorPackets(imu);
  const dts = imu.slice(1).map((r, i) => r[0] - imu[i][0]).sort((a, b) => a - b);
  const intervalMs = +(dts[Math.floor(dts.length / 2)] * 1000).toFixed(3);
  const frameDts = frames.slice(1).map((f, i) => f.t - frames[i].t).sort((a, b) => a - b);
  const nominalFps = Math.round(1 / frameDts[Math.floor(frameDts.length / 2)]);
  const fixture = { config: { width: rec.width, height: rec.height }, nominalFps, scale: scenario.scale || 'responsive',
    frames: frames.map((f) => ({ index: f.index, physicalTimestamp: f.t })),
    sensorPackets: sensors.packets.map((p) => ({ ...p, intervalMs })) };
  const fixtureBytes = Buffer.from(JSON.stringify(fixture));
  return { scenario: scenarioName, scenarioInfo: scenario, dir, width: rec.width, height: rec.height, frames, imuRows: imu.length, fixture, fixtureBytes,
    fixtureSha256: sha(fixtureBytes), sourceSha256: sha(bytes), ua: scenario.ua || 'pixel7', nominalFps, intervalMs,
    async gray(index) {
      const f = frames[index];
      if (!f) throw new Error('unknown frame');
      const out = parsePGM(await fs.readFile(f.file), rec.width, rec.height);
      if (sha(out) !== f.sha256) throw new Error(`frame ${index} sha256 differs from the record`);
      return out;
    } };
}

export const UA = {
  pixel7: 'Mozilla/5.0 (Linux; Android 13; Pixel 7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
  reduced: 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
};
