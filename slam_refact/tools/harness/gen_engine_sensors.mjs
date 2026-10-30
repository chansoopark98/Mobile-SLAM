#!/usr/bin/env node
// Engine-input sensor stream for a TUM fixture without an original reference run.
//
// Writes reference/runs/<scenario>/engine-input-sensors.jsonl.gz with exactly the record format and
// adapter that run-reference.mjs uses (buildSensorPackets on the fixture IMU rows, rows filtered to
// t <= last frame + 0.05 s), so prepare_inputs.py can build replay bundles for held-out sequences.
//
//   node slam_refact/tools/harness/gen_engine_sensors.mjs --scenario tum-room2-full
import {createHash} from 'node:crypto';
import fs from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import zlib from 'node:zlib';

import {buildSensorPackets} from '../../../scripts/dev/references/xr_reference/sensor_adapter.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
const {values} = parseArgs({options: {scenario: {type: 'string'}}});
if (!values.scenario) throw new Error('--scenario required');
const scenarios = JSON.parse(await fs.readFile(path.join(root, 'slam_refact/tools/re/runtime/scenarios.json')));
const scenario = scenarios[values.scenario];
if (!scenario || scenario.kind !== 'tum') throw new Error('TUM scenario required');
const dir = path.resolve(root, scenario.dataset);
const input = JSON.parse(await fs.readFile(path.join(dir, 'input.json')));
if (input.schema !== 'mobile-slam-synthetic-vio-v1') throw new Error('unexpected fixture schema');
const lastT = input.frames.at(-1).physicalTimestamp;
const imu = input.imu.filter(r => r[0] <= lastT + 0.05);
const sensors = buildSensorPackets(imu);
const lines = sensors.packets.map(p => JSON.stringify({t: p.sourceTimestamp, accelIncludingGravity: p.deviceAcceleration,
  rotationRateDegS: [p.rotationRate.alpha, p.rotationRate.beta, p.rotationRate.gamma],
  orientation: [p.orientation.alpha, p.orientation.beta, p.orientation.gamma],
  qEarthFromDevice: p.quaternionEarthFromDevice})).join('\n') + '\n';
const outDir = path.join(root, 'build/slam-refact-20261015/reference/runs', values.scenario);
await fs.mkdir(outDir, {recursive: true});
const file = path.join(outDir, 'engine-input-sensors.jsonl.gz');
await fs.writeFile(file, zlib.gzipSync(lines, {level: 9, mtime: 0}), {flag: 'wx'});
const meta = {schema: 'slam-refact-engine-sensors-v1', scenario: values.scenario, dataset: scenario.dataset,
  rows: sensors.packets.length, sha256Uncompressed: createHash('sha256').update(lines).digest('hex'),
  provenance: sensors.provenance, generator: 'slam_refact/tools/harness/gen_engine_sensors.mjs',
  note: 'no original reference run exists for this scenario; engine-only evaluation'};
await fs.writeFile(path.join(outDir, 'engine-input-sensors.meta.json'), JSON.stringify(meta, null, 1) + '\n', {flag: 'wx'});
console.log(JSON.stringify({file: path.relative(root, file), rows: meta.rows}));
