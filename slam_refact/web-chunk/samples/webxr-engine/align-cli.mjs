// Offline: engine vs ARCore reference (NOT ground truth) from a JSON exported by the webxr-engine page.
//   node slam_refact/web-chunk/samples/webxr-engine/align-cli.mjs <webxr-engine-*.json> [--status NORMAL,LIMITED] [--rpe 1]
// Prints the compareTrajectories() report as JSON (SE(3) primary, Sim(3) scale diagnostic).
import fs from 'node:fs';
import { parseArgs } from 'node:util';
import { compareTrajectories, pairsFromRecords } from './trajectory-align.js';

const { values, positionals } = parseArgs({ allowPositionals: true, options: { status: { type: 'string', default: 'NORMAL' }, rpe: { type: 'string', default: '1' } } });
if (positionals.length !== 1) { console.error('usage: align-cli.mjs <export.json> [--status NORMAL,LIMITED] [--rpe 1]'); process.exit(2); }
const data = JSON.parse(fs.readFileSync(positionals[0], 'utf8'));
const records = data.phaseB && data.phaseB.log ? data.phaseB.log.records : [];
const pairs = pairsFromRecords(records, { statuses: values.status.split(',') });
const report = compareTrajectories(pairs, { refFrames: records.filter((r) => r.ref).length, rpeDelta: Number(values.rpe) });
console.log(JSON.stringify({ source: positionals[0], statuses: values.status.split(','), config: data.phaseB && data.phaseB.config, report }, null, 1));
