// The record of one gate invocation that lives OUTSIDE the output directory (OR-1 v2 post-run review F-9): the command, the names of the
// environment variables that steer the run (values only as sha256), every HELD check line, the marker, and the exit status. The file is created
// exclusively (a log is never reused) and made read-only when the invocation ends. Nothing here is a verdict.
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';

// environment names that can change a gate run; a value is never written, only its sha256
export const STEERING_ENV = /^(SLAM_REFACT_|SR_|OR1_|RM4C_|US010_|NODE_|BROWSER_|PLAYWRIGHT_|PRECHECK_|MOBILE_SLAM_)/;

export const envRecord = (env = process.env, extraNames = []) => {
  const names = Object.keys(env).filter((k) => STEERING_ENV.test(k) || extraNames.includes(k)).sort();
  return Object.fromEntries(names.map((k) => [k, { set: env[k] !== undefined && env[k] !== '', valueSha256: createHash('sha256').update(String(env[k] ?? '')).digest('hex') }]));
};

export class RunLog {
  constructor(file, { out = null } = {}) {
    this.file = path.resolve(file);
    if (out) {
      const rel = path.relative(path.resolve(out), this.file);
      if (!rel.startsWith('..') && !path.isAbsolute(rel)) throw new Error(`the run log ${file} must be outside the output directory ${out}`);
    }
    fs.mkdirSync(path.dirname(this.file), { recursive: true });
    fs.writeFileSync(this.file, '', { flag: 'wx', mode: 0o644 });  // exclusive: an existing log is refused
    this.closed = false;
  }

  write(event, data = {}) {
    if (this.closed) throw new Error('run log is closed');
    fs.appendFileSync(this.file, `${JSON.stringify({ at: new Date().toISOString(), event, ...data })}\n`);
  }

  close(status) {
    if (this.closed) return;
    this.write('end', { status });
    this.closed = true;
    fs.chmodSync(this.file, 0o444);
  }
}
