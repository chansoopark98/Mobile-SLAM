import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/imu.js', import.meta.url), 'utf8');
function fixture({ generic = true, motion = true } = {}) {
    let now = 2000;
    const listeners = new Map();
    class Sensor {
        constructor() { this.handlers = {}; this.stopped = false; }
        addEventListener(name, handler) { this.handlers[name] = handler; }
        start() {}
        stop() { this.stopped = true; }
        read(timestamp, values = [0, 9.81, 0]) {
            this.timestamp = timestamp; [this.x, this.y, this.z] = values;
            this.handlers.reading();
        }
        fail() { this.handlers.error({ error: new Error('sensor unavailable') }); }
    }
    const context = vm.createContext({ Float64Array, setTimeout,
        console: { log() {}, warn() {}, error() {} },
        navigator: { userAgent: 'Android', platform: '', maxTouchPoints: 0 },
        performance: { now: () => now, timeOrigin: 1700000000000 },
        window: { addEventListener: (name, handler) => listeners.set(name, handler),
            removeEventListener: name => listeners.delete(name) },
        ...(generic ? { Accelerometer: Sensor, Gyroscope: Sensor } : {}),
        ...(motion ? { DeviceMotionEvent: class {} } : {}),
    });
    vm.runInContext(source.replace(/export /g, '') + '\nglobalThis.IMU = IMU;', context);
    return { imu: new context.IMU(), listeners, setNow: value => { now = value; } };
}
const motionSample = timeStamp => ({ timeStamp, accelerationIncludingGravity: { x: 1, y: 2, z: 3 },
    rotationRate: { beta: 90, gamma: 180, alpha: -90 } });

test('Future Generic Sensor clock cannot seed missing-past input and falls back to valid motion samples', () => {
    const { imu, listeners } = fixture(); imu.start();
    const accel = imu._accel, gyro = imu._gyro;
    // Every reading would be newer than a camera frame at 2s, even though delivered now.
    accel.read(62000); gyro.read(62000);
    assert.equal(imu.flush().count, 0, 'future samples never enter the transferable engine ring');
    assert.equal(imu.getSensorType(), 'device_motion');
    assert.equal(accel.stopped, true); assert.equal(gyro.stopped, true);
    const diagnostics = imu.getDiagnostics();
    assert.equal(diagnostics.clockFailure.reason, 'source_timestamp_in_future');
    assert.equal(diagnostics.clockFailure.timestampMs, 62000);
    assert.equal(diagnostics.clockFailure.arrivalMs, 2000);
    assert.equal(diagnostics.clockFailure.aheadMs, 60000);
    assert.equal(diagnostics.sourceSwitches, 1);
    assert.equal(diagnostics.status, 'fallback_motion');
    listeners.get('devicemotion')(motionSample(1990));
    const { data, count } = imu.flush(); assert.equal(count, 1);
    assert.equal(data[0], 1.99); assert.deepEqual(Array.from(data.slice(1, 4)), [1, 2, 3]);
    // W3C rotationRate alpha=x, beta=y, gamma=z; unlike orientation Euler names.
    assert.ok(Math.abs(data[4] + Math.PI / 2) < 1e-12);
    assert.ok(Math.abs(data[5] - Math.PI / 2) < 1e-12);
    assert.ok(Math.abs(data[6] - Math.PI) < 1e-12);
});

test('Delayed source timestamps are preserved and 1ms clock quantization is allowed', () => {
    const { imu } = fixture(); imu.start();
    imu._accel.read(1000); imu._gyro.read(1000);
    imu._accel.read(2001); imu._gyro.read(2001);
    const { data, count } = imu.flush(); assert.equal(count, 2);
    assert.deepEqual([data[0], data[7]], [1, 2.001]);
    assert.equal(imu.getSensorType(), 'generic_sensor');
});

test('Motion clock rejects future relative and epoch timestamps without inventing replacements', () => {
    const { imu, listeners } = fixture({ generic: false }); imu.start();
    const read = listeners.get('devicemotion');
    read(motionSample(2002)); read(motionSample(1700000002002));
    assert.equal(imu.flush().count, 0);
    assert.equal(imu.getDiagnostics().drops.timestamp, 2);
    assert.equal(imu.getDiagnostics().clockFailure.timestampMs, 1700000002002);
    read(motionSample(1500));
    assert.equal(imu.flush().data[0], 1.5);
});

test('Fallback keeps source ordering and ignores callbacks from stopped sensor generations', () => {
    const { imu, listeners, setNow } = fixture(); imu.start();
    const accel = imu._accel, gyro = imu._gyro;
    accel.read(1900); gyro.read(1900); gyro.fail();
    const oldMotion = listeners.get('devicemotion');
    oldMotion(motionSample(1890)); oldMotion(motionSample(1910));
    assert.doesNotThrow(() => { accel.read(1990); gyro.read(1990); gyro.fail(); });
    assert.equal(imu.getDiagnostics().sourceSwitches, 1);
    const { data, count } = imu.flush(); assert.equal(count, 2);
    assert.deepEqual([data[0], data[7]], [1.9, 1.91]);
    imu.stop(); imu.start(); setNow(3000);
    oldMotion(motionSample(2500)); gyro.fail(); accel.read(2500);
    assert.equal(imu.flush().count, 0);
    assert.equal(imu.getDiagnostics().sourceSwitches, 0);
    assert.equal(imu.getDiagnostics().clockFailure, null);
    imu._accel.read(2990); imu._gyro.read(2990);
    assert.equal(imu.flush().data[0], 2.99);
});

test('Invalid Generic Sensor clock stops cleanly when DeviceMotion is unavailable', () => {
    const { imu } = fixture({ motion: false }); imu.start();
    imu._accel.read(9000);
    assert.equal(imu.running, false);
    assert.equal(imu.getSensorType(), 'none');
    assert.equal(imu.getDiagnostics().status, 'error');
    assert.equal(imu.flush().count, 0);
});
