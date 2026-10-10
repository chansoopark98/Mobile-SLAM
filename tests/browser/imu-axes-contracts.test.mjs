import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/imu.js', import.meta.url), 'utf8');
function fixture({ generic = false, ios = false } = {}) {
    const listeners = new Map();
    class Sensor {
        constructor() { this.handlers = {}; }
        addEventListener(name, handler) { this.handlers[name] = handler; }
        start() {} stop() {}
        read(values) { this.timestamp = 1000; [this.x, this.y, this.z] = values; this.handlers.reading(); }
    }
    const context = vm.createContext({ Float64Array,
        console: { log() {}, warn() {}, error() {} },
        navigator: { userAgent: ios ? 'iPhone' : 'Android', platform: '', maxTouchPoints: 0 },
        performance: { now: () => 2000, timeOrigin: 1700000000000 },
        window: { addEventListener: (name, handler) => listeners.set(name, handler), removeEventListener: name => listeners.delete(name) },
        DeviceMotionEvent: class {}, ...(generic ? { Accelerometer: Sensor, Gyroscope: Sensor } : {}),
    });
    vm.runInContext(source.replace(/export /g, '') + '\nglobalThis.IMU = IMU;', context);
    const imu = new context.IMU(); imu.start();
    return { imu, motion: rotationRate => listeners.get('devicemotion')({ timeStamp: 1000,
        accelerationIncludingGravity: { x: 0, y: ios ? -9.81 : 9.81, z: 0 }, rotationRate }) };
}

// W3C DeviceMotionEventRotationRate: alpha=x, beta=y, gamma=z (degrees/s).
// https://www.w3.org/TR/orientation-event/#devicemotion-rotationrate
// DeviceOrientation's Euler angle order is a separate API contract.
for (const [axis, fields, expected] of [
    ['x', { alpha: 90, beta: 0, gamma: 0 }, [Math.PI / 2, 0, 0]],
    ['y', { alpha: 0, beta: 90, gamma: 0 }, [0, Math.PI / 2, 0]],
    ['z', { alpha: 0, beta: 0, gamma: 90 }, [0, 0, Math.PI / 2]],
]) {
    test(`DeviceMotion device ${axis}-axis 90deg/s reaches only the matching raw gyro axis`, () => {
        for (const ios of [false, true]) {
            const { imu, motion } = fixture({ ios }); motion(fields);
            const { data, count } = imu.flush(); assert.equal(count, 1);
            Array.from(data.slice(4, 7)).forEach((value, index) => assert.ok(Math.abs(value - expected[index]) < 1e-12));
            Array.from(data.slice(1, 4)).forEach((value, index) => assert.ok(Math.abs(value - [0, 9.81, 0][index]) < 1e-12));
        }
    });
}

test('Generic Sensor and DeviceMotion deliver the same physical device angular velocity', () => {
    const expected = [Math.PI / 2, -Math.PI, Math.PI / 4];
    const generic = fixture({ generic: true });
    generic.imu._accel.read([0, 9.81, 0]); generic.imu._gyro.read(expected);
    const genericRow = generic.imu.flush().data;
    for (const ios of [false, true]) {
        const motion = fixture({ ios }); motion.motion({ alpha: 90, beta: -180, gamma: 45 });
        const motionRow = motion.imu.flush().data;
        Array.from(motionRow).forEach((value, index) => assert.ok(Math.abs(value - genericRow[index]) < 1e-12));
    }
});
