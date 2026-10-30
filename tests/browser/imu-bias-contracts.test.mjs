import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/imu.js', import.meta.url), 'utf8');
function fixture({ generic = false } = {}) {
    let now = 1000;
    const listeners = new Map(), pendingTimers = [];
    class Sensor {
        constructor() { this.handlers = {}; }
        addEventListener(name, handler) { this.handlers[name] = handler; }
        start() {} stop() {}
        read(timestamp, values) { this.timestamp = timestamp; [this.x, this.y, this.z] = values; this.handlers.reading(); }
    }
    const context = vm.createContext({ Float64Array,
        console: { log() {}, warn() {}, error() {} },
        setTimeout: resolve => pendingTimers.push(resolve),
        navigator: { userAgent: 'Android', platform: '', maxTouchPoints: 0 },
        performance: { now: () => now, timeOrigin: 1700000000000 },
        window: { addEventListener: (name, handler) => listeners.set(name, handler),
            removeEventListener: name => listeners.delete(name) },
        DeviceMotionEvent: class {},
        ...(generic ? { Accelerometer: Sensor, Gyroscope: Sensor } : {}),
    });
    vm.runInContext(source.replace(/export /g, '') + '\nglobalThis.IMU = IMU;', context);
    const imu = new context.IMU(); imu.start();
    const emit = gyro => {
        now += 20;
        if (generic) {
            imu._accel.read(now, [0, 9.81, 0]); imu._gyro.read(now, gyro);
            return;
        }
        listeners.get('devicemotion')({ timeStamp: now, accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 },
            rotationRate: { alpha: gyro[0] * 180 / Math.PI, beta: gyro[1] * 180 / Math.PI, gamma: gyro[2] * 180 / Math.PI } });
    };
    const candidate = async gyro => {
        const promise = imu.calibrate(1500);
        for (let i = 0; i < 75; i++) emit(gyro);
        pendingTimers.shift()();
        return promise;
    };
    return { imu, emit, candidate };
}

test('Constant rotation accepted as a bias candidate cannot fabricate rotation after stopping', async () => {
    const { imu, emit, candidate } = fixture();
    const result = await candidate([0.31, 0, 0]);
    assert.ok(result, 'existing mean acceptance condition permits slow constant rotation');
    emit([0, 0, 0]);
    const sample = imu.flush(); assert.equal(sample.count, 1);
    assert.deepEqual(Array.from(sample.data.slice(4, 7)), [0, 0, 0]);
    assert.deepEqual([imu.latest.gyro_x, imu.latest.gyro_y, imu.latest.gyro_z], [0, 0, 0]);
});

test('Raw nonzero angular velocity retains axes and radians after candidate collection', async () => {
    for (const generic of [false, true]) {
        const { imu, emit, candidate } = fixture({ generic });
        await candidate([-0.30890298, 0.0337092, 0.0410355]);
        const expected = [0.2, -0.1, 0.05]; emit(expected);
        const gyro = Array.from(imu.flush().data.slice(4, 7));
        gyro.forEach((value, axis) => assert.ok(Math.abs(value - expected[axis]) < 1e-12));
    }
});

test('Candidate metadata is explicitly unverified and unapplied; repeated means use raw input', async () => {
    const { imu, candidate } = fixture();
    const first = await candidate([0.31, 0, 0]);
    assert.equal(first.verified, false); assert.equal(first.applied, false);
    assert.equal(imu.isCalibrated(), false);
    const diagnostics = imu.getDiagnostics();
    assert.equal(diagnostics.calibration, 'bias_candidate_unverified');
    assert.equal(diagnostics.gyroBiasApplied, false);
    assert.ok(Math.abs(diagnostics.gyroBiasCandidate.x - 0.31) < 1e-12);
    const second = await candidate([0.1, 0, 0]);
    assert.ok(Math.abs(second.bias.x - 0.1) < 1e-12);
    const copy = imu.getDiagnostics().gyroBiasCandidate; copy.x = 99;
    assert.ok(Math.abs(imu.getGyroBias().x - 0.1) < 1e-12);
    imu.stop();
    assert.equal(imu.getDiagnostics().gyroBiasCandidate, null);
    assert.equal(imu.getDiagnostics().gyroBiasApplied, false);
    assert.equal(imu.isCalibrated(), false);
});
