import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/imu.js', import.meta.url), 'utf8');
function fixture() {
    const listeners = new Map();
    class Sensor {
        constructor() { this.handlers = {}; }
        addEventListener(name, handler) { this.handlers[name] = handler; }
        start() {} stop() {}
        read(timestamp, values) {
            this.timestamp = timestamp; [this.x, this.y, this.z] = values;
            this.handlers.reading();
        }
        fail() { this.handlers.error({ error: new Error('sensor unavailable') }); }
    }
    const context = vm.createContext({ Float64Array, setTimeout,
        console: { log() {}, warn() {}, error() {} },
        navigator: { userAgent: 'Android', platform: '', maxTouchPoints: 0 },
        performance: { now: () => 2000, timeOrigin: 1700000000000 },
        window: { addEventListener: (name, handler) => listeners.set(name, handler),
            removeEventListener: name => listeners.delete(name) },
        Accelerometer: Sensor, Gyroscope: Sensor, DeviceMotionEvent: class {},
    });
    vm.runInContext(source.replace(/export /g, '') + '\nglobalThis.IMU = IMU;', context);
    const imu = new context.IMU(); imu.start(60);
    return { imu, listeners };
}
const gyro = [3, 0, 0];
const acceleration = angle => [0, 9.81 * Math.cos(angle), -9.81 * Math.sin(angle)];
function seed(imu) { imu._accel.read(1000, acceleration(0)); imu._gyro.read(1000, gyro); imu.flush(); }
function values(imu) { return Array.from(imu.flush().data ?? []); }

test('Same sensor samples emit identical IMU regardless of callback order during rotation', () => {
    const outputs = [];
    for (const order of ['gyro-first', 'accel-first']) {
        const { imu } = fixture(); seed(imu);
        const t = 1000 + 1000 / 60;
        if (order === 'gyro-first') {
            imu._gyro.read(t, gyro);
            assert.equal(imu.flush().count, 0, 'previous-period acceleration cannot be relabeled as current');
            imu._accel.read(t, acceleration(3 / 60));
        } else {
            imu._accel.read(t, acceleration(3 / 60)); imu._gyro.read(t, gyro);
        }
        outputs.push(values(imu));
    }
    assert.deepEqual(outputs[0], outputs[1]);
    assert.deepEqual(outputs[0], [1 + 1 / 60, ...acceleration(3 / 60), ...gyro]);
});

test('Delayed acceleration resolves queued gyro timestamps once and in order', () => {
    const { imu } = fixture(); seed(imu);
    imu._gyro.read(1010, [1, 0, 0]); imu._gyro.read(1020, [2, 0, 0]);
    assert.equal(imu.flush().count, 0);
    imu._accel.read(1010, [1, 10, 0]); imu._accel.read(1020, [2, 11, 0]);
    assert.deepEqual(values(imu), [1.01, 1, 10, 0, 1, 0, 0, 1.02, 2, 11, 0, 2, 0, 0]);
    assert.equal(imu.flush().count, 0);
    assert.equal(imu.getDiagnostics().drops.pairing, 0);
});

test('Asynchronous source phases interpolate acceleration at gyro source time in either callback order', () => {
    for (const gyroFirst of [false, true]) {
        const { imu } = fixture();
        imu._accel.read(1000, [0, 10, 20]);
        if (gyroFirst) imu._gyro.read(1010, gyro);
        imu._accel.read(1020, [4, 14, 24]);
        if (!gyroFirst) imu._gyro.read(1010, gyro);
        assert.deepEqual(values(imu), [1.01, 2, 12, 22, ...gyro]);
        const sample = imu.getDiagnostics().lastSample;
        assert.equal(sample.accelInterpolated, true);
        assert.equal(sample.accelTimestampS, null, 'interpolated values are not a measured accel sample at the gyro time');
        assert.equal(sample.accelEvaluationTimestampS, 1.01);
        assert.equal(sample.skewS, null);
    }
});

test('A gyro 1ms after the left accel waits for the 15.7ms right bracket in either callback order', () => {
    for (const gyroFirst of [false, true]) {
        const { imu } = fixture();
        imu._accel.read(999, [0, 9, 20]);
        if (gyroFirst) {
            imu._gyro.read(1000, gyro);
            assert.equal(imu.flush().count, 0);
        }
        imu._accel.read(1015.7, [16.7, 25.7, 36.7]);
        if (!gyroFirst) imu._gyro.read(1000, gyro);
        const actual = values(imu), expected = [1, 1, 10, 21, ...gyro];
        expected.forEach((value, i) => assert.ok(Math.abs(actual[i] - value) < 1e-11));
        assert.equal(imu.getDiagnostics().lastSample.accelBeforeTimestampS, .999);
        assert.equal(imu.getDiagnostics().lastSample.accelAfterTimestampS, 1.0157);
    }
});

test('Missing acceleration drops a gyro only once the monotonic accel bracket proves it impossible', () => {
    const { imu } = fixture(); seed(imu);
    imu._gyro.read(1010, gyro); imu._gyro.read(1040, gyro);
    assert.equal(imu.flush().count, 0);
    assert.equal(imu.getDiagnostics().drops.pairing, 0, 'newer gyros cannot prove that an earlier accel callback is missing');
    imu._accel.read(1040, [0, 8, -2]);
    assert.deepEqual(values(imu), [1.04, 0, 8, -2, ...gyro]);
    assert.equal(imu.getDiagnostics().drops.pairing, 1);
});

test('Sensor callback bursts spanning multiple periods preserve all exact pairs in either order', () => {
    const timestamps = [1010, 1020, 1040];
    const outputs = [];
    for (const order of ['gyro-first', 'accel-first', 'interleaved']) {
        const { imu } = fixture(); seed(imu);
        const readGyro = time => imu._gyro.read(time, [time / 1000, 0, 0]);
        const readAccel = time => imu._accel.read(time, [0, time / 100, -1]);
        if (order === 'interleaved') for (const time of timestamps) { readGyro(time); readAccel(time); }
        else {
            for (const time of timestamps) (order === 'gyro-first' ? readGyro : readAccel)(time);
            for (const time of timestamps) (order === 'gyro-first' ? readAccel : readGyro)(time);
        }
        outputs.push(values(imu));
        assert.equal(imu.getDiagnostics().drops.pairing, 0);
    }
    const expected = timestamps.flatMap(time => [time / 1000, 0, time / 100, -1, time / 1000, 0, 0]);
    for (const output of outputs) assert.deepEqual(output, expected);
});

test('Source capacity evictions are explicit and keep the same retained exact pairs in either burst order', () => {
    for (const gyroFirst of [false, true]) {
        const { imu } = fixture();
        const readGyros = () => { for (let i = 0; i < 600; i++) imu._gyro.read(1000 + i, [i, 0, 0]); };
        const readAccels = () => { for (let i = 0; i < 600; i++) imu._accel.read(1000 + i, [0, i, 0]); };
        if (gyroFirst) { readGyros(); readAccels(); } else { readAccels(); readGyros(); }
        const sampleValues = values(imu), diagnostics = imu.getDiagnostics();
        assert.equal(sampleValues.length, 512 * 7);
        assert.equal(sampleValues[0], 1.088); assert.equal(sampleValues[2], 88);
        assert.equal(sampleValues.at(-7), 1.599); assert.equal(sampleValues.at(-5), 599);
        assert.equal(diagnostics.pairingPending, 0);
        assert.equal(diagnostics.drops.pairing, 88);
        assert.equal(diagnostics.drops.pairingOverflow, gyroFirst ? 88 : 0);
        assert.equal(diagnostics.drops.pairingAccelOverflow, gyroFirst ? 0 : 88);
        assert.ok(diagnostics.pairingAccelBuffered <= diagnostics.pairingCapacity);
    }
});

test('Interpolation never crosses a stale acceleration bracket and duplicate gyro is not emitted twice', () => {
    const { imu } = fixture(); seed(imu);
    imu._gyro.read(1100, gyro); imu._accel.read(1110, [0, 8, -2]);
    assert.equal(imu.flush().count, 0);
    assert.equal(imu.getDiagnostics().drops.pairing, 1);
    imu._gyro.read(1110, gyro); imu._gyro.read(1110, gyro);
    assert.deepEqual(values(imu), [1.11, 0, 8, -2, ...gyro]);
    assert.equal(imu.getDiagnostics().drops.outOfOrder, 1);
});

test('Fallback discards pending generic pairs and cannot emit a late old-generation acceleration', () => {
    const { imu, listeners } = fixture(); seed(imu);
    const accel = imu._accel, genericGyro = imu._gyro;
    genericGyro.read(1010, gyro); genericGyro.fail();
    assert.equal(imu.getDiagnostics().drops.pairing, 1);
    accel.read(1010, [0, 8, -2]);
    listeners.get('devicemotion')({ timeStamp: 1020, accelerationIncludingGravity: { x: 0, y: 9, z: -1 },
        rotationRate: { alpha: 0, beta: 0, gamma: 0 } });
    assert.deepEqual(values(imu), [1.02, 0, 9, -1, 0, 0, 0]);
    assert.equal(imu.getSensorType(), 'device_motion');
});

test('Discard removes every queued gyro pair before source collection resumes', () => {
    const { imu } = fixture(); seed(imu);
    imu._gyro.read(1010, gyro); imu._gyro.read(1020, gyro); imu.discard();
    assert.equal(imu.getDiagnostics().drops.pairing, 2);
    imu._accel.read(1020, [0, 8, -2]);
    assert.equal(imu.flush().count, 0);
    imu._gyro.read(1030, gyro); imu._accel.read(1030, [0, 7, -3]);
    assert.deepEqual(values(imu), [1.03, 0, 7, -3, ...gyro]);
});

test('A burst without acceleration bounds pending storage and accounts for every unmatched gyro', () => {
    const { imu } = fixture();
    for (let i = 0; i < 2000; i++) imu._gyro.read(1000 + i * .0001, gyro);
    const diagnostics = imu.getDiagnostics();
    assert.equal(imu.flush().count, 0);
    assert.ok(diagnostics.pairingPending <= 512);
    assert.equal(diagnostics.receivedGyro, diagnostics.pairingPending + diagnostics.drops.pairing);
    imu.stop();
    assert.equal(imu.getDiagnostics().pairingPending, 0);
    assert.equal(imu.getDiagnostics().drops.pairing, 2000);
});

test('A future acceleration clock fails closed while discarding pending source pairs', () => {
    const { imu } = fixture(); seed(imu);
    imu._gyro.read(1010, gyro); imu._accel.read(2002, [0, 9, 0]);
    const diagnostics = imu.getDiagnostics();
    assert.equal(imu.getSensorType(), 'device_motion');
    assert.equal(imu.flush().count, 0);
    assert.equal(diagnostics.clockFailure.reason, 'source_timestamp_in_future');
    assert.equal(diagnostics.drops.timestamp, 1);
    assert.equal(diagnostics.drops.pairing, 1);
    assert.equal(diagnostics.pairingPending, 0);
});

test('Discard cannot make a previously emitted gyro timestamp eligible again', () => {
    const { imu } = fixture(); seed(imu); imu.discard();
    imu._accel.read(999, [0, 9, 0]); imu._gyro.read(999, gyro);
    assert.equal(imu.flush().count, 0);
    assert.equal(imu.getDiagnostics().drops.outOfOrder, 1);
    assert.equal(imu.getDiagnostics().lastSample.timestampS, 1);
    imu._accel.read(1010, [0, 8, -1]); imu._gyro.read(1010, gyro);
    assert.deepEqual(values(imu), [1.01, 0, 8, -1, ...gyro]);
});
