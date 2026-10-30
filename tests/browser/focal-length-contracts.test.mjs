import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/app.js', import.meta.url), 'utf8');
const start = source.indexOf('function estimateFocalLength(');
const end = source.indexOf('\n/**', start);
assert.ok(start >= 0 && end > start, 'Actual focal length estimator must be present');

function estimator(search = '') {
    const context = vm.createContext({ URL_PARAMS: new URLSearchParams(search), console: { log() {}, warn() {} } });
    vm.runInContext(source.slice(start, end) + '\nglobalThis.estimate = estimateFocalLength;', context);
    return context.estimate;
}

function track(focusDistance, settings = { zoom: 1 }) {
    return { getSettings: () => settings, getCapabilities: () => ({ focusDistance }) };
}

test('Focus capability cannot fabricate a 100 degree field of view', () => {
    const estimate = estimator();
    for (const focusDistance of [{ min: 0, max: 10 }, { min: 0.001, max: 10 }, { min: 0.02, max: 10 }, { min: 0, max: 0 }]) {
        const result = estimate(track(focusDistance), 480, 640, null);
        assert.ok(Math.abs(result.fx - 465.60288917518244) < 1e-10, `focus range ${JSON.stringify(focusDistance)}`);
        assert.equal(result.method, 'default 69° (ref=640)');
    }
});

test('Focus capability cannot override an explicit focal length factor', () => {
    const estimate = estimator();
    for (const min of [0, 0.001, 0.02]) {
        const result = estimate(track({ min, max: 10 }), 480, 640, 0.8);
        assert.equal(result.fx, 512);
        assert.equal(result.method, 'config factor 0.8');
    }
});

test('URL focal length keeps priority over track FOV and explicit factor', () => {
    const result = estimator('?fx=600')(track({ min: 0, max: 10 }, { fieldOfView: 90, zoom: 1 }), 480, 640, 0.8);
    assert.equal(result.fx, 600);
    assert.equal(result.method, 'URL override ?fx=600');
});

test('Explicit track FOV keeps priority over focus range and factor', () => {
    const result = estimator()(track({ min: 0, max: 10 }, { fieldOfView: 90, zoom: 1 }), 480, 640, 0.8);
    assert.ok(Math.abs(result.fx - 320) < 1e-10);
    assert.equal(result.method, 'track FOV 90°');
});

test('Default remains the same heuristic across image rotations without focus metadata', () => {
    for (const [width, height] of [[480, 640], [640, 480]]) {
        const result = estimator()(null, width, height, null);
        assert.ok(Math.abs(result.fx - 465.60288917518244) < 1e-10);
        assert.equal(result.method, 'default 69° (ref=640)');
    }
});
