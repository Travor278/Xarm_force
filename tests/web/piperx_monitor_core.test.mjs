import test from 'node:test';
import assert from 'node:assert/strict';

import {
  RingBuffer,
  driverAlarms,
  pearsonCorrelation,
  sampleAgeState,
  traceSegments,
} from '../../web/piperx_monitor/monitor-core.js';


test('RingBuffer evicts samples outside duration and point bounds', () => {
  const buffer = new RingBuffer({ maxDurationMs: 60_000, maxPoints: 3 });
  buffer.push({ timestampMs: 0, value: 0 });
  buffer.push({ timestampMs: 1_000, value: 1 });
  buffer.push({ timestampMs: 2_000, value: 2 });
  buffer.push({ timestampMs: 3_000, value: 3 });
  assert.deepEqual(buffer.values().map((sample) => sample.value), [1, 2, 3]);

  buffer.push({ timestampMs: 70_000, value: 70 });
  assert.deepEqual(buffer.values().map((sample) => sample.value), [70]);
});


test('pearsonCorrelation rejects short and zero-variance windows', () => {
  assert.equal(pearsonCorrelation([1, 2], [2, 4]), null);
  assert.equal(
    pearsonCorrelation(Array(20).fill(1), Array.from({ length: 20 }, (_, i) => i)),
    null,
  );
});


test('pearsonCorrelation identifies positive and negative trends', () => {
  const x = Array.from({ length: 30 }, (_, i) => i - 15);
  assert.ok(Math.abs(pearsonCorrelation(x, x.map((value) => 3 * value + 2)) - 1) < 1e-12);
  assert.ok(Math.abs(pearsonCorrelation(x, x.map((value) => -2 * value)) + 1) < 1e-12);
});


test('traceSegments breaks lines across invalid or nonfinite samples', () => {
  const segments = traceSegments([
    { x: 0, y: 1, valid: true },
    { x: 1, y: 2, valid: true },
    { x: 2, y: 99, valid: false },
    { x: 3, y: 3, valid: true },
    { x: 4, y: null, valid: true },
  ]);

  assert.deepEqual(segments, [
    [{ x: 0, y: 1, valid: true }, { x: 1, y: 2, valid: true }],
    [{ x: 3, y: 3, valid: true }],
  ]);
});


test('sampleAgeState has explicit live stale and critical boundaries', () => {
  assert.equal(sampleAgeState(100), 'live');
  assert.equal(sampleAgeState(101), 'stale');
  assert.equal(sampleAgeState(500), 'stale');
  assert.equal(sampleAgeState(501), 'critical');
  assert.equal(sampleAgeState(Number.POSITIVE_INFINITY), 'critical');
});


test('driverAlarms reports official Piper status and stale telemetry', () => {
  const alarms = driverAlarms([
    { fresh: true, collision: true, overcurrent: false, driver_error: true, enabled: true },
    { fresh: false, collision: false, overcurrent: true, driver_error: false, enabled: false },
    null,
  ]);

  assert.deepEqual(alarms, ['J1 碰撞保护', 'J1 驱动错误', 'J2 驱动遥测过期', 'J2 过流', 'J2 未使能']);
});
