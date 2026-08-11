export class RingBuffer {
  constructor({ maxDurationMs = 60_000, maxPoints = 6_000 } = {}) {
    if (!(maxDurationMs > 0) || !(maxPoints > 0)) {
      throw new RangeError('RingBuffer limits must be positive');
    }
    this.maxDurationMs = maxDurationMs;
    this.maxPoints = maxPoints;
    this.items = [];
  }

  push(sample) {
    if (!Number.isFinite(sample?.timestampMs)) {
      throw new TypeError('sample.timestampMs must be finite');
    }
    this.items.push(sample);
    const cutoff = sample.timestampMs - this.maxDurationMs;
    let first = 0;
    while (first < this.items.length && this.items[first].timestampMs < cutoff) {
      first += 1;
    }
    if (first > 0) this.items.splice(0, first);
    if (this.items.length > this.maxPoints) {
      this.items.splice(0, this.items.length - this.maxPoints);
    }
  }

  values() {
    return this.items.slice();
  }

  clear() {
    this.items.length = 0;
  }
}


export function pearsonCorrelation(xs, ys, { minSamples = 20, minVariance = 1e-12 } = {}) {
  if (!Array.isArray(xs) || !Array.isArray(ys)) return null;
  const pairs = [];
  for (let index = 0; index < Math.min(xs.length, ys.length); index += 1) {
    if (Number.isFinite(xs[index]) && Number.isFinite(ys[index])) {
      pairs.push([xs[index], ys[index]]);
    }
  }
  if (pairs.length < minSamples) return null;
  const meanX = pairs.reduce((sum, pair) => sum + pair[0], 0) / pairs.length;
  const meanY = pairs.reduce((sum, pair) => sum + pair[1], 0) / pairs.length;
  let varianceX = 0;
  let varianceY = 0;
  let covariance = 0;
  for (const [x, y] of pairs) {
    const dx = x - meanX;
    const dy = y - meanY;
    varianceX += dx * dx;
    varianceY += dy * dy;
    covariance += dx * dy;
  }
  if (varianceX <= minVariance || varianceY <= minVariance) return null;
  return covariance / Math.sqrt(varianceX * varianceY);
}


export function estimateDisplayState(sample) {
  const hasFiniteExternal = sample?.tau_external_nm?.some(Number.isFinite) === true;
  if (!hasFiniteExternal) return 'unavailable';
  if (sample?.valid === true) return 'valid';
  if (sample?.reason === 'outside_calibrated_workspace') return 'extrapolated';
  return 'unavailable';
}


export function externalTracePresentation(samples) {
  const displayable = samples.filter(
    (sample) => estimateDisplayState(sample) !== 'unavailable',
  );
  const extrapolatedOnly = displayable.length > 0 && displayable.every(
    (sample) => estimateDisplayState(sample) === 'extrapolated',
  );
  return {
    colorKey: 'external',
    dash: extrapolatedOnly ? [7, 5] : [],
    glow: extrapolatedOnly ? 1 : 3,
  };
}


export function traceSegments(points) {
  const segments = [];
  let segment = [];
  for (const point of points) {
    if (point?.valid !== false && Number.isFinite(point?.x) && Number.isFinite(point?.y)) {
      segment.push(point);
      continue;
    }
    if (segment.length) segments.push(segment);
    segment = [];
  }
  if (segment.length) segments.push(segment);
  return segments;
}


export function sampleAgeState(ageMs) {
  if (!Number.isFinite(ageMs) || ageMs > 500) return 'critical';
  if (ageMs > 100) return 'stale';
  return 'live';
}


export function medianFinite(values) {
  const finite = values.filter(Number.isFinite).sort((left, right) => left - right);
  if (!finite.length) return null;
  const middle = Math.floor(finite.length / 2);
  return finite.length % 2
    ? finite[middle]
    : (finite[middle - 1] + finite[middle]) / 2;
}


export function driverAlarms(drivers) {
  const alarms = [];
  for (let index = 0; index < (drivers?.length ?? 0); index += 1) {
    const driver = drivers[index];
    if (!driver) continue;
    const joint = `J${index + 1}`;
    if (!driver.fresh) alarms.push(`${joint} 驱动遥测过期`);
    if (driver.voltage_low) alarms.push(`${joint} 欠压`);
    if (driver.motor_overheat) alarms.push(`${joint} 电机过温`);
    if (driver.overcurrent) alarms.push(`${joint} 过流`);
    if (driver.driver_overheat) alarms.push(`${joint} 驱动过温`);
    if (driver.collision) alarms.push(`${joint} 碰撞保护`);
    if (driver.driver_error) alarms.push(`${joint} 驱动错误`);
    if (driver.stall) alarms.push(`${joint} 堵转保护`);
    if (!driver.enabled) alarms.push(`${joint} 未使能`);
  }
  return alarms;
}
