import {
  RingBuffer,
  driverAlarms,
  medianFinite,
  pearsonCorrelation,
  sampleAgeState,
  traceSegments,
} from './monitor-core.js';

const COLORS = {
  external: '#55d8d0', effort: '#ff6b4a', model: '#f0bb52',
  position: '#5e91d8', velocity: '#c6dc72', current: '#ff6b4a', tracking: '#b77bd7',
  grid: '#2d3539', text: '#748084', zero: '#465156',
};
const arms = ['left', 'right'];
const buffers = new Map(arms.map((arm) => [arm, new RingBuffer()]));
const latest = new Map();
const receivedAt = new Map();
let activeArm = 'left';
let activeJoint = 0;
let windowSeconds = 20;
let paused = false;
let connectionState = 'connecting';
let runtimeMode = 'receive_only';

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
const canvases = {
  torque: $('#torqueCanvas'), position: $('#positionCanvas'), velocity: $('#velocityCanvas'),
  current: $('#currentCanvas'), tracking: $('#trackingCanvas'),
};

function finite(value) { return Number.isFinite(value) ? value : null; }
function fixed(value, digits = 3) { return Number.isFinite(value) ? value.toFixed(digits) : '--'; }
function sampleValue(sample, field, joint, scale = 1) {
  const value = sample?.[field]?.[joint];
  return Number.isFinite(value) ? value * scale : null;
}

function acceptSample(sample) {
  if (sample?.schema !== 'piperx-monitor-v1' || !buffers.has(sample.arm)) return;
  setRuntimeMode(sample.runtime_mode ?? runtimeMode);
  receivedAt.set(sample.arm, performance.now());
  if (paused) return;
  const normalized = { ...sample, timestampMs: sample.timestamp_ns / 1e6 };
  buffers.get(sample.arm).push(normalized);
  latest.set(sample.arm, normalized);
}

function setRuntimeMode(mode) {
  runtimeMode = mode === 'standalone_teleop' ? 'standalone_teleop' : 'receive_only';
  const standalone = runtimeMode === 'standalone_teleop';
  $('#modeEyebrow').textContent = standalone ? 'STANDALONE TELEOP / SHARED TELEMETRY' : 'PASSIVE TELEMETRY / RECEIVE ONLY';
  $('#controlMode').textContent = standalone ? '普通遥操 / CONTROL + MONITOR' : '无 / RX ONLY';
  $('#controlMode').classList.toggle('safe', !standalone);
}

async function hydrate() {
  try {
    const response = await fetch('/api/snapshot', { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const snapshot = await response.json();
    setRuntimeMode(snapshot.mode);
    Object.values(snapshot.arms ?? {}).forEach(acceptSample);
  } catch (error) {
    setConnection('error', `快照失败 · ${error.message}`);
  }
}

function setConnection(state, label) {
  connectionState = state;
  $('#connectionBadge').dataset.state = state;
  $('#connectionLabel').textContent = label;
}

function connectStream() {
  const source = new EventSource('/stream');
  source.addEventListener('open', () => setConnection('live', '实时连接'));
  source.addEventListener('ready', (event) => {
    try { setRuntimeMode(JSON.parse(event.data).mode); } catch { /* status is optional */ }
    setConnection('live', '实时连接');
  });
  source.addEventListener('snapshot', (event) => {
    try { acceptSample(JSON.parse(event.data)); setConnection('live', '实时连接'); }
    catch { setConnection('error', '数据格式错误'); }
  });
  source.onerror = () => setConnection('error', '重连中');
}

function prepareCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.max(1, window.devicePixelRatio || 1);
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width; canvas.height = height;
  }
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { context, width: rect.width, height: rect.height };
}

function visibleSamples() {
  const values = buffers.get(activeArm).values();
  if (!values.length) return [];
  const end = values.at(-1).timestampMs;
  return values.filter((sample) => sample.timestampMs >= end - windowSeconds * 1000);
}

function boundsFor(series, symmetric = false) {
  const values = series.flatMap((trace) => trace.points.map((point) => point.y)).filter(Number.isFinite);
  if (!values.length) return [-1, 1];
  let min = Math.min(...values); let max = Math.max(...values);
  if (symmetric) { const limit = Math.max(Math.abs(min), Math.abs(max), .05) * 1.12; return [-limit, limit]; }
  if (Math.abs(max - min) < 1e-9) { const margin = Math.max(Math.abs(max) * .1, .1); return [min - margin, max + margin]; }
  const margin = (max - min) * .12; return [min - margin, max + margin];
}

function drawChart(canvas, traces, { symmetric = false, decimals = 2 } = {}) {
  const { context: ctx, width, height } = prepareCanvas(canvas);
  ctx.clearRect(0, 0, width, height);
  const pad = { left: 46, right: 12, top: 13, bottom: 22 };
  const plotW = Math.max(1, width - pad.left - pad.right);
  const plotH = Math.max(1, height - pad.top - pad.bottom);
  const samples = visibleSamples();
  const end = samples.at(-1)?.timestampMs ?? 0;
  const start = end - windowSeconds * 1000;
  const [minY, maxY] = boundsFor(traces, symmetric);
  const x = (time) => pad.left + ((time - start) / (windowSeconds * 1000)) * plotW;
  const y = (value) => pad.top + (1 - (value - minY) / (maxY - minY)) * plotH;

  ctx.lineWidth = 1;
  ctx.strokeStyle = COLORS.grid;
  ctx.fillStyle = COLORS.text;
  ctx.font = '9px "Cascadia Mono", monospace';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let row = 0; row <= 4; row += 1) {
    const py = pad.top + (plotH * row) / 4;
    ctx.beginPath(); ctx.moveTo(pad.left, py); ctx.lineTo(width - pad.right, py); ctx.stroke();
    const value = maxY - ((maxY - minY) * row) / 4;
    ctx.fillText(value.toFixed(decimals), pad.left - 7, py);
  }
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (let column = 0; column <= 4; column += 1) {
    const px = pad.left + (plotW * column) / 4;
    ctx.beginPath(); ctx.moveTo(px, pad.top); ctx.lineTo(px, height - pad.bottom); ctx.stroke();
    ctx.fillText(`${(-windowSeconds + (windowSeconds * column) / 4).toFixed(0)}s`, px, height - pad.bottom + 6);
  }
  if (minY < 0 && maxY > 0) {
    ctx.strokeStyle = COLORS.zero; ctx.setLineDash([5, 5]);
    ctx.beginPath(); ctx.moveTo(pad.left, y(0)); ctx.lineTo(width - pad.right, y(0)); ctx.stroke(); ctx.setLineDash([]);
  }

  ctx.save(); ctx.beginPath(); ctx.rect(pad.left, pad.top, plotW, plotH); ctx.clip();
  for (const trace of traces) {
    if (!trace.visible) continue;
    ctx.strokeStyle = trace.color; ctx.lineWidth = trace.width ?? 1.5;
    ctx.shadowColor = trace.color; ctx.shadowBlur = trace.glow ?? 0;
    for (const segment of traceSegments(trace.points)) {
      ctx.beginPath();
      segment.forEach((point, index) => { if (index === 0) ctx.moveTo(x(point.x), y(point.y)); else ctx.lineTo(x(point.x), y(point.y)); });
      ctx.stroke();
    }
  }
  ctx.restore();
}

function tracePoints(samples, field, scale = 1, requireValid = false) {
  return samples.map((sample) => ({
    x: sample.timestampMs,
    y: sampleValue(sample, field, activeJoint, scale),
    valid: !requireValid || sample.valid,
  }));
}

function drawAllCharts() {
  const samples = visibleSamples();
  const traceEnabled = Object.fromEntries($$('[data-trace]').map((input) => [input.dataset.trace, input.checked]));
  drawChart(canvases.torque, [
    { points: tracePoints(samples, 'tau_external_nm', 1, true), color: COLORS.external, visible: traceEnabled.external, width: 2.1, glow: 3 },
    { points: tracePoints(samples, 'tau_effort_nm'), color: COLORS.effort, visible: traceEnabled.effort },
    { points: tracePoints(samples, 'tau_model_nm'), color: COLORS.model, visible: traceEnabled.model },
  ], { symmetric: true, decimals: 2 });
  drawChart(canvases.position, [{ points: tracePoints(samples, 'q_rad', 180 / Math.PI), color: COLORS.position, visible: true }], { decimals: 1 });
  drawChart(canvases.velocity, [{ points: tracePoints(samples, 'qd_rad_s'), color: COLORS.velocity, visible: true }], { symmetric: true, decimals: 1 });
  drawChart(canvases.current, [{ points: tracePoints(samples, 'current_a'), color: COLORS.current, visible: true }], { symmetric: true, decimals: 1 });
  drawChart(canvases.tracking, [{ points: tracePoints(samples, 'tracking_error_rad', 180 / Math.PI), color: COLORS.tracking, visible: true }], { symmetric: true, decimals: 2 });
}

function updateStatus() {
  const now = performance.now();
  for (const arm of arms) {
    const samples = buffers.get(arm).values();
    const end = samples.at(-1)?.timestampMs ?? 0;
    const rate = medianFinite(
      samples
        .filter((item) => item.timestampMs >= end - 1000)
        .map((item) => item.frequency_hz),
    );
    $(`#${arm}Rate`).textContent = fixed(rate, 0);
  }
  const sample = latest.get(activeArm);
  const age = receivedAt.has(activeArm) ? now - receivedAt.get(activeArm) : Number.POSITIVE_INFINITY;
  const ageState = sampleAgeState(age);
  $('#estimateState').textContent = !sample ? '等待数据' : sample.valid ? 'VALID / 已标定' : `INVALID / ${sample.reason ?? '未知原因'}`;
  $('#estimateState').style.color = sample?.valid ? 'var(--cyan)' : 'var(--danger)';
  $('#sampleAge').textContent = Number.isFinite(age) ? `${age.toFixed(0)} ms` : '-- ms';
  $('#invalidStamp').classList.toggle('visible', Boolean(sample && !sample.valid));
  if (connectionState === 'live' && ageState === 'critical') setConnection('error', '遥测超时');

  const firmware = sample?.firmware ?? { status: 'unknown', version: null, scale_verified: false };
  $('#firmwareCell').dataset.state = firmware.status;
  $('#firmwareVersion').textContent = firmware.version ?? '--';
  $('#firmwareState').textContent = firmware.status === 'current' ? '比例已核对' : firmware.status === 'legacy' ? '旧固件需 J1–J3 ×4' : '固件未知';
  $('#activeIdentity').textContent = `${activeArm.toUpperCase()} · J${activeJoint + 1}`;

  $('#externalNow').textContent = fixed(sampleValue(sample, 'tau_external_nm', activeJoint));
  $('#effortNow').textContent = fixed(sampleValue(sample, 'tau_effort_nm', activeJoint));
  $('#modelNow').textContent = fixed(sampleValue(sample, 'tau_model_nm', activeJoint));
  $('#currentNow').textContent = fixed(sampleValue(sample, 'current_a', activeJoint));
}

function updateCorrelation() {
  const samples = visibleSamples().filter((sample) => sample.valid);
  const effort = samples.map((sample) => sampleValue(sample, 'tau_effort_nm', activeJoint));
  const external = samples.map((sample) => sampleValue(sample, 'tau_external_nm', activeJoint));
  const correlation = pearsonCorrelation(effort, external);
  $('#correlationValue').textContent = correlation === null ? '--' : correlation.toFixed(2);
  const arc = $('#correlationArc');
  const fraction = correlation === null ? 0 : (correlation + 1) / 2;
  arc.style.strokeDashoffset = String(270 * (1 - fraction));
  arc.style.stroke = correlation === null ? '#657074' : correlation < 0 ? '#ff6b4a' : '#55d8d0';
}

function rowValue(sample, field, joint, digits = 3, scale = 1) { return fixed(sampleValue(sample, field, joint, scale), digits); }
function updateMatrix() {
  const rows = [];
  for (const arm of arms) {
    const sample = latest.get(arm);
    for (let joint = 0; joint < 6; joint += 1) {
      const driver = sample?.driver?.[joint];
      const valid = Boolean(sample?.valid && driver?.fresh !== false);
      const stateClass = !sample ? 'waiting' : valid ? '' : 'invalid';
      const state = !sample ? 'WAIT' : sample.valid ? driver?.fresh === false ? 'STALE' : 'VALID' : 'INVALID';
      rows.push(`<tr>
        <td><span class="arm-label ${arm}">${arm.toUpperCase()}</span></td><td>J${joint + 1}</td>
        <td>${rowValue(sample, 'tau_external_nm', joint)}</td><td>${rowValue(sample, 'tau_effort_nm', joint)}</td>
        <td>${rowValue(sample, 'current_a', joint)}</td><td>${rowValue(sample, 'q_rad', joint, 2, 180 / Math.PI)}</td>
        <td>${rowValue(sample, 'qd_rad_s', joint)}</td><td>${fixed(driver?.motor_temp_c, 0)}</td><td>${fixed(driver?.foc_temp_c, 0)}</td>
        <td><span class="state-chip ${stateClass}">${state}</span></td></tr>`);
    }
  }
  $('#jointTableBody').innerHTML = rows.join('');
}

function updateAlarms() {
  const sample = latest.get(activeArm);
  const alarms = sample ? driverAlarms(sample.driver) : ['等待遥测数据'];
  if (sample && !sample.valid) alarms.unshift(`估计无效：${sample.reason ?? '未知原因'}`);
  if (sample?.firmware?.status === 'unknown') alarms.push('固件未知：电流换算比例未核对');
  if (sample?.firmware?.status === 'legacy') alarms.push('旧固件：J1–J3 effort 需要额外 ×4 核对');
  $('#alarmCount').textContent = sample ? String(alarms.length) : '0';
  $('#alarmCount').classList.toggle('has-alarms', Boolean(sample && alarms.length));
  $('#alarmList').innerHTML = alarms.map((alarm) => `<li class="${sample ? '' : 'empty'}">${alarm}</li>`).join('');
}

function selectArm(arm) {
  activeArm = arm;
  $$('[data-arm]').forEach((button) => button.classList.toggle('active', button.dataset.arm === arm));
}
function selectJoint(joint) {
  activeJoint = joint;
  $$('[data-joint]').forEach((button) => button.classList.toggle('active', Number(button.dataset.joint) === joint));
}

$$('[data-arm]').forEach((button) => button.addEventListener('click', () => selectArm(button.dataset.arm)));
$$('[data-joint]').forEach((button) => button.addEventListener('click', () => selectJoint(Number(button.dataset.joint))));
$('#windowSelect').addEventListener('change', (event) => { windowSeconds = Number(event.target.value); });
$('#pauseButton').addEventListener('click', () => {
  paused = !paused; $('#pauseButton').classList.toggle('active', paused);
  $('#pauseButton').lastChild.textContent = paused ? ' RESUME' : ' PAUSE';
});
window.addEventListener('keydown', (event) => {
  if (/^[1-6]$/.test(event.key)) selectJoint(Number(event.key) - 1);
  if (event.key.toLowerCase() === 'l') selectArm('left');
  if (event.key.toLowerCase() === 'r') selectArm('right');
});

let slowFrame = 0;
function render() {
  drawAllCharts(); updateStatus(); updateCorrelation();
  if (slowFrame++ % 10 === 0) { updateMatrix(); updateAlarms(); }
  $('#clock').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  requestAnimationFrame(render);
}

hydrate().finally(connectStream);
requestAnimationFrame(render);
