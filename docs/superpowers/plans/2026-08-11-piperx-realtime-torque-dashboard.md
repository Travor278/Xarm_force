# PiperX Realtime Torque Dashboard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Serve a receive-only, dual-arm PiperX torque dashboard on `.166` and view it locally through an SSH port forward during EvoStudio teleoperation.

**Architecture:** Enrich the existing pure CAN decoder and coherent state assembler with auxiliary current, driver, command, and passive firmware telemetry. One acquisition thread per follower publishes finite JSON snapshots to a bounded event hub; a loopback-only FastAPI application exposes health, snapshots, static assets, and SSE. A dependency-free canvas dashboard renders synchronized traces and diagnostic-only correlation in the local browser through SSH.

**Tech Stack:** Python 3.10+, NumPy, Pinocchio 3.9, FastAPI, uvicorn, Linux SocketCAN, SSE, HTML/CSS/ES modules/Canvas, pytest, Node's built-in test runner. Production target: the existing `evo-rl` Conda environment on `dell@192.168.105.166`.

## Global Constraints

- Do not modify any file under `D:\Code\Work\evostudio-piperx`.
- All CAN access remains receive-only: no `send`, `sendto`, `sendmsg`, `write`, SDK connect/control/query, CAN setup, role switch, enable, motion, gripper, or MIT command.
- The HTTP service binds to `127.0.0.1` unless `--allow-network-bind` is explicitly supplied, and exposes no mutating robot route.
- Label Piper SDK-style effort as `电流换算力矩（参考）`; it is current times a fixed coefficient, not an independent sensor.
- Unknown firmware leaves `firmware_scale_verified=false`; do not silently apply the legacy J1-J3 four-times adjustment to an existing calibration.
- Invalid or stale estimates serialize as JSON `null` and break chart lines; never carry them forward as valid or draw them as zero.
- Keep frontend assets local with no CDN or internet runtime dependency.

---

### Task 1: Decode passive auxiliary Piper feedback

**Files:**
- Modify: `robot_control/piperx/protocol.py`
- Modify: `tests/piperx/test_protocol.py`

**Interfaces:**
- Produces: `HighSpeedSample.current_a: float`
- Produces: `LowSpeedSample(joint_index, voltage_v, foc_temp_c, motor_temp_c, status_code, bus_current_a)`
- Produces: `JointCommandPair(first_joint, positions_rad)`
- Produces: `FirmwareFragment(data: bytes)`
- Produces: `decode_low_speed`, `decode_joint_command`, and updated `decode_frame`

- [ ] **Step 1: Write failing protocol tests**

Add tests that require signed current conversion, exact low-speed units and
status byte preservation, millidegree command conversion for IDs `0x155..157`,
and byte-preserving `0x4AF` firmware fragments:

```python
sample = decode_high_speed(0x251, bytes.fromhex("03e8fc1800000000"))
assert sample.current_a == pytest.approx(-1.0)
assert sample.effort_nm == pytest.approx(-1.18125)

slow = decode_low_speed(0x264, bytes.fromhex("01f4002d2cf0007b"))
assert slow.voltage_v == pytest.approx(50.0)
assert slow.foc_temp_c == 45
assert slow.motor_temp_c == 44
assert slow.status_code == 0xF0
assert slow.bus_current_a == pytest.approx(0.123)

command = decode_joint_command(0x155, bytes.fromhex("000003e8fffff830"))
assert command.first_joint == 0
assert command.positions_rad == pytest.approx((math.radians(1), math.radians(-2)))
assert decode_frame(0x4AF, b"S-V1.8-2") == FirmwareFragment(b"S-V1.8-2")
```

- [ ] **Step 2: Run the protocol tests and verify RED**

Run: `python -m pytest tests/piperx/test_protocol.py -q`

Expected: FAIL because the new types/functions and `current_a` do not exist.

- [ ] **Step 3: Implement minimal pure decoders**

Use big-endian structures `>hhi`, `>HhbBH`, and `>ii`. Preserve the repository's
signed-current behavior because the official parser calls its signed 16-bit
conversion despite a contradictory uint16 comment. Return typed dataclasses
from `decode_frame`, and continue returning `None` for unrelated IDs.

- [ ] **Step 4: Run protocol and full PiperX tests**

Run:

```powershell
python -m pytest tests/piperx/test_protocol.py -q
python -m pytest tests/piperx -q
```

Expected: all tests pass, with only the existing platform skips.

- [ ] **Step 5: Commit**

```powershell
git add robot_control/piperx/protocol.py tests/piperx/test_protocol.py
git commit -m "feat: decode PiperX auxiliary telemetry"
```

### Task 2: Enrich coherent states without slowing the estimator

**Files:**
- Modify: `robot_control/piperx/socketcan.py`
- Modify: `tests/piperx/test_socketcan.py`

**Interfaces:**
- Produces: `DriverTelemetry` immutable dataclass
- Extends: `PiperState.current_a`, `driver`, `q_command_rad`, `firmware`, and auxiliary freshness fields
- Preserves: coherent state emission depends only on position and high-speed revisions

- [ ] **Step 1: Write failing state-assembly tests**

Build synthetic frames and assert that a state is emitted without any
low-speed or command frames, then assert that latest optional values appear
without affecting the next coherent revision:

```python
assembler.update(0x264, bytes.fromhex("01f4002d2c40007b"), 1)
assembler.update(0x155, bytes.fromhex("000003e8000007d0"), 2)
assembler.update(0x4AF, b"S-V1.8-2", 3)
state = feed_one_complete_cycle(assembler, start_ns=10)
assert state.current_a[0] == pytest.approx(-1.0)
assert state.driver[3].motor_temp_c == 44
assert state.q_command_rad[:2] == pytest.approx((math.radians(1), math.radians(2)))
assert state.firmware == "S-V1.8-2"
```

Also assert missing optional telemetry is represented by `None`, and slow
telemetry older than its independent threshold is marked stale.

- [ ] **Step 2: Run and verify RED**

Run: `python -m pytest tests/piperx/test_socketcan.py -q`

Expected: FAIL because enriched fields do not exist.

- [ ] **Step 3: Implement auxiliary state storage**

Update all optional fields on receipt but increment only the existing position
and motor revisions used by coherence. Accumulate at most 64 passive firmware
bytes, extract `S-V` plus up to seven printable version characters, and never
send a query. Copy immutable tuples into each emitted state.

- [ ] **Step 4: Run focused and regression tests**

Run:

```powershell
python -m pytest tests/piperx/test_socketcan.py tests/piperx/test_estimator.py -q
python -m pytest tests/piperx -q
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```powershell
git add robot_control/piperx/socketcan.py tests/piperx/test_socketcan.py
git commit -m "feat: enrich PiperX coherent states"
```

### Task 3: Serialize diagnostic snapshots and fan out bounded events

**Files:**
- Create: `robot_control/piperx/monitoring.py`
- Create: `tests/piperx/test_monitoring.py`
- Modify: `robot_control/piperx/__init__.py`
- Modify: `tests/piperx/test_public_api.py`

**Interfaces:**
- Produces: `firmware_effort_status(version: str | None) -> FirmwareEffortStatus`
- Produces: `serialize_snapshot(arm_name: str, state: PiperState, estimate: Estimate, sequence: int) -> dict[str, object]`
- Produces: `LatestEventHub.subscribe()`, `publish(event)`, `unsubscribe(queue)`
- Produces: `sse_message(event: str, data: object, event_id: int | None = None) -> str`

- [ ] **Step 1: Write failing serializer and hub tests**

Tests must assert six-element arrays, tracking error, status-bit expansion,
finite JSON, and firmware classifications `unknown`, `legacy`, and `current`.
They must also fill a one-slot subscriber queue twice and assert only the newest
event remains:

```python
hub = LatestEventHub(queue_size=1)
subscriber = hub.subscribe()
hub.publish({"sequence": 1})
hub.publish({"sequence": 2})
assert subscriber.get_nowait() == {"sequence": 2}

payload = serialize_snapshot("left", state, estimate, sequence=7)
encoded = json.dumps(payload, allow_nan=False)
assert 'NaN' not in encoded
assert payload["tau_external_nm"] == [None] * 6
```

- [ ] **Step 2: Run and verify RED**

Run: `python -m pytest tests/piperx/test_monitoring.py tests/piperx/test_public_api.py -q`

Expected: FAIL because `monitoring.py` and its exports do not exist.

- [ ] **Step 3: Implement minimal monitoring primitives**

Sanitize every scalar recursively using `math.isfinite`. Expand status bits to
the official names while retaining `status_code`. Parse firmware numerically so
`S-V1.8-2` and earlier are legacy; newer recognized versions are current;
missing/unparseable values are unknown. Implement hub subscription with
`queue.Queue(maxsize=queue_size)` and drop-old-before-put behavior under a lock.

- [ ] **Step 4: Run focused and full tests**

Run:

```powershell
python -m pytest tests/piperx/test_monitoring.py tests/piperx/test_public_api.py -q
python -m pytest tests/piperx -q
```

Expected: all pass.

- [ ] **Step 5: Commit**

```powershell
git add robot_control/piperx/monitoring.py robot_control/piperx/__init__.py tests/piperx/test_monitoring.py tests/piperx/test_public_api.py
git commit -m "feat: publish finite PiperX monitor snapshots"
```

### Task 4: Build the loopback-only acquisition and SSE application

**Files:**
- Create: `robot_control/piperx/web_monitor.py`
- Create: `scripts/piperx_torque_web.py`
- Create: `tests/piperx/test_web_monitor.py`
- Create: `tests/piperx/test_web_cli.py`
- Modify: `tests/piperx/test_receive_only_audit.py`
- Modify: `requirements.txt`
- Modify: `setup.py`

**Interfaces:**
- Produces: `ArmMonitorConfig(name, serial, calibration_path)`
- Produces: `ArmAcquisitionWorker(config, dynamics_factory, reader_factory, hub, ui_rate_hz, firmware_override=None)`
- Produces: `MonitorRuntime.start()`, `stop()`, `status()`, and `snapshot()`
- Produces: `create_app(runtime, asset_root) -> FastAPI`
- CLI: `python scripts/piperx_torque_web.py --arm NAME,SERIAL,CALIBRATION ... --urdf PATH`

- [ ] **Step 1: Write failing runtime, API, CLI, and safety tests**

Use a fake receive-only reader yielding synthetic frames and a fake estimator;
do not mock the event hub. Assert worker downsampling, clean stop, exception
health, `/healthz`, `/api/status`, `/api/snapshot`, SSE content type and framing,
static index delivery, duplicate arm rejection, calibration parsing, and
non-loopback rejection without the explicit flag.

Extend the audit source list to include `monitoring.py`, `web_monitor.py`, and
`scripts/piperx_torque_web.py`.

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python -m pytest tests/piperx/test_web_monitor.py tests/piperx/test_web_cli.py tests/piperx/test_receive_only_audit.py -q
```

Expected: FAIL because the application does not exist.

- [ ] **Step 3: Implement acquisition and read-only routes**

Reuse `discover_interface`, `ReadOnlySocketCan`, `PiperStateAssembler`,
`PinocchioDynamics`, `CalibrationArtifact.load`, and
`VelocityDerivativeFilter`. Build one independent estimator per arm. Publish at
most `ui_rate_hz` while still passing every frame through the assembler. SSE
must yield an initial `ready` event, snapshot events, and 10-second heartbeat
comments, and unsubscribe in `finally`.

Use FastAPI lifespan to start/stop runtime. Add only `fastapi>=0.110` and
`uvicorn>=0.29` runtime dependencies already present on `.166`.

- [ ] **Step 4: Run focused and regression tests**

Run:

```powershell
python -m pytest tests/piperx/test_web_monitor.py tests/piperx/test_web_cli.py tests/piperx/test_receive_only_audit.py -q
python -m pytest tests/piperx -q
python scripts/piperx_torque_web.py --help
```

Expected: all tests pass and CLI help lists the safety-related bind option.

- [ ] **Step 5: Commit**

```powershell
git add robot_control/piperx/web_monitor.py scripts/piperx_torque_web.py tests/piperx/test_web_monitor.py tests/piperx/test_web_cli.py tests/piperx/test_receive_only_audit.py requirements.txt setup.py
git commit -m "feat: serve PiperX monitor over loopback SSE"
```

### Task 5: Implement the industrial realtime dashboard

**Files:**
- Create: `web/piperx_monitor/index.html`
- Create: `web/piperx_monitor/styles.css`
- Create: `web/piperx_monitor/monitor-core.js`
- Create: `web/piperx_monitor/app.js`
- Create: `tests/web/piperx_monitor_core.test.mjs`
- Modify: `tests/piperx/test_web_monitor.py`

**Interfaces:**
- Produces: `RingBuffer`, `pearsonCorrelation`, `traceSegments`, `sampleAgeState`, and `driverAlarms` ES-module exports
- Consumes: `/api/snapshot` and `/stream` schema version `piperx-monitor-v1`

- [ ] **Step 1: Write failing frontend logic tests**

Use `node:test` with a direct import of `monitor-core.js`. Assert bounded
buffers, correlation `null` for fewer than 20 points or near-zero variance,
known perfect positive/negative correlations, invalid samples splitting trace
segments, and stale/critical age thresholds at 100/500 ms.

- [ ] **Step 2: Run and verify RED**

Run: `node --test tests/web/piperx_monitor_core.test.mjs`

Expected: FAIL because the ES module does not exist.

- [ ] **Step 3: Implement pure frontend data functions**

Keep `monitor-core.js` independent of DOM APIs so Node tests exercise real
browser logic. Correlation filters paired finite points and returns `null` when
sample count/variance is insufficient. Trace segments omit invalid points
instead of coercing values to zero.

- [ ] **Step 4: Build the dashboard markup, styling, and canvas rendering**

Implement the specification's status rail, arm/joint selectors, main torque
strip, aligned q/qd/current/tracking strips, six-joint matrix, trace toggles,
pause/window controls, and diagnostic correlation warning. Use local font
fallbacks, CSS variables, visible keyboard focus, semantic buttons, and no
remote assets. Render all charts from a common timestamp transform and
`requestAnimationFrame`.

- [ ] **Step 5: Verify frontend logic and static delivery**

Run:

```powershell
node --test tests/web/piperx_monitor_core.test.mjs
python -m pytest tests/piperx/test_web_monitor.py -q
python -m pytest tests/piperx -q
```

Expected: all pass.

- [ ] **Step 6: Commit**

```powershell
git add web/piperx_monitor tests/web/piperx_monitor_core.test.mjs tests/piperx/test_web_monitor.py
git commit -m "feat: visualize PiperX torque telemetry"
```

### Task 6: Document, deploy, and hardware-verify the monitor

**Files:**
- Create: `scripts/open_piperx_monitor.ps1`
- Modify: `docs/piperx_external_torque.md`
- Create: `docs/piperx_dashboard_hardware_validation_2026-08-11.md`
- Modify: `/home/dell/piperx-force-validation/*` deployment copy

**Interfaces:**
- PowerShell helper opens `ssh -N -L LOCAL:127.0.0.1:REMOTE dell@HOST` and then the local browser; it never contains a password
- Remote service command uses `/home/dell/anaconda3/bin/conda run -n evo-rl`

- [ ] **Step 1: Add operation and truth-semantics documentation**

Document exact remote launch and Windows tunnel commands, each chart's units,
status meanings, firmware caveat, correlation limitation, stop procedure, log
location, and coexistence with EvoStudio. The helper validates local port range
and invokes Windows OpenSSH without storing credentials.

- [ ] **Step 2: Run all local verification**

Run:

```powershell
python -m pytest tests/piperx -q
node --test tests/web/piperx_monitor_core.test.mjs
python scripts/piperx_external_torque.py --help
python scripts/piperx_torque_web.py --help
git diff --check
```

Expected: all pass, no whitespace errors.

- [ ] **Step 3: Render and inspect synthetic dashboard states**

Start the app with a deterministic synthetic runtime, inspect at 1440x900 and
1280x720, and capture reconnect, valid, invalid, firmware-warning, and driver-
alarm states. Fix clipping, unreadable labels, contrast, focus, and chart gaps;
rerun frontend and server tests after every fix.

- [ ] **Step 4: Deploy an exact copy to `.166`**

Copy only the repository files needed by the application into
`/home/dell/piperx-force-validation`, preserve the existing calibration/data/
reports directories, and write the deployed commit to `DEPLOYMENT_COMMIT`.
Verify the selected URDF SHA-256 matches both calibration artifacts before
starting.

- [ ] **Step 5: Start loopback service and validate through SSH**

Start the remote process on `127.0.0.1:8765`, write PID/log files, open a local
SSH forward, and check `/healthz`, `/api/status`, `/api/snapshot`, and `/stream`.
Confirm both arms emit q, qd, signed current, current-derived effort, model,
bias, external estimate, temperatures, and status at the requested UI rate.

- [ ] **Step 6: Audit CAN and teleoperation coexistence**

Measure `/sys/class/net/can2/statistics/{rx_packets,rx_dropped}` and the same for
`can3` before/during the monitor. Verify no TX method exists in process sources,
no RX-drop increase attributable to monitoring, and no rate/control degradation
during a safe real teleoperation observation. Record exact findings and the
remaining lack of independent torque ground truth in the hardware-validation
document.

- [ ] **Step 7: Commit documentation and launcher**

```powershell
git add scripts/open_piperx_monitor.ps1 docs/piperx_external_torque.md docs/piperx_dashboard_hardware_validation_2026-08-11.md
git commit -m "docs: deploy PiperX realtime dashboard"
```

## Plan Self-Review

- Every design requirement maps to a task: protocol/firmware (1-2), finite
  schema and bounded fanout (3), receive-only loopback service (4), visual and
  correlation behavior (5), deployment/safety/hardware evidence (6).
- Production functions are introduced only after a focused failing test.
- Shared names are consistent across tasks: `PiperState`, `serialize_snapshot`,
  `LatestEventHub`, `MonitorRuntime`, schema `piperx-monitor-v1`, and the four
  read-only API endpoints.
- EvoStudio is outside every file list, and external force/torque truth remains
  explicitly out of scope for this visualization-only release.
