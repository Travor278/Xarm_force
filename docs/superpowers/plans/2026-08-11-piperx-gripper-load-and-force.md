# PiperX gripper payload and force implementation plan

**Goal:** Include the official Piper gripper rigid payload in six-joint inverse dynamics and add receive-only gripper torque/status plus calibrated fingertip-force visualization.

**Architecture:** A validated payload artifact is appended to joint 6 in Pinocchio and becomes part of the dynamics/calibration identity. CAN ID 0x2A8 and SDK feedback populate auxiliary gripper telemetry. A separate force-calibration module converts feedback torque to N only inside a validated empirical envelope. Monitoring carries the result to a dedicated web panel.

**Tech stack:** Python 3.10+, NumPy, Pinocchio, SocketCAN, Piper SDK, FastAPI/SSE, vanilla JavaScript, pytest, Node test runner.

---

### Task 1: Payload artifact and Pinocchio integration

**Files:**
- Create: `config/piperx_gripper_payload.json`
- Create: `robot_control/piperx/payload.py`
- Modify: `robot_control/piperx/dynamics.py`
- Test: `tests/piperx/test_payload.py`
- Test: `tests/piperx/test_dynamics.py`

1. Write failing tests for artifact validation, canonical SHA identity, and joint-6 attachment.
2. Implement the immutable payload model and JSON loader.
3. Append validated Pinocchio inertia to joint 6 without changing `nq=nv=6`.
4. Expose a combined dynamics identity and verify old/no-payload behavior remains explicit.

### Task 2: Calibration identity binding

**Files:**
- Modify: `robot_control/piperx/calibration.py`
- Modify: `robot_control/piperx/estimator.py`
- Modify: `scripts/piperx_external_torque.py`
- Modify: `scripts/piperx_torque_web.py`
- Modify: `scripts/piperx_teleop_web.py`
- Test: `tests/piperx/test_calibration.py`
- Test: `tests/piperx/test_cli.py`
- Test: `tests/piperx/test_web_cli.py`
- Test: `tests/piperx/test_teleop_web_cli.py`

1. Write failing mismatch tests for payload-bound dynamics.
2. Bump the calibration artifact and include payload SHA/dynamics identity.
3. Add explicit `--payload` configuration to fitting, evaluation, passive monitor, and standalone teleop monitor.
4. Fail closed when an old no-payload calibration is used with a payload model.

### Task 3: 0x2A8 decode and coherent state assembly

**Files:**
- Modify: `robot_control/piperx/protocol.py`
- Modify: `robot_control/piperx/socketcan.py`
- Test: `tests/piperx/test_protocol.py`
- Test: `tests/piperx/test_socketcan.py`

1. Write exact positive/negative travel and torque decode tests and status-bit tests.
2. Implement `GripperSample` and pure decoding.
3. Carry latest gripper telemetry as non-blocking auxiliary state with freshness.
4. Verify receive-only and 200 Hz arm coherence are unchanged.

### Task 4: Force-calibration core and CLI

**Files:**
- Create: `robot_control/piperx/gripper_force.py`
- Create: `scripts/piperx_gripper_force.py`
- Test: `tests/piperx/test_gripper_force.py`
- Test: `tests/piperx/test_gripper_force_cli.py`

1. Write tests for fitting, prediction, device mismatch, range rejection, stale feedback, and fault rejection.
2. Implement a versioned calibration artifact and affine direction-aware fit.
3. Implement CLI ingestion of CSV/JSON known-force samples and an evaluation report.
4. Keep N output invalid when no artifact is supplied.

### Task 5: SDK teleop parity and monitor transport

**Files:**
- Modify: `robot_control/piperx/sdk_teleop.py`
- Modify: `robot_control/piperx/monitoring.py`
- Modify: `robot_control/piperx/web_monitor.py`
- Modify: `scripts/piperx_torque_web.py`
- Modify: `scripts/piperx_teleop_web.py`
- Test: `tests/piperx/test_sdk_teleop.py`
- Test: `tests/piperx/test_monitoring.py`
- Test: `tests/piperx/test_web_monitor.py`

1. Write failing SDK extraction and strict JSON schema tests.
2. Read gripper feedback torque/status in the SDK follower path.
3. Apply optional force calibration during serialization.
4. Preserve raw torque/status when force is unavailable.

### Task 6: Gripper web panel

**Files:**
- Modify: `web/piperx_monitor/index.html`
- Modify: `web/piperx_monitor/styles.css`
- Modify: `web/piperx_monitor/app.js`
- Modify: `web/piperx_monitor/monitor-core.js`
- Test: `tests/web/piperx_monitor_core.test.mjs`

1. Write failing view-model tests for calibrated, uncalibrated, stale, and fault states.
2. Add gripper summary values, health chips, and torque/force history.
3. Keep J1-J6 selection and charts unchanged.
4. Verify narrow and wide layouts manually.

### Task 7: Documentation, deployment, and hardware acceptance

**Files:**
- Modify: `docs/piperx_external_torque.md`
- Modify: `scripts/open_piperx_monitor.ps1`
- Modify: `scripts/open_piperx_teleop_monitor.ps1`
- Modify: `scripts/run_piperx_teleop_web_remote.sh`
- Create: `docs/piperx_gripper_force_calibration.md`

1. Document payload provenance, commands, force-gauge procedure, and validity rules.
2. Run focused Python and Node tests, then the full suite.
3. Deploy only `arm_force` changes to `.166` and observe 0x2A8 receive-only telemetry on both CAN interfaces.
4. Verify arm update rate, gripper status, and raw torque trends; report force as uncalibrated until known-force data exists.
5. Record static-pose before/after payload residual evidence and the remote acceptance result.
