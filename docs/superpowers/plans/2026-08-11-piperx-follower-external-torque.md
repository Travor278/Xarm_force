# PiperX Follower External Joint-Torque Estimation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and hardware-validate a standalone, receive-only estimator for the six external joint torques on each PiperX follower arm, without changing EvoStudio or its LeRobot dataset writer.

**Architecture:** A Linux SocketCAN boundary discovers a USB-CAN adapter by stable serial number, decodes only Piper feedback frames, and assembles coherent SI-unit states. A Pinocchio backend computes rigid-body inverse dynamics; a bounded derivative filter supplies acceleration; a per-arm ridge artifact models repeatable no-contact residuals. A single CLI records, fits, monitors, replays, and scores data while enforcing serial/URDF/workspace compatibility.

**Tech Stack:** Python 3.10+, NumPy, Linux SocketCAN, Pinocchio 3.9, Rich, pytest. Production target: the existing `evo-rl` Conda environment on `dell@192.168.105.166`.

## Global Constraints

- Treat `D:\Code\Work\evostudio-piperx` as read-only throughout this plan.
- Never call `send`, `sendto`, `write`, Piper SDK connect/enable/control methods, `ip link`, or any CAN setup command from estimator code.
- Resolve follower interfaces by USB serial. Interface names such as `can2` and `can3` are diagnostic output, not configuration identity.
- Use radians, radians/second, radians/second squared, Nm, and monotonic nanoseconds internally.
- Define external torque as `tau_model + tau_bias - tau_measured`.
- Reject stale, incomplete, derivative-spike, metadata-mismatch, and out-of-workspace samples rather than silently degrading.
- Do not vendor the PiperX URDF. The available upstream PiperX model has no usable license declaration; require a caller-provided URDF and bind calibration to its SHA-256.
- Never store SSH passwords, private host configuration, or credentials in the repository or generated artifacts.
- Use test-first red/green cycles for every implementation task and commit after each completed task.

---

### Task 1: Decode Piper feedback frames into SI units

**Files:**

- Create: `robot_control/piperx/__init__.py`
- Create: `robot_control/piperx/protocol.py`
- Create: `tests/piperx/test_protocol.py`

- [ ] Write failing decoder tests first.

Cover signed big-endian values, position pairs `0x2A5..0x2A7`, high-speed motor frames `0x251..0x256`, an unknown identifier, short payloads, and current-to-effort conversion. Assert these exact conversions:

```python
assert decode_position_pair(0x2A5, bytes.fromhex("000003e8fffff830")) == (0, (1.0, -2.0))
sample = decode_high_speed(0x251, bytes.fromhex("03e8fc1800000000"))
assert sample.joint_index == 0
assert sample.velocity_rad_s == pytest.approx(1.0)
assert sample.effort_nm == pytest.approx(-1.18125)
```

- [ ] Run `python -m pytest tests/piperx/test_protocol.py -q` and confirm failure because the package does not exist.
- [ ] Implement strict pure decoders.

Use constants:

```python
POSITION_IDS = {0x2A5: 0, 0x2A6: 2, 0x2A7: 4}
HIGH_SPEED_IDS = {0x251 + index: index for index in range(6)}
EFFORT_NM_PER_RAW = np.array([1.18125, 1.18125, 1.18125,
                              0.95844, 0.95844, 0.95844]) * 1e-3
```

Decode position as signed `>ii` millidegrees and convert with `np.deg2rad(raw / 1000)`. Decode motor speed/current as signed `>hh`; speed is `raw * 1e-3 rad/s`, effort is `current_raw * EFFORT_NM_PER_RAW[joint]`. Return immutable dataclasses and raise `FrameDecodeError` for recognized malformed frames.

- [ ] Re-run the focused test and commit.

```text
git add robot_control/piperx tests/piperx/test_protocol.py
git commit -m "feat: decode PiperX feedback frames"
```

### Task 2: Discover a CAN adapter by serial and assemble read-only states

**Files:**

- Create: `robot_control/piperx/socketcan.py`
- Create: `tests/piperx/test_socketcan.py`

- [ ] Write failing tests for serial discovery, duplicate/missing serials, SocketCAN frame unpacking, coherent-state assembly, staleness, and receive-only behavior.

Use fake `/sys/class/net` entries/property loaders and a fake socket whose `send`, `sendto`, and `sendmsg` methods raise immediately. A successful test must observe only `bind`, `settimeout`, `recv`, and `close` calls. State assembly must require all three position-pair revisions and all six motor revisions before emitting one state.

- [ ] Run the focused tests and confirm the expected import failures.
- [ ] Implement `discover_interface(serial, sys_class_net=..., property_loader=...)`.

Enumerate interfaces under `/sys/class/net`; load properties through the argument-vector command below, never through a shell:

```python
subprocess.run(
    ["udevadm", "info", "--query=property", f"--path={path}"],
    check=True,
    capture_output=True,
    text=True,
)
```

Match `ID_SERIAL_SHORT` first and `ID_SERIAL` second. Require exactly one result.

- [ ] Implement `ReadOnlySocketCan` using Linux `AF_CAN`, `SOCK_RAW`, `CAN_RAW`, `bind((interface,))`, and `recv(16)`. Decode `struct.Struct("=IB3x8s")`; mask the CAN ID with `CAN_EFF_MASK`. Do not expose a transmit method.
- [ ] Implement `PiperStateAssembler` with per-field revisions and receive timestamps. Emit a `PiperState` only when every component advanced, `max(timestamp)-min(timestamp) <= max_skew_ns`, and all values are finite. Include source interface, serial, receive timestamp, measured frequencies, completeness, freshness, and reason.
- [ ] Re-run tests and commit.

```text
git add robot_control/piperx/socketcan.py tests/piperx/test_socketcan.py
git commit -m "feat: add receive-only PiperX state source"
```

### Task 3: Add the Pinocchio dynamics and derivative pipeline

**Files:**

- Create: `robot_control/piperx/dynamics.py`
- Create: `robot_control/piperx/filtering.py`
- Create: `tests/piperx/test_dynamics.py`
- Create: `tests/piperx/test_filtering.py`

- [ ] Write failing unit tests for SHA-256 computation, six-joint validation, base-orientation gravity transform, RNEA output shape/finite checks, startup invalidity, normal constant-acceleration tracking, non-monotonic timestamps, excessive gaps, and derivative spikes.
- [ ] Use a minimal six-revolute-joint URDF generated as a pytest fixture; skip only the Pinocchio integration assertions when `pinocchio` is unavailable locally. Pure gravity-transform tests must never skip.
- [ ] Run both focused files and observe failures.
- [ ] Implement `PinocchioDynamics` with a lazy `import pinocchio as pin`, `pin.buildModelFromUrdf`, `model.nq == model.nv == 6`, and `pin.rnea(model, data, q, qd, qdd)`.

For configured base roll/pitch/yaw, compute `R_world_base = Rz(yaw) @ Ry(pitch) @ Rx(roll)` and set:

```python
model.gravity.linear = R_world_base.T @ np.array([0.0, 0.0, -9.81])
```

Reject incorrect vector shapes and non-finite results.

- [ ] Implement `VelocityDerivativeFilter` with first-order low-pass filtering, monotonic time, configurable startup count, `max_gap_s`, and per-joint `max_acceleration_rad_s2`. Invalid samples return a reason and are excluded downstream.
- [ ] Re-run tests and commit.

```text
git add robot_control/piperx/dynamics.py robot_control/piperx/filtering.py tests/piperx/test_dynamics.py tests/piperx/test_filtering.py
git commit -m "feat: compute PiperX model torque safely"
```

### Task 4: Fit and enforce per-arm residual calibration

**Files:**

- Create: `robot_control/piperx/calibration.py`
- Create: `tests/piperx/test_calibration.py`

- [ ] Write failing tests for feature dimensions, recovery of known synthetic coefficients, contiguous group splitting, train/validation metrics, JSON round trip, adapter mismatch, URDF mismatch, feature-version mismatch, and workspace rejection.

Define versioned features from one `(q, qd)` sample:

```python
q_chain = np.cumsum(q[1:])
features = np.concatenate((
    [1.0], np.sin(q), np.cos(q),
    np.sin(q_chain), np.cos(q_chain),
    np.tanh(qd / friction_velocity_scale),
))
```

The calibration target for explicitly no-contact samples is `tau_measured - tau_model`. Fit all six targets with ridge regression, do not penalize the constant column, and validate on whole held-out contiguous groups rather than random frames.

- [ ] Run the focused test and confirm failure.
- [ ] Implement `CalibrationArtifact` with schema version, feature version, adapter serial, URDF SHA-256, base RPY, coefficients, ridge strength, velocity scale, train/validation metrics, q/qd workspace min/max, creation time, and source-log hash.
- [ ] Add `validate_runtime(serial, urdf_sha256, base_rpy)` and `predict(q, qd)`. Runtime metadata mismatch raises; workspace violation returns an invalid prediction reason.
- [ ] Re-run tests and commit.

```text
git add robot_control/piperx/calibration.py tests/piperx/test_calibration.py
git commit -m "feat: calibrate PiperX no-contact residuals"
```

### Task 5: Compose the estimator and deterministic records

**Files:**

- Create: `robot_control/piperx/estimator.py`
- Create: `robot_control/piperx/records.py`
- Create: `tests/piperx/test_estimator.py`
- Create: `tests/piperx/test_records.py`

- [ ] Write failing estimator tests with fake dynamics/calibration objects. Prove the exact sign convention with:

```python
tau_model = np.array([2, 2, 2, 2, 2, 2], dtype=float)
tau_bias = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
tau_measured = np.array([3, 1, 2.5, 2, 4, 0], dtype=float)
assert tau_external == pytest.approx([-0.5, 1.5, 0, 0.5, -1.5, 2.5])
```

Also test startup/stale/incomplete/filter/calibration/workspace invalidity propagation without losing diagnostic raw arrays.

- [ ] Write failing record tests for NPZ + sidecar JSON atomic save/load, required metadata, equal row counts, finite-data masks, and file SHA-256.
- [ ] Run focused tests and confirm failures.
- [ ] Implement `ExternalTorqueEstimator` and immutable `Estimate` data. Never clip stored torque; provide a separate bounded display helper.
- [ ] Implement deterministic record serialization with arrays `timestamp_ns`, `q`, `qd`, `qdd`, `tau_measured`, `tau_model`, `tau_bias`, `tau_external`, `valid`, `group_id`, and `reason`. Write to a sibling temporary file then use `os.replace`; reject partial or dimensionally inconsistent logs.
- [ ] Re-run tests and commit.

```text
git add robot_control/piperx/estimator.py robot_control/piperx/records.py tests/piperx/test_estimator.py tests/piperx/test_records.py
git commit -m "feat: compose PiperX torque estimator"
```

### Task 6: Build the passive record, fit, monitor, evaluate, and known-load CLI

**Files:**

- Create: `scripts/piperx_external_torque.py`
- Create: `tests/piperx/test_cli.py`
- Modify: `requirements.txt`
- Create: `requirements-dev.txt`
- Modify: `setup.py`

- [ ] Write failing CLI parser and offline-command tests. Hardware-free tests must cover `fit`, `evaluate`, metadata mismatch exit code, acceptance threshold failure exit code, and known-load sign/magnitude scoring.
- [ ] Define these commands:

```text
discover --serial SERIAL
record --arm NAME --serial SERIAL --urdf PATH --seconds N --output PATH --confirm-no-contact
fit --input PATH --output PATH [--ridge 1e-3] [--validation-fraction 0.2]
monitor --arm NAME,SERIAL,CALIBRATION ... --urdf PATH [--seconds N]
evaluate --input PATH --calibration PATH [--stationary]
known-load --input PATH --calibration PATH --expected-torque NM,NM,NM,NM,NM,NM
```

`record` must refuse to start without the literal `--confirm-no-contact` flag. `monitor` may attach to one or both follower arms, using one receive thread per arm and no transmitter. All commands log discovered interface, serial, URDF hash, completeness rate, valid rate, and reasons.

- [ ] Implement statistics per joint: signed mean, MAE, RMSE, standard deviation, p95 absolute error, sample count, and direction agreement for nonzero known-load joints.
- [ ] Make `evaluate` enforce stationary limits `0.30 Nm` for joints 2/3 and `0.15 Nm` for the others, and free-motion p95 `0.50 Nm`. Make `known-load` enforce `max(0.30 Nm, 15% * abs(expected))` plus 95% direction agreement.
- [ ] Add `pin>=3.9.0` to runtime requirements and `pytest>=8.0` to development requirements. Export only package/library APIs from `robot_control.piperx`; keep the CLI runnable as `python scripts/piperx_external_torque.py`.
- [ ] Run CLI tests, then the complete local suite, and commit.

```text
python -m pytest tests/piperx -q
git add scripts/piperx_external_torque.py tests/piperx/test_cli.py requirements.txt requirements-dev.txt setup.py robot_control/piperx/__init__.py
git commit -m "feat: add PiperX torque validation CLI"
```

### Task 7: Document operation and perform a source-level safety audit

**Files:**

- Create: `docs/piperx_external_torque.md`
- Modify: `README.md` only if it exists; otherwise keep the standalone guide
- Create: `tests/piperx/test_receive_only_audit.py`

- [ ] Write a source-level safety test that scans `robot_control/piperx` and `scripts/piperx_external_torque.py` for prohibited calls/tokens: `.send(`, `.sendto(`, `.sendmsg(`, `ip link`, `EnableArm`, `MotionCtrl`, `JointCtrl`, `GripperCtrl`, and `JointMitCtrl`.
- [ ] Document the physical meaning/sign convention, serial discovery, environment setup, each CLI flow, artifact compatibility rules, acceptance thresholds, troubleshooting, and the known-load setup using a measured mass/force and lever arm.
- [ ] State prominently that the result is joint torque, not a Cartesian force/wrench; no-contact calibration cannot prove absolute load accuracy; and EvoStudio integration stays blocked until known-load validation passes.
- [ ] Run the safety audit and whole suite, inspect `git diff --check`, and commit.

```text
python -m pytest tests/piperx -q
git diff --check
git add docs/piperx_external_torque.md tests/piperx/test_receive_only_audit.py
git commit -m "docs: explain PiperX torque validation workflow"
```

### Task 8: Deploy to `.166` and validate without disturbing EvoStudio

**Files (generated outside git on `.166`):**

- `/home/dell/piperx-force-validation/` deployment copy
- `/home/dell/piperx-force-validation/data/*.npz`
- `/home/dell/piperx-force-validation/calibration/*.json`
- `/home/dell/piperx-force-validation/reports/*.json`

- [ ] Record the pre-test `evostudio-client` service state and per-follower feedback frequency. Do not stop/restart the service and do not reconfigure CAN.
- [ ] Transfer a clean source archive over SSH/SFTP without credentials in filenames, scripts, shell history, or repository files. Install only missing Python dependencies in the existing `evo-rl` environment.
- [ ] Run the remote unit suite and CLI import/help smoke tests.
- [ ] Run `discover` for both follower serials and assert they resolve to the deployed left/right follower interfaces.
- [ ] Passively record a short static interval from each arm, confirm roughly 200 Hz complete feedback, finite SI units, no estimator transmit path, and unchanged EvoStudio health/rate.
- [ ] If normal user-driven no-contact motion is already occurring, record it; otherwise use stationary/ambient follower data only. Never initiate robot motion from this estimator task.
- [ ] Fit each arm independently, score held-out groups, and save exact metrics and artifact hashes. Do not reuse one arm's artifact for the other.
- [ ] Re-check `evostudio-client` state and CAN feedback frequency after the test. Any degradation fails the safety criterion and stops further validation.
- [ ] Run known-load scoring only if a physically measured load interval already exists or can be applied safely by a present operator. Do not fabricate a pass from no-contact data. If no physical load is available, report this criterion as an explicit remaining validation gate.
- [ ] Compare every obtained metric with the approved thresholds. Do not modify EvoStudio regardless of outcome in this phase.
- [ ] Commit only repository-side fixes found during hardware validation, each after a new failing regression test, then repeat the local and remote verification.

### Task 9: Final verification and handoff

**Files:**

- Modify: `docs/piperx_external_torque.md` only if observed deployment details require correction

- [ ] Invoke `superpowers:verification-before-completion` and run fresh evidence commands:

```text
python -m pytest tests/piperx -q
python scripts/piperx_external_torque.py --help
git diff --check
git status --short
```

- [ ] Report the original xArm repository diagnosis, implemented PiperX architecture, safety evidence, exact local/remote test results, per-joint error metrics, known-load status, files/artifacts, commit hashes, and any remaining gate.
- [ ] Reiterate the future EvoStudio contract only; do not implement it now:

```text
complementary_info.left_follower_joint_torque_external: float[6]
complementary_info.right_follower_joint_torque_external: float[6]
complementary_info.follower_joint_torque_external_valid: bool
```

Keep existing 14-dimensional `observation.state` and `action` unchanged.
