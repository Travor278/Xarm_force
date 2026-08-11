# PiperX Standalone Teleop Web Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run bimanual PiperX leader/follower teleoperation and torque visualization in one non-EvoStudio process that owns one Piper SDK connection per arm.

**Architecture:** Four `C_PiperInterface_V2` objects own can0–can3 without reconfiguring interfaces. A single coordinator copies each leader's control state to its follower at 200 Hz, constructs follower `PiperState` values from the same SDK objects, and publishes estimates through the existing loopback FastAPI/SSE application at 25 Hz. A visible SSH wrapper stops the idle EvoStudio client before control and restores it after the combined process exits.

**Tech Stack:** Python 3.10+, official `piper_sdk`, NumPy, Pinocchio, FastAPI, uvicorn, pytest, PowerShell/OpenSSH, systemd.

## Global Constraints

- Do not modify `D:\Code\Work\evostudio-piperx`.
- Never run two PiperX controllers concurrently; EvoStudio must report `idle` and its service must be stopped before standalone control starts.
- Do not call `ip link down`, `ip link set`, or otherwise reconfigure a live CAN interface.
- Use exactly one SDK/CAN connection per physical arm and derive Web telemetry from the follower controller's existing SDK connection.
- Require four distinct USB-CAN serials and firmware at least `S-V1.8-9`.
- Refuse startup when any initial leader/follower joint delta exceeds 15 degrees or gripper delta exceeds 10 mm.
- Preserve the existing estimate validity gates; never turn `outside_calibrated_workspace` into a valid value.
- Bind HTTP only to loopback and store no SSH or sudo password.
- On loss of leader commands, hold the last follower target; fail the session after one second without disabling motors.
- Run the remote interpreter directly, not through `conda run`, so SSH teardown propagates to the Python process.

---

### Task 1: Define SDK state conversion and safety gates

**Files:**
- Create: `robot_control/piperx/sdk_teleop.py`
- Create: `tests/piperx/test_sdk_teleop.py`

**Interfaces:**
- Produces: `ArmIdentity(name, serial, role, interface=None)`
- Produces: `TeleopPairConfig(name, leader, follower, calibration_path)`
- Produces: `operator_target(interface) -> OperatorTarget`
- Produces: `follower_state(interface, identity, target, now_ns) -> PiperState`
- Produces: `require_aligned(operator, follower, max_joint_deg=15, max_gripper_mm=10)`
- Produces: `parse_firmware_version(value) -> tuple[int, int, int]`

- [ ] **Step 1: Write failing conversion and safety tests**

Use small fake SDK message objects with official fields. Assert 0.001-degree to radians, 0.001 A, 0.001 rad/s, 0.001 Nm, low-speed units/status, six-axis target conversion, four distinct serial validation, firmware ordering, and exact 15-degree/10-mm boundaries.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python -m pytest tests/piperx/test_sdk_teleop.py -q`

Expected: collection fails because `sdk_teleop` does not exist.

- [ ] **Step 3: Implement immutable value conversion and gates**

Keep all conversion functions independent of the concrete SDK class. Raise `TeleopSafetyError` for missing/nonfinite data, duplicate identities, old firmware, or unsafe alignment.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `python -m pytest tests/piperx/test_sdk_teleop.py -q`

Expected: all conversion and safety tests pass.

- [ ] **Step 5: Commit**

```powershell
git add robot_control/piperx/sdk_teleop.py tests/piperx/test_sdk_teleop.py
git commit -m "feat: define safe PiperX SDK teleop state"
```

### Task 2: Implement one-owner teleop and monitoring runtime

**Files:**
- Modify: `robot_control/piperx/sdk_teleop.py`
- Modify: `tests/piperx/test_sdk_teleop.py`
- Modify: `robot_control/piperx/web_monitor.py`
- Modify: `tests/piperx/test_web_monitor.py`

**Interfaces:**
- Produces: `StandaloneTeleopRuntime(pair_configs, estimators, hub, sdk_factory, interface_resolver, clock_ns, control_rate_hz, ui_rate_hz)`
- Implements: `start()`, `stop()`, `status()`, `snapshot()`, `healthy()`, `workers`, and `hub` for `create_app`
- SDK factory contract: `factory(interface, judge_flag=True, can_auto_init=True)`

- [ ] **Step 1: Add failing coordinator tests**

Fake four SDK interfaces and assert: each is constructed once, `ConnectPort(piper_init=False)` is used, firmware query/role/mode/enable ordering is safe, leader commands reach only the paired follower, gripper effort is forced to 1000, the same follower SDK getter data produces Web snapshots, and stopping disconnects every SDK interface.

- [ ] **Step 2: Add failing watchdog and lifecycle tests**

Advance an injected clock to 250 ms and 1 s. Assert the coordinator holds the last target without inventing motion, then reports an error and stops. Assert any SDK/control exception makes health false and closes all four interfaces.

- [ ] **Step 3: Run focused tests and verify RED**

Run: `python -m pytest tests/piperx/test_sdk_teleop.py tests/piperx/test_web_monitor.py -q`

Expected: failures identify missing coordinator/runtime behavior.

- [ ] **Step 4: Implement the minimal shared runtime**

Resolve serials without changing link state. Open and connect all four SDK objects, call only the firmware query plus explicit role/mode/enable/control methods, align before enabling, control at 200 Hz, estimate/publish at 25 Hz, and expose two arm views to the existing read-only HTTP application.

- [ ] **Step 5: Run focused and full PiperX tests**

```powershell
python -m pytest tests/piperx/test_sdk_teleop.py tests/piperx/test_web_monitor.py -q
python -m pytest tests/piperx -q
```

Expected: all pass with only existing platform skips.

- [ ] **Step 6: Commit**

```powershell
git add robot_control/piperx/sdk_teleop.py robot_control/piperx/web_monitor.py tests/piperx/test_sdk_teleop.py tests/piperx/test_web_monitor.py
git commit -m "feat: share PiperX teleop feedback with dashboard"
```

### Task 3: Add CLI, lifecycle wrapper, and truthful UI mode

**Files:**
- Create: `scripts/piperx_teleop_web.py`
- Create: `scripts/open_piperx_teleop_monitor.ps1`
- Create: `tests/piperx/test_teleop_web_cli.py`
- Modify: `web/piperx_monitor/index.html`
- Modify: `web/piperx_monitor/app.js`
- Modify: `docs/piperx_external_torque.md`

**Interfaces:**
- CLI: repeated `--pair NAME,LEADER_SERIAL,FOLLOWER_SERIAL,CALIBRATION`, `--urdf`, `--speed-ratio 10`, `--gripper-effort 1000`, `--control-rate 200`, `--ui-rate 25`, loopback host/port
- PowerShell: `open_piperx_teleop_monitor.ps1 -LocalPort 8765 -RemotePort 18765`

- [ ] **Step 1: Write failing CLI parsing and safety tests**

Assert exactly two named pairs, four distinct serials, numeric ranges, loopback-only bind, and command generation with the direct Evo-RL Python path. Assert the launcher checks `idle`, stops EvoStudio, restores it in cleanup, and contains no password.

- [ ] **Step 2: Run CLI tests and verify RED**

Run: `python -m pytest tests/piperx/test_teleop_web_cli.py -q`

Expected: fail because the CLI does not exist.

- [ ] **Step 3: Implement CLI and PowerShell wrapper**

Keep the SSH window visible for password/sudo prompts. The remote shell installs a cleanup trap, stops only `evostudio-client`, starts Python directly, and starts `evostudio-client` again on every normal/error/interrupt exit.

- [ ] **Step 4: Update UI mode copy and documentation**

Show `普通遥操 / CONTROL + MONITOR` when runtime status says `standalone_teleop`; retain `无 / RX ONLY` for the original monitor. Explain that closing the combined SSH window stops control/Web and restores EvoStudio.

- [ ] **Step 5: Run CLI, Web, and full regression tests**

```powershell
python -m pytest tests/piperx/test_teleop_web_cli.py tests/piperx/test_web_monitor.py -q
node --test tests/web/piperx_monitor_core.test.mjs
python -m pytest tests/piperx -q
git diff --check
```

Expected: all pass.

- [ ] **Step 6: Commit**

```powershell
git add scripts/piperx_teleop_web.py scripts/open_piperx_teleop_monitor.ps1 tests/piperx/test_teleop_web_cli.py web/piperx_monitor docs/piperx_external_torque.md
git commit -m "feat: launch standalone PiperX teleop dashboard"
```

### Task 4: Deploy and perform guarded hardware verification

**Files:**
- Modify: `/home/dell/piperx-force-validation/*`
- Modify: `docs/piperx_dashboard_hardware_validation_2026-08-11.md`

**Interfaces:**
- Remote process: direct `/home/dell/anaconda3/envs/evo-rl/bin/python scripts/piperx_teleop_web.py ...`
- Local access: `127.0.0.1:8765 -> .166:127.0.0.1:18765`

- [ ] **Step 1: Run local final tests and deploy exact files**

Upload only changed application/test/Web files, write the deployed commit, and run remote pytest with the Evo-RL interpreter.

- [ ] **Step 2: Verify the control guard before motion**

Confirm EvoStudio phase is `idle`, stop its service, verify no prior CAN raw receiver or `piperx_torque_web` process remains, verify four serial mappings, and record CAN counters.

- [ ] **Step 3: Start with aligned stationary arms**

Start the combined service on loopback. It must fail closed if firmware, role, feedback, or alignment gates do not pass. Do not override a failed gate.

- [ ] **Step 4: Validate Web and low-amplitude operator motion**

Check health/status/snapshot/SSE, then let the on-site user move each leader slowly through a small range. Confirm correct side follows and Web q/qd/current changes synchronously. Stop immediately on side swap, jump, stale warning, or driver fault.

- [ ] **Step 5: Verify teardown and restore EvoStudio**

Exit the SSH session, confirm no combined process/18765 listener remains, no new `rx_dropped`, and `evostudio-client` is active and reports `idle`/hardware ready.

- [ ] **Step 6: Record evidence and commit**

```powershell
git add docs/piperx_dashboard_hardware_validation_2026-08-11.md
git commit -m "docs: validate standalone PiperX teleop dashboard"
```

## Plan Self-Review

- The design's one-owner CAN requirement maps to Tasks 1–2.
- Control exclusivity, direct-Python lifecycle, and truthful UI labeling map to Task 3.
- Firmware, alignment, watchdog, interface, and restoration gates are covered by tests before hardware use.
- EvoStudio source remains read-only; only its running systemd service is stopped and restored during the explicitly non-EvoStudio session.
- No placeholder or compatibility path is included.
