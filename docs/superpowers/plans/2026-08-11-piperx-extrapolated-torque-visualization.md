# PiperX Extrapolated Torque Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Display finite out-of-workspace joint-torque extrapolations beside the current-derived effort proxy without claiming calibration validity or independent ground truth.

**Architecture:** Preserve estimator behavior and expose finite values at the monitoring serialization boundary. Add a browser-only three-state classifier so charts, readouts, correlation, alarms, and the joint table consistently distinguish valid, extrapolated, and unavailable samples.

**Tech Stack:** Python 3.10, NumPy, pytest, ES modules, Node test runner, Canvas 2D, FastAPI static assets.

## Global Constraints

- `valid=false` and `reason=outside_calibrated_workspace` must remain unchanged for extrapolated samples.
- Stale, incomplete, nonfinite, dynamics-error, and calibration-mismatch estimates must remain unavailable.
- `tau_effort_nm` must be labelled as a current-derived proxy, not ground truth.
- All displayed torque values remain joint torques in `N·m`; do not claim Cartesian force accuracy in `N`.
- Do not change LeRobot recording or EvoStudio files.

---

### Task 1: Monitoring payload preserves finite workspace extrapolations

**Files:**
- Modify: `tests/piperx/test_monitoring.py`
- Modify: `robot_control/piperx/monitoring.py:96-106`

**Interfaces:**
- Consumes: `Estimate.tau_external_nm`, `Estimate.valid`, and `Estimate.reason`.
- Produces: existing JSON field `tau_external_nm: list[float | None]`; no schema rename.

- [ ] **Step 1: Write the failing serializer tests**

Change `_estimate` to accept explicit external values, then add one test with
`valid=False`, reason `outside_calibrated_workspace`, and six finite values that
expects those values in `payload["tau_external_nm"]`. Keep a second test with
`NaN`/`Inf` values that expects `null` for nonfinite entries.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python -m pytest tests/piperx/test_monitoring.py -q`

Expected: the finite workspace-invalid test fails because the serializer
currently returns six `None` values.

- [ ] **Step 3: Implement the serialization boundary**

Replace validity-gated serialization with finite sanitization:

```python
external = _finite_list(estimate.tau_external_nm)
```

Do not change the `valid`, `calibrated`, or `reason` fields.

- [ ] **Step 4: Verify GREEN and commit**

Run: `python -m pytest tests/piperx/test_monitoring.py -q`

Then commit:

```text
feat: expose finite torque extrapolations
```

### Task 2: Browser renders extrapolated trend without claiming validity

**Files:**
- Modify: `tests/web/piperx_monitor_core.test.mjs`
- Modify: `web/piperx_monitor/monitor-core.js`
- Modify: `web/piperx_monitor/app.js`
- Modify: `web/piperx_monitor/index.html`
- Modify: `web/piperx_monitor/styles.css`

**Interfaces:**
- Consumes: snapshot fields `valid`, `reason`, `tau_external_nm`, and `tau_effort_nm`.
- Produces: `estimateDisplayState(sample): "valid" | "extrapolated" | "unavailable"`.

- [ ] **Step 1: Write failing classification tests**

Add Node tests asserting:

```javascript
assert.equal(estimateDisplayState({ valid: true, tau_external_nm: [1] }), 'valid');
assert.equal(estimateDisplayState({ valid: false, reason: 'outside_calibrated_workspace', tau_external_nm: [1] }), 'extrapolated');
assert.equal(estimateDisplayState({ valid: false, reason: 'stale_feedback', tau_external_nm: [1] }), 'unavailable');
assert.equal(estimateDisplayState({ valid: false, reason: 'outside_calibrated_workspace', tau_external_nm: [null] }), 'unavailable');
```

- [ ] **Step 2: Run Node tests and verify RED**

Run: `node --test tests/web/piperx_monitor_core.test.mjs`

Expected: import failure because `estimateDisplayState` does not exist.

- [ ] **Step 3: Implement classification and consistent UI semantics**

Add `estimateDisplayState` to `monitor-core.js`. In `app.js`, use it to:

- render finite extrapolated torque points instead of requiring `sample.valid`;
- include finite extrapolated pairs in Pearson correlation;
- show `EXTRAPOLATED / 标定区外` in amber;
- show numeric instantaneous and table values with `EXTRAP` chips;
- report an amber extrapolation warning while retaining driver alarms;
- use a dashed external trace whenever the selected window contains only
  extrapolated samples.

Update copy in `index.html` to say `电流换算力矩代理` and update the stamp to
`TORQUE EXTRAPOLATED`. Add amber stamp/chip/legend styles without changing the
existing industrial visual language.

- [ ] **Step 4: Verify browser tests and commit**

Run: `node --test tests/web/piperx_monitor_core.test.mjs`

Then commit:

```text
feat: visualize extrapolated PiperX torque trends
```

### Task 3: Regression, deployment, and live acceptance

**Files:**
- Deploy only the files changed in Tasks 1 and 2 to `/home/dell/piperx-force-validation`.

**Interfaces:**
- Consumes: official Evo-RL PR #33/#34 teleoperation and passive SocketCAN monitor.
- Produces: live dashboard at `http://127.0.0.1:8765`.

- [ ] **Step 1: Run complete local verification**

Run:

```text
python -m pytest tests/piperx -q
node --test tests/web/piperx_monitor_core.test.mjs
git diff --check
```

- [ ] **Step 2: Deploy and run remote focused tests**

Copy the committed monitoring and Web files only. Run remote
`python -m pytest tests/piperx/test_monitoring.py -q` in the `evo-rl`
environment.

- [ ] **Step 3: Restart the official teleop plus passive monitor session**

Reuse `/home/dell/Evo-RL` PR #33/#34 `lerobot-teleoperate --fps=30` and the
receive-only `piperx_torque_web.py`; do not start `sdk_teleop.py`.

- [ ] **Step 4: Live acceptance**

For at least 20 seconds, verify API sequences advance, `valid` remains false
with `outside_calibrated_workspace`, `tau_external_nm` is finite, the Web page
shows `EXTRAPOLATED`, and rolling correlation becomes numeric for a moving
joint. Confirm both remote processes remain alive and CAN interfaces do not
enter bus-off.
