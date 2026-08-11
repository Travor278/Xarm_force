# PiperX Follower External Joint-Torque Estimation Design

## Goal

Add a standalone, read-only PiperX external joint-torque estimator to this
repository and validate it on the two follower arms connected to
`dell@192.168.105.166`. The estimator must be proven accurate enough before
any changes are made to `D:\Code\Work\evostudio-piperx` or to EvoStudio's
LeRobot dataset schema.

The target is the torque caused by contact between each follower arm and its
environment. Operator input torque on the leader arms is out of scope.

## Evidence and Constraints

- The original xArm implementation estimates joint torque, not a Cartesian
  force/wrench. Its active monitor subtracts a rigid-body model and hard-coded
  xArm firmware bias levels from the measured actuator torque.
- The production EvoStudio collector is the pure-Rust `evostudio-client`.
  Its PiperX path currently records six joint positions and one gripper
  position per arm. It does not decode motor effort.
- The deployed `.166` configuration resolves the adapters as follows:
  - left leader: `can0`, serial `0040002C4148570C20343133`
  - right leader: `can1`, serial `003900454148571320343133`
  - left follower: `can2`, serial `004B00204148570D20343133`
  - right follower: `can3`, serial `003F002D4148571320343133`
- Passive hardware probing showed that `can2` and `can3` publish joint,
  high-speed motor, and status feedback at about 200 Hz. Their SDK `effort`
  values are derived from motor current and reported in 0.001 Nm units.
- Leader-mode `can0` and `can1` do not provide the same high-speed effort
  stream. They are not estimator inputs.
- The existing EvoStudio source and its dirty worktree are reference-only for
  this phase. No file under `D:\Code\Work\evostudio-piperx` will be changed.

## Selected Approach

Use a hybrid estimator consisting of:

1. the Piper SDK's measured follower joint effort;
2. Pinocchio rigid-body inverse dynamics using the PiperX URDF;
3. per-arm, position-dependent residual calibration; and
4. bounded filtering and validity checks.

A pure URDF estimator is rejected because the initial passive sample had
roughly 2 Nm static residual on joints 2 and 3. A purely data-driven estimator
is rejected because it would require substantially more labeled load data and
would be harder to audit outside the calibrated workspace.

## Sign Convention and Model

The rigid-body equation is defined as:

```text
M(q) qdd + C(q, qd) qd + g(q) = tau_actuator + tau_external
```

The reported external torque is therefore:

```text
tau_external = tau_model(q, qd, qdd) + tau_bias(q, qd) - tau_measured
```

Positive external torque acts in the positive joint-coordinate direction.
The calibration term represents stable actuator-effort scale/offset error,
friction, mechanical balancing, and URDF mismatch. It is fitted only from
explicitly marked no-contact samples.

The implementation will keep all internal angles in radians, velocities in
radians per second, accelerations in radians per second squared, torques in
Nm, and timestamps in monotonic nanoseconds. Conversion from Piper protocol
units occurs once at the hardware boundary.

## Components

### Stable CAN discovery

Resolve follower SocketCAN names through the USB-CAN adapter serial numbers,
not hard-coded `can2` or `can3` names. The default serials may be supplied by a
local configuration file or command-line arguments, but discovered interface
names are always logged.

### Read-only Piper source

Attach to an already configured SocketCAN interface and consume feedback
without transmitting any CAN frame. In particular, the estimator must never
send firmware queries, role changes, enable commands, motion-mode commands,
joint commands, gripper commands, or MIT torque commands.

Each complete state contains:

- six joint positions;
- six motor velocities;
- six measured efforts;
- source timestamps/frequencies when available;
- the local monotonic receive timestamp; and
- freshness and completeness flags.

### Pinocchio dynamics backend

Load the PiperX no-gripper URDF, validate that it has six actuated joints, and
compute inverse dynamics with `rnea`. Base orientation is configurable and is
stored in calibration metadata. The backend exposes pure functions so it can
be tested without CAN hardware.

### State derivative/filter pipeline

Use measured motor velocity where available. Estimate acceleration with a
bounded filtered derivative of velocity. A sample is invalid during startup,
on stale/incomplete feedback, on an excessive timestamp gap, or when a
derivative spike exceeds configured physical limits. Invalid samples remain in
diagnostic logs but are not used for calibration or accuracy scoring.

### Per-arm calibration

Calibration records no-contact trajectories while the normal EvoStudio
leader/follower loop moves the follower through the intended workspace. The
estimator remains an independent passive listener.

Fit each arm independently. Use ridge regression over a compact feature set:

- constant term;
- `sin(q_i)` and `cos(q_i)`;
- adjacent/cumulative angle terms needed by the serial chain; and
- a small velocity-dependent friction term.

Training and validation samples are split by contiguous time/pose groups, not
random individual frames, to avoid overstating generalization. Calibration
artifacts include adapter serial, URDF hash, feature version, fit metrics,
workspace bounds, base orientation, and creation time. An artifact is rejected
if its serial or URDF hash does not match the runtime arm.

### Monitor, recorder, and replay evaluator

The CLI supports:

- passive live monitoring of one or both follower arms;
- raw no-contact calibration recording;
- fitting a calibration artifact;
- replaying a log through the estimator;
- reporting per-joint mean error, MAE, RMSE, standard deviation, and percentile
  errors; and
- recording a known-load validation interval and its expected joint torque.

CSV output contains raw and derived values. NPZ output preserves arrays and
metadata for deterministic offline analysis.

## Validation

Validation proceeds without modifying or restarting EvoStudio:

1. Run unit tests with synthetic Piper frames, dynamics outputs, filtering,
   calibration fitting, metadata mismatch, stale input, and sign convention.
2. Run an import/CLI smoke test in the `.166` `evo-rl` environment.
3. Attach passively to both follower CAN buses and verify approximately 200 Hz
   complete feedback while the EvoStudio service continues running.
4. Record no-contact calibration motion through the normal teleoperation path.
5. Evaluate held-out no-contact poses/motions.
6. Validate absolute magnitude and sign with a known external load applied at
   a measured lever arm, or with a calibrated force gauge.

Acceptance criteria are:

- stationary no-contact MAE at or below 0.30 Nm for joints 2 and 3;
- stationary no-contact MAE at or below 0.15 Nm for the other joints;
- 95th-percentile free-motion no-contact absolute error at or below 0.50 Nm;
- known-load error at or below the greater of 0.30 Nm and 15% of the expected
  torque;
- correct known-load direction for at least 95% of valid samples; and
- no feedback-rate degradation or control change attributable to the passive
  estimator.

No-contact validation proves residual rejection and repeatability, but it does
not prove absolute load accuracy. EvoStudio integration remains blocked until
the known-load requirement is completed.

## Failure Handling and Safety

- Fail closed on missing dependencies, URDF mismatch, stale feedback,
  incomplete frames, wrong adapter serial, or an out-of-workspace calibration.
- Never silently substitute a calibration from the other arm.
- Never update calibration bias online during an unlabelled run; doing so could
  absorb a real sustained contact force.
- Cap display/log values for presentation without clipping the raw stored
  measurement.
- Disconnect only the estimator's receive resources on exit. Do not change arm
  role, enable state, CAN link configuration, or pose.
- Do not expose or store SSH credentials.

## Future EvoStudio Dataset Contract

After hardware validation passes, design a separate EvoStudio change that
decodes effort and synchronizes calibrated follower external torque with each
dataset frame. The proposed fields are:

```text
complementary_info.left_follower_joint_torque_external
complementary_info.right_follower_joint_torque_external
complementary_info.follower_joint_torque_external_valid
```

Each torque vector has width six. Existing 14-dimensional
`observation.state` and `action` remain unchanged, preserving current policy
input/output schemas and dataset compatibility.

That integration is explicitly outside the current implementation phase.
