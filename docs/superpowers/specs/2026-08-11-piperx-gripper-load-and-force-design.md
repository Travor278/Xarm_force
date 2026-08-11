# PiperX gripper payload and fingertip-force design

## Goal

Improve J2-J5 external-torque estimation by including the installed Piper gripper as an end payload, and expose receive-only gripper travel, feedback torque, health, and calibrated fingertip force in the existing real-time monitor.

The implementation stays inside `arm_force`; it does not modify EvoStudio or its LeRobot dataset path.

## Evidence and constraints

- Piper's official six-axis model currently used by the estimator omits the gripper.
- The official `piper_ros` model at commit `ac41fcbcdda598f01b51cf6175ed9a24d0dacadc` describes a 0.45 kg gripper base and two 0.025 kg fingers, fixed to `link6` through the gripper base.
- The official 0x2A8 feedback frame reports signed gripper travel, signed feedback torque in 0.001 N.m, and an eight-bit health/status byte. It does not report gripper current.
- Motor feedback torque is not an independently calibrated fingertip force. A force value in N is valid only after empirical calibration against known force.
- The existing estimator and datasets use exactly six arm joints. The gripper must not become a seventh arm joint.

## Options considered

### 1. Replace the arm URDF with the full eight-joint gripper URDF

Rejected. This changes `nq`/`nv`, breaks the six-joint estimator contract, and introduces two passive symmetric finger coordinates that are irrelevant to arm inverse dynamics.

### 2. Add only the 0.50 kg mass at the flange origin

Rejected. This improves static load but loses the approximately 37.6 mm Z center-of-mass offset and the official inertia tensor, leaving avoidable J2-J5 model error.

### 3. Attach an equivalent rigid payload to J6

Selected. Aggregate the official base and finger inertias in the `link6` frame and append the resulting rigid inertia to joint 6 in Pinocchio. This preserves six arm coordinates while correcting gravity and inertial terms.

The two equal fingers move symmetrically, so their combined center of mass is invariant with opening. Their combined inertia varies slightly with opening. Use the 35 mm total-opening midpoint for the fixed equivalent inertia. This approximation preserves the exact mass and center of mass; the opening-dependent inertia difference is small compared with the gravity correction and can be revisited if hardware validation shows it matters.

## Payload artifact

Store the payload in a versioned JSON document containing:

- schema/version and payload name;
- source repository, source commit, and source model path;
- reference opening;
- mass, COM in `link6`, and symmetric 3x3 inertia about the COM in `link6`;
- a SHA-256 identity derived from canonical payload content.

Official-model aggregate at 35 mm total opening:

- mass: 0.500 kg;
- COM: `[-0.000165426446, 0.000072452984, 0.037590003400]` m;
- inertia about COM in `link6`, kg.m^2:

```
[[ 0.00124729527,  0.000000807077, -0.000007830484],
 [ 0.000000807077, 0.000997374239,  0.000000247302],
 [-0.000007830484, 0.000000247302, 0.000447116481]]
```

Validate positive mass, finite vectors, symmetry, and positive-definite inertia before changing the Pinocchio model. The runtime dynamics identity must include both the base URDF SHA-256 and payload SHA-256. An old no-payload residual calibration must fail closed instead of silently compensating for a different model.

## Gripper feedback model

Add a pure 0x2A8 decoder:

- bytes 0-3: signed big-endian int32 travel, 0.001 mm;
- bytes 4-5: signed big-endian int16 feedback torque, 0.001 N.m;
- byte 6: low voltage, motor overheat, driver overcurrent, driver overheat, sensor abnormal, driver error, enabled, homed;
- byte 7: reserved.

Treat gripper feedback as auxiliary telemetry. It must never block the 200 Hz coherent six-joint state emission. Each emitted arm state carries the latest gripper sample and freshness flag.

The SDK teleoperation path reads the same values from `GetArmGripperMsgs()` so the standalone teleop monitor and passive SocketCAN monitor expose one schema.

## Force calibration

Use empirical direct calibration from feedback torque to total fingertip clamping force instead of assuming an undocumented transmission ratio.

The first calibration model is affine and direction-aware:

```
F_N = max(0, slope_N_per_Nm * direction * (tau_feedback_Nm - zero_Nm) + intercept_N)
```

The artifact stores device identity, fitted coefficients, calibrated travel/torque ranges, sample count, validation MAE, and creation metadata. A reported force is valid only when:

- the calibration artifact is loaded and matches the configured gripper/arm;
- the 0x2A8 sample is fresh;
- the gripper is enabled and no voltage, temperature, overcurrent, sensor, or driver fault bit is set;
- travel and torque lie inside calibrated ranges.

Outside those conditions the monitor still shows raw travel and feedback torque, but force is `null` with an explicit reason. The calibration CLI accepts recorded known-force points so a force gauge or scale can provide ground truth without changing teleoperation control.

## Monitor schema and UI

Bump the monitor schema and add a `gripper` object with travel, feedback torque, decoded status, freshness, force value, force validity, calibration status, and reason.

Add a dedicated gripper panel rather than pretending the gripper is arm joint J7. Show:

- live opening in mm;
- feedback motor torque in N.m;
- calibrated total fingertip force in N, or `--` with the reason;
- status chips for enabled/homed/fault/fresh;
- synchronized torque and force trend traces.

The existing J1-J6 charts and receive-only guarantees remain unchanged.

## Safety and verification

- No CAN transmit call is added to passive monitoring.
- Unit tests cover exact signed frame decoding, stale/fault validity, payload validation, dynamics identity, calibration fitting/range rejection, strict JSON, and browser presentation.
- Remote acceptance is receive-only: observe 0x2A8 on both CAN buses, deploy the updated monitor, and verify that arm feedback remains near 200 Hz while gripper telemetry updates.
- Fingertip force remains labelled uncalibrated until physical known-force samples are collected. Payload integration can be validated immediately through static-pose torque residual comparisons before and after adding the 0.50 kg equivalent payload.
