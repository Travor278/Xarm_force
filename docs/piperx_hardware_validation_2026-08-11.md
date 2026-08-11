# PiperX Hardware Validation — 2026-08-11

## Scope and outcome

The receive-only estimator was deployed to
`/home/dell/piperx-force-validation` on the PiperX host and tested while the
production EvoStudio client remained active. Static repeatability passed the
approved thresholds for both follower arms. Free-motion accuracy and absolute
known-load accuracy remain untested because no operator-driven motion or
measured physical load was available during this session.

These results do **not** authorize EvoStudio dataset integration yet.

## Identity

| Arm | Adapter serial | Discovered interface |
|---|---|---|
| left follower | `004B00204148570D20343133` | `can2` |
| right follower | `003F002D4148571320343133` | `can3` |

URDF SHA-256 for both artifacts:

```text
d6dc6d31b0fa56341b2323fee5e8a375c2500ae6fb02e31fb52e8e917b608439
```

Calibration artifact SHA-256:

```text
left  21c24da4e4dae5b2a3ed21b95c5f9f0da6423b5b722446f6b421989f066359f9
right 8a418dc6f043a24d7b21cec79cd61bb76582bdfe71bcbe67b44f76fb902d6ff8
```

## Safety and rate evidence

- Remote test suite: 66 passed, 0 failed, 0 skipped.
- EvoStudio service before and after: `active`.
- EvoStudio main PID before and after: `324139`.
- EvoStudio restart count before and after: `0`.
- Initial three-second CAN RX increments: `can2=9428`, `can3=9423`.
- Final three-second CAN RX increments: `can2=9438`, `can3=9427`.
- Final three-second RX-drop increments: `can2=0`, `can3=0`.
- Recorder complete-state rate:
  - left calibration: `199.9995 Hz`;
  - right calibration: `200.0020 Hz`;
  - left independent validation: `200.0016 Hz`;
  - right independent validation: `200.0008 Hz`.
- The estimator source audit found no CAN send, arm-enable, motion-control,
  gripper-control, or MIT-torque command path.

## Data

Each arm was observed concurrently for 30 seconds at the current unloaded,
stationary pose for fitting and then for a separate 15-second validation
interval. The tool did not command motion.

| Arm/log | Complete samples | Finite valid samples |
|---|---:|---:|
| left calibration | 6000 | 5998 |
| right calibration | 6000 | 5998 |
| left validation | 3000 | 2998 |
| right validation | 3000 | 2998 |

Validation NPZ SHA-256:

```text
left  1abc98497842399b8a60748148dff8bce7d514699e505be15571a7e3d09f11de
right b3d118e33b68e3adaee146052a30f1f50321a85490bce204d1688699f1ce11fa
```

## Raw model residual

Before per-arm bias calibration, `tau_model - tau_measured` had the following
stationary signed mean (Nm):

```text
left  [0.0270, 2.1425, -2.0574, 0.6221, 0.0216, -0.0199]
right [0.0745, 2.2406, -2.0513, 0.7002, 0.0701, -0.0013]
```

This proves a raw URDF subtraction is not acceptable on these arms,
particularly for joints 2, 3, and 4, and that an xArm-specific fixed bias must
not be copied to PiperX.

## Independent static validation

Thresholds were MAE <= 0.30 Nm on joints 2/3 and <= 0.15 Nm on the other
joints.

### Left follower

```text
MAE      [0.005858, 0.003543, 0.003616, 0.003676, 0.004349, 0.003877] Nm
p95 abs  [0.014114, 0.008740, 0.009366, 0.009761, 0.012921, 0.010818] Nm
```

Result: pass. Scored samples: 2998. Outside-workspace samples: 0.

### Right follower

```text
MAE      [0.002701, 0.002439, 0.004052, 0.004871, 0.005096, 0.003995] Nm
p95 abs  [0.007046, 0.006059, 0.010708, 0.012546, 0.016283, 0.013866] Nm
```

Result: pass. Scored samples: 2995. Three of 2998 source-valid samples were
outside the calibrated workspace and were correctly excluded.

## Limits and remaining gates

The observed joint-position span during calibration was 0 degrees on all six
joints. Consequently, these artifacts establish only same-pose static
repeatability. They must not be treated as general workspace calibrations.

Before EvoStudio is changed, both arms still require:

1. operator-driven, explicitly no-contact trajectories covering the intended
   teleoperation workspace;
2. held-out free-motion validation with per-joint p95 absolute error <= 0.50
   Nm; and
3. a measured load/force and lever-arm validation with error <= max(0.30 Nm,
   15% of expected torque) and >= 95% direction agreement.

The CLI now records known-load intervals with both `--label known_load` and
`--confirm-known-load`, which prevents those samples from being accepted by
the no-contact calibration command.
