# PiperX extrapolated torque visualization design

## Goal

Keep calibration validity fail-closed while allowing operators to inspect the
trend of a finite external-joint-torque extrapolation during normal Evo-RL
teleoperation. The electrical comparison is a current-derived joint-torque
proxy, not an independent ground-truth torque sensor.

## Chosen approach

The estimator and calibration rules remain unchanged. The monitoring payload
will preserve a finite `tau_external_nm` result even when `valid` is false due
only to `outside_calibrated_workspace`. Nonfinite, stale, incomplete, dynamics,
or calibration-mismatch results remain unavailable as `null` values.

The existing industrial dashboard will render the preserved estimate as an
amber/cyan dashed **EXTRAPOLATED** trace and continue to render
`tau_effort_nm` as the red **current-derived proxy**. The status, snapshot
table, alarm panel, and instantaneous value will say `EXTRAPOLATED`, never
`VALID`. Rolling Pearson correlation will use finite paired samples regardless
of workspace validity and will be labelled as trend-only evidence.

Changing `valid` to true was rejected because it would erase the distinction
between an in-workspace estimate and extrapolation. Adding Cartesian wrench
estimation was rejected for this iteration because it requires Jacobian/frame
and force-ground-truth validation beyond the requested joint-torque trend view.

## Data flow and safety

1. `ExternalTorqueEstimator` computes the same finite joint-torque residual it
   already computes: `tau_model + tau_bias - tau_effort`.
2. `serialize_snapshot` exposes that finite result independently of the
   calibration-validity boolean.
3. The browser classifies each sample as valid, extrapolated, or unavailable.
4. Charts and correlation accept valid and extrapolated finite samples;
   recording, offline validation, and LeRobot data remain unchanged.

The dashboard must explicitly state that correlation measures shared trend and
does not establish absolute accuracy or independent ground truth. Values stay
in joint torque units (`N·m`); the user's `2–5 N` Cartesian-force tolerance is
not converted or claimed here.

## Error handling

- `outside_calibrated_workspace` plus finite external torque: display as
  extrapolated with an amber warning.
- Any other invalid reason, or any nonfinite external torque: show unavailable
  and break the chart trace.
- Driver faults and stale CAN telemetry retain their existing error severity.

## Tests and acceptance

- Python serializer test: a finite, workspace-invalid estimate keeps all six
  external-torque numbers while retaining `valid=false` and its reason.
- Python serializer test: nonfinite invalid estimates still serialize as
  `null`.
- JavaScript tests: display classification distinguishes valid,
  workspace-extrapolated, and unavailable samples; correlation accepts finite
  extrapolated pairs after the existing minimum sample count.
- Existing Python and browser test suites remain green.
- Live acceptance: while official PR #33/#34 teleoperation runs, the Web page
  shows numeric extrapolated torque and a trend correlation without changing
  the teleoperation process or writing data.
