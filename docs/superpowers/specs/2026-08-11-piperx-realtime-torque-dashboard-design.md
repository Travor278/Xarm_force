# PiperX Realtime Torque Dashboard Design

## Objective

Build a standalone, receive-only dashboard for the two PiperX follower arms.
The monitor runs on `dell@192.168.105.166`, reads the already active
SocketCAN interfaces, computes the existing external-joint-torque estimate,
and serves a loopback-only web application. An SSH local-forward carries the
HTTP event stream to a browser on the Windows workstation. EvoStudio remains
unchanged and can run teleoperation concurrently.

The dashboard is a diagnostic instrument, not a metrology substitute. Piper's
SDK `effort` is current multiplied by a fixed coefficient; it is not an
independent joint-torque sensor. The UI must therefore label it
`电流换算力矩（参考）`, label the model output `外力矩估计`, and never describe
their correlation as absolute accuracy.

## Selected Architecture

Use a remote FastAPI/uvicorn service with a server-sent-events (SSE) stream and
static HTML/CSS/JavaScript assets. SSE is one-way by construction, reconnects
natively in browsers, and is sufficient because the web application has no
control action. The service binds to `127.0.0.1:8765`; the documented access
path is:

```text
PiperX CAN feedback -> read-only SocketCAN -> estimator on .166
                    -> loopback HTTP/SSE -> SSH local forward
                    -> http://127.0.0.1:8765 on Windows
```

Alternatives rejected for this version:

- WebSocket plus a local relay adds bidirectional behavior and another
  process without a monitoring requirement that needs it.
- Streaming JSON over SSH stdout makes reconnect, health endpoints, multiple
  clients, and static asset delivery unnecessarily brittle.

## Safety Boundary

- Reuse `ReadOnlySocketCan`; do not import or instantiate Piper SDK control
  interfaces.
- Do not expose `send`, `sendto`, `sendmsg`, CAN configuration, arm enable,
  motion, role switching, gripper, MIT torque, or parameter-query operations.
- Extend the source-level receive-only audit to include the web-monitor code.
- Bind to loopback by default and reject non-loopback binding unless an
  explicit `--allow-network-bind` flag is present.
- Serve no HTTP route that mutates robot, estimator, or process state.
- Treat stale/incomplete/out-of-calibration-workspace samples as invalid and
  keep their reason visible rather than carrying the last valid estimate
  forward.

## Piper Feedback Semantics

Consume the existing feedback and additionally decode the following passive
frames:

- `0x251..0x256`: joint velocity, signed motor current, and SDK-style
  current-derived effort;
- `0x261..0x266`: driver voltage, FOC temperature, motor temperature, driver
  status bits, and bus current;
- `0x2A5..0x2A7`: six joint positions;
- `0x155..0x157`, when visible during master/slave teleoperation: commanded
  follower joint positions, shown as optional tracking error and never
  required for estimator validity;
- `0x4AF`, when another process requests firmware information: passively
  accumulate and display the firmware string. The monitor must not transmit a
  firmware query.

Piper's official Q&A states that firmware `1.8-2` and earlier needs an
additional four-times correction for J1-J3 feedback torque. Until a firmware
string is observed or supplied explicitly, expose the current-derived effort
as `firmware_scale_verified=false`. Do not silently alter the estimator's
calibration artifact; show a prominent warning if the firmware is unknown or
known to be legacy.

## Remote Components

### Protocol and state enrichment

Pure protocol functions decode high-speed current, low-speed driver telemetry,
optional joint commands, and passive firmware fragments. The state assembler
continues to emit coherent estimator states based only on position and
high-speed revisions. Auxiliary fields use their most recent values and carry
freshness timestamps so slow telemetry never blocks the 200 Hz state path.

### Snapshot serialization

A focused serializer converts one estimator output and its matching Piper
state into finite, versioned JSON. Each sample contains:

- schema version, arm name, adapter serial, interface, and monotonic timestamp;
- validity, calibration state, invalid reason, measured frequency, and stream
  sequence;
- six values each for position, filtered velocity, raw current,
  current-derived effort, model torque, bias, and estimated external torque;
- optional six-value joint command and tracking error;
- six motor temperatures, FOC temperatures, voltage, bus current, and raw
  driver status bytes;
- firmware string and firmware-scale verification state.

JSON uses `null`, never non-standard `NaN`, for unavailable values.

### Acquisition and event hub

Run one receive thread per follower. Each thread owns one SocketCAN socket, one
state assembler, and one estimator. The thread consumes every CAN frame,
publishes coherent samples at a configurable UI rate (default 50 Hz), and
continues to update slow telemetry between samples.

A bounded latest-value event hub gives every SSE client an independent queue.
Slow clients drop old samples and receive the newest state; they must never
back-pressure CAN acquisition. A heartbeat event is sent during idle periods.

### HTTP application and CLI

`scripts/piperx_torque_web.py` accepts repeated
`--arm NAME,SERIAL,CALIBRATION`, the URDF and base orientation, host, port, UI
rate, and optional firmware version. Routes are read-only:

- `/` and `/assets/*`: dashboard files;
- `/api/status`: service identity, arm configuration, and latest health;
- `/api/snapshot`: latest samples for initial hydration;
- `/stream`: SSE telemetry stream;
- `/healthz`: process and acquisition liveness.

## Browser Experience

Adopt a dense industrial-test-bench aesthetic: graphite background, warm
instrument-paper panels, amber validity accents, cyan/vermillion traces,
tabular numerals, and restrained grid/noise texture. The memorable element is
a synchronized dual-arm strip-chart that feels like a hardware oscilloscope,
not a generic analytics dashboard.

The single page provides:

1. A top status rail showing SSH/stream connection, both CAN rates, sample age,
   calibration validity, firmware status, and driver alarms.
2. Left/right arm selection and J1-J6 selection, with keyboard shortcuts.
3. A 20-second synchronized canvas chart for estimated external torque,
   current-derived effort, and model torque. Traces have explicit units and
   independent visibility toggles.
4. Smaller aligned charts for joint angle, velocity, current, and optional
   command tracking error.
5. A six-joint numeric matrix containing the latest values, temperature,
   validity, and driver status for both arms.
6. A correlation panel that reports rolling Pearson correlation only when the
   window has enough variance and samples. It is titled `趋势相关性（非精度）`
   and includes the warning that shared current is not independent ground
   truth.
7. Pause/resume and time-window controls that affect only local presentation.

The browser keeps a bounded 60-second ring buffer, renders at animation-frame
rate, decimates visually when necessary, reconnects with backoff, and is usable
at 1280x720 and larger. No CDN or internet asset is required.

## Accuracy and Failure Presentation

- Invalid estimator samples break the torque line instead of drawing zero.
- Stale samples turn the arm status amber after 100 ms and red after 500 ms.
- Unknown firmware, legacy firmware, out-of-workspace calibration, high
  temperature, collision, driver error, overcurrent, stall, and disabled-driver
  bits remain visible until a newer clean sample arrives.
- Correlation is computed between current-derived effort and estimated external
  torque only as a diagnostic. Low variance returns `--`, and no pass/fail
  accuracy claim is generated.
- A future independent F/T sensor can be added as a new optional series without
  changing CAN acquisition or the estimator interface.

## Testing and Acceptance

- Protocol unit tests cover signed current, low-speed units/status bits,
  command position, malformed frames, and firmware fragments.
- State tests prove auxiliary telemetry does not block coherent state emission
  and becomes stale independently.
- Serialization tests prove stable field names, six-element arrays, `null`
  sanitization, and explicit firmware verification.
- Event-hub and FastAPI tests prove bounded drop-old behavior, SSE framing,
  loopback binding, health behavior, and read-only routes.
- Frontend JavaScript tests cover ring-buffer bounds, correlation edge cases,
  invalid gaps, and reconnect state without requiring live hardware.
- The receive-only audit scans every new Python source.
- Browser verification checks desktop layout, 1280x720 layout, reconnect and
  invalid-state rendering using synthetic telemetry.
- On `.166`, passive validation must show both adapter serials, no CAN RX-drop
  increase attributable to the monitor, current/temperature values matching
  direct frame decoding, and simultaneous operation without control traffic.

## Deployment and Operation

Deploy only into `/home/dell/piperx-force-validation`. Run in the existing
`evo-rl` Conda environment, using the existing per-arm calibration files and
URDF hash checks. Provide a systemd-independent launch command and PID/log
files under the deployment directory; do not install a persistent boot service
in this version.

The Windows-side documented flow is two terminals:

```powershell
ssh dell@192.168.105.166 "<start loopback monitor command>"
ssh -N -L 8765:127.0.0.1:8765 dell@192.168.105.166
```

Then open `http://127.0.0.1:8765`. The start command and an optional local
PowerShell helper must not embed a password.
