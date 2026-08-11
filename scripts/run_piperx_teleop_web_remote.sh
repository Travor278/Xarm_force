#!/usr/bin/env bash
set -u

deployment="/home/dell/piperx-force-validation"
python_bin="/home/dell/anaconda3/envs/evo-rl/bin/python"
urdf="/home/dell/Evo-RL.before-pr-sync/src/lerobot/assets/piper_x_description/urdf/piper_x_description_no_gripper.urdf"
payload="$deployment/config/piperx_gripper_payload.json"
remote_port="${1:-18765}"

case "$remote_port" in
    ''|*[!0-9]*) echo "invalid remote port: $remote_port" >&2; exit 2 ;;
esac
if (( remote_port < 1 || remote_port > 65535 )); then
    echo "invalid remote port: $remote_port" >&2
    exit 2
fi

status_json="$(curl -fsS http://127.0.0.1:8766/client/status)" || {
    echo "cannot read EvoStudio client status" >&2
    exit 2
}
phase="$(printf '%s' "$status_json" | "$python_bin" -c 'import json,sys; print(json.load(sys.stdin)["phase"])')"
if [[ "$phase" != "idle" ]]; then
    echo "EvoStudio phase is not \"idle\": $phase" >&2
    exit 2
fi

child_pid=""
cleanup() {
    exit_code=$?
    trap - EXIT HUP INT TERM
    if [[ -n "$child_pid" ]] && kill -0 "$child_pid" 2>/dev/null; then
        kill -TERM "$child_pid"
        wait "$child_pid" 2>/dev/null || true
    fi
    exit "$exit_code"
}
trap cleanup EXIT HUP INT TERM

session_pid="$$"
guard_unit="piperx-teleop-guard-${session_pid}"
sudo -v
sudo -n systemd-run --quiet --collect \
    --unit="$guard_unit" --property=Type=simple \
    /bin/bash -c '
        trap "systemctl start evostudio-client" EXIT
        systemctl stop evostudio-client
        while kill -0 "$1" 2>/dev/null; do sleep 1; done
    ' bash "$session_pid"

evostudio_stopped=false
for _attempt in {1..100}; do
    if ! systemctl is-active --quiet evostudio-client; then
        evostudio_stopped=true
        break
    fi
    sleep 0.1
done
if [[ "$evostudio_stopped" != true ]]; then
    echo "guard did not stop EvoStudio within 10 seconds" >&2
    exit 2
fi

if pgrep -f '[p]iperx_torque_web.py|[p]iperx_teleop_web.py' >/dev/null; then
    echo "another PiperX monitor or teleop process is already running" >&2
    exit 2
fi

for interface in can0 can1 can2 can3; do
    if ! ip link show dev "$interface" | head -n 1 | grep -q 'state UP'; then
        sudo -n ip link set dev "$interface" type can bitrate 1000000
        sudo -n ip link set dev "$interface" up
    fi
done

cd "$deployment"
"$python_bin" scripts/piperx_teleop_web.py \
    --urdf "$urdf" \
    --payload "$payload" \
    --pair left,0040002C4148570C20343133,004B00204148570D20343133,calibration/left.json \
    --pair right,003900454148571320343133,003F002D4148571320343133,calibration/right.json \
    --speed-ratio 10 --gripper-effort 1000 \
    --control-rate 200 --ui-rate 25 \
    --host 127.0.0.1 --port "$remote_port" &
child_pid=$!
wait "$child_pid"
