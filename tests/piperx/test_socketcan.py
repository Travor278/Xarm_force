from __future__ import annotations

from pathlib import Path
import struct

import pytest

from robot_control.piperx.socketcan import (
    CanDiscoveryError,
    CanReceiveError,
    PiperStateAssembler,
    ReadOnlySocketCan,
    discover_interface,
)


def _net_tree(tmp_path: Path, *names: str) -> Path:
    root = tmp_path / "net"
    for name in names:
        (root / name).mkdir(parents=True)
    return root


def test_discovery_matches_stable_short_serial(tmp_path):
    root = _net_tree(tmp_path, "can0", "can2", "lo")
    properties = {
        "can0": {"ID_SERIAL_SHORT": "leader"},
        "can2": {"ID_SERIAL_SHORT": "follower", "ID_SERIAL": "vendor_follower"},
        "lo": {},
    }

    interface = discover_interface(
        "follower", root, lambda path: properties[path.name]
    )

    assert interface == "can2"


@pytest.mark.parametrize(
    ("properties", "message"),
    [
        ({"can0": {"ID_SERIAL_SHORT": "other"}}, "not found"),
        (
            {
                "can0": {"ID_SERIAL_SHORT": "same"},
                "can1": {"ID_SERIAL_SHORT": "same"},
            },
            "multiple",
        ),
    ],
)
def test_discovery_rejects_missing_or_duplicate_serial(tmp_path, properties, message):
    root = _net_tree(tmp_path, *properties)

    with pytest.raises(CanDiscoveryError, match=message):
        discover_interface("same", root, lambda path: properties[path.name])


class _ReceiveSocket:
    def __init__(self, packet: bytes):
        self.packet = packet
        self.bound = None
        self.timeout = None
        self.closed = False

    def bind(self, address):
        self.bound = address

    def settimeout(self, timeout):
        self.timeout = timeout

    def recv(self, size):
        assert size == 16
        return self.packet

    def close(self):
        self.closed = True

    def send(self, *_args):
        raise AssertionError("receive-only reader attempted CAN transmission")

    sendto = send
    sendmsg = send


def test_socketcan_reader_unpacks_one_frame_without_transmitting():
    packet = struct.pack("=IB3x8s", 0x251, 8, bytes.fromhex("03e8000100000000"))
    fake = _ReceiveSocket(packet)
    factory_args = []

    def factory(*args):
        factory_args.append(args)
        return fake

    reader = ReadOnlySocketCan(
        "can2", timeout_s=0.25, socket_factory=factory, clock_ns=lambda: 123456
    )
    frame = reader.recv_frame()
    reader.close()

    assert frame.can_id == 0x251
    assert frame.payload == bytes.fromhex("03e8000100000000")
    assert frame.timestamp_ns == 123456
    assert fake.bound == ("can2",)
    assert fake.timeout == 0.25
    assert fake.closed
    assert len(factory_args) == 1
    assert not hasattr(reader, "send")


def test_socketcan_reader_rejects_truncated_kernel_frame():
    reader = ReadOnlySocketCan(
        "can2", socket_factory=lambda *_: _ReceiveSocket(bytes(15))
    )

    with pytest.raises(CanReceiveError, match="16 bytes"):
        reader.recv_frame()


@pytest.mark.parametrize("flag", [0x80000000, 0x40000000, 0x20000000])
def test_socketcan_reader_ignores_extended_rtr_and_error_frames(flag):
    packet = struct.pack("=IB3x8s", flag | 0x251, 8, bytes(8))
    reader = ReadOnlySocketCan(
        "can2", socket_factory=lambda *_: _ReceiveSocket(packet), clock_ns=lambda: 99
    )

    frame = reader.recv_frame()

    assert frame.can_id == -1
    assert frame.payload == b""
    assert frame.timestamp_ns == 99


def _position_payload(first_mdeg: int, second_mdeg: int) -> bytes:
    return struct.pack(">ii", first_mdeg, second_mdeg)


def _motor_payload(speed_raw: int, current_raw: int) -> bytes:
    return struct.pack(">hhi", speed_raw, current_raw, 0)


def _feed_complete_cycle(assembler, start_ns: int):
    frames = [
        (0x2A5, _position_payload(1000, 2000)),
        (0x2A6, _position_payload(3000, 4000)),
        (0x2A7, _position_payload(5000, 6000)),
        *((0x251 + index, _motor_payload(100 + index, -1000 + index)) for index in range(6)),
    ]
    state = None
    for offset, (can_id, payload) in enumerate(frames):
        emitted = assembler.update(can_id, payload, start_ns + offset)
        if emitted is not None:
            state = emitted
    return state


def test_assembler_emits_only_after_every_feedback_component_advances():
    assembler = PiperStateAssembler("can2", "serial-left", max_skew_ns=20)
    frames = [
        (0x2A5, _position_payload(1000, 2000)),
        (0x2A6, _position_payload(3000, 4000)),
        (0x2A7, _position_payload(5000, 6000)),
        *((0x251 + index, _motor_payload(100 + index, 10 + index)) for index in range(6)),
    ]

    for offset, (can_id, payload) in enumerate(frames[:-1]):
        assert assembler.update(can_id, payload, 1000 + offset) is None

    state = assembler.update(frames[-1][0], frames[-1][1], 1008)

    assert state is not None
    assert state.interface == "can2"
    assert state.adapter_serial == "serial-left"
    assert state.complete
    assert state.fresh
    assert state.reason is None
    assert state.timestamp_ns == 1008
    assert state.q_rad[0] == pytest.approx(0.017453292519943295)
    assert state.qd_rad_s == pytest.approx([0.1, 0.101, 0.102, 0.103, 0.104, 0.105])
    assert assembler.update(0x251, _motor_payload(200, 20), 1010) is None


def test_auxiliary_feedback_enriches_state_without_blocking_emission():
    assembler = PiperStateAssembler(
        "can2", "serial-left", max_skew_ns=20, auxiliary_stale_after_ns=50
    )
    assembler.update(0x264, bytes.fromhex("01f4002d2c40007b"), 1)
    assembler.update(0x155, _position_payload(1000, 2000), 2)
    assembler.update(0x156, _position_payload(3000, 4000), 3)
    assembler.update(0x157, _position_payload(5000, 6000), 4)
    assembler.update(0x4AF, b"S-V1.8-2", 5)
    assembler.update(0x2A8, bytes.fromhex("000088b8007bc100"), 6)

    state = _feed_complete_cycle(assembler, 10)

    assert state is not None
    assert state.current_a == pytest.approx([-1.0, -0.999, -0.998, -0.997, -0.996, -0.995])
    assert state.driver[3] is not None
    assert state.driver[3].voltage_v == pytest.approx(50.0)
    assert state.driver[3].foc_temp_c == 45
    assert state.driver[3].motor_temp_c == 44
    assert state.driver[3].status_code == 0x40
    assert state.driver[3].bus_current_a == pytest.approx(0.123)
    assert state.driver_fresh[3]
    assert state.q_command_rad == pytest.approx(
        [0.0174532925, 0.0349065850, 0.0523598776,
         0.0698131701, 0.0872664626, 0.1047197551]
    )
    assert state.command_fresh
    assert state.firmware == "S-V1.8-2"
    assert state.gripper is not None
    assert state.gripper.travel_mm == pytest.approx(35.0)
    assert state.gripper.torque_nm == pytest.approx(0.123)
    assert state.gripper.status_code == 0xC1
    assert state.gripper_fresh


def test_slow_auxiliary_feedback_becomes_stale_without_blocking_next_state():
    assembler = PiperStateAssembler(
        "can2", "serial-left", max_skew_ns=20, auxiliary_stale_after_ns=50
    )
    assembler.update(0x261, bytes.fromhex("01f4002520400000"), 1)
    for can_id in (0x155, 0x156, 0x157):
        assembler.update(can_id, bytes(8), 1)
    assert _feed_complete_cycle(assembler, 10) is not None

    state = _feed_complete_cycle(assembler, 100)

    assert state is not None
    assert not state.driver_fresh[0]
    assert not state.command_fresh


def test_missing_auxiliary_feedback_is_explicitly_unavailable():
    state = _feed_complete_cycle(
        PiperStateAssembler("can2", "serial-left", max_skew_ns=20), 10
    )

    assert state is not None
    assert state.driver == (None,) * 6
    assert state.driver_fresh == (False,) * 6
    assert state.q_command_rad is None
    assert not state.command_fresh
    assert state.firmware is None
    assert state.gripper is None
    assert not state.gripper_fresh


def test_stale_gripper_feedback_does_not_block_arm_state():
    assembler = PiperStateAssembler(
        "can2", "serial-left", max_skew_ns=20, auxiliary_stale_after_ns=50
    )
    assembler.update(0x2A8, bytes.fromhex("000088b8007b4000"), 1)

    state = _feed_complete_cycle(assembler, 100)

    assert state is not None
    assert state.gripper is not None
    assert not state.gripper_fresh


def test_assembler_reports_windowed_rate_instead_of_last_interval_jitter():
    assembler = PiperStateAssembler("can2", "serial-left", max_skew_ns=20)
    starts = [1_000_000_000, 1_004_000_000, 1_009_000_000, 1_015_000_000]

    for start in starts:
        state = _feed_complete_cycle(assembler, start)

    assert state is not None
    assert state.frequency_hz == pytest.approx(200.0, rel=1e-6)


def test_assembler_waits_for_a_coherent_set_after_excessive_skew():
    assembler = PiperStateAssembler("can2", "serial-left", max_skew_ns=10)
    for index, can_id in enumerate((0x2A5, 0x2A6, 0x2A7)):
        assembler.update(can_id, bytes(8), index)
    for can_id in range(0x251, 0x257):
        assert assembler.update(can_id, bytes(8), 100) is None

    for can_id in (0x2A5, 0x2A6, 0x2A7):
        assembler.update(can_id, bytes(8), 200)
    for index, can_id in enumerate(range(0x251, 0x257)):
        state = assembler.update(can_id, bytes(8), 201 + index)

    assert state is not None
    assert state.fresh


def test_snapshot_marks_last_state_stale_without_mutating_measurements():
    assembler = PiperStateAssembler(
        "can2", "serial-left", max_skew_ns=20, stale_after_ns=50
    )
    for can_id in (0x2A5, 0x2A6, 0x2A7):
        assembler.update(can_id, bytes(8), 100)
    for can_id in range(0x251, 0x257):
        state = assembler.update(can_id, bytes(8), 100)

    stale = assembler.snapshot(151)

    assert state is not None
    assert stale is not None
    assert not stale.fresh
    assert stale.reason == "stale_feedback"
    assert stale.q_rad == pytest.approx(state.q_rad)
