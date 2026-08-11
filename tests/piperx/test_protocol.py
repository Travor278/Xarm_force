import math

import pytest

from robot_control.piperx.protocol import (
    FirmwareFragment,
    FrameDecodeError,
    HighSpeedSample,
    JointCommandPair,
    LowSpeedSample,
    decode_frame,
    decode_high_speed,
    decode_joint_command,
    decode_low_speed,
    decode_position_pair,
)


def test_position_pair_converts_signed_millidegrees_to_radians():
    first_joint, positions = decode_position_pair(
        0x2A5, bytes.fromhex("000003e8fffff830")
    )

    assert first_joint == 0
    assert positions == pytest.approx((math.radians(1.0), math.radians(-2.0)))


def test_each_position_id_maps_to_its_first_joint():
    payload = bytes(8)

    assert decode_position_pair(0x2A5, payload)[0] == 0
    assert decode_position_pair(0x2A6, payload)[0] == 2
    assert decode_position_pair(0x2A7, payload)[0] == 4


@pytest.mark.parametrize(
    ("can_id", "coefficient"),
    [(0x251, 1.18125), (0x253, 1.18125), (0x254, 0.95844), (0x256, 0.95844)],
)
def test_high_speed_converts_signed_speed_and_current(can_id, coefficient):
    sample = decode_high_speed(can_id, bytes.fromhex("03e8fc1800000000"))

    assert sample == HighSpeedSample(
        joint_index=can_id - 0x251,
        velocity_rad_s=1.0,
        current_a=-1.0,
        effort_nm=-coefficient,
    )


def test_low_speed_converts_units_and_preserves_status_bits():
    sample = decode_low_speed(0x264, bytes.fromhex("01f4002d2cf0007b"))

    assert sample == LowSpeedSample(
        joint_index=3,
        voltage_v=50.0,
        foc_temp_c=45,
        motor_temp_c=44,
        status_code=0xF0,
        bus_current_a=0.123,
    )


def test_joint_command_converts_signed_millidegrees_to_radians():
    sample = decode_joint_command(0x155, bytes.fromhex("000003e8fffff830"))

    assert sample == JointCommandPair(
        first_joint=0,
        positions_rad=(math.radians(1.0), math.radians(-2.0)),
    )


def test_decode_frame_preserves_passive_firmware_fragment():
    assert decode_frame(0x4AF, b"S-V1.8-2") == FirmwareFragment(b"S-V1.8-2")


def test_decode_frame_ignores_unknown_identifier():
    assert decode_frame(0x123, bytes(8)) is None


@pytest.mark.parametrize("can_id", [0x2A5, 0x251, 0x261, 0x155, 0x4AF])
def test_recognized_frame_rejects_short_payload(can_id):
    with pytest.raises(FrameDecodeError, match="8 bytes"):
        decode_frame(can_id, bytes(7))


def test_specific_decoder_rejects_wrong_identifier_family():
    with pytest.raises(FrameDecodeError, match="position"):
        decode_position_pair(0x251, bytes(8))

    with pytest.raises(FrameDecodeError, match="high-speed"):
        decode_high_speed(0x2A5, bytes(8))

    with pytest.raises(FrameDecodeError, match="low-speed"):
        decode_low_speed(0x251, bytes(8))

    with pytest.raises(FrameDecodeError, match="joint-command"):
        decode_joint_command(0x2A5, bytes(8))
