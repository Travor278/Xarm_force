import numpy as np
import pytest

from robot_control.piperx.filtering import VelocityDerivativeFilter


def _filter(**overrides):
    options = {
        "velocity_time_constant_s": 0.0,
        "acceleration_time_constant_s": 0.0,
        "startup_samples": 2,
        "max_gap_s": 0.2,
        "max_acceleration_rad_s2": np.full(6, 10.0),
    }
    options.update(overrides)
    return VelocityDerivativeFilter(**options)


def test_first_sample_is_invalid_during_derivative_startup():
    result = _filter().update(1_000_000_000, np.zeros(6))

    assert not result.valid
    assert result.reason == "derivative_startup"
    assert result.qd_filtered_rad_s == pytest.approx(np.zeros(6))


def test_second_sample_recovers_constant_acceleration():
    derivative = _filter()
    derivative.update(1_000_000_000, np.zeros(6))

    result = derivative.update(1_100_000_000, np.full(6, 0.1))

    assert result.valid
    assert result.reason is None
    assert result.qd_filtered_rad_s == pytest.approx(np.full(6, 0.1))
    assert result.qdd_rad_s2 == pytest.approx(np.ones(6))


def test_non_monotonic_timestamp_is_rejected():
    derivative = _filter()
    derivative.update(1_000_000_000, np.zeros(6))

    result = derivative.update(1_000_000_000, np.zeros(6))

    assert not result.valid
    assert result.reason == "non_monotonic_timestamp"


def test_excessive_gap_restarts_derivative_baseline():
    derivative = _filter(max_gap_s=0.05)
    derivative.update(1_000_000_000, np.zeros(6))

    result = derivative.update(1_100_000_000, np.full(6, 0.1))

    assert not result.valid
    assert result.reason == "excessive_timestamp_gap"
    restarted = derivative.update(1_110_000_000, np.full(6, 0.11))
    assert restarted.valid
    assert restarted.qdd_rad_s2 == pytest.approx(np.ones(6))


def test_derivative_spike_is_invalid_instead_of_clipped_and_used():
    derivative = _filter(
        max_acceleration_rad_s2=np.full(6, 0.5), max_gap_s=2.0
    )
    derivative.update(1_000_000_000, np.zeros(6))

    result = derivative.update(2_000_000_000, np.ones(6))

    assert not result.valid
    assert result.reason == "acceleration_limit"
    assert result.qdd_rad_s2 == pytest.approx(np.ones(6))


def test_nonfinite_velocity_is_rejected():
    result = _filter().update(1_000_000_000, np.full(6, np.nan))

    assert not result.valid
    assert result.reason == "nonfinite_velocity"
