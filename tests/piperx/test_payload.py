from __future__ import annotations

import hashlib
import json

import numpy as np
import pytest

from robot_control.piperx.payload import PayloadError, RigidPayload


def _document() -> dict[str, object]:
    return {
        "schema_version": 1,
        "name": "test-gripper",
        "parent_joint": "joint6",
        "reference_opening_m": 0.035,
        "mass_kg": 0.5,
        "com_m": [0.0, 0.0, 0.04],
        "inertia_kg_m2": [
            [0.0012, 0.0, 0.0],
            [0.0, 0.0010, 0.0],
            [0.0, 0.0, 0.00045],
        ],
        "source": {
            "repository": "https://example.test/official.git",
            "commit": "a" * 40,
            "path": "model.urdf",
        },
    }


def test_payload_round_trip_has_canonical_content_identity(tmp_path):
    document = _document()
    path = tmp_path / "payload.json"
    path.write_text(json.dumps(document, indent=4), encoding="utf-8")

    payload = RigidPayload.load(path)
    canonical = json.dumps(
        document, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")

    assert payload.mass_kg == pytest.approx(0.5)
    assert payload.com_m == pytest.approx([0.0, 0.0, 0.04])
    assert payload.sha256 == hashlib.sha256(canonical).hexdigest()


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("mass_kg", 0.0, "mass"),
        ("com_m", [0.0, float("nan"), 0.04], "finite"),
        (
            "inertia_kg_m2",
            [[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "symmetric",
        ),
        (
            "inertia_kg_m2",
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
            "positive definite",
        ),
    ],
)
def test_payload_rejects_unsafe_inertia_documents(field, value, message):
    document = _document()
    document[field] = value

    with pytest.raises(PayloadError, match=message):
        RigidPayload.from_dict(document)


def test_official_payload_constants_are_finite_and_positive():
    payload = RigidPayload.load("config/piperx_gripper_payload.json")

    assert payload.name == "piper-standard-parallel-gripper"
    assert payload.mass_kg == pytest.approx(0.5)
    assert payload.com_m == pytest.approx(
        [-0.000165426446012032, 0.000072452984002012, 0.03759000340047896]
    )
    assert np.linalg.eigvalsh(payload.inertia_kg_m2).min() > 0
