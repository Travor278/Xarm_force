from __future__ import annotations

import asyncio
from dataclasses import dataclass
from pathlib import Path
import queue
import socket
import struct
import warnings

import numpy as np
import pytest
from starlette.exceptions import StarletteDeprecationWarning

warnings.filterwarnings(
    "ignore",
    message="Using `httpx` with `starlette.testclient` is deprecated.*",
    category=StarletteDeprecationWarning,
)

from fastapi.testclient import TestClient

from robot_control.piperx.estimator import Estimate
from robot_control.piperx.monitoring import LatestEventHub
from robot_control.piperx.socketcan import CanFrame
from robot_control.piperx.web_monitor import (
    ArmAcquisitionWorker,
    ArmMonitorConfig,
    MonitorRuntime,
    create_app,
)


def _frames() -> list[CanFrame]:
    frames = []
    timestamp = 1_000_000_000
    for can_id in (0x2A5, 0x2A6, 0x2A7):
        frames.append(CanFrame(can_id, struct.pack(">ii", 1000, 2000), timestamp))
    for index, can_id in enumerate(range(0x251, 0x257)):
        frames.append(CanFrame(can_id, struct.pack(">hhi", index, 100, 0), timestamp))
    return frames


class _Reader:
    def __init__(self, frames):
        self.frames = queue.Queue()
        for frame in frames:
            self.frames.put(frame)
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True

    def recv_frame(self):
        try:
            return self.frames.get(timeout=0.01)
        except queue.Empty as error:
            raise socket.timeout from error


class _Estimator:
    def estimate(self, state):
        zeros = np.zeros(6)
        return Estimate(
            interface=state.interface,
            adapter_serial=state.adapter_serial,
            timestamp_ns=state.timestamp_ns,
            q_rad=np.asarray(state.q_rad),
            qd_filtered_rad_s=np.asarray(state.qd_rad_s),
            qdd_rad_s2=zeros,
            tau_measured_nm=np.asarray(state.tau_measured_nm),
            tau_model_nm=zeros,
            tau_bias_nm=zeros,
            tau_external_nm=-np.asarray(state.tau_measured_nm),
            valid=True,
            calibrated=True,
            reason=None,
        )


def test_acquisition_worker_publishes_real_assembled_snapshot_and_stops_cleanly(tmp_path):
    hub = LatestEventHub()
    subscriber = hub.subscribe()
    reader = _Reader(_frames())
    config = ArmMonitorConfig("left", "serial-left", tmp_path / "left.json", "can2")
    worker = ArmAcquisitionWorker(
        config,
        estimator=_Estimator(),
        hub=hub,
        reader_factory=lambda _interface, timeout_s: reader,
        ui_rate_hz=50.0,
    )

    worker.start()
    event = subscriber.get(timeout=1.0)
    worker.stop()

    assert event["arm"] == "left"
    assert event["interface"] == "can2"
    assert event["current_a"] == [0.1] * 6
    assert event["tau_effort_nm"][:3] == pytest.approx([0.118125] * 3)
    assert event["tau_effort_nm"][3:] == pytest.approx([0.095844] * 3)
    assert worker.latest == event
    assert reader.closed
    assert not worker.status()["running"]


@dataclass
class _WebWorker:
    name: str = "left"
    started: bool = False
    stopped: bool = False

    @property
    def latest(self):
        return {"schema": "piperx-monitor-v1", "arm": self.name, "sequence": 4}

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def status(self):
        return {
            "arm": self.name,
            "running": self.started and not self.stopped,
            "error": None,
            "sequence": 4,
        }


def test_read_only_web_app_serves_health_snapshot_assets_and_sse(tmp_path):
    (tmp_path / "index.html").write_text("<h1>PiperX monitor</h1>", encoding="utf-8")
    (tmp_path / "styles.css").write_text("body{}", encoding="utf-8")
    hub = LatestEventHub()
    worker = _WebWorker()
    runtime = MonitorRuntime([worker], hub)
    app = create_app(runtime, tmp_path)

    with TestClient(app) as client:
        assert client.get("/").text == "<h1>PiperX monitor</h1>"
        assert client.get("/assets/styles.css").text == "body{}"
        assert client.get("/healthz").json() == {"ok": True, "arms": 1}
        assert client.get("/api/status").json()["arms"][0]["arm"] == "left"
        assert client.get("/api/snapshot").json()["arms"]["left"]["sequence"] == 4
        route = next(route for route in app.routes if getattr(route, "path", None) == "/stream")
        response = asyncio.run(route.endpoint())
        first = asyncio.run(anext(response.body_iterator))
        asyncio.run(response.body_iterator.aclose())
        assert response.media_type == "text/event-stream"
        assert first.startswith("event: ready\ndata:")
        assert all(getattr(route, "methods", {"GET"}) == {"GET"} for route in app.routes if getattr(route, "path", "").startswith("/api/"))

    assert worker.started
    assert worker.stopped
