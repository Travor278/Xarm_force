"""Receive-only acquisition runtime and loopback web application."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
import queue
import socket
import threading
from typing import Callable, Protocol, Sequence

from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .estimator import Estimate
from .gripper_force import GripperForceCalibration
from .monitoring import SCHEMA_VERSION, LatestEventHub, serialize_snapshot, sse_message
from .socketcan import (
    PiperState,
    PiperStateAssembler,
    ReadOnlySocketCan,
    discover_interface,
)


@dataclass(frozen=True)
class ArmMonitorConfig:
    name: str
    serial: str
    calibration_path: Path
    interface: str | None = None
    firmware_override: str | None = None


class _Estimator(Protocol):
    def estimate(self, state: PiperState) -> Estimate: ...


class _Reader(Protocol):
    def __enter__(self) -> "_Reader": ...
    def __exit__(self, *_args: object) -> None: ...
    def recv_frame(self): ...


class ArmAcquisitionWorker:
    """Own one passive CAN reader and publish downsampled monitor snapshots."""

    def __init__(
        self,
        config: ArmMonitorConfig,
        *,
        estimator: _Estimator,
        hub: LatestEventHub,
        reader_factory: Callable[..., _Reader] = ReadOnlySocketCan,
        interface_resolver: Callable[[str], str] = discover_interface,
        ui_rate_hz: float = 50.0,
        gripper_calibration: GripperForceCalibration | None = None,
    ) -> None:
        if not config.name or not config.serial:
            raise ValueError("arm name and adapter serial must not be empty")
        if ui_rate_hz <= 0 or ui_rate_hz > 200:
            raise ValueError("ui_rate_hz must be in (0, 200]")
        self.config = config
        self.name = config.name
        self.estimator = estimator
        self.hub = hub
        self.reader_factory = reader_factory
        self.interface_resolver = interface_resolver
        self.publish_period_ns = int(1e9 / ui_rate_hz)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: dict[str, object] | None = None
        self._running = False
        self._error: str | None = None
        self._interface: str | None = config.interface
        self._sequence = 0
        self.gripper_calibration = gripper_calibration

    @property
    def latest(self) -> dict[str, object] | None:
        with self._lock:
            return self._latest

    def status(self) -> dict[str, object]:
        with self._lock:
            latest_timestamp = (
                self._latest.get("timestamp_ns") if self._latest is not None else None
            )
            return {
                "arm": self.name,
                "serial": self.config.serial,
                "interface": self._interface,
                "running": self._running,
                "error": self._error,
                "sequence": self._sequence,
                "latest_timestamp_ns": latest_timestamp,
            }

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=f"piperx-web-{self.name}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)

    def _run(self) -> None:
        with self._lock:
            self._running = True
            self._error = None
        try:
            interface = self.config.interface or self.interface_resolver(self.config.serial)
            with self._lock:
                self._interface = interface
            assembler = PiperStateAssembler(interface, self.config.serial)
            last_publish_ns: int | None = None
            with self.reader_factory(interface, timeout_s=0.1) as reader:
                while not self._stop.is_set():
                    try:
                        frame = reader.recv_frame()
                    except socket.timeout:
                        continue
                    state = assembler.update(
                        frame.can_id, frame.payload, frame.timestamp_ns
                    )
                    if state is None:
                        continue
                    if (
                        last_publish_ns is not None
                        and state.timestamp_ns - last_publish_ns < self.publish_period_ns
                    ):
                        continue
                    estimate = self.estimator.estimate(state)
                    self._sequence += 1
                    event = serialize_snapshot(
                        self.name,
                        state,
                        estimate,
                        self._sequence,
                        firmware_override=self.config.firmware_override,
                        gripper_calibration=self.gripper_calibration,
                    )
                    with self._lock:
                        self._latest = event
                    self.hub.publish(event)
                    last_publish_ns = state.timestamp_ns
        except Exception as error:  # surfaced through health/status
            with self._lock:
                self._error = f"{type(error).__name__}: {error}"
        finally:
            with self._lock:
                self._running = False


class MonitorRuntime:
    def __init__(
        self,
        workers: Sequence[ArmAcquisitionWorker],
        hub: LatestEventHub,
    ) -> None:
        self.workers = tuple(workers)
        self.hub = hub

    def start(self) -> None:
        for worker in self.workers:
            worker.start()

    def stop(self) -> None:
        for worker in self.workers:
            worker.stop()

    def status(self) -> dict[str, object]:
        return {
            "schema": SCHEMA_VERSION,
            "arms": [worker.status() for worker in self.workers],
            "subscribers": self.hub.subscriber_count,
        }

    def snapshot(self) -> dict[str, object]:
        return {
            "schema": SCHEMA_VERSION,
            "arms": {
                worker.name: worker.latest
                for worker in self.workers
                if worker.latest is not None
            },
        }

    def healthy(self) -> bool:
        statuses = [worker.status() for worker in self.workers]
        return bool(statuses) and all(
            status["running"] and status["error"] is None for status in statuses
        )


def create_app(runtime: MonitorRuntime, asset_root: str | Path) -> FastAPI:
    assets = Path(asset_root).resolve(strict=True)
    index = assets / "index.html"
    if not index.is_file():
        raise FileNotFoundError(f"dashboard index is missing: {index}")

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        runtime.start()
        try:
            yield
        finally:
            runtime.stop()

    app = FastAPI(
        title="PiperX receive-only torque monitor",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
        lifespan=lifespan,
    )
    app.mount("/assets", StaticFiles(directory=assets), name="assets")

    @app.get("/", include_in_schema=False)
    def dashboard() -> FileResponse:
        return FileResponse(index)

    @app.get("/healthz")
    def health() -> JSONResponse:
        ok = runtime.healthy()
        return JSONResponse(
            {"ok": ok, "arms": len(runtime.workers)},
            status_code=200 if ok else 503,
        )

    @app.get("/api/status")
    def status() -> dict[str, object]:
        return runtime.status()

    @app.get("/api/snapshot")
    def snapshot() -> dict[str, object]:
        return runtime.snapshot()

    async def events():
        subscriber = runtime.hub.subscribe()
        try:
            yield sse_message("ready", runtime.status())
            while True:
                try:
                    event = await asyncio.to_thread(subscriber.get, True, 10.0)
                except queue.Empty:
                    yield ": heartbeat\n\n"
                    continue
                yield sse_message(
                    "snapshot", event, event_id=int(event.get("sequence", 0))
                )
        finally:
            runtime.hub.unsubscribe(subscriber)

    @app.get("/stream")
    async def stream() -> StreamingResponse:
        return StreamingResponse(
            events(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )

    return app
