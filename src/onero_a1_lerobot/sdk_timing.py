"""Opt-in timing of the parent loop and completed follower transactions."""

from __future__ import annotations

import json
import logging
import threading
import time
from functools import wraps

logger = logging.getLogger(__name__)
_MISSING = object()


class FollowTiming:
    """Install after connection; close before shutting down the session."""

    def __init__(self, session):
        self.session = session
        self._thread = threading.get_ident()
        self._patches = []
        self._stats = {}
        self._worker_frames = {}
        self._unavailable = []
        self._following = session.state == "following"
        self._closed = False
        for obj, method, name in (
            (session, "tick", "session.tick"),
            (session.teleop, "get_action", "teleop.get_action"),
            (session.robot, "get_observation", "robot.get_observation"),
            (session.robot, "send_action", "robot.send_action"),
            (getattr(session, "recorder", None), "submit", "recorder.submit"),
            (session, "log_diagnostics", "session.log_diagnostics"),
        ):
            self._wrap(obj, method, name)
        for side, device in session.robot.devices.items():
            prefix = f"follower.{side}"
            if callable(getattr(device, "begin_command", None)):
                worker_name = f"{prefix}.worker.command"
                self._stats[worker_name] = [0, 0, 0, 0]
                self._wrap(device, "begin_command", f"{prefix}.submit")
                self._wrap(
                    device, "finish_command", f"{prefix}.wait",
                    on_success=lambda d=device, n=worker_name: self._record_worker(d, n),
                )
                continue
            self._wrap(device, "command", f"{prefix}.command")
            arm = getattr(device, "arm", None)
            self._wrap(arm, "_gravity_torques", f"{prefix}.arm._gravity_torques")
            bus = serial = None
            try:
                backend, seen = getattr(arm, "_backend", None), set()
                while backend is not None and id(backend) not in seen:
                    seen.add(id(backend))
                    bus = getattr(backend, "_bus", None)
                    if bus is not None:
                        break
                    backend = getattr(backend, "_inner", None)
                serial = getattr(getattr(bus, "_bus", None), "serialPortOrig", None)
            except Exception:
                pass  # SDK transport details may differ between installations.
            for method in ("send", "send_batch", "recv_batch"):
                if method == "send_batch" and not callable(getattr(bus, method, None)):
                    continue
                self._wrap(bus, method, f"{prefix}.can.{method}")
            for method in ("write", "flush"):
                self._wrap(serial, method, f"{prefix}.serial.{method}")
        self._reset()

    def _record_worker(self, device, name):
        report = device.last_command_timing
        if not report or self._worker_frames.get(name) == report["frame_id"]:
            return
        self._worker_frames[name] = report["frame_id"]
        wall, cpu = int(report["wall_ms"] * 1e6), int(report["cpu_ms"] * 1e6)
        stats = self._stats[name]
        stats[0] += 1
        stats[1] += wall
        stats[2] += cpu
        stats[3] = max(stats[3], wall)

    def _wrap(self, obj, method, name, *, on_success=None):
        stats = self._stats[name] = [0, 0, 0, 0]
        try:
            original = getattr(obj, method)
            if not callable(original):
                raise TypeError("Not callable")
            previous = vars(obj).get(method, _MISSING)

            @wraps(original)
            def measured(*args, **kwargs):
                if self._closed or threading.get_ident() != self._thread:
                    return original(*args, **kwargs)
                try:
                    if name == "session.tick" and not self._following:
                        self._reset()
                        self._following = self.session.state == "following"
                    wall, cpu = time.perf_counter_ns(), time.thread_time_ns()
                except Exception:
                    return original(*args, **kwargs)
                try:
                    result = original(*args, **kwargs)
                    if on_success is not None:
                        try:
                            on_success()
                        except Exception:
                            pass
                    return result
                finally:
                    try:
                        elapsed = time.perf_counter_ns() - wall
                        cpu_elapsed = time.thread_time_ns() - cpu
                        stats[0] += 1
                        stats[1] += elapsed
                        stats[2] += cpu_elapsed
                        stats[3] = max(stats[3], elapsed)
                    except Exception:
                        pass  # Timing must preserve the original result/exception.

            setattr(obj, method, measured)
            self._patches.append((obj, method, previous, measured))
        except Exception:
            self._unavailable.append(name)

    def _reset(self, now=None):
        self._started = time.monotonic() if now is None else now
        self._frame = self.session.frames
        for stats in self._stats.values():
            stats[:] = [0, 0, 0, 0]

    def maybe_log(self):
        """Call every loop iteration, including while paused or faulted."""
        if self._closed or threading.get_ident() != self._thread:
            return
        try:
            now = time.monotonic()
            if self.session.state != "following":
                self._following = False
                self._reset(now)
                return
            elapsed = now - self._started
            frames = self.session.frames - self._frame
            if elapsed < 1.0 or frames <= 0:
                return
            report = {
                "frame": self.session.frames,
                "frames_in_window": frames,
                "actual_hz": round(frames / elapsed, 2),
                "requested_hz": self.session.config.robot.control_hz,
                "window_s": round(elapsed, 4),
                "stages": {
                    name: {
                        "calls": calls,
                        "wall_ms_per_frame": round(wall / frames / 1e6, 4),
                        "cpu_ms_per_frame": round(cpu / frames / 1e6, 4),
                        "max_wall_ms": round(maximum / 1e6, 4),
                    }
                    for name, (calls, wall, cpu, maximum) in self._stats.items()
                },
                "unavailable": self._unavailable,
                "note": "Parent thread and completed worker transactions; nested/parallel times cannot be added together.",
            }
            self._reset(now)
            logger.info("Follow timing %s", json.dumps(report, separators=(",", ":")))
        except Exception:
            pass

    def close(self):
        """Restore instance overrides without touching methods changed elsewhere."""
        self._closed = True
        for obj, method, previous, measured in reversed(self._patches):
            try:
                if getattr(obj, method) is measured:
                    if previous is _MISSING:
                        delattr(obj, method)
                    else:
                        setattr(obj, method, previous)
            except Exception:
                pass
        self._patches.clear()
