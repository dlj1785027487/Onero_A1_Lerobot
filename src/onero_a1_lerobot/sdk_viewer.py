"""One MuJoCo window showing actual SDK simulation states side by side."""

from __future__ import annotations

import logging
import queue
import threading
import time
from pathlib import Path

import numpy as np

from .sdk_window import SdkSimulationWindow

logger = logging.getLogger(__name__)


class SdkSimulationViewer:
    """Compose SDK URDFs for display; physics stays in SDK Arm backends.

    The viewport only runs forward kinematics on copies of SDK qpos/qvel.
    It never integrates a second physics simulation or drives CAN devices.
    One main-thread window handles key press, release and focus loss; input
    callbacks only enqueue events, and the session sends control frames.
    """

    def __init__(self, session):
        self.session = session
        self._view = None
        self.model = self.data = None
        self.bindings = []
        self._keys: queue.SimpleQueue[tuple] = queue.SimpleQueue()
        self._held_keys: dict[int, float] = {}
        self._last_jog_time = None
        self._joint = 0
        self._source = next(iter(session.teleop.config.arms))
        self._next_sync = 0.0

    @staticmethod
    def preflight(config) -> None:
        arms = (*config.robot.arms.values(), *config.teleop.arms.values())
        if any(arm.backend != "mujoco" for arm in arms):
            raise ValueError("--viewer requires all simulated arms: use --simulate mujoco --viewer")
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("MuJoCo viewer must run on the main thread")
        try:
            import glfw
            import mujoco  # noqa: F401
        except ImportError as exc:
            raise RuntimeError("Install the SDK sim extra (mujoco) to open the viewer") from exc
        if not glfw.init():
            raise RuntimeError(
                "Cannot open the desktop display. Run from a graphical terminal, "
                "or omit --viewer for headless SDK MuJoCo simulation."
            )
        glfw.terminate()

    def build_scene(self) -> None:
        import mujoco
        from onero_sdk.mounting import get_mounting_rotation, resolve_mounting

        if not hasattr(mujoco, "MjSpec"):
            raise RuntimeError("The combined viewer requires MuJoCo with MjSpec support")
        entries = []
        for target, mapping in self.session.teleop.config.mappings.items():
            entries.extend(
                (
                    ("LEADER", mapping.source, self.session.teleop.devices[mapping.source]),
                    ("FOLLOWER", target, self.session.robot.devices[target]),
                )
            )
        if any(d.config.backend != "mujoco" for _, _, d in entries):
            raise ValueError("Viewer scenes must all use SDK MuJoCo backends")

        spec = mujoco.MjSpec()
        # Same asset resolution as the SDK's MjSpec scene builder.
        spec.compiler.meshdir = str(Path(entries[0][2].mujoco_model_path()).parent)
        spec.worldbody.add_light(pos=[1, -1, 3], dir=[-0.3, 0.3, -1])
        spec.worldbody.add_light(pos=[-1, 1, 2], dir=[0.3, -0.3, -1])
        spec.worldbody.add_geom(
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=[0, 0, 0],
            size=[3, 3, 0.01],
            rgba=[0.18, 0.20, 0.23, 1],
        )
        self.bindings.clear()
        for i, (role, name, device) in enumerate(entries):
            prefix = f"arm{i}_"
            child = mujoco.MjSpec.from_file(device.mujoco_model_path())
            pos = [0.0, (i - (len(entries) - 1) / 2) * 0.9, 0.9]
            mounting = device.config.mounting or (
                "humanoid_left" if device.config.model == "A1L" else "humanoid_right"
            )
            rotation = get_mounting_rotation(resolve_mounting(mounting))
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, np.asarray(rotation).ravel())
            site = spec.worldbody.add_site(name=f"mount_{i}", pos=pos, quat=quat)
            spec.attach(child, prefix=prefix, site=site)
            self.bindings.append(dict(role=role, name=name, device=device, prefix=prefix, pos=pos))

        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        for binding in self.bindings:
            native_model, _ = binding["device"].mujoco_scene()
            addresses, velocities = [], []
            for i in range(native_model.njnt):
                joint = mujoco.mj_id2name(native_model, mujoco.mjtObj.mjOBJ_JOINT, i)
                j = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, binding["prefix"] + joint
                )
                if j < 0:
                    raise RuntimeError(f"SDK joint not found in viewer: {joint}")
                addresses.append(self.model.jnt_qposadr[j])
                velocities.append(self.model.jnt_dofadr[j])
            if len(addresses) != 7 or native_model.nq != 7:
                raise RuntimeError("Viewer requires the SDK's seven-joint A1 models")
            binding["qpos"] = np.asarray(addresses)
            binding["qvel"] = np.asarray(velocities)
            color = [0.9, 0.65, 0.3] if binding["role"] == "LEADER" else [0.3, 0.65, 0.85]
            for g in range(self.model.ngeom):
                body = (
                    mujoco.mj_id2name(
                        self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[g])
                    )
                    or ""
                )
                if body.startswith(binding["prefix"]):
                    self.model.geom_rgba[g, :3] = color
        self.update_scene()

    def update_scene(self) -> None:
        import mujoco

        latest_time = 0.0
        for binding in self.bindings:
            _, source = binding["device"].mujoco_scene()
            self.data.qpos[binding["qpos"]] = source.qpos
            self.data.qvel[binding["qvel"]] = source.qvel
            latest_time = max(latest_time, source.time)
        self.data.time = latest_time
        mujoco.mj_forward(self.model, self.data)

    def open(self) -> None:
        import mujoco

        self.build_scene()
        try:
            self._view = SdkSimulationWindow(
                self.model,
                self.data,
                key_callback=self._key_event,
                focus_callback=lambda focused: self._keys.put(("focus", focused)),
            )
            for device in self.session.teleop.devices.values():
                device.set_simulation_auto(False)
            with self._view.lock():
                self._view.cam.lookat[:] = self.model.stat.center
                self._view.cam.lookat[2] = 0.7
                self._view.cam.distance = max(float(self.model.stat.extent) * 1.4, 1.2)
                self._view.cam.azimuth = 155
                self._view.cam.elevation = -20
                scene = self._view.user_scn
                if scene is not None:
                    for b in self.bindings:
                        geom = scene.geoms[scene.ngeom]
                        mujoco.mjv_initGeom(
                            geom,
                            mujoco.mjtGeom.mjGEOM_LABEL,
                            np.zeros(3),
                            np.asarray(b["pos"]) + [0, 0, 0.18],
                            np.eye(3).ravel(),
                            np.ones(4),
                        )
                        geom.label = f"{b['role']} {b['name']}"
                        scene.ngeom += 1
            self.sync(force=True)
            logger.info(
                "Opened MuJoCo viewer with %d SDK arms; orange=leader, blue=follower",
                len(self.bindings),
            )
        except BaseException:
            self.close()
            raise

    @property
    def is_running(self) -> bool:
        return self._view is not None and self._view.is_running()

    def _key_event(self, key: int, action: int) -> None:
        self._keys.put(("key", key, action))

    def _clear_held(self) -> None:
        self._held_keys.clear()
        self._last_jog_time = None

    def poll_commands(self) -> list[str]:
        if self._view:
            self._view.poll_events()
        now = time.monotonic()
        commands = []
        for _ in range(256):
            try:
                event = self._keys.get_nowait()
            except queue.Empty:
                break
            if event[0] == "focus":
                if not event[1]:
                    self._clear_held()
                continue
            _, key, action = event
            if action == 0:  # GLFW_RELEASE
                self._held_keys.pop(key, None)
                continue
            if action != 1:  # Ignore GLFW_REPEAT; speed uses elapsed time.
                continue
            if ord("1") <= key <= ord("7"):
                self._clear_held()
                self._joint = key - ord("1")
            elif key in (ord("["), ord("]")):
                self._clear_held()
                sources = list(self.session.teleop.devices)
                i = sources.index(self._source) + (1 if key == ord("]") else -1)
                self._source = sources[i % len(sources)]
            elif key in (ord("Q"), ord("P"), ord("R")):
                self._clear_held()
                commands.append(chr(key).lower())
            elif key in (ord("A"), ord("-"), ord("="), 333, 334):
                if self.session.state != "following":
                    logger.info("Resume following before changing the simulated leader")
                    continue
                device = self.session.teleop.devices[self._source]
                if key == ord("A"):
                    self._clear_held()
                    self.session.set_simulation_auto(self._source, not device.simulation_auto)
                else:
                    if device.simulation_auto:
                        self.session.set_simulation_auto(self._source, False)
                    delta = -0.02 if key in (ord("-"), 333) else 0.02
                    device.jog_simulation_joint(self._joint, delta)
                    self._held_keys[key] = now
                    self._last_jog_time = now
        if commands or self.session.state != "following" or not self.is_running:
            self._clear_held()
        elif self._held_keys:
            dt = min(max(now - self._last_jog_time, 0), 0.05)
            # Keep the existing 0.02 rad tap. Holding past 250 ms adds motion
            # at 0.15 rad/s, independent of OS repeat delay/rate or frame rate.
            negative = any(key in (ord("-"), 333) for key in self._held_keys)
            positive = any(key in (ord("="), 334) for key in self._held_keys)
            if now - max(self._held_keys.values()) >= 0.25:
                direction = int(positive) - int(negative)
                if direction:
                    self.session.teleop.devices[self._source].jog_simulation_joint(
                        self._joint, direction * 0.15 * dt,
                    )
            self._last_jog_time = now
        return commands

    def sync(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now < self._next_sync:
            return
        self._next_sync = now + 1 / 30
        if not self.is_running:
            return
        with self._view.lock():
            self.update_scene()
        source = self.session.teleop.devices[self._source]
        mode = "AUTO" if source.simulation_auto else "MANUAL"
        if hasattr(self._view, "set_texts"):
            import mujoco

            controls = (
                f"Leader: {self._source} | J{self._joint + 1} | {mode}\n"
                "1-7: select joint  -/=: tap/hold jog  A: auto/manual\n"
                "[/]: select leader  P: pause  R: record  Q: quit\n"
                f"Session: {self.session.state} | Recording: {self.session.recorder.status}"
            )
            self._view.set_texts(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                    "Onero SDK simulation\nOrange: leader  Blue: follower",
                    controls,
                )
            )
        self._view.sync()

    def close(self) -> None:
        self._clear_held()
        if self._view is not None:
            try:
                self._view.close()
            finally:
                self._view = None
