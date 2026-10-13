"""``scripts/can_precheck.py`` 的纯逻辑测试：不打开任何 CAN 通道。

脚本不属于 ``onero_a1_lerobot`` 包，因此这里按文件路径加载它，并替换掉
``CanBackend`` / ``get_model``，只验证参数解析、通道推导和超时传递。
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

MODULE_PATH = Path(__file__).resolve().parent.parent / "scripts" / "can_precheck.py"

EXPECTED_TARGETS = [
    ("can4", "A1L", "leader left"),
    ("can3", "A1R", "leader right"),
    ("can2", "A1L", "follower left"),
    ("can1", "A1R", "follower right"),
]


@pytest.fixture(scope="module")
def precheck():
    spec = importlib.util.spec_from_file_location("can_precheck", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules["can_precheck"] = module
    spec.loader.exec_module(module)
    return module


class _FakeMotor:
    _can_id = 0x01

    def __init__(self, seen, reply):
        self._seen, self._reply = seen, reply

    def get_feedback(self, timeout=None):
        self._seen.append(timeout)
        return self._reply


def _install_fake_backend(precheck, monkeypatch, *, replies):
    seen: list[float] = []

    class FakeBackend:
        def __init__(self, *_args, **_kwargs):
            self._motors = [_FakeMotor(seen, reply) for reply in replies]
            self.closed = False

        def close(self):
            self.closed = True

    class FakeModel:
        joints = [object()]

    monkeypatch.setattr(precheck, "CanBackend", FakeBackend)
    monkeypatch.setattr(precheck, "get_model", lambda _name: FakeModel())
    return seen


def test_default_channels_cover_all_four_arms(precheck):
    assert precheck.DEFAULT_CHANNELS == EXPECTED_TARGETS


def test_describe_maps_every_motor_state(precheck):
    from onero_sdk.protocol import MotorError

    assert precheck.describe(None) == "NO REPLY"
    state = SimpleNamespace(error=MotorError.DISABLED, position=0.25)
    assert precheck.describe(state) == "DISABLED q=+0.2500"
    state = SimpleNamespace(error=MotorError.ENABLED, position=-1.0)
    assert precheck.describe(state) == "ENABLED q=-1.0000"
    state = SimpleNamespace(error=MotorError.OVERLOAD, position=0.0)
    assert precheck.describe(state) == "ERROR(OVERLOAD) q=+0.0000"


def test_channels_from_config_orders_leaders_before_followers(precheck, config_dir):
    for name in ("native_dual_joint_follow_record.json", "sdk_dual_joint_follow_cameras.json"):
        assert precheck.channels_from_config(str(config_dir / name)) == EXPECTED_TARGETS


def test_channels_from_config_rejects_a_config_without_channels(precheck, tmp_path):
    path = tmp_path / "empty.json"
    path.write_text(json.dumps({"robot": {"arms": {"left": {"model": "A1L"}}}}), encoding="utf-8")
    with pytest.raises(SystemExit, match="没有带 channel"):
        precheck.channels_from_config(str(path))


def test_channels_from_config_reports_unreadable_files(precheck, tmp_path):
    missing = tmp_path / "missing.json"
    with pytest.raises(SystemExit, match="无法读取配置"):
        precheck.channels_from_config(str(missing))
    broken = tmp_path / "broken.json"
    broken.write_text("{not json", encoding="utf-8")
    with pytest.raises(SystemExit, match="无法读取配置"):
        precheck.channels_from_config(str(broken))


def test_timeout_is_forwarded_to_every_query(precheck, monkeypatch, capsys):
    seen = _install_fake_backend(precheck, monkeypatch, replies=[None])
    monkeypatch.setattr(sys, "argv", ["can_precheck.py", "can1", "-n", "2", "-t", "0.25"])
    assert precheck.main() == 1
    assert seen == [0.25, 0.25]
    assert "2 次无应答" in capsys.readouterr().out


def test_replied_motors_exit_zero(precheck, monkeypatch, capsys):
    from onero_sdk.protocol import MotorError

    state = SimpleNamespace(error=MotorError.DISABLED, position=0.0)
    _install_fake_backend(precheck, monkeypatch, replies=[state])
    monkeypatch.setattr(sys, "argv", ["can_precheck.py"])
    assert precheck.main() == 0
    assert "全部应答" in capsys.readouterr().out


@pytest.mark.parametrize("arguments", [["-n", "0"], ["-t", "0"], ["-t", "-1"]])
def test_invalid_numbers_are_rejected(precheck, monkeypatch, arguments):
    monkeypatch.setattr(sys, "argv", ["can_precheck.py", *arguments])
    with pytest.raises(SystemExit) as excinfo:
        precheck.main()
    assert excinfo.value.code == 2


def test_open_backend_uses_socketcan_names(precheck, monkeypatch):
    seen = {}

    class FakeBackend:
        def __init__(self, channel, joint_specs):
            seen["channel"] = channel

    monkeypatch.setattr(precheck, "CanBackend", FakeBackend)
    precheck.open_backend("can3", [object()])
    assert seen["channel"] == "can3"


def test_open_backend_opens_device_paths_as_slcan(precheck, monkeypatch):
    import onero_sdk

    seen = {}

    class FakePythonCanBus:
        def __init__(self, **kwargs):
            seen["bus_kwargs"] = kwargs

    class FakeBackend:
        def __init__(self, channel, joint_specs, bus=None):
            seen["channel"] = channel
            seen["bus_arg"] = bus

    monkeypatch.setattr(onero_sdk, "PythonCanBus", FakePythonCanBus)
    monkeypatch.setattr(precheck, "CanBackend", FakeBackend)
    monkeypatch.setattr(precheck, "SLCAN_SETTLE_S", 0)

    device = "/dev/serial/by-id/usb-CANable2_2097368C4343-if00"
    assert isinstance(precheck.open_backend(device, [object()]), FakeBackend)
    assert seen["bus_kwargs"] == {
        "channel": device, "interface": "slcan", "bitrate": 1_000_000,
        "tty_baudrate": 1_000_000, "buffered_slcan": True,
    }
    assert seen["channel"] is None
    assert isinstance(seen["bus_arg"], FakePythonCanBus)


def test_watch_reports_the_moved_joints(precheck, monkeypatch, capsys):
    class FakeMotor:
        _can_id = 0x01

        def __init__(self):
            self.calls = 0

        def get_feedback(self, timeout=None):
            self.calls += 1
            return SimpleNamespace(error=0, position=1.5 if self.calls >= 3 else 0.0)

    class FakeBackend:
        def __init__(self):
            self._motors = [FakeMotor()]

        def close(self):
            pass

    monkeypatch.setattr(precheck, "open_backend", lambda *a, **k: FakeBackend())
    monkeypatch.setattr(precheck, "get_model", lambda _name: SimpleNamespace(joints=[object()]))

    precheck.watch("can4", "A1L", "leader left", 0.05, 0.0)
    output = capsys.readouterr().out
    assert "can4 (A1L, leader left)" in output
    assert "J1 变化 1.500 rad" in output


def test_watch_option_runs_on_every_target(precheck, monkeypatch):
    calls = []
    monkeypatch.setattr(precheck, "watch", lambda *args: calls.append(args))
    monkeypatch.setattr(sys, "argv", ["can_precheck.py", "--watch", "3", "can1"])
    assert precheck.main() == 0
    assert calls == [("can1", "A1R", "follower right", 3.0, precheck.DEFAULT_TIMEOUT)]


def test_nonpositive_watch_seconds_are_rejected(precheck, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["can_precheck.py", "--watch", "0"])
    with pytest.raises(SystemExit) as excinfo:
        precheck.main()
    assert excinfo.value.code == 2
