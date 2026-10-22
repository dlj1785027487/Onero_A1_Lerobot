"""只读 CAN 链路预检：复现 SDK 使能前的电机状态查询，不发任何使能/运动帧。

对每条通道逐个电机发送 SDK 同样的被动反馈查询（PARAM_FRAME_ID -> response_id），
报告哪些电机在给定的应答超时内回复、是否锁存了错误码。用它可以在启动跟随程序前发现
"某个电机偶发不应答" 或 "整条臂失联" 这类问题。

通道既可以是 SocketCAN 接口名（`can3`），也可以是 `/dev/serial/by-id/...` 直连串口
（直接打开 SLCAN，不需要 slcand；正式双臂配置走的是 slcand 转出的 SocketCAN 接口）。

用法：
    python scripts/can_precheck.py                 # 按内置的四条通道检查
    python scripts/can_precheck.py --config configs/native_dual_joint_follow_record.json
                                                   # 按配置里的 robot/teleop arms 推导通道与型号
    python scripts/can_precheck.py can3 -n 20 -t 0.1
                                                   # 只查 can3，每个电机查 20 次，超时 100 ms

更换适配器或交换 CAN 线后必须重新确认左右对应关系：让操作人员用手转动目标机械臂，
同时观察哪条通道的关节读数在变，即可确认它是不是你要的那条臂：

    python scripts/can_precheck.py --watch 10 /dev/ttyACM4 /dev/ttyACM5

退出码 0 表示全部应答；1 表示存在无应答或打不开的通道。
DISABLED 是未使能的正常状态；任何 NO REPLY / ERROR(...) 都需要先排查再启动跟随程序。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from onero_sdk.can_backend import CanBackend
from onero_sdk.model_registry import get_model
from onero_sdk.protocol import MotorError

DEFAULT_TIMEOUT = 0.05
DEFAULT_CHANNELS = [("can1", "A1L", "leader left"), ("can2", "A1R", "leader right"),
                    ("can3", "A1L", "follower left"), ("can4", "A1R", "follower right")]
SLCAN_SETTLE_S = 0.5  # USB CDC-ACM 适配器枚举后需要一点时间才能稳定收发
WATCH_MIN_DELTA_RAD = 0.02  # 判定"这个关节被转动过"的最小角度变化


def open_backend(channel: str, joint_specs):
    """按通道名打开总线：`/dev/...` 走直连 SLCAN，其余按 SocketCAN 接口。"""
    if channel.startswith("/dev/"):
        try:
            from onero_sdk import PythonCanBus
        except ImportError as exc:  # pragma: no cover - 依赖缺失时的提示
            raise RuntimeError(
                "直连串口需要 onero-sdk[python-can] 与 pyserial"
            ) from exc
        bus = PythonCanBus(
            channel=channel, interface="slcan", bitrate=1_000_000,
            tty_baudrate=1_000_000, buffered_slcan=True,
        )
        time.sleep(SLCAN_SETTLE_S)
        return CanBackend(None, joint_specs, bus=bus)
    return CanBackend(channel, joint_specs)


def describe(state) -> str:
    if state is None:
        return "NO REPLY"
    name = MotorError(state.error).name if state.error in tuple(MotorError) else state.error
    if state.error == MotorError.ENABLED:
        status = "ENABLED"
    elif state.error == MotorError.DISABLED:
        status = "DISABLED"
    else:
        status = f"ERROR({name})"
    return f"{status} q={state.position:+.4f}"


def channels_from_config(path: str) -> list[tuple[str, str, str]]:
    """按配置里 teleop（主臂）先、robot（从臂）后的顺序推导 (channel, model, role)。

    同时接受本项目的 `--session-config` 和原生 `lerobot-record` 的 `--config_path`
    两种 JSON 格式，二者都用 `arms.<left|right>.channel` / `.model`。
    """
    source = Path(path).expanduser()
    try:
        data = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"无法读取配置 {source}: {type(exc).__name__}: {exc}") from exc
    targets: list[tuple[str, str, str]] = []
    for section, role in (("teleop", "leader"), ("robot", "follower")):
        arms = (data.get(section) or {}).get("arms") or {}
        for side, arm in arms.items():
            channel = arm.get("channel")
            if not channel:
                continue
            targets.append((channel, arm.get("model", "A1L"), f"{role} {side}"))
    if not targets:
        raise SystemExit(f"{source}: 配置里没有带 channel 的机械臂")
    return targets


def check(channel: str, model_name: str, role: str, rounds: int, timeout: float) -> int:
    backend = open_backend(channel, get_model(model_name).joints)
    misses = 0
    try:
        results = []
        for index, motor in enumerate(backend._motors):
            states = [motor.get_feedback(timeout=timeout) for _ in range(rounds)]
            failed = sum(state is None for state in states)
            misses += failed
            last = next((state for state in reversed(states) if state is not None), None)
            detail = describe(last)
            if failed:
                detail += f"  !! {failed}/{rounds} 次无应答"
            results.append(f"J{index + 1}(0x{motor._can_id:02X})={detail}")
        print(f"{channel} ({model_name}, {role}):")
        for line in results:
            print(f"   {line}")
    finally:
        backend.close()
    return misses


def watch(channel: str, model_name: str, role: str, seconds: float, timeout: float) -> None:
    """观察窗口内哪些关节被手动转动过，用来确认通道与机械臂的对应关系。

    只读取被动反馈，不使能、不下发运动指令；机械臂可以自由被手推动。
    """
    backend = open_backend(channel, get_model(model_name).joints)
    try:
        baseline = []
        for motor in backend._motors:
            state = motor.get_feedback(timeout=timeout)
            baseline.append(None if state is None else state.position)
        deltas: dict[int, float] = {}
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for index, motor in enumerate(backend._motors):
                state = motor.get_feedback(timeout=timeout)
                if state is None or baseline[index] is None:
                    continue
                moved = abs(state.position - baseline[index])
                if moved > deltas.get(index, 0.0):
                    deltas[index] = moved
        moved_joints = [
            f"J{index + 1} 变化 {delta:.3f} rad"
            for index, delta in sorted(deltas.items()) if delta >= WATCH_MIN_DELTA_RAD
        ]
        detail = "，".join(moved_joints) if moved_joints else "没有关节被明显转动"
        print(f"{channel} ({model_name}, {role}): {detail}")
    finally:
        backend.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="只读 CAN 链路预检（不使能、不运动）")
    parser.add_argument("channels", nargs="*", help="只查指定通道；缺省查内置四条或 --config 的通道")
    parser.add_argument("-n", "--rounds", type=int, default=1, help="每个电机查询次数，默认 1")
    parser.add_argument(
        "-t", "--timeout", type=float, default=DEFAULT_TIMEOUT,
        help=f"每次查询等待应答的秒数，默认 {DEFAULT_TIMEOUT}",
    )
    parser.add_argument("--config", help="从会话/原生 JSON 配置的 arms 推导通道与型号")
    parser.add_argument(
        "--watch", type=float,
        help="观察若干秒，报告每条通道被手动转动的关节，用于确认通道与机械臂的对应关系",
    )
    args = parser.parse_args()

    if args.rounds < 1:
        parser.error("--rounds 必须是正整数")
    if args.timeout <= 0:
        parser.error("--timeout 必须大于 0")
    if args.watch is not None and args.watch <= 0:
        parser.error("--watch 必须是正数秒")

    known = {channel: (model, role) for channel, model, role in DEFAULT_CHANNELS}
    if args.channels:
        targets = [(name, *known.get(name, ("A1L", "unknown role"))) for name in args.channels]
    elif args.config:
        targets = channels_from_config(args.config)
    else:
        targets = DEFAULT_CHANNELS

    if args.watch is not None:
        print(f"开始观察 {args.watch:g} 秒：请用手转动你要识别的那条机械臂（不会被使能或受力）。")
        failed = 0
        for channel, model_name, role in targets:
            try:
                watch(channel, model_name, role, args.watch, args.timeout)
            except Exception as exc:
                failed += 1
                print(f"{channel} ({role}): 打开失败：{type(exc).__name__}: {exc}")
        if failed:
            print(f"{failed}/{len(targets)} 条通道无法打开；请先排查，再据读数判断对应关系。")
        else:
            print("读数在变化的那条通道就是你在转动的那条机械臂。")
        return 1 if failed else 0

    started = time.monotonic()
    total_misses = 0
    for channel, model_name, role in targets:
        try:
            total_misses += check(channel, model_name, role, args.rounds, args.timeout)
        except Exception as exc:
            print(f"{channel} ({role}): 打开失败：{type(exc).__name__}: {exc}")
            total_misses += 1
    print(f"\n预检完成：{len(targets)} 条通道 × {args.rounds} 次，应答超时 {args.timeout:.3f}s，"
          f"{'全部应答' if not total_misses else f'{total_misses} 次无应答'}"
          f"（{time.monotonic() - started:.1f}s）")
    print("DISABLED 是未使能的正常状态；任何 NO REPLY / ERROR(...) 都需要先排查再启动跟随程序。")
    return 1 if total_misses else 0


if __name__ == "__main__":
    sys.exit(main())
