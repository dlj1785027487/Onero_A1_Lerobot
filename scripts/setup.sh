#!/usr/bin/env bash
# One-click setup for A1 direct CAN control, SDK simulation, and LeRobot
# recording (including video encode/decode dependencies) in a virtual env.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
VENV="${A1_LEROBOT_VENV:-$HOME/.venvs/a1-lerobot}"
SDK_SOURCE="${A1_ARM_SDK_PATH:-$PROJECT_DIR/../armsdk}"
PIP_NETWORK_OPTS=(--timeout 120 --retries 3)
# torch 轮子源：x86_64 与 aarch64 的 PyPI Linux 轮子都绑定 CUDA（会额外拉
# 1GB+ 的 nvidia-cuda 依赖，arm64 开发板用不上），统一用官方 CPU 源。
# 显式设置 A1_TORCH_INDEX_URL 时优先使用（空字符串强制用 PyPI 默认源）。
ARCH="$(uname -m)"
TORCH_INDEX_URL="${A1_TORCH_INDEX_URL-https://download.pytorch.org/whl/cpu}"
# 平台差异：x86 主机装全量（含 MuJoCo 仿真、rerun/foxglove 可视化、训练依赖）；
# arm64 开发板只保留遥操与数据采集所需，不装 MuJoCo / viz / training。
case "${ARCH}" in
    x86_64|amd64)
        PLATFORM=x86
        SDK_EXTRAS="dynamics,sim,python-can"
        MUJOCO_PKGS=('mujoco==3.12.0')
        LEROBOT_PKGS=(-e "$PROJECT_DIR"'[lerobot]')
        ;;
    *)
        PLATFORM=arm64
        SDK_EXTRAS="dynamics,python-can"
        MUJOCO_PKGS=()
        # 不用 lerobot[dataset]：它在 aarch64 上强制拉 torchcodec，而 PyPI 的
        # torchcodec aarch64 轮子是 CUDA 构建（dlopen libnvrtc.so.13，CPU 板
        # 加载即失败）。所以按 dataset extra 的清单手工补齐其他依赖
        # （datasets / pandas / pyarrow / jsonlines / av），只跳过 torchcodec。
        # LeRobotDataset 本身在 lerobot 基础包里，但数据集 IO 依赖不会跟着来：
        # 缺 datasets 时录制过程正常、收尾保存数据集才报
        # "LeRobotDataset import failed"，所以这几个包不能省。
        # 录制/回放走 PyAV（适配层显式 video_backend="pyav"，lerobot 在
        # torchcodec 缺失时也回退 pyav；av 轮子自带 libx264/libx265）。
        # 也不带 hardware extra：其 pynput 在 Linux 上拉 evdev（需编译 C 扩展，
        # 板载 Python 的 sysconfig 编译器名不可用会构建失败），A1 用不到。
        LEROBOT_PKGS=(-e "$PROJECT_DIR" 'lerobot==0.6.1' 'datasets>=4.8,<5' \
            'pandas>=2,<3' 'pyarrow>=21,<30' 'jsonlines>=4,<5' 'av>=15,<16')
        ;;
esac
INSTALL_RECORD=1

usage() {
    cat <<'EOF'
用法: bash scripts/setup.sh [--base]

一键创建或复用虚拟环境，默认安装：
  - 本地 armsdk、Pinocchio（CAN 拖动所需动力学）
  - python-can、pyserial（直连 SLCAN）
  - Onero A1 LeRobot 适配器
  - LeRobot 0.6.1 录制依赖（含 PyAV 视频编码、torchcodec 视频解码）
  - CPU 版 PyTorch；系统 ffmpeg（torchcodec 解码所需，缺少时用 apt 安装）

x86_64 主机另装 MuJoCo 3.12.0（SDK 仿真）、rerun/foxglove 可视化与训练依赖。
arm64 开发板自动精简：只装遥操与数据采集所需，不装 MuJoCo / viz / training。
arm64 的录制依赖按 lerobot 的 dataset extra 手工补齐（datasets / pandas /
pyarrow / jsonlines / av），只跳过绑定 CUDA 的 torchcodec，视频编解码走 PyAV。

录制依赖要求 Python >= 3.12：系统里挑不到 3.12+ 时直接报错并要求先安装，
不会降级；只做遥操时显式加 --base（要求 3.10+）。
已有 venv 的 Python 不满足本次安装要求（录制 3.12+、基础控制 3.10+）时，
直接删除该 venv 并用合适的解释器重建，venv 内原有包需重新安装。
--base 只安装基础控制环境（Python 3.10+，不装 PyTorch / LeRobot）。
安装后只检查软件、SDK 自带的 A1 模型和视频编解码链路，不连接或使能真机。
--sdk 是默认流程的兼容别名。

可选环境变量：
  A1_LEROBOT_VENV    虚拟环境位置（默认 ~/.venvs/a1-lerobot）
  A1_ARM_SDK_PATH    SDK 源码（默认本项目相邻的 armsdk）
  A1_LEROBOT_PYTHON  指定 Python 解释器（默认自动挑选 3.12+）
  A1_TORCH_INDEX_URL torch 轮子源；默认官方 CPU 源（x86_64 与 aarch64 的
                     PyPI Linux 轮子均绑定 CUDA），置空强制用 PyPI 默认源

在项目目录使用 .venv：
  A1_LEROBOT_VENV="$PWD/.venv" bash scripts/setup.sh
EOF
}

die() {
    printf 'ERROR: %s\n' "$1" >&2
    exit 1
}

ensure_venv() {
    if [[ -x "$VENV/bin/python" && "${VENV_REBUILD:-0}" -eq 1 ]]; then
        # 只删除确认过的 venv（有 pyvenv.cfg），避免误删用户自己的目录。
        [[ -n "$VENV" && "$VENV" != "/" && -f "$VENV/pyvenv.cfg" ]] \
            || die "$VENV 不是可重建的虚拟环境（缺少 pyvenv.cfg），拒绝删除；请手动处理或用 A1_LEROBOT_VENV 换位置。"
        printf '  删除并重建虚拟环境：%s（原 Python %s，本次需要 %s）\n' \
            "$VENV" "$(venv_python_version)" "$MIN_PY_DESC"
        rm -rf -- "$VENV" || die "无法删除 $VENV；请手动删除后重跑本脚本。"
    fi
    if [[ ! -x "$VENV/bin/python" ]]; then
        "$PY_BIN" -m venv "$VENV" \
            || die "无法创建 venv；请确认已安装 python3-venv。"
    else
        printf '  复用已有虚拟环境：%s\n' "$VENV"
    fi
    [[ -f "$VENV/pyvenv.cfg" ]] || die "$VENV 不是 Python venv。"
    "$VENV/bin/python" -I -c 'import sys; assert sys.prefix != sys.base_prefix' \
        || die "$VENV/bin/python 未运行于虚拟环境。"
    VENV_PY_VERSION="$(venv_python_version)"
    [[ -n "$VENV_PY_VERSION" && "${VENV_PY_VERSION%%.*}" -eq 3 \
        && "${VENV_PY_VERSION#*.}" -ge "${MIN_PY_MINOR:-10}" ]] \
        || die "虚拟环境 $VENV 的 Python 为 ${VENV_PY_VERSION:-未知}，本次安装需要 ${MIN_PY_DESC:-3.10+}；请手动删除该 venv 后重跑。"
    install -m 755 "$SCRIPT_DIR/venv_deactivate.sh" "$VENV/bin/deactivate"
}

# 选择 Python 解释器：优先 3.12+（录制依赖需要），其次任意 3.10+。
# 直接设置全局 PY_BIN / PY_VERSION，不使用命令替换。
pick_python() {
    local candidates=("${A1_LEROBOT_PYTHON:-}" python3.13 python3.12 python3 python3.11 python3.10)
    local fallback="" fallback_version="" candidate version
    for candidate in "${candidates[@]}"; do
        command -v "$candidate" >/dev/null 2>&1 || continue
        version="$("$candidate" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)" || continue
        if [[ "${version%%.*}" -eq 3 && "${version#*.}" -ge 12 ]]; then
            PY_BIN="$candidate"
            PY_VERSION="$version"
            return 0
        fi
        if [[ "${version%%.*}" -eq 3 && "${version#*.}" -ge 10 && -z "$fallback" ]]; then
            fallback="$candidate"
            fallback_version="$version"
        fi
    done
    if [[ -n "$fallback" ]]; then
        PY_BIN="$fallback"
        PY_VERSION="$fallback_version"
        return 0
    fi
    return 1
}

# 已有 venv 时返回它的 Python 版本（major.minor），否则返回空。
# 复用的 venv 可能是旧解释器创建的，判定录制依赖时必须以它为准。
venv_python_version() {
    local version=""
    if [[ -x "$VENV/bin/python" ]]; then
        version="$("$VENV/bin/python" -I -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || true)"
    fi
    printf '%s' "$version"
}

# 安装缺失的系统依赖：venv/pip 始终需要；ffmpeg 供 torchcodec 解码视频。
ensure_system_deps() {
    local apt_pkgs=()
    "$PY_BIN" -I -m ensurepip --version >/dev/null 2>&1 || apt_pkgs+=(python3-venv python3-pip)
    if ((INSTALL_RECORD)) && ! command -v ffmpeg >/dev/null 2>&1; then
        apt_pkgs+=(ffmpeg)
    fi
    ((${#apt_pkgs[@]})) || return 0
    printf '\n需要安装系统依赖：%s（sudo apt-get）\n' "${apt_pkgs[*]}"
    sudo apt-get update -qq || die "apt-get update 失败；请手动安装：sudo apt-get install ${apt_pkgs[*]}"
    sudo -E apt-get install -y --no-install-recommends "${apt_pkgs[@]}" \
        || die "apt 安装失败；请手动安装：sudo apt-get install ${apt_pkgs[*]}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sdk) ;;
        --base) INSTALL_RECORD=0 ;;
        -h|--help) usage; exit 0 ;;
        *) die "不支持参数：$1。使用 --help 查看说明。" ;;
    esac
    shift
done

if [[ "${EUID}" -eq 0 ]]; then
    die "不要使用 sudo 启动整个脚本。请运行：bash scripts/setup.sh"
fi

PY_BIN=""
PY_VERSION=""
pick_python || die "未找到可用的 Python 3.10+；请安装 python3 或用 A1_LEROBOT_PYTHON 指定。"
if ((INSTALL_RECORD)) && [[ "${PY_VERSION%%.*}" -lt 3 || "${PY_VERSION#*.}" -lt 12 ]]; then
    die "系统 Python 为 $PY_VERSION（$PY_BIN），录制依赖需要 Python 3.12+。请先安装 Python 3.12+ 再重跑本脚本，或用 A1_LEROBOT_PYTHON=/path/to/python3.12 指定解释器；只做遥操时请显式加 --base。"
fi
# 本次安装对 venv 的 Python 要求：录制 3.12+，基础控制 3.10+。
if ((INSTALL_RECORD)); then
    MIN_PY_MINOR=12
    MIN_PY_DESC="3.12+（录制依赖）"
else
    MIN_PY_MINOR=10
    MIN_PY_DESC="3.10+"
fi
# 已有 venv 的 Python 不满足要求时直接删除重建（在 [3/5] 执行），
# 不把 lerobot 0.6.1（要求 >= 3.12）装进 3.10 venv 后在 pip 层失败。
VENV_REBUILD=0
EXISTING_VENV_PY="$(venv_python_version)"
if [[ -n "$EXISTING_VENV_PY" ]] && (( ${EXISTING_VENV_PY#*.} < MIN_PY_MINOR )); then
    VENV_REBUILD=1
fi
if ((INSTALL_RECORD)); then
    MODE_DESC="控制 + 录制 + 视频编解码"
else
    MODE_DESC="基础控制（--base）"
fi
[[ "$PLATFORM" == arm64 ]] && MODE_DESC="$MODE_DESC，arm64 精简（无仿真/可视化）"
printf 'Onero A1 一键部署：%s\n' "$MODE_DESC"
((VENV_REBUILD)) && printf '  已有 venv 的 Python %s 不满足本次安装（需要 %s）：将删除 %s 并重建\n' \
    "$EXISTING_VENV_PY" "$MIN_PY_DESC" "$VENV"
printf '  Python：%s（%s）\n\n' "$PY_BIN" "$PY_VERSION"

cat <<'EOF'
安装范围说明：
  - 所有 Python 包均装入虚拟环境（venv 隔离），不改系统 Python，
    不向系统或用户级 site-packages 写任何包
  - armsdk 安装为源码快照副本，不修改 armsdk 源项目文件
  - 已有 venv 的 Python 不满足本次安装时会删除该 venv 并重建
    （venv 内原有包需重新安装）；满足时只复用，不动现有环境
  - 不创建、不修改任何 CAN 接口或其配置；机械臂现有 CAN 配置保持原样，
    需要配置 CAN 时单独运行：sudo bash scripts/setup_can.sh
  - 仅当系统缺少运行库时才用 sudo apt 安装：python3-venv、ffmpeg
    （系统工具包，与 Python 环境无关）
EOF

printf '[1/5] 检查本地 SDK\n'
[[ -f "$SDK_SOURCE/pyproject.toml" && -d "$SDK_SOURCE/onero_sdk" ]] \
    || die "未找到 armsdk：$SDK_SOURCE；可用 A1_ARM_SDK_PATH 指定。"
printf '  SDK 源码：%s\n' "$SDK_SOURCE"

printf '\n[2/5] 安装缺失的系统依赖\n'
ensure_system_deps
printf '  系统依赖就绪。\n'

printf '\n[3/5] 创建或复用 Python 虚拟环境\n'
ensure_venv
"$VENV/bin/python" -I -c 'import sys; assert sys.version_info >= (3, 10)' \
    || die "A1 安装需要 Python >= 3.10。"

printf '\n[4/5] 安装 SDK、动力学、仿真及本项目\n'
# Build a snapshot in a temporary directory so setuptools does not leave
# build/ or egg-info files in the independent SDK source project.
SDK_BUILD_DIR="$(mktemp -d "${TMPDIR:-/tmp}/onero-sdk-setup.XXXXXX")"
trap 'rm -rf -- "$SDK_BUILD_DIR"' EXIT
"$VENV/bin/python" -I - "$SDK_SOURCE" "$SDK_BUILD_DIR/armsdk" <<'PY'
import shutil
import sys

shutil.copytree(
    sys.argv[1], sys.argv[2],
    ignore=shutil.ignore_patterns(
        ".git", ".venv", "venv", "__pycache__", "*.pyc", "*.egg-info",
        "build", "dist", ".pytest_cache", ".ruff_cache",
    ),
)
PY
if ((INSTALL_RECORD)); then
    # 先装 CPU 版 torch/torchvision（版本上限与 lerobot 0.6.1 的 torch 约束一致），
    # 后续安装会保留已满足约束的版本，避免从默认源拉取数 GB 的 CUDA 轮子。
    if [[ -n "$TORCH_INDEX_URL" ]]; then
        printf '  预装 torch / torchvision（CPU 版，%s，%s）\n' "$TORCH_INDEX_URL" "$ARCH"
        "$VENV/bin/python" -I -m pip install "${PIP_NETWORK_OPTS[@]}" \
            'torch>=2.7,<2.12' 'torchvision>=0.22,<0.27' --index-url "$TORCH_INDEX_URL"
    else
        printf '  预装 torch / torchvision（PyPI 默认源，注意会拉取 CUDA 依赖）\n'
        "$VENV/bin/python" -I -m pip install "${PIP_NETWORK_OPTS[@]}" \
            'torch>=2.7,<2.12' 'torchvision>=0.22,<0.27'
    fi
    "$VENV/bin/python" -I -m pip install "${PIP_NETWORK_OPTS[@]}" \
        "${LEROBOT_PKGS[@]}" \
        "$SDK_BUILD_DIR/armsdk[${SDK_EXTRAS}]" "${MUJOCO_PKGS[@]}" pyserial
else
    "$VENV/bin/python" -I -m pip install "${PIP_NETWORK_OPTS[@]}" \
        -e "$PROJECT_DIR" \
        "$SDK_BUILD_DIR/armsdk[${SDK_EXTRAS}]" "${MUJOCO_PKGS[@]}" pyserial
fi

printf '\n[5/5] 验证安装（不访问 CAN）\n'
PLATFORM="$PLATFORM" REQUIRE_RECORD="$INSTALL_RECORD" "$VENV/bin/python" -I - <<'PY'
import importlib.metadata as metadata
import os
from importlib.resources import files

import numpy as np
import pinocchio
import can
import serial
from onero_sdk.dynamics import DynamicsModel
from onero_sdk.model_registry import get_model

import onero_a1_lerobot

# arm64 板子不装 MuJoCo，跳过仿真后端验证
SKIP_SIM = os.environ.get("PLATFORM") == "arm64"
# 录制模式下录制依赖必须齐备，不能只打印“跳过”就放过（缺 datasets 时
# 录制过程正常、收尾保存数据集才失败，自检必须在这里挡住）。
REQUIRE_RECORD = os.environ.get("REQUIRE_RECORD") == "1"
if not SKIP_SIM:
    from onero_sdk.mujoco_backend import MujocoBackend

for name, side in (("A1L", "l"), ("A1R", "r")):
    assert get_model(name).num_joints == 7
    urdf = files("onero_sdk") / "urdf" / "A1" / f"a1_{side}.urdf"
    dynamics = DynamicsModel(str(urdf))
    gravity = dynamics.gravity_torques([0.0] * 7)
    assert len(gravity) == 7 and np.isfinite(gravity).all()
    if not SKIP_SIM:
        backend = MujocoBackend(model_name=name)
        try:
            assert backend.num_joints == 7
        finally:
            backend.close()
    print(f"  {name}: 动力学 / URDF{' / 仿真模型' if not SKIP_SIM else ''} OK")

packages = ["onero-sdk", "python-can", "pyserial"]
if not SKIP_SIM:
    packages.append("mujoco")
packages.append("lerobot_robot_onero_a1")
for package in packages:
    print(f"  {package}: {metadata.version(package)}")
print("  Pinocchio:", pinocchio.__version__)
print("  Adapter:", onero_a1_lerobot.__file__)

try:
    import av
    import lerobot
    # 数据集工具链（datasets / pandas / pyarrow 等由 lerobot 的 dataset extra
    # 提供，arm64 分支手工补齐）：导入失败说明录制收尾会报错。
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    import os
    import tempfile

    del LeRobotDataset  # 只验证可导入，不使用

    path = tempfile.mktemp(suffix=".mp4")
    with av.open(path, "w") as container:
        stream = container.add_stream("libx264", rate=10)
        stream.width, stream.height = 64, 48
        stream.pix_fmt = "yuv420p"
        for i in range(10):
            frame = av.VideoFrame.from_ndarray(
                np.full((48, 64, 3), (i * 25) % 255, dtype=np.uint8), format="rgb24")
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    if SKIP_SIM:
        # arm64 精简环境不装 torchcodec（aarch64 轮子绑定 CUDA），解码走 PyAV，
        # 与适配层录制/回放的 video_backend="pyav" 一致。
        with av.open(path) as container:
            decoded = [f.to_ndarray() for f in container.decode(container.streams.video[0])]
        assert len(decoded) == 10, len(decoded)
        decode_backend = "PyAV"
        packages = ("lerobot", "datasets", "av", "torch")
    else:
        from torchcodec.decoders import VideoDecoder

        frames = VideoDecoder(path)[:]
        assert frames.shape[0] == 10, frames.shape
        decode_backend = "torchcodec"
        packages = ("lerobot", "datasets", "av", "torchcodec", "torch")
    os.remove(path)

    for package in packages:
        print(f"  {package}: {metadata.version(package)}")
    print("  LeRobotDataset: 可导入")
    print(f"  视频编码(PyAV) / 解码({decode_backend}) 回环 OK")
except ImportError as exc:
    if REQUIRE_RECORD:
        raise SystemExit(
            f"ERROR: 录制依赖不完整（缺少 {exc.name}），录制收尾保存数据集会失败；"
            "请重跑 scripts/setup.sh 安装录制依赖后重试。"
        ) from None
    print(f"  跳过录制与视频依赖检查（未安装：{exc.name}）")
PY
printf '\n安装完成。虚拟环境：%s\n' "$VENV"
printf 'SDK 安装为当前源码快照；SDK 更新后重新运行本命令。\n'
if [[ "$PLATFORM" == arm64 ]]; then
    printf '\narm64 精简环境：CPU 版 PyTorch，不含 MuJoCo 仿真、rerun/foxglove 可视化、训练依赖与 torchcodec。\n'
    printf '录制/回放视频走 PyAV；无硬件自检仍可用 --simulate（内存模拟）；数据集建议拷回 x86 主机用 scripts/viz.sh 查看。\n'
fi
printf '\n每次使用前执行：\n'
printf '  cd %q\n' "$PROJECT_DIR"
printf '  source %q/bin/activate\n' "$VENV"
printf '退出虚拟环境：deactivate，或 source %q/bin/deactivate\n' "$VENV"
printf '\n无硬件验证（遥操流程）：\n'
printf '  onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_right_follow.json --simulate --duration 5\n'
if ((INSTALL_RECORD)); then
    printf '\n无硬件验证（录制流程）：\n'
    printf '  onero-a1-lerobot-record --session-config configs/sdk_left_leader_right_follow.json --simulate \\\n'
    printf '      --repo-id local/a1_test --task "sim test" --root ./datasets/local/a1_test --duration 5\n'
    printf '\n确认配置中的 CAN 通道和安装方向后运行真机遥操/录制：\n'
    printf '  onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_right_follow.json\n'
else
    printf '\n当前为 --base 环境，不含录制依赖；需要数据采集时在 Python 3.12+ 下重跑：\n'
    printf '  bash scripts/setup.sh\n'
    printf '\n确认配置中的 CAN 通道和安装方向后运行真机：\n'
    printf '  onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_right_follow.json\n'
fi
printf '\n部署与使用详见 docs/usage_sdk_follow.md。\n'
