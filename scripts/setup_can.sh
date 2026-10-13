#!/usr/bin/env bash
# Configure one SLCAN adapter for armsdk without enabling any motor.
set -Eeuo pipefail

usage() {
    cat <<'EOF'
用法: sudo bash scripts/setup_can.sh <接口名> <串口路径>
      USB 串口适配器（SLCAN）：把串口映射成 CAN 接口
用法: sudo bash scripts/setup_can.sh --native <接口名>
      手臂直接接硬件 CAN 口（板载 SocketCAN）：设速率并启动接口，无需映射
示例: sudo bash scripts/setup_can.sh can0 /dev/ttyACM0
      sudo bash scripts/setup_can.sh --native can0
已有接口只调整发送队列: sudo bash scripts/setup_can.sh --tx-queue can0 can1

建议串口使用 /dev/serial/by-id/ 下的稳定路径。
CAN 速率为 1 Mbit/s（串口速率 1 Mbaud），与 armsdk 配置一致。
缺少 can-utils 时自动安装；只创建/启动接口，不使能或控制机械臂。
每条臂独占一个适配器和接口；左右型号由会话 JSON 指定。
发送队列至少设置为 1000 帧，为七轴突发发送留出余量；已有更大队列不缩减。
--tx-queue 不重启接口、不改变速率、不发送 CAN 帧；运行前先退出控制程序。
--native 会先关闭接口再按 1 Mbit/s 重新启动；请先退出控制程序。
本脚本不设置开机启动；重启或重新插拔后需重新配置。
EOF
}

die() { printf 'ERROR: %s\n' "$1" >&2; exit 1; }

validate_iface_name() {
    [[ "$1" =~ ^[a-zA-Z0-9_][a-zA-Z0-9_.-]{0,14}$ ]] \
        || die "接口名无效：$1"
}

ensure_tx_queue() {
    local iface="$1" current
    current="$(< "/sys/class/net/$iface/tx_queue_len")"
    # freedrive/跟随循环会突发发送；内核默认 10、小队列会被打满
    # （send 报 [Errno 105] No buffer space available 并触发 SDK 急停）。
    # 1000 帧 ≈ 1 Mbit/s 下 200ms 缓冲，足以吸收 SLCAN 串口转发的抖动。
    if ((current < 1000)); then
        ip link set dev "$iface" txqueuelen 1000
        printf '%s: txqueuelen %s -> 1000\n' "$iface" "$current"
    else
        printf '%s: txqueuelen=%s，保留现有设置\n' "$iface" "$current"
    fi
}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    usage
    exit 0
fi
if [[ "${1:-}" == --tx-queue ]]; then
    shift
    [[ $# -ge 1 ]] || { usage >&2; exit 1; }
    [[ "$EUID" -eq 0 ]] || die "配置网络接口需要管理员权限，请使用 sudo bash 运行。"
    # Validate the entire group before changing any interface. ARPHRD_CAN=280.
    for CAN_IFACE in "$@"; do
        validate_iface_name "$CAN_IFACE"
        [[ -r "/sys/class/net/$CAN_IFACE/type" ]] || die "接口不存在：$CAN_IFACE"
        [[ "$(< "/sys/class/net/$CAN_IFACE/type")" == 280 ]] \
            || die "$CAN_IFACE 不是 CAN 接口，未更改。"
    done
    for CAN_IFACE in "$@"; do
        ensure_tx_queue "$CAN_IFACE"
        ip -details -statistics link show dev "$CAN_IFACE"
    done
    exit 0
fi
if [[ "${1:-}" == --native ]]; then
    shift
    [[ $# -eq 1 ]] || { usage >&2; exit 1; }
    [[ "$EUID" -eq 0 ]] || die "配置网络接口需要管理员权限，请使用 sudo bash 运行。"
    CAN_IFACE="$1"
    validate_iface_name "$CAN_IFACE"
    # ARPHRD_CAN=280：必须是硬件/驱动提供的 CAN 接口，而不是 SLCAN 映射目标。
    [[ -r "/sys/class/net/$CAN_IFACE/type" ]] || die "接口不存在：$CAN_IFACE"
    [[ "$(< "/sys/class/net/$CAN_IFACE/type")" == 280 ]] \
        || die "$CAN_IFACE 不是 CAN 接口，未更改。若为 USB 串口适配器请用双参数映射用法。"
    if ! command -v candump >/dev/null; then
        apt-get update
        apt-get install -y --no-install-recommends can-utils
    fi
    ensure_tx_queue "$CAN_IFACE"
    # 先关闭再设速率，保证重复执行结果一致；运行控制程序前执行，关闭无副作用。
    ip link set dev "$CAN_IFACE" down
    if ! ip link set dev "$CAN_IFACE" type can bitrate 1000000 up; then
        die "$CAN_IFACE 设定 1 Mbit/s 并启动失败；请确认驱动与比特率参数。"
    fi
    ip -details -statistics link show dev "$CAN_IFACE"
    printf '\n接口已启动；未发送电机使能或运动指令。\n'
    printf '只接收检查：timeout 5 candump %q\n' "$CAN_IFACE"
    exit 0
fi
[[ $# -eq 2 ]] || { usage >&2; exit 1; }
CAN_IFACE="$1"
validate_iface_name "$CAN_IFACE"
TTY_DEVICE="$(readlink -f -- "$2")"
[[ -c "$TTY_DEVICE" ]] || die "串口设备不存在：$2"
[[ "$EUID" -eq 0 ]] || die "配置网络接口需要管理员权限，请使用 sudo bash 运行。"

if ip link show dev "$CAN_IFACE" >/dev/null 2>&1; then
    die "$CAN_IFACE 已存在，未更改。请先检查其状态和设备对应关系。"
fi
if fuser -- "$TTY_DEVICE" >/dev/null 2>&1; then
    die "$TTY_DEVICE 正被其他进程使用，未更改。"
fi

if ! command -v slcand >/dev/null || ! command -v candump >/dev/null; then
    apt-get update
    apt-get install -y --no-install-recommends can-utils
fi

modprobe slcan
printf '配置 %s <- %s，CAN 1 Mbit/s\n' "$CAN_IFACE" "$TTY_DEVICE"
slcand -o -s8 -t hw -S 1000000 "$TTY_DEVICE" "$CAN_IFACE"
for ((attempt = 0; attempt < 20; attempt++)); do
    if ip link show dev "$CAN_IFACE" >/dev/null 2>&1; then
        ensure_tx_queue "$CAN_IFACE"
        ip link set dev "$CAN_IFACE" up
        ip -details -statistics link show dev "$CAN_IFACE"
        printf '\n接口已启动；未发送电机使能或运动指令。\n'
        printf '只接收检查：timeout 5 candump %q\n' "$CAN_IFACE"
        exit 0
    fi
    sleep 0.1
done
die "slcand 启动后未出现 $CAN_IFACE；请检查系统日志。"
