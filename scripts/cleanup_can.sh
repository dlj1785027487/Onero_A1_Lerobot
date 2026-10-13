#!/usr/bin/env bash
# 一键清理 CAN 环境：
#   1. 结束占用 CAN 串口（ttyACM*/ttyUSB*）的残留进程（异常退出的遥操/采集会话）
#   2. 停止 slcand 守护进程
#   3. 删除 slcan 映射出的 CAN 接口（硬件 SocketCAN 接口保留不动）
# 用法：sudo bash scripts/cleanup_can.sh
set -u

say() { echo "$*"; }

if [ "$(id -u)" -ne 0 ]; then
  say "请用 sudo 运行：sudo bash scripts/cleanup_can.sh"
  exit 1
fi

# 1. 结束占用 CAN 串口的进程
serials=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
if [ -z "$serials" ]; then
  say "没有发现 CAN 串口设备"
else
  killed_any=0
  for s in $serials; do
    pids=$(fuser "$s" 2>/dev/null | tr -d ' ')
    if [ -n "$pids" ]; then
      say "串口 $s 被占用，结束进程: $(ps -o pid=,cmd= -p $(echo $pids | tr ' ' ',') 2>/dev/null | tr '\n' '; ')"
      kill $pids 2>/dev/null
      killed_any=1
    fi
  done
  if [ "$killed_any" -eq 1 ]; then
    sleep 1
    # 仍存活则强杀
    for s in $serials; do
      pids=$(fuser "$s" 2>/dev/null | tr -d ' ')
      [ -n "$pids" ] && kill -9 $pids 2>/dev/null
    done
    say "已结束所有占用串口的进程"
  else
    say "串口无进程占用"
  fi
fi

# 2. 停止 slcand 守护进程
if pgrep -x slcand >/dev/null 2>&1; then
  pkill -x slcand
  say "已停止 slcand 守护进程"
else
  say "没有运行中的 slcand 守护进程"
fi

# 3. 删除 slcan 映射出的 CAN 接口（硬件 SocketCAN 保留）
removed=0
failed=0
for ifpath in /sys/class/net/can*; do
  [ -d "$ifpath" ] || continue
  name=$(basename "$ifpath")
  if ip -d link show "$name" 2>/dev/null | grep -qi slcan; then
    # Report a failed deletion explicitly instead of silently counting zero and
    # then claiming there was nothing to delete.
    if ip link del "$name"; then
      say "已删除映射接口 $name"
      removed=$((removed + 1))
    else
      say "删除映射接口 $name 失败，请手动检查：sudo ip link del $name"
      failed=$((failed + 1))
    fi
  else
    say "保留 $name（非 slcan，硬件 SocketCAN）"
  fi
done
[ "$removed" -eq 0 ] && [ "$failed" -eq 0 ] && say "没有 slcan 映射接口需要删除"

say "清理完成。重新映射运行：sudo bash scripts/setup_can.sh <接口名> <串口路径>"
