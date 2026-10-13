# CAN 接口丢失后的手动修复

本页用于重建被内核注销的 SocketCAN 接口。正式双臂配置 `configs/sdk_dual_joint_follow.json`
使用四条 SocketCAN 接口：主臂 `can4`/`can3`、从臂 `can2`/`can1`；接口丢失后按本页重新
创建或恢复，不要用外接适配器覆盖已被其他设备使用的接口。
（只有改用 `/dev/serial/by-id/...` 让 SDK 直接打开 SLCAN 的部署才不需要创建接口。）

以下用独立接口名 `can_ext_l/can_ext_r` 举例，修复接口无需激活 venv，也无需运行 armsdk。

典型故障：USB 集线器断开后，内核注销外接 CAN 接口；重连时串口可能从
`ttyACM0/1` 变为 `ttyACM2/3`，原来的 slcand 进程仍绑定失效串口。
USB 设备已经重新识别，但 CAN 网络接口需要重新创建。

## 1. 支撑机械臂，退出控制服务

先确保机械臂有可靠支撑，再用 `q` + Enter 或 Ctrl+C 退出跟随服务。退出会请求失能。
接口修复过程中不要运行遥操作、录制控制、回放或其他控制机械臂的程序。

以下命令均在本项目根目录执行。

## 2. 检查 USB、串口和 CAN 接口

```bash
lsusb -d 16d0:117e
ls -l /dev/serial/by-id/
ip -brief link show type can
pgrep -a -x slcand
```

下面的手动 SocketCAN 示例按以下对应关系配置，实际部署须核对接线：

| 机械臂 | CAN 接口 | 适配器序列号 |
|---|---|---|
| 左臂 A1L | `can_ext_l` | `209636A34343` |
| 右臂 A1R | `can_ext_r` | `209836AD4343` |

序列号对应 `/dev/serial/by-id/` 中以该序列号加 `-if00` 结尾的设备链接。
不要用 `ttyACM0/1/2/3` 的编号判断左右臂；它们会随插拔变化。
如果更换适配器或交换了适配器与机械臂之间的 CAN 线，需要重新确认左右对应关系：
转动目标机械臂，用
`python scripts/can_precheck.py --watch 10 <通道...>` 看哪条通道的关节读数在变。
上表的左右对应关系只是示例，必须以现场实测为准。

- USB 和稳定串口链接都存在，但 `can_ext_l/can_ext_r` 缺失：继续第 3 步。
- 接口仍存在、对应关系正确但为 DOWN：跳到第 4 步启动接口，不必杀进程或重建。
- 接口已 UP 且对应关系正确：无需重建；可跳到第 5 步检查。
- USB 或对应串口链接缺失：先检查 USB 线、集线器、供电和插接，再重新检查。

## 3. 清理失效 slcand，并重建缺失的接口

以下命令只清理 **接口已经消失** 的 `can_ext_l/can_ext_r` 对应 slcand，不使用固定 PID，也不清理其他 CAN 接口。
执行前必须已完成第 1 步，并确认第 2 步的设备链接存在。

```bash
for CAN_IFACE in can_ext_l can_ext_r; do
  if ! ip link show dev "$CAN_IFACE" >/dev/null 2>&1; then
    sudo pkill -TERM -f -- "^([^ ]*/)?slcand[[:space:]].*[[:space:]]${CAN_IFACE}$" || true
  fi
done
```

按序列号创建接口。下面的通配符匹配已确认的稳定设备链接，串口改名后不需要改命令：

```bash
if ! ip link show dev can_ext_l >/dev/null 2>&1; then
  sudo bash scripts/setup_can.sh can_ext_l /dev/serial/by-id/*_209636A34343-if00
fi

if ! ip link show dev can_ext_r >/dev/null 2>&1; then
  sudo bash scripts/setup_can.sh can_ext_r /dev/serial/by-id/*_209836AD4343-if00
fi
```

脚本会加载 slcan，以 CAN 1 Mbps、串口 1 Mbaud 创建接口，设置发送队列至少为 1000 并置为 UP。
它不发送电机使能或运动指令。若报告设备不存在或串口被占用，先排查提示，不要强制重置其他设备。
如果接口原本存在，应结合 `pgrep` 输出和稳定链接的指向确认左右对应关系，不能仅凭接口名称判断。

## 4. 启动已有接口并检查队列

确认两个接口存在且对应关系正确后执行；如果刚刚由脚本成功创建，这一步可省略。

```bash
sudo ip link set dev can_ext_l up
sudo ip link set dev can_ext_r up
sudo bash scripts/setup_can.sh --tx-queue can_ext_l can_ext_r
```

## 5. 验证恢复结果

```bash
ip -brief link show type can
ip -statistics link show dev can_ext_l
ip -statistics link show dev can_ext_r
pgrep -a -x slcand
```

应看到 `can_ext_l`、`can_ext_r` 都是 UP，队列长度 `qlen` 至少为 1000；两个 slcand 分别对应当前稳定链接指向的串口。

可选的被动监听只接收、不发送 CAN 帧：

```bash
timeout 5 candump can_ext_l can_ext_r
```

五秒无数据、命令超时返回 124，不足以判定通信故障；电机未主动上报时可能没有帧。
接口 UP 只证明电脑端接口已恢复，不能单独证明机械臂供电、CAN 接线或电机反馈正常。

若接口又消失，查看近期 USB/SLCAN 日志：

```bash
journalctl -k -b --since '10 minutes ago' --no-pager | rg -i 'usb|ttyACM|slcan|can_ext_[lr]|disconnect'
```

如果没有主动拔插却反复出现 USB disconnect，需要检查 USB 线、集线器或其供电；仅重建接口不能修复物理掉线。
当前脚本不安装开机服务，也不负责自动热插拔重连。

## 6. 恢复跟随

若按本页为外接主臂适配器重建接口，正式配置 `configs/sdk_dual_joint_follow.json` 中对应
端点的 `channel` 必须指向重建出的接口名（部署现场是主臂 `can4`/`can3`）。若要改用
`/dev/serial/by-id/...` 让 SDK 直接打开 SLCAN，则必须停掉占用该串口的 slcand，两者不能
并存。只用本体 `can1`/`can2` 的单臂配置不需要本页的外接接口重建。
确认实际通道和机械臂周围空间后，再运行所需正式配置；下面的命令会使能机械臂并开始跟随：

```bash
source .venv/bin/activate
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json
```

本体单臂跟随使用 `configs/sdk_left_leader_right_follow.json`，镜像使用 `configs/sdk_single_mirror.json`。
串口数字编号变化不影响稳定 by-id 链接；更换适配器或接线时仍须更新配置。
