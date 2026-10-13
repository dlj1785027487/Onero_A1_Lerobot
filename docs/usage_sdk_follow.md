# CAN 直连主从跟随：单臂与双臂

本项目使用本地 `armsdk` 的公共 `Arm` API 直接访问 A1L/A1R 的 CAN 通道。
LeRobot 设备类型为 `onero_sdk` / `onero_sdk_teleop`，Python 包为 `onero_a1_lerobot`。
机械臂真机控制使用 CAN，仿真保留 mock 和 SDK MuJoCo 后端；模型及网格均由 SDK 提供。
可选 USB 相机直接接入部署电脑，图像采集与 CAN 跟随独立运行。

提供 `mirror` 镜像和 `joint` 逐关节映射，每条臂有七个关节位置目标。
遥操类型由 `teleop.reference` 一处切换：`relative` 增量（从臂保持起始位姿，只复现主臂位移）、
`absolute` 全量（每个关节直接复现主臂自身角度），见第 3 节。
CAN 真机且 `with_gripper=true` 时，同时启用夹爪控制与 `{side}_arm.gripper.pos` 特征，
范围为 0~1，1 为张开；主臂夹爪开合度传给从臂并进入录制数据。
无夹爪时单臂/双臂为 7/14 维，两臂均启用夹爪时为 8/16 维；mock/mujoco 不含夹爪特征。
启用相机后观测增加独立 RGB 图像，不改变关节及夹爪特征的维度。

实体夹爪沿用 SDK 的两个独立配置项，两者互不推导，也不会改动 `with_gripper`（仅选择重力补偿档位）：

`gripper_protocol` —— 实体 CAN 协议：

- `"onero"`：自研夹爪，DM 兼容 MIT 协议，默认值；端点标定按本臂明确型号 `A1L`/`A1R` 选择。
- `"omnipicker"`：旧款 8 字节协议，可用 `gripper_can_id`、`gripper_force`、`gripper_velocity`
  覆盖默认的 `0x08` / `0x20` / `0xFF`。
- `"none"`：不发送任何夹爪 CAN 帧（此时 `with_gripper=true` 只作用于重力补偿）。

`gripper_model` —— Link7 上的可视化工具模型，默认 `"woan_picker"`，可选 `"omnipicker"`、`"none"`。
它只影响外观，不选择协议、不改 TCP 或重力补偿；与 SDK 的 `--gripper-model` 同义。

**这两个字段只对从臂（`robot.arms`）生效。** 主臂的开合度来自末端改装的按压式手柄，不按装配选型，
所以 `gripper_protocol`、`gripper_model`（以及 `gripper_can_id`/`gripper_force`/`gripper_velocity`）
在主臂上**会在字段校验之前被丢掉**，写什么都不会导致启动失败，主臂实际固定使用
`onero` + `woan_picker`（丢弃时会记一条 INFO 日志）。主臂真正需要填写的是手柄实测行程
`handle_open_code` / `handle_closed_code`，见下节。配置文件里主臂因此不写夹爪选型项，
只有从臂按现场装配填写，例如：

```json
"robot":  { "arms": { "right": { "model": "A1R", "backend": "can", "channel": "can2",
                                 "mounting": "humanoid_right", "with_gripper": true,
                                 "gripper_protocol": "omnipicker",
                                 "gripper_model": "omnipicker" } } },
"teleop": { "arms": { "left":  { "model": "A1L", "backend": "can", "channel": "can1",
                                 "mounting": "humanoid_left", "with_gripper": true } } }
```

配套的规则与校验：

- **主臂固定**：主臂的夹爪字段不参与选型，加载时整组丢弃并回落默认的 `onero` / `woan_picker`。
  从臂上的非法取值（例如拼错的 `"woan"`、越界的 `gripper_force`）仍然会被字段校验拦住。
- **主从必须同时启用或同时停用夹爪**：从臂的夹爪动作由主臂的开合度读数驱动，只有一侧有
  `gripper_channel` 会报错，而不是留到运行时才因动作维度不一致失败。
- **主臂不能在 CAN 上被动读取 OmniPicker**：OmniPicker 在 CAN/CAN-FD 下没有只读查询帧
  （智元手册 6.3.2："每当夹爪收到一次控制指令，会立即上报一帧状态数据"），只能回报最近一次
  命令的值。主臂既然已被固定为 `onero`，这条只在直接构造设备配置时才会遇到，
  此时连接会在动 CAN 之前报错，而不是静默按 0（全闭）下发给从臂。

### 主臂手柄的来源与标定

**这一节适用于末端是手动开合手柄的主臂，也就是成对填写了 `handle_open_code` /
`handle_closed_code` 的那些主臂。** 现场几只主臂装的都是同一类手柄，所以五份会话配置的
主臂条目都写了这两项：`sdk_left_leader_right_follow.json`（`can1`）、
`sdk_right_leader_left_follow.json`（`can1`）、`sdk_single_mirror.json`（`can2`），
以及两份双臂配置（`can4`/`can3`）。没写这两项的主臂会退回普通夹爪驱动——既不使用实测
端点，也会跟随机械臂一起被使能——见本节第 3 条。

手柄是一个手动开合机构，末端电机与从臂夹爪同型（DM 电机，命令 `0x08`、反馈 `0x18`），
所以 `gripper_protocol` 固定为 `onero`，读取方式也是同一套被动状态查询。
**填写了端点的**主臂按**只读**方式驱动：`set_position()` 直接拒绝，任何情况下都不会把位置
命令下发给主臂手柄，所以手柄不会被拖走、也不会自己跑。但"松"并不是靠不给它通电实现的：
实测这类手柄电机上电后若处于"通电但未收到指令"的状态，驱动器会把三相短接，反电动势在
绕组里环流形成很强的制动，手一拨就很顶，只有整机断电才松；而 DM 的失能帧对这颗电机无效
（它连应答都没有）。所以连接时改为**先发 DM 使能帧、紧跟一帧 `kp=kd=力矩=0` 的目标**：
电机被使能，驱动器则主动把反电动势电流抵消掉，轴几乎没有可感知的阻力。位置项不带增益，
所以既不会顶住手、也不会自己跑。运行中若读到它又掉回制动状态（`status=0`），最多再重新
声明 5 次。
差别在机械零位：手柄的行程端点与夹爪型号标定（`A1L`/`A1R`）并不重合，
因此必须用实测端点替换型号标定，否则手柄整段行程只会映射成满量程的百分之几。

在 `teleop.arms` 里成对给出端点（16 位位置码）：

```json
"teleop": { "arms": { "right": { "model": "A1R", "backend": "can", "channel": "can3",
                               "mounting": "humanoid_right", "with_gripper": true,
                               "handle_open_code": 29385,
                               "handle_closed_code": 32821 } } }
```

- 码值随闭合增大，与夹爪自身端点方向一致，因此 `handle_open_code` 是**较小**的值
  （当前校验只要求两项不相等，写反不会被拦下，需按实测结果确认方向）；
- 两项必须同时给出，且只允许写在主臂条目上，写在从臂上会被配置校验直接拒绝；
- **是不是手柄由这两项决定**：写了才走上面的只读手柄驱动、并用实测端点换算开合度；
  不写就按普通夹爪驱动，用夹爪型号标定，而且**跟随机械臂一起被使能**。所以在真机手柄
  主臂上漏填，表现为电机把手柄锁住、用手推不动，同时开合度只会映射成满量程的百分之几，
  而不是"从臂夹爪几乎不动"。

标定方法（保持 CAN 空闲，不需要额外工具）：

1. 用手把该手柄从完全闭合压到完全张开，来回几次；
2. 同时读该通道的夹爪反馈——向 `0x7FF` 发送 `08 00 CC 00 00 00 00 00`，
   接收 `0x18` 回帧，位置码为 `data[1] << 8 | data[2]`；
3. 全程最小码值填入 `handle_open_code`，最大码值填入 `handle_closed_code`。

左右两只手柄的机构若不一致，各自标定，不要互相套用。

运行时的夹爪保护：驱动上报非零 `fault_code`（Onero 的 DM 状态 8~14、OmniPicker 的 D0
故障码），或状态查询**连续无响应达到 `gripper_query_failures_before_fault` 次（默认 3，
约 150 ms；设为 1 恢复"首次无响应即报"的严格行为）**时，会锁存一个**独立的夹爪故障**，
随后拒绝夹爪命令、从臂/主臂进程各自报出故障并让会话停位；下一次成功查询（故障码为 0）
会自动解除，并把连续计数清零。计数存在的理由：状态查询是单发 CAN 往返、SDK 驱动不重试，
而真机上 CANable2 走 slcan（USB CDC-ACM）时偶发单帧丢失（SDK 自己也报过
`can4: Joint 5: 3 consecutive missed frames`），一次丢帧不应该终止整场录制；未达阈值期间
只记 WARNING 并沿用上一次读数。它刻意不走机械臂故障通道——机械臂一侧仍然健康、必须继续
保位，而机械臂故障会让 `hold()`
报 `POSITION HOLD UNAVAILABLE` 并跳过保位。OmniPicker 容忍回帧丢失，
若从臂自首次命令起始终没有任何反馈，会记录一次警告（此时记录的是命令目标值而非实测开合度）。

若旧配置只写了 `with_gripper`，未写 `gripper_protocol` 时按默认 `onero` 处理，保持原有行为。

当前单臂配置按现场两臂均带夹爪、无额外负载的情况显式设置了
`with_gripper=true`，并在从臂上写明 `gripper_model` 与 `gripper_protocol`。拆掉夹爪时需相应改为 false；扩展双臂时各端点也应按实际装配填写。

## 1. 先在当前源码目录试运行

无需 CAN、SDK、LeRobot 或 GPU；需要 Python 3.10+ 和 NumPy：

```bash
# 左主臂 → 右从臂，合成主臂轨迹，运行 5 秒，不录制
PYTHONPATH=src python3 -m onero_a1_lerobot.cli.lerobot_teleoperate \
  --session-config configs/sdk_single_mirror.json --simulate --duration 5

# 左主臂 → 右从臂，逐关节方向映射，运行 5 秒，不录制
PYTHONPATH=src python3 -m onero_a1_lerobot.cli.lerobot_teleoperate \
  --session-config configs/sdk_left_leader_right_follow.json --simulate --duration 5
```

`--simulate` 会将配置中的所有机械臂和相机替换成内存模拟端点，不打开 CAN 或 USB 相机。
`--simulate mujoco` 仅将机械臂换成 SDK 物理仿真，相机仍生成合成图像，不是 MuJoCo 渲染。
它验证控制流程、数据接口和配置，不代表真实动力学或真机性能。
退出时会输出控制帧数、录制帧数及录制错误。

### SDK MuJoCo 可视化仿真

想看到机械臂运动，使用 `--simulate mujoco --viewer`。先使用统一设置脚本安装 SDK 环境：

```bash
# 在本项目根目录执行；已有 .venv 时直接复用
A1_LEROBOT_VENV="$PWD/.venv" bash scripts/setup.sh
source .venv/bin/activate

# 在有图形桌面的终端运行，无需启动 SDK 服务
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_left_leader_right_follow.json --simulate mujoco --viewer

# 镜像跟随，使用同一个仿真窗口
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_single_mirror.json --simulate mujoco --viewer
```

x86_64 默认安装包含 MuJoCo；arm64 的精简部署不默认安装仿真依赖。
已安装 SDK 仿真依赖时只需激活同一个 venv。
仿真使用 SDK 的 `MujocoBackend` 与 `Arm.servoj()`；不修改 SDK、不打开 CAN。
仿真前馈使用 MuJoCo 已计算的动力学偏置，不要求 Pinocchio。
CAN 真机仍使用 SDK 的重力补偿，设置脚本会安装其所需的 Pinocchio。

窗口里橙色是主臂，蓝色是从臂，并显示角色标签。窗口默认从手动静止状态开始；
按 `A` 才启动自动演示。无窗口的仿真仍自动产生主臂轨迹。
点击窗口后可以直接按以下键，无需 Enter：

| 按键 | 功能 |
|---|---|
| `1`–`7` | 选择当前主臂的关节 |
| `-` / `=` | 短按微调 0.02 rad；长按超过 0.25 s 后以 0.15 rad/s 连续减少 / 增加 |
| `A` | 切换自动演示 / 手动模式，并重新捕获跟随参考 |
| `[` / `]` | 双主臂场景中切换要操作的主臂 |
| `P` | 暂停 / 恢复整个跟随会话 |
| `R` | 开始 / 结束录制，需要提前提供数据集参数 |
| `Q` | 退出 |

长按松开后停止增加目标，虚拟伺服会收敛到最后目标。切出窗口、暂停、切换关节或主臂时，
清除长按状态，需重新按下才继续；不依赖操作系统的按键连发速度。小键盘 `-` / `+` 也可用。
从自动切到手动时会重新捕获双方参考，避免从臂继续追赶自动演示留下的旧目标。

鼠标左键拖动用于转动视角，右键拖动用于平移，滚轮用于缩放；手动关节控制使用上述按键。虚拟主臂收到目标后经过 SDK
动力学仿真，实际关节反馈再经主从映射发送给虚拟从臂。

单个窗口复用 SDK 的 URDF、网格和左右肩展示坐标，并排呈现各后端的真实 qpos/qvel；
展示场景只做正运动学，不另跑一套物理引擎，也不会通过修改画面直接改变 SDK 状态。
各臂的物理仿真相互独立，该画面不能用于验证跨臂碰撞。
MuJoCo 物理步进周期采用会话的 `robot.control_hz`，例如 50 Hz 时每次控制推进 20 ms，
与控制循环的时间一致；不是以默认 5 ms 步长运行一个 20 ms 周期的程序。仿真后端使用原始 SDK 单臂
基座坐标，不使用真机 CAN 通道。JSON 的 `mounting` 用于窗口中的安装方向，
不会施加到单臂仿真的动力学基座。`simulation_initial` 只设置模拟初始关节角，
让逐关节跟随示例从屈肘姿态开始；真机忽略该字段，以实际反馈为起点，不执行预设对齐动作。

这两条命令没有 `--duration`，窗口会持续运行；关闭窗口或按 `Q` 退出。
如果仍传入 `--duration 5`，会在窗口就绪后运行五秒再关闭。
无桌面环境时可以省略 `--viewer`，继续运行 SDK MuJoCo 物理仿真。
单独 `--simulate` 等价于 `--simulate mock`，仍是无界面的内存测试。

安装本项目后可以将上面的 `PYTHONPATH=src python3 -m ...` 替换成：

```bash
onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_right_follow.json --simulate
```

## 2. 接入真机

统一使用 `scripts/setup.sh` 创建或复用 venv，并安装
本地 SDK、Pinocchio 及本项目；x86_64 同时安装 MuJoCo。SDK 源码默认位于相邻的 `../armsdk`：

```bash
# 在本项目根目录执行
A1_LEROBOT_VENV="$PWD/.venv" bash scripts/setup.sh
source .venv/bin/activate
```

也可以使用默认虚拟环境位置 `~/.venvs/a1-lerobot`：

```bash
bash scripts/setup.sh
source ~/.venvs/a1-lerobot/bin/activate
```

`A1_LEROBOT_VENV` 省略时默认 `~/.venvs/a1-lerobot`；
两种命令选择一种即可。SDK 不在相邻目录时，通过 `A1_ARM_SDK_PATH` 指定源码路径。
SDK 从临时副本构建后安装进 venv，不在 SDK 项目里生成构建文件；更新 SDK 后重新运行
安装命令。本项目采用 editable 安装，修改本项目源码后直接生效。

默认一键安装要求 Python 3.12+ 和可用的 `venv` / `pip`，同时安装 CPU 版
PyTorch、LeRobot 0.6.1 录制依赖及视频编解码组件（系统缺 ffmpeg 时
自动用 apt 安装）；仅需控制时用 `bash scripts/setup.sh --base` 只装基础环境
（Python 3.10+，不装 PyTorch / LeRobot），录制依赖再按第 4 节补充。
x86_64 默认包含 MuJoCo 3.12.0、可视化、训练及 torchcodec；arm64 使用 PyAV，
不安装 torchcodec 或需要本机编译的 hardware extra。两种架构的 PyTorch 默认使用官方 CPU 源。
脚本验证依赖、模型和视频编解码链路，不访问 CAN 或 USB 相机。平台专用和旧仿真安装脚本已移除，统一使用此入口。
安装包含 `python-can` 和 `pyserial`，用于可选的 `/dev/serial/by-id/...` 直连 SLCAN
（SDK 与 `can_precheck.py` 都支持这种写法）；正式双臂配置本身走 slcand 转出的 SocketCAN 接口。
`--sdk` 参数保留为默认安装流程的别名。

每次新开终端，在项目目录激活所选的 venv 后直接使用命令，无需设置 `PYTHONPATH`
或单独运行 armsdk 服务。CAN 通道必须预先在系统中配置好。

正式双臂配置 `configs/sdk_dual_joint_follow.json` 使用四条互不相同的 SocketCAN 接口：
主臂是外接 CANable2 适配器（SN `209636A34343` / `209836AD4343`）经 slcand 转出的接口
（左 `can4`、右 `can3`），从臂是本体原生 CAN 口（左 `can2`、右 `can1`）。
那对适配器串口已由主臂的 slcand 占用，所以不能再把 `/dev/serial/by-id/...` 写进 `channel`：
slcand 与 pyserial 同时打开同一串口会在 `tcflush` 上报 `ENOTTY`。这些通道必须按实际 CAN
接线核对，不能根据接口编号推断机械臂身份。
只有改用 `/dev/serial/by-id/...` 让 SDK 直接打开 SLCAN 的部署才跳过创建接口，但必须先把
串口从 slcand 手里释放出来，并相应修改 JSON 的 `channel`：

```bash
# 查看适配器的稳定设备路径，再用实际路径替换下面的占位符
ls -l /dev/serial/by-id/
sudo bash scripts/setup_can.sh can_ext_l /dev/serial/by-id/实际左臂适配器名称

# 本体已有 can2/can1 时，只调整发送队列，不重建接口（先退出跟随程序）
sudo bash scripts/setup_can.sh --tx-queue can2 can1

# 只接收总线数据，五秒后退出；没有帧时可能因超时返回 124
timeout 5 candump can2
```

脚本缺少 `can-utils` 时会安装它，以 SDK 使用的 CAN 1 Mbit/s、串口 1 Mbaud
创建接口，并将发送队列至少设为 1000 帧，为七轴集中发送留出余量。
不使能电机，不发送运动指令。此操作是系统网络配置，无需激活 venv。
手动创建的 `can_ext_l` 是接口名称，设备序号不代表左臂或右臂。只有一条左臂时，
它也可能是 `/dev/ttyACM0`。创建模式遇到已有接口或串口被占用时会退出，不重置其他连接。
`--tx-queue` 只调整已有 CAN 接口的队列，不修改波特率或 UP/DOWN 状态，已有更大队列不会缩减。
重启或重新插拔适配器后需重新配置；当前不安装开机服务。
接收不到数据不能单独证明机械臂离线，尤其是尚未使能、没有主动反馈时。
USB 重连后接口消失、串口编号变化时，可按 [CAN 接口丢失后的手动修复](can_recovery.md)
检查、清理失效 slcand，并按已确认的设备序列号重建接口。

检查并按现场情况修改 JSON 文件中的型号、通道和安装方向：

| 配置文件 | 主臂通道 | 从臂通道 | 关系 |
|---|---|---|---|
| `configs/sdk_left_leader_right_follow.json` | 本体左 `can1`（A1L） | 本体右 `can2`（A1R） | 逐关节方向转换后跟随 |
| `configs/sdk_right_leader_left_follow.json` | 本体右 `can1`（A1R） | 本体左 `can2`（A1L） | 逐关节方向转换后跟随 |
| `configs/sdk_single_mirror.json` | 本体左 `can2`（A1L） | 本体右 `can1`（A1R） | 七个关节增量全部取反 |
| `configs/sdk_dual_joint_follow.json` | 外接左 `can4`、右 `can3`（CANable2 经 slcand） | 本体左 `can2`、右 `can1` | 同型号同侧逐关节跟随 |
| `configs/sdk_dual_joint_follow_cameras.json` | 与双臂配置相同 | 与双臂配置相同 | 双臂跟随并采集两路 USB 腕部相机 |

前四份配置不启用相机，只有 `sdk_dual_joint_follow_cameras.json` 在双臂配置上增加 `robot.cameras`。
相机设备名必须按部署电脑修改，详见 [USB 双腕相机与视频录制](usage_usb_cameras.md)。
另有 `configs/native_dual_joint_follow_{teleoperate,record}.json` 两份配置给原生
`lerobot-teleoperate` / `lerobot-record` 使用，它们走 `--config_path` 而不是
`--session-config`，见第 5 节。
外接主臂那对 CANable2 适配器的稳定串口路径为（已由 slcand 转成 `can4`/`can3`）：

```text
/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209636A34343-if00
/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209836AD4343-if00
```

通道、型号及安装方向必须与部署机器的实际接线一致；适配器序列号不是通用默认值。
SDK 作为独立依赖维护，本项目配置调整不修改其源码。

每条物理臂独占一个 CAN 通道。SDK 默认两臂的电机 ID 相同，不能直接在同一总线上
区分两条臂；多通道 CAN 适配器可以提供多个独立通道。程序会拒绝重复通道配置。
示例采用 `humanoid_left/right` 安装方向，台架或其他装配必须改成实际对应的 SDK 安装配置。

确认配置后，省略 `--simulate` 即运行真机：

```bash
onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_right_follow.json
```

这条命令使能主臂并直接进入重力补偿拖动模式，从臂使能后保持当前位置。
全部机械臂连接完成后采集实际姿态作为本次运动参考，再开始跟随；主臂启动不增加位置保位到拖动的切换。
会话内的相机连接与录制器启动独立并行，不作为机械臂启动的前置条件。
隔离 CAN 示例的四条臂并行连接，先就绪的从臂持续保持，等待其他机械臂就绪。
CAN worker 使用纯 SDK 设备配置和按需包导出，避免四个工作进程重复导入 LeRobot/PyTorch；
仍使用 `spawn` 和原有 SLCAN 驱动约 2 秒的稳定等待。`Startup:` 记录进程、导入和设备连接耗时，
本次优化的实际改善尚未测量，不承诺固定几秒启动。
示例显式配置 `robot.send_action=true`；SDK Robot 类本身默认 `false`。
`false` 只关闭跟随动作，连接仍会使能从臂并建立初始保持；无硬件验证请使用 `--simulate`。
从臂控制先做有限值检查、首帧距离检查和按实际周期计算的速度/加速度限制。
本项目的关节位置裁剪是可选项，示例均设置 `robot.clip_joint_limits=false`，省略时也默认关闭。
开启后按 SDK 型号的关节范围裁剪目标，并为当前位移及后续制动预留空间，提前减速停在边界内。
SDK CAN 路径在打开设备前检查动力学依赖，缺少 Pinocchio 时直接报错。

退出会话时，程序结束拖动和跟随，尝试停止后，通过 SDK `close(disable=True)`
向所有已连接的主臂、从臂发送失能指令，再关闭通信。单臂示例会失能两条臂，
双臂示例会失能四条臂；退出前须确保机械臂有可靠支撑，因为失能会卸掉电机扭矩。
`q` + Enter、Ctrl+C、`--duration` 到期和 SIGTERM 均进入此流程，
机械臂连接失败仍会清理已连接设备；运行控制异常则保留 CLI 进程和可控臂的位置保持，
等待操作人员处理，不再因异常自动退出并失能全部机械臂。
`p` 暂停仍持续保持姿态，不失能；仅结束录制也不会失能。

### 运行异常时保位

这些配置复用同一套单臂/双臂故障处理，无需增加启动参数。
正常跟随保留速度/加速度、状态时效及跟踪误差检查；本项目的目标位置裁剪由配置开关控制。
主臂不增加基于 0.05 rad 预留距离或 0.02 rad 运动量的暂停规则，也没有额外的手动回撤状态。

机械臂控制异常时停止整组跟随和录制，进入 `fault`，CLI 保留进程。
相机故障单独结束当前录制，不使 CAN 跟随进入控制故障状态。
各臂并发开始停止/保持，随后持续刷新仍可控制的主臂、从臂，保持指令使用零速度前馈。
每五秒提醒停止原因和保位状态；`p` 不能解除故障锁定，排查后再重启。
普通 `p` 暂停仍可用 `p` 恢复，恢复重新捕获相对参考且不自动开始录制。

保位失败的端点单独打印 `POSITION HOLD UNAVAILABLE`，其他可控臂继续保持。
SDK 急停可能先失能电机再通知应用；已经失能的臂无法保位，程序不会自动重新使能。
本修改防止应用控制异常导致整个进程退出并失能其他健康臂，不能承诺 SDK 急停、
通信中断、驱动故障、掉电或底层 I/O 卡死时仍有支撑力。
`q`、Ctrl+C、SIGTERM、窗口关闭和显式 `--duration` 到期仍会请求失能，退出前须支撑机械臂。

## 3. 遥操类型、相对跟随与配置扩展

### 遥操类型开关（增量 / 全量）

遥操类型写在会话配置的 `teleop` 段，一个字段同时作用于本次会话的所有 mapping：

```json
"teleop": {
  "reference": "relative",
  "arms": { "...": "..." },
  "mappings": { "...": "..." }
}
```

- `"relative"`（增量）：会话开始时记录主从**双方**的实际角度，之后只复现主臂的位移。
  从臂的起点由它自己决定，启动时两臂位姿不必相同：

  ```text
  q_follower = q_follower_start + scale × signs × (q_leader − q_leader_start)
  ```

- `"absolute"`（全量）：每个关节直接复现主臂**自身**的角度，等于把主臂的位姿整体交给从臂：

  ```text
  q_follower = scale × signs × q_leader + offset
  ```

省略该字段时，每条 mapping 保持自己在 `teleop.mappings.<side>.reference` 里声明的值，
混合配置（一条臂全量、另一条增量）用这种写法。显式写出该字段时它覆盖所有 mapping——
于是"同一个启动命令、只改配置文件"即可切换全量/增量，自研 CLI 与原生
`lerobot-teleoperate` / `lerobot-record` 两条路径行为一致（两处都在
`OneroSdkTeleopConfig.validate()` 应用）。五份会话配置都显式写了这个字段，
因此改它一处就能整场切换，配置里也看不到被覆盖的逐臂 `reference`。

全量的三个前提，缺一不可：

1. **`scale` 必须是 1。** 全量下 `scale` 乘的是主臂自己的关节角，改变的是从臂**零点**而不是
   行程，结果是一组没人要的位姿；`mode="joint"` 时该组合在加载期直接报错。
   主从零点或安装差异请写进七维 `offset`。
2. **`mode="joint"` 的 `joint_signs` 必须现场核对。** 方向表由主从型号与安装方式决定，
   全量下它作用在绝对角上，不能直接沿用增量时标定出的那张表；先按下面的肩部参考姿态核对方向，
   再低速试运行。
3. **启动时从臂必须已经在主臂附近。** 全量的第一条指令就是主臂当前位姿，所以
   `robot.startup_max_delta_rad`（默认 0.15 rad ≈ 8.6°）会逐关节比较首帧目标与从臂实际角度：
   超限即拒绝跟随、锁存故障并保位，报错里逐个列出超限关节的 `target` / `follower` / `delta`。
   程序**不自动对齐**。要么先把从臂手动摆到该阈值内再启动，要么显式调大
   `startup_max_delta_rad`，让限幅器把从臂拉过去（那会真的动臂，仍受 `max_velocity` /
   `max_acceleration` 约束）。

首帧保护只在第一条指令生效，之后由 `robot.max_tracking_error_rad` 负责跟踪误差保护。
暂停恢复时全量不需要重建参考，增量会重新捕获。

### 可选的关节位置裁剪

在会话配置的 `robot` 中设置：

```json
"clip_joint_limits": false
```

- `false`（默认）：本项目不裁剪跟随目标，也不按模型位置边界提前制动；仍限制速度、加速度和连续模式的单次步长。
- `true`：启用本项目的目标位置裁剪、边界减速和 `target clipped` 日志。单条从臂的七个关节、双条从臂的十四个关节统一使用该开关。

这只控制本项目的跟随规划，不关闭 SDK 内部限位、速度限制或急停，也不改变启动/暂停/异常时的保位目标处理。
即使设为 `false`，真机和 MuJoCo 的 SDK 仍可能裁剪收到的目标，从臂实际活动范围不会因此扩大。
关闭时应用使用 SDK 名义范围内的参考检查跟踪误差，避免把 SDK 的正常边界裁剪误报为伺服故障；
实际反馈偏离该参考过大仍会进入故障保位。
本项目的未裁剪轨迹可能继续向范围外推进，从超限目标反向返回时可能增加恢复跟随的等待时间。
此开关无法解决主臂 SDK 拖动看门狗急停导致的失能问题。

### 逐关节方向映射（对应关节动作复刻）

`configs/sdk_left_leader_right_follow.json` 使用 `mode="joint"`。主臂反馈直接生成从臂关节目标：

```text
q_right[i] = q_right_start[i] + joint_signs[i] × scale × (q_left[i] - q_left_start[i])
```

```json
"teleop": {
  "reference": "relative",
  "mappings": {
    "right": {
      "source": "left", "mode": "joint", "scale": 1.0,
      "joint_signs": [-1, 1, 1, -1, 1, -1, 1]
    }
  }
}
```

遥操类型不写在这里：它由 `teleop.reference` 统一给出（上例是增量）。

该方向表根据 SDK A1L/A1R 模型的标准肩部参考姿态核对：J1、J4、J6 反号，
J2、J3、J5、J7 同号。不同安装或关节顺序不能直接套用该表；型号和 mounting 仍填写实际装配。
只改变主臂某一关节的反馈时，映射只改变从臂对应关节目标，不通过逆解联动其他关节。
同款电机并不意味着左右安装后，编码器正方向在空间里也相同。

此模式不使用 IK；内存模拟也不需要 SDK 或 Pinocchio。真机仍使用 SDK 的 CAN 和动力学补偿。
首帧和暂停恢复按双方实际角度建立参考，不要求两臂数值角度相等，不执行自动对齐。
正式配置的请求频率均为 `robot.control_hz=200`（5 ms/周期），
`max_velocity=[3,3,3,3,3,3,3]` rad/s，`max_acceleration=[80,80,80,80,80,80,80]` rad/s²。
关节按主臂输入运动，并非自动满速。速度和加速度支持标量或按 J1–J7 排列的七项值；
连续模式逐关节保留 SDK 型号上限与单步限制。
这些是应用侧的轨迹限制，不是电机增益，也不会改变 SDK 主臂拖动的 200 Hz。
正式配置启用主从 CAN 进程隔离、`robot.control_cpu_affinity="auto"`，默认不录制。
实际完成频率仍取决于计算、收发和调度耗时，不保证等于请求频率。

镜像、单臂和双臂逐关节配置均已启用 `robot.continuous_follow=true`。应用按连续移动目标
生成位置和速度：对目标差分速度做 20 ms 短时滤波，再限制估计值，避免反馈量化或缓存重复
被误判为反复停走；不再每一帧都预留到该目标的完整停车距离。
速度前馈由 `robot.velocity_feedforward_scale` 缩放后，通过 SDK 公共
`Arm.servoj(..., velocities=...)` 发送。正式配置为 `[1,1,1,1,0,0,0]`：J1–J4 保留前馈，
仅 J5–J7 的速度前馈设为零。它不关闭重力补偿或阻尼，不改变限幅器内部速度、
位置轨迹及加减速度约束，也不改变录制的 7/14 维位置字段。
该参数支持 0–1 标量或七项数组；省略时默认 1。电机增益和 SDK 源码不变。

暂停、退出和连接初始保持均使用零速度前馈；恢复时重新建立目标历史。
最大速度、加速度、反馈陈旧及跟踪误差保护仍保留；本项目的关节边界制动仅在 `clip_joint_limits=true` 时启用。连续模式还采用 SDK 的
模型速度上限和单次位置步长上限，并预留可变周期下的减速余量；
控制间隔超出 `max_control_gap_s` 时停止跟随，不继续追赶旧目标。

低滞后的取舍：主臂若突然停下或反向得比从臂允许的减速度更快，从臂会继续制动，
可能短暂越过当前目标后回到它。按 3 rad/s、80 rad/s² 估算，
匀速骤停的指令制动距离约 0.056 rad（3.2°），实际机械响应还受负载和伺服影响。
因此不能承诺任何动作下零延迟或停手即停。正常连续、平滑的拖动才是此模式的主要用法。

配置省略 `continuous_follow` 或设为 false 时，仍可使用原来的逐点减速、零速度前馈策略。
主臂运动超过从臂的速度或加速度上限时仍会出现落后；提高循环频率不能消除这种限幅。
若当前参数下仍明显跟不上，先用诊断区分软件限速和实际伺服跟踪误差，再决定如何调整。

目标仍经过速度、加速度和跟踪误差检查，并由 SDK 执行最终的型号限位。左右臂关节范围不同，
特别是向内摆动时可用范围可能较小；关闭本项目裁剪也不能保证主臂所有角度都能复制。

这是对应关节动作的约定，不保证任意组合姿态下末端始终沿相同空间轨迹运动。
两种映射均不包含碰撞检测，运行时应给从臂留出无阻挡的运动空间。

### 原来的镜像跟随

`configs/sdk_single_mirror.json` 使用 `mode="mirror"`，七个关节增量全部取反：

```text
q_target = q_follower_start - scale × (q_leader - q_leader_start)
```

两种映射（`mirror` 与 `joint`）都支持 `reference="relative"`：首帧保持从臂当前位置，
暂停恢复时重新捕获参考；也都支持 `reference="absolute"` 和七维 `offset`——全量的前提与
启动保护见本节开头的"遥操类型开关"。
位置伺服增益使用 SDK 模型默认值。镜像配置的控制参数已与逐关节配置同步：
200 Hz、每关节 3 rad/s、80 rad/s²、`continuous_follow=true`，仅 J5–J7 关闭速度前馈；
映射仍为七个关节增量全部取反。

### 扩展为一对主臂控制另一对从臂

直接使用 `configs/sdk_dual_joint_follow.json`：外接主左 A1L / `can4` → 本体从左 A1L / `can2`，
外接主右 A1R / `can3` → 本体从右 A1R / `can1`；两只主臂是 SN `209636A34343` / `209836AD4343`
的 CANable2 适配器经 slcand 转出的 SocketCAN 接口。
默认不录制，运动限制与速度前馈设置沿用两份单臂配置。
两只从臂在独立进程内执行控制，每轮等待双方反馈事务完成后再推进，避免堆积旧目标。
实际循环频率不保证达到请求的 200 Hz。

```bash
# 无需接入真机，同一窗口显示四条机械臂；[ / ] 切换要操作的主臂
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_dual_joint_follow.json --simulate mujoco --viewer

# 四条物理臂及其独立 CAN 接口准备好后运行真机
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json
```

配置在 `teleop.arms`、`robot.arms` 中各声明左右两臂，
四个端点使用独立 CAN 通道。主左→从左、主右→从右在同型号、同安装方式下
可用 `joint` 模式配合七个 `+1`：

```json
"reference": "relative",
"mappings": {
  "left": {
    "source": "left", "mode": "joint",
    "joint_signs": [1, 1, 1, 1, 1, 1, 1]
  },
  "right": {
    "source": "right", "mode": "joint",
    "joint_signs": [1, 1, 1, 1, 1, 1, 1]
  }
}
```

两条路由复用同一套设备、限速、录制及退出失能流程。四个端点均按肩部安装、带夹爪配置，
接入前按实际装配核对型号、通道、安装方向、`with_gripper` 和 `gripper_protocol`。退出时会请求失能四条臂。

### CAN 发送队列异常

`[Errno 105] No buffer space available` 表示发送时遇到缓冲／队列资源不足。
发送队列仅有 `txqueuelen=10` 时，七轴集中发送可能造成队列丢包并触发 SDK 急停。
在 7 轴 A1 模型中，SDK 后台拖动按 200 Hz 运行，每周期集中发送 7 条电机目标；
`robot.control_hz` 只控制本项目的跟随主循环，不会降低 SDK 后台拖动的频率。
启动日志会分别打印实际拖动频率和可读取的接口队列长度。

先退出控制程序，再使用上述 `--tx-queue` 将短队列扩至至少 1000 帧。
这只是为瞬时发送突发增加余量，不代表已经排除 USB、串口、CAN 物理链路或持续吞吐问题；
也不宜用很长的队列掩盖持续拥塞，否则旧运动指令会排队。
可在运行前后只读比较以下统计，重点看新增 dropped 和运行中的 backlog：

```bash
ip -details -statistics link show dev can2
tc -statistics qdisc show dev can2
ip -details -statistics link show dev can1
tc -statistics qdisc show dev can1
```

当前 SDK 在发送遇到此错误时已有有限重试；失败后仍急停，应用不吞掉故障或自动重新使能。
SDK 将这类异常包装成 `Hardware fault on joint 0: code=0x0`，不能据此判定某个电机损坏。
收到 SDK 故障回调的臂不再发送位置保持命令，日志明确提示无法保位；
运行中的 CLI 保留进程并持续保持其他可控臂，主动退出时才通过 `close(disable=True)` 清理所有端点。

### 跟随诊断

若慢速跟随仍有滞后，应分别比较 `mapped_target` 与 `sdk_target`（软件限速造成的落后），
以及 `sdk_target` 与 `feedback`（伺服和通信等造成的实际跟踪偏差）。提高循环频率不能
消除过低的速度、加速度限制；也不要通过关闭限位或故障保护追求响应。

若出现“主臂动了、从臂不明显跟随”，可加诊断参数观察整条控制链：

```bash
onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_right_follow.json \
  --diagnostics-interval 1
```

每秒打印一条 `Follow diagnostics`。以下关节数组按 J1–J7 排列、单位为 rad：
`leader` 是参与本次映射的主臂角度，`mapped_target` 是映射目标，`sdk_target` 是
本项目限速后提交 SDK 的目标，`feedback` 是该控制周期开始时的从臂反馈，
`clip_joint_limits` 标明本项目的裁剪开关；`clipped_joints` 仅在开关启用时列出目标被本层裁剪的关节，
关闭时为空。`out_of_bounds_joints` 在两种配置下均列出映射目标超出 SDK 型号范围的关节编号。
SDK 内部仍可能进一步约束目标，
`sdk_target` 不代表已在 CAN 线上验证的最终指令。诊断只读取缓存，不增加 CAN 查询，
不启动数据集录制；暂停期间不打印旧帧。启动和恢复时会打印映射模式及参考角度，
开启本项目裁剪时输出 `target clipped`，每条臂最多每五秒一次；关闭时不输出该日志。
`continuous_follow` 标明跟踪策略，`velocity_feedforward` 为本帧提交的七维速度前馈
（rad/s），已包含 `velocity_feedforward_scale`；关闭连续模式时该字段为零。
它不代表 CAN 编码量化后或 SDK 再次约束后的实测速度。

双臂使用两个映射，源码中没有另一套双臂控制程序。无夹爪时目标动作按左臂 7 维、右臂 7 维排序：

```text
left_arm.joint1-l.pos ... left_arm.joint7-l.pos
right_arm.joint1-r.pos ... right_arm.joint7-r.pos
```

启用夹爪时，每侧七个关节键后附加该侧的 `{side}_arm.gripper.pos`，单臂/双臂可为 8/16 维。
关节映射在 Teleoperator 中完成一次；Robot、
录制后的回放和策略执行始终消费从臂物理目标角，不重复映射。

## 4. 录制默认关闭，可在跟随中开关

### 可选 USB 相机

`configs/sdk_dual_joint_follow_cameras.json` 配置 `left_wrist` 和 `right_wrist` 两路相机，
默认 MJPEG 1600 × 1200、30 fps。`/dev/videoLeft` / `/dev/videoRight` 只是参考别名，
新电脑应使用 `ls -l /dev/v4l/by-id/ /dev/v4l/by-path/` 查到的实际路径。
采集通过后台 V4L2 MJPEG + mmap，OpenCV 仅解 JPEG，控制循环读取最新 RGB 缓存。

默认部署已带所需依赖，`--base` 环境使用 `python -m pip install -e '.[camera]'`
补齐采集依赖；录制需另装 LeRobot，可使用 `python -m pip install -e '.[camera,lerobot]'`。
在主从会话中，相机连接、机械臂连接和录制器独立并行启动。机械臂全部就绪后就开始跟随，
相机或录制器未就绪只等待录制，不推迟跟随。相机缺失、权限或格式错误只使当前录制尝试失败；
排查后输入 `r` 会异步重试相机并请求新的 episode。运行中掉线由相机后台尝试重连，
帧过期会结束当前录制，CAN 跟随继续。

直接调用原生 `OneroSdk.connect()` 仍要求配置的相机可用，以满足策略的完整图像观测要求。
其连接行为与 `SdkSession` 独立管理相机、机械臂和录制就绪状态的行为不同。

格式检查、设备权限、相机配置、模拟图像和真实 USB 配模拟臂的步骤见
[USB 双腕相机与视频录制](usage_usb_cameras.md)。不配置相机时，下面的关节录制流程保持不变。

### 开始与结束录制

纯跟随关闭录制且不提供 `repo_id` / `task` 时，不创建录制进程或数据集。
启用启动录制或提供任一数据集参数时，会话会准备录制进程；提供完整参数但没有开启录制时，
只预先启动空闲进程，首次输入 `r` 才创建或打开数据集。
若需要在本次会话中随时录制，先提供这些参数，仍不加 `--record`：

```bash
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_single_mirror.json \
  --repo-id local/a1_follow --task "mirror following" \
  --root ./datasets/a1_follow
```

交互命令需要按 Enter：

| 命令 | 行为 |
|---|---|
| `r` | 请求录制，异步等待相机及录制器就绪；再次输入结束并保存当前 episode，跟随继续 |
| `p` | 暂停/恢复跟随；恢复重新捕获参考且不自动录制；不能解除 `fault` |
| `q` 或 Ctrl+C | 停止整个会话，失能主臂和从臂，再完成录制收尾 |

只有交互终端启用这些输入命令；无人值守运行可使用 `--duration` 和启动录制参数。
`--duration` 仍表示跟随会话运行时间，异步录制准备也占用这段时间，不表示一定录满指定秒数。
保存期间不能开始下一个 episode，待状态从 saving 变为 off 后再输入 `r`。
停止录制不重置机械臂、不退出拖动模式。

启动时即请求录制，录制初始化与机械臂连接并行进行：

```bash
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_single_mirror.json --record \
  --repo-id local/a1_follow --task "mirror following" --root ./datasets/a1_follow

# 也可使用 record 包装入口，默认开启录制
onero-a1-lerobot-record \
  --session-config configs/sdk_single_mirror.json \
  --repo-id local/a1_follow --task "mirror following" --root ./datasets/a1_follow
```

`--no-record` 可覆盖配置中的启动录制开关。JSON 的 `recording.enabled` 默认 false，
`recording.fps` 默认 30；未指定控制频率时默认 100 Hz，逐关节示例显式请求 200 Hz。
LeRobot 导入与数据集初始化在独立录制进程中完成，
需 Python 3.12+ 及项目固定的 LeRobot 0.6.1 数据集依赖；默认一键安装已包含，
使用 `--base` 安装的环境在已激活的 venv 中补充：

```bash
python -m pip install 'lerobot[dataset]==0.6.1'
```

数据为标准 LeRobot 数据集：

- `observation.state`：从臂实际关节角。
- `action`：本项目限制后提交给 SDK 的目标角；SDK 内部还可能继续限制它。
- `control.time_s`：主机单调时钟的真实采样时间，补充固定帧率索引。
- 启用相机时，`observation.images.<name>` 为标准视频特征，输入图像是 RGB HWC `uint8`。
- `camera.<name>.time_s` / `camera.<name>.sequence` 为独立 float64 字段，记录主机取帧时间和帧序号，不扩大关节状态维度。

录制期间，后台只保存 USB 相机原始 JPEG 字节和同组关节、动作、时间戳；mock 图像保存原始 RGB。
原始记录位于数据集父目录的 `.a1-recordings/<数据集名>-<随机标识>/`，包含 `session.json`、
`samples.jsonl` 和 `frames/`。相机解码仍独立运行，策略图像接口保持 RGB。

停止录制后，先排空待写队列，再加载 LeRobot/PyTorch、逐组读取原始图像并生成 H.264 MP4、
统计与数据集。采集阶段不做视频编码，也不通过共享槽传输完整 USB RGB 图像。
`r` 停止录制时跟随继续，生成完成前不能开始下一段；退出整个会话时关闭机械臂后等待生成完成。
等待 `Post-record conversion complete` 和 `Episode writer finished ... status=saved` 表示数据集可用。

`recording.video_encoder` 支持 `auto`（默认）、`h264_rkmpp`、`libx264`，仅在录制后生成阶段使用。
自动模式检测 Rockchip MPP 设备及系统 FFmpeg 编码器，否则使用 PyAV/libx264。
软件参数为 `ultrafast`、`zerolatency`、`bf=0`，硬件为 CQP、QP=23，两者 GOP 均为 `recording.fps`。
完整样本按原顺序转换，视频帧数与数据行对应，不静默丢帧。
这一组参数是本项目的编码契约：原生 `lerobot-record` 不写 `dataset.rgb_encoder` 时会回落到
LeRobot 默认的 `libsvtav1`（`video.codec=av1`、GOP 2、crf 30、preset 12），因此
`configs/native_dual_joint_follow_record.json` 显式写入同一组 H.264 参数，两条路径的落盘编码一致；
`tests/test_native_record_parity.py` 与 `tests/test_video_encoding.py` 会在参数漂移时失败。
硬件输出允许记录为 `yuv420p` 或 `yuvj420p`；追加时读取新视频的实际格式并与已有格式严格比较，
检查在提交新 episode 前完成。错误信息包含具体相机及不一致字段，不改写旧数据的格式标记。

`recording.queue_size` 默认 300 组，`recording.max_queue_bytes` 默认 256 MiB。
预算包含排队/正在写入的载荷和为 IPC 副本预留的最大单组大小的 4 倍；JPEG 按压缩后大小计费。
整段数据留在磁盘，内存只缓冲有限待写组；没有共享 RGB 槽，也不再固定只能积压约 22 组。
磁盘持续慢写、磁盘错误或相机异常仍会停止当前录制并报告错误，CAN 跟随继续。

`recording.finalize_timeout_s` 默认 300 秒，限制连续无进度的等待时间，而非生成总时长。
队列写盘、逐帧转换及阶段切换会续期；持续有进度时可以等待超过 300 秒。依赖加载、最终统计等
无中间进度的阶段仍受此限制。实际停滞或子进程退出才报错，保存成功以子进程完成回复为准。
成功生成后清理原始文件，生成失败/无进度超时保留源目录，恢复命令见
[USB 相机接入](usage_usb_cameras.md#6-队列保存与恢复)。手动恢复要求新的输出目录，不连接硬件。
原始采集文件不是最终 LeRobot 数据集，须在转换完成后用于训练。

`Capture frame` 和 `Capture throughput` 报告图像 IPC 与写盘速度；录制停止后才出现
`Post-record video generation`、`Dataset frame`、`Video group` 与转换进度。
控制、相机与录制频率独立，视频时长仍是实际保存样本数除以 fps；`--duration` 是会话时长。
CPU 亲和性仍按可用核心动态分配，控制超时保护不变。

已有目录必须匹配全部特征、相机名称/尺寸及 fps；单臂数据集不能直接续录双臂数据，
无图像数据集不能直接追加视频。当前不另存主臂原始角度或逐关节反馈有效标志。

## 5. 回放与 LeRobot 集成

```bash
onero-a1-replay-episode \
  --session-config configs/sdk_single_mirror.json \
  --repo-id local/a1_follow --root ./datasets/a1_follow --episode 0
```

回放仅连接从臂，不连接主臂，也不执行主臂映射。含相机的配置同样可用于回放，
但不会打开 USB 相机；读取动作不解码录制视频。可以加 `--simulate` 用内存从臂验证。
回放前校验动作名称、顺序及首帧的有限值，随后连接从臂并自动平滑对齐到首帧。
对齐使用起止速度/加速度为零的五次插值，从当前反馈位置开始，以 `robot.control_hz` 发送目标。
默认至少 3 秒，关节速度不超过 0.2 rad/s、加速度不超过 0.5 rad/s²，且不超过配置及 SDK 的限制；
较大的位移自动延长过渡时间。夹爪启用时从当前开合度平滑过渡到首帧，按 0.25/s 的开合度速度限值计算时长。

| 回放参数 | 默认值 | 含义 |
|---|---|---|
| `--align-duration` | `3` | 对齐轨迹的最短时长，秒 |
| `--align-max-velocity` | `0.2` | 对齐关节速度上限，rad/s |
| `--align-max-acceleration` | `0.5` | 对齐关节加速度上限，rad/s² |
| `--align-timeout` | `15` | 计划轨迹结束后允许等待稳定的额外时间，秒 |
| `--align-tolerance` | 沿用 `robot.startup_max_delta_rad` | 关节反馈到位容差，rad；显式设置时不超过首帧保护阈值 |
| `--no-align` | 不启用 | 跳过自动对齐，仍检查首帧与当前姿态的距离 |

关节到位容差默认与首帧保护一致，当前默认配置为 0.15 rad（约 8.59°），不另行折半。
需要更精确的对齐时可用 `--align-tolerance` 收紧；跟踪误差、反馈时效和控制断流保护仍独立生效。
夹爪误差须在 0.03 内，关节命令到位、命令速度接近零，并连续稳定 0.25 秒才进入回放。
稳定窗口内实测关节变化范围不得超过 `min(0.005 rad, 到位容差 / 4)`，夹爪变化范围不得超过 0.01。
日志会显示误差最大的关节、目标值、实测值和稳定时间，便于定位未到位原因。
对齐结束后保留最后的命令位置恢复正常运动限制，避免将保持目标改成存在偏差的实测位置；
回放首帧仍重新检查实际距离，不修改录制动作或全局保护阈值。
当前姿态或首帧超出 SDK 关节范围、反馈过期、控制断流、跟踪异常或对齐超时时会停止，不进入数据回放。
Ctrl+C 可中止对齐/回放，退出时仍请求失能从臂。对齐仅在关节空间插值，不做碰撞检测或避障，需确保运动路径畅通。

LeRobot 插件发行包为 `lerobot_robot_onero_a1`，工厂设备类型为
`onero_sdk` / `onero_sdk_teleop`。本项目的 teleoperate、record、print 和 replay
入口都要求 `--session-config`。relative 映射需要双方参考，由 `SdkSession`
准备流程捕获；直接使用原生 LeRobot 独立构造两设备时不会自动捕获参考，
原生 CLI 用下面的配置和首帧自举开关绕过这一限制。

### 用原生 `lerobot-teleoperate` / `lerobot-record` 跟踪双臂

原生 CLI 不接受 `--session-config`，它用 draccus 的 `--config_path` 读一份
`robot` / `teleop`（录制时再加 `dataset`）结构的 JSON。项目提供两份真机配置，
机械臂、通道、夹爪和相机与 `configs/sdk_dual_joint_follow_cameras.json` 一致：

| 配置文件 | 用于 | 差异 |
|---|---|---|
| `configs/native_dual_joint_follow_teleoperate.json` | `lerobot-teleoperate` | 只有 `robot` / `teleop` |
| `configs/native_dual_joint_follow_record.json` | `lerobot-record` | 另加 `dataset` 段，含两路腕部相机与同参数的 H.264 `rgb_encoder` |

```bash
# 纯跟随，Ctrl+C 退出（teleop_time_s 为 null）
lerobot-teleoperate --config_path=configs/native_dual_joint_follow_teleoperate.json

# 录制；episode 时长、数量、数据集目录按现场修改 dataset 段
lerobot-record --config_path=configs/native_dual_joint_follow_record.json
```

相对会话配置只有四处结构性差异，都是原生路径的要求：

1. 多一层 `"type": "onero_sdk"` / `"type": "onero_sdk_teleop"`（draccus 用它选择设备类）。
2. 遥操类型不写会话级的 `teleop.reference`，而是逐条臂在 `teleop.mappings.<side>.reference`
   写 `"absolute"`。原生路径读的是同一份 `OneroSdkTeleopConfig`，会话级开关同样生效，
   但逐臂写法把"原生必须全量"这条依赖写在依赖它的那一层。
3. `robot.bootstrap_relative_follow` 为 `true`。
4. 录制配置多一个 `dataset` 段（见下表与后文）。

视频编码参数不构成差异：`dataset.rgb_encoder` 已与会话配置的 H.264 参数对齐。

**唯一阻塞点与两条路径。** 原生 CLI 自己构造 teleop 和 robot，流程是
`teleop.connect()` → `robot.connect()` → 循环内 `teleop.get_action()` /
`robot.send_action()`，**从不调用 `OneroSdkTeleop.prepare()`**
（`Teleoperator` 基类里也没有这个方法）。`ArmMapper` 因此拿不到主从参考，
`reference="relative"` 会在第一帧抛
`Relative (incremental) mapping requires prepare() in a shared SDK session; ...`，
报文里同时给出改法（改成 `reference="absolute"` 并配合 `bootstrap_relative_follow`）。
会话配置里把 `bootstrap_relative_follow` 与相对映射写在一起，会在加载期就被拦下。

- **A（零代码改动）**：`reference="absolute"`（`scale=1`、`offset=0` 时目标就是主臂
  反馈角）。从臂第一步会直接跳到主臂当前姿态，所以必须在启动前把主臂摆到与从臂相差
  `startup_max_delta_rad`（默认 0.15 rad）以内，否则首个动作被首帧保护拦下，报文里逐个
  列出超限关节的 `target` / `follower` / `delta`。加
  `--robot.bootstrap_relative_follow=false` 可复现这条路径。
- **B（默认，推荐）**：`bootstrap_relative_follow=true` 时，`OneroSdk` 在 `prepare()`
  之后的第一个动作上锁存「从臂当时实测位置 q0」和「该帧目标 first_target」，之后把每个
  目标改写为 `q0 + (target - first_target)`。这正是 relative 映射用一对参考产生的角度
  增量，所以既不需要 `prepare()`，也不需要事先对齐：第一帧命令就是从臂当前位置，随后按
  主臂的增量运动。会打印一次
  `Bootstrap relative follow for <side>: follower q0=... first target=...`。

该开关只改写关节位置；夹爪开合度是主臂的绝对读数，原样传递。锁存值在 `prepare()` 和断开
时清空，因此连接、暂停恢复、仿真切回手动都会重新锁存，语义与 relative 映射重新捕获参考
一致。把它与相对映射同时写进会话配置会在**加载期**被拒绝
（`bootstrap_relative_follow requires reference='absolute' on every mapping`）：
相对映射自己就提交从臂相对目标，且依赖 `prepare()` 捕获的参考，两者组合没有意义。
默认值为 `false`：策略 rollout 与数据集回放提交的是绝对目标，不能被平移。

确定性等价验证（本机 mock、固定 5 ms 假时钟、同一主臂轨迹、80 帧）：relative 映射的目标
与 B 改写后的目标逐帧相同，下发到从臂的命令也逐帧相同（差值 0），从臂实际位移等于主臂位移
（0.2513 rad，说明不是被限速吃掉了差异）；不开开关时（路径 A）同样 0.5 rad 的主从偏差会被
首帧保护拒绝。

**与自研 CLI 的差别，真机前先确认。**

- **控制频率**：原生循环跑在 `fps` / `dataset.fps`（这两份配置是 30 Hz），自研 CLI 跑在
  `robot.control_hz`（200 Hz）。SDK 的单步位置上限是 `max_delta_per_step=0.03 rad`，
  30 Hz 下每个关节的稳态速度被压到 `0.03 × 30 = 0.9 rad/s`，而不是配置里的 3 rad/s；
  200 Hz 下该上限是 6 rad/s，实际由配置的 3 rad/s 决定。要更接近自研 CLI 的跟手程度，
  需提高原生循环频率（它同时是数据集与视频帧率）或改用自研 CLI。
- **数据集 `action` 列的含义**：原生 `record_loop` 保存 teleop 原始动作，
  不是 `send_action()` 实际下发的目标；所以 B 生效时 `action` 是主臂绝对角
  （实测例：0.5002 rad），而 `observation.state` 是从臂实测（0.0）。自研 CLI 保存的是
  限制后提交给 SDK 的从臂目标角（`action` ≈ 从臂当时位置）。两者记录同一段主臂运动，但
  原点约定不同：用原生数据做 rollout 时配置里必须同样打开 `bootstrap_relative_follow`，
  起点的常数偏移才会抵消。
- **视频编码与额外列**：原生 `lerobot-record` 默认用 `libsvtav1`（`video.codec=av1`、GOP 2、
  crf 30、preset 12），本项目已在 `dataset.rgb_encoder` 写入与自研管线相同的 H.264
  （`video.codec=h264`、GOP=帧率、crf 23、preset ultrafast、`tune=zerolatency`、`bf=0`），
  所以两条路径的 MP4 编码一致；不写这段就会出现 av1/h264 混用。此外自研管线额外记录
  `control.time_s`、`camera.<name>.time_s`、`camera.<name>.sequence`，并在视频 `info` 里写
  `a1.video_encoder` 标记，原生录制没有这三列和该标记（只有标准的 `timestamp` /
  `frame_index` / `episode_index` / `index` / `task_index`）。用原生数据训练时不要依赖这些键。
- **相机是连接前置条件**：原生 `OneroSdk.connect()` 要求配置的相机都能出帧；
  自研 `SdkSession` 允许相机后到、失败只影响录制。
- **CPU 亲和性**：`control_cpu_affinity` 只由自研 `run_session_cli` 应用，原生路径不会
  设置。原生 `lerobot-record` 在**主进程**里做数据集写入与图像编码（LeRobot 默认
  `num_image_writer_processes=0`、每相机 4 个编码线程），这些线程会和 CAN worker 抢同一批
  核心，是原生更容易撞上单帧超时的主要原因。这两份原生配置因此把图像写入改为
  `num_image_writer_processes=2`、`num_image_writer_threads_per_camera=1`
  （见 `configs/native_dual_joint_follow_record.json` 的 `dataset` 段，可按现场调回默认）。
- **单帧丢失的容错**：真机主臂那对 CAN 是 CANable2 走 slcan（USB CDC-ACM），负载下偶发
  连续丢帧（SDK 自己会报 `canN: Joint j: 3 consecutive missed frames`）。因此从臂连接在
  **SDK 预使能状态检查**失败时会自动重试一次（SDK 报的 `... before enable` 系列，
  例如 `Cannot verify motor state before enable`：那时一条使能帧都还没发、电机仍全是
  DISABLED，重试是安全的；其他错误不重试），夹爪/手柄
  状态查询则按 `gripper_query_failures_before_fault`（默认 3）计数后才报故障。这两处容错
  只在故障路径生效，链路正常时行为与之前完全一致。

`onero-a1-lerobot-rollout` 委托原生 LeRobot 策略入口，使用原生 robot/policy 参数，
默认设备类型为 `onero_sdk`，不接受 `--session-config`。模型输入输出必须与训练数据
的关节特征名称、顺序、弧度单位及可选夹爪开合度特征一致，并匹配配置的相机名称和图像尺寸。
`OneroSdk.get_observation()` 可提供 RGB 图像，但完整视觉策略训练与真机部署尚未端到端验证。

查看反馈时也需要会话配置：

```bash
onero-a1-print-observation --session-config configs/sdk_left_leader_right_follow.json \
  --simulate --count 5 --rate 1
```

该入口只连接从臂，要求 `robot.send_action=true`；真机连接会使能从臂，随后按控制频率
持续保持位置，按 `--rate` 打印反馈，退出时请求失能。它不是被动 CAN 监听。
`--count 0` 表示持续打印，`--rate` 不能超过 `robot.control_hz`。

`scripts/record.sh` 和 `scripts/replay.sh` 只向对应 A1 命令转发参数，数据集 ID
和 `--root` 会原样使用。`scripts/viz.sh` 是离线 LeRobot 数据集可视化入口，
其 `--root` 指向包含 `<repo-id>` 子目录的数据集缓存根目录，不连接机械臂。

## 6. 无硬件测试与 CAN 预检

仓库自带一套不需要 CAN、USB 相机或 SDK 服务的用例，覆盖配置加载、遥操类型开关与全量/增量映射
数学、启动首帧保护与报错内容、两套录制路径的编码一致性、会话配置接线，以及 `can_precheck`
的参数行为。先准备好 venv（含 `dev` extra）：

```bash
bash scripts/setup.sh
source ~/.venvs/a1-lerobot/bin/activate
python -m pip install -e '.[dev]'   # 已装过可跳过
python -m pytest                    # 秒级完成，不访问 CAN 或相机
```

`pythonpath` 和 ROS 插件屏蔽写在 `pyproject.toml` 的 `[tool.pytest.ini_options]`，在机器人上
直接 `python -m pytest` 即可。终端若 source 过 `/opt/ros/jazzy`，ROS 的 `launch_testing`
插件会和本项目的 venv 冲突，已用 `-p no:launch_testing` / `-p no:launch_ros` 关掉；
需要完全隔离 ROS 环境变量时用 `PYTHONPATH= python -m pytest`。

启动真机跟随前，可以先用只读预检看每条通道是否应答（不发使能、不发运动帧）。
通道既可以是 SocketCAN 接口名（正式双臂配置用的就是 `can4`/`can3`/`can2`/`can1`），
也可以是 `/dev/serial/by-id/...` 直连串口（直接打开 SLCAN，不依赖 slcand）：

```bash
python scripts/can_precheck.py                  # 内置四条通道：can4/can3 主臂、can2/can1 从臂
python scripts/can_precheck.py --config configs/native_dual_joint_follow_record.json
                                               # 按配置里的 arms 推导通道与型号
python scripts/can_precheck.py can3 -n 20 -t 0.1
                                               # 只查 can3，每电机查 20 次，应答超时 100 ms
python scripts/can_precheck.py --watch 10 /dev/ttyACM4 /dev/ttyACM5
                                               # 观察 10 秒：你手动转动哪条臂，哪条通道读数就变
```

`DISABLED` 是未使能的正常状态；任何 `NO REPLY` / `ERROR(...)` 都应先排查再启动跟随程序。
存在无应答或打不开的通道时脚本以退出码 1 结束。更换或重插适配器、交换 CAN 线之后，
通道与左右臂的对应关系必须重新确认，`--watch` 就是为此准备的：运行后用手转动目标机械臂，
读数在变化的那条通道就是它。它只读，不会使能机械臂或让它受力。

## 7. 验证范围和边界

- 此前双视频数据集验证不覆盖本次原始图像落盘和录制后生成流程。本次已加入免硬件用例并在 mock 下跑通（见第 6 节）：覆盖配置加载、遥操类型开关与全量/增量映射数学、首帧保护的判定与报错、两套录制路径的编码一致性、`can_precheck` 参数与 CLI 解析；真机的图像落盘与吞吐、CAN 异常和故障行为仍需部署方测量。
- 全量（`teleop.reference="absolute"`）的开关优先级、映射数学与启动保护有免硬件用例覆盖；`joint_signs`、`offset` 的现场取值与"从臂先就位"的可达性没有真机验收，首次使用必须低速、空载、留足运动空间试运行。
- `--simulate` 会替换相机为 mock，并清空 V4L2 曝光项；MuJoCo 模式的相机图像同样为合成图像，不表示渲染视角或真实摄像头画面。

- 内存模拟不测试 CAN 延迟、真实关节限位或动力学；模型限位在实际 SDK 路径中读取。
- 主臂和从臂均可配置 `backend="mujoco"`（删除 channel）；CAN 主臂配 MuJoCo 从臂仍受支持。
  图形窗口目前仅用于全部端点均为 MuJoCo 的仿真会话；需安装 SDK sim 依赖。
  SDK MuJoCo 时间使用仿真时钟，本适配器分别处理它和 CAN 的主机时钟。
- 运行控制异常会锁定跟随，CLI 继续运行并保持所有仍可控制的主臂、从臂；
  无法保位的臂会单独报警，不自动重新使能。库调用的首次控制异常仍向调用方抛出，
  后续 `tick()` 可维持故障保位；独立使用库的调用方需自行安排持续调用和最终退出。
  进程隔离时双路跟随并发执行，并等待双方完成；它仍不是原子 CAN 事务。
- 当前 SDK 的整帧时间戳不能证明每个关节反馈都是新数据；这里只检查可验证的状态及故障。
  控制间隔超时在下一次调用时检查，不能代替独立硬件看门狗。
- SDK 故障回调的锁重入已有无硬件复现，本修改不修补或绕过 SDK；底层 I/O 卡死时，
  应用层无法保证调用能及时返回或发出停止帧，真机故障行为尚需单独验收。
- 本项目的 SDK Robot / Teleoperator 在断开时显式调用 SDK `close(disable=True)`，
  请求失能其所有已连接的臂；回放只连接、失能从臂。
  单条臂清理报错时仍尝试清理其他臂，并记录错误。通信断开、底层 I/O 卡死、
  SIGKILL 或掉电时，软件无法保证失能指令送达；发送指令不等同于已验证所有电机失能。
