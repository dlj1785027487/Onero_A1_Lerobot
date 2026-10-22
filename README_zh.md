# Onero A1 LeRobot

[English](README.md) · [使用速查](docs/usage.md)

A1 单臂/双臂遥操与数据采集框架，支持 CAN/SLCAN 直连、USB 相机、LeRobot 数据集和动作回放。

## 安装

使用 Linux、Python 3.12+；在项目根目录执行（`armsdk` 从 PyPI 安装）：

```bash
A1_LEROBOT_VENV="$PWD/.venv" bash scripts/setup.sh
source .venv/bin/activate
```

SDK 从 PyPI 安装（`onero-sdk>=0.3,<0.4`）；想改用本地源码时设置 `A1_ARM_SDK_PATH`。每次新开终端先激活虚拟环境；仅需遥操可安装 `--base` 环境。
退出环境用 `deactivate`，也可执行 `source .venv/bin/deactivate`；退出脚本需要用 `source` 加载。

## 配置设备

四条手臂的接口在全部配置里已经固定，按[使用速查](docs/usage.md)第 2 章接好线即可，不需要改 JSON：

| 物理臂 | 型号 | 配置位置 | 接口 |
|---|---|---|---|
| 左主臂 | A1L | `teleop.arms.left` | `can1` |
| 右主臂 | A1R | `teleop.arms.right` | `can2` |
| 左从臂 | A1L | `robot.arms.left` | `can3` |
| 右从臂 | A1R | `robot.arms.right` | `can4` |

| 配置文件 | 用途 |
|---|---|
| `configs/sdk_left_leader_right_follow.json` | 左主臂 → 右从臂（`joint` 逐关节映射） |
| `configs/sdk_right_leader_left_follow.json` | 右主臂 → 左从臂（`joint` 逐关节映射） |
| `configs/sdk_single_mirror.json` | 左主臂 → 右从臂（`mirror` 增量取反） |
| `configs/sdk_left_leader_left_follow.json` | 左主臂 → 左从臂（同侧，`joint`） |
| `configs/sdk_right_leader_right_follow.json` | 右主臂 → 右从臂（同侧，`joint`） |
| `configs/sdk_dual_joint_follow.json` | 双臂遥操 |
| `configs/sdk_dual_joint_follow_cameras.json` | 双臂遥操与相机采集 |
| `configs/native_dual_joint_follow_teleoperate.json` | 原生 `lerobot-teleoperate`（`--config_path`） |
| `configs/native_dual_joint_follow_record.json` | 原生 `lerobot-record`（另含数据集参数） |
| `configs/native_dual_joint_follow_rollout.json` | 策略 rollout（`onero-a1-lerobot-rollout`，只连从臂） |
| `configs/native_dual_joint_follow_rollout_mock.json` | 无硬件 rollout 干跑（mock 臂与相机） |

只有现场无法按上表接线时才需要改配置：

- `teleop.arms`：主臂；`robot.arms`：从臂。每条机械臂使用独立通道。
- `channel`：SocketCAN 接口名或 SLCAN 的 `/dev/serial/by-id/...` 路径。串口直连时不要同时运行 `slcand`。
- `robot.cameras`：相机设备路径、分辨率和帧率。建议使用稳定的 `/dev/v4l/by-path/...` 路径。
- `teleop.reference`：整场遥操类型。`"relative"` 让从臂停在自己启动时的位姿、只复现主臂位移；`"absolute"` 把主臂自身位姿逐关节交给从臂。五份会话配置默认都是 `"relative"`；启动命令加 `--reference absolute`（或改这个字段）即可切全量，原生配置则逐臂声明。

CAN 绑定命令、接线校验与相机 USB 口约定见[使用速查](docs/usage.md)第 2 章。

## 遥操

```bash
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_dual_joint_follow.json
```

只需模拟运行时，加 `--simulate --duration 5`，不会连接机械臂或 USB 相机。

全量遥操（`"reference": "absolute"`）要求 `scale=1`、`joint_signs` 在真机核对，且启动时从臂与
主臂的每个关节都在 `robot.startup_max_delta_rad` 以内——第一条指令就是主臂位姿，程序不会替你
对齐。该字段默认 0.15 rad，两份双臂配置写 3.0 rad，让限幅器把从臂牵引过去而不是直接拒绝会话，
随后由 `robot.max_tracking_error_rad`（0.5 rad）接管这次牵引。见[使用速查](docs/usage.md)。

同一套双臂遥操与录制也可以用 LeRobot 原生 CLI 触发：

```bash
lerobot-teleoperate --config_path=configs/native_dual_joint_follow_teleoperate.json
lerobot-record --config_path=configs/native_dual_joint_follow_record.json
```

原生路径不调用 `prepare()`，靠 `robot.bootstrap_relative_follow` 做首帧自举相对跟随，跟随
动作与自研 CLI 相同；但循环频率为 `dataset.fps`（示例 30 Hz）、数据集 `action` 列的含义
与自研 CLI 不同，差异见[使用速查](docs/usage.md)。

## 数据采集

```bash
onero-a1-lerobot-record \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_cameras --task "双臂操作采集" \
  --root ./datasets/local/a1_cameras
```

默认持续录制，**Ctrl+C 停止并自动保存**。加 `--duration 60` 可限制会话运行 60 秒；录制准备时间也计入会话时长。
`--root` 是数据集的完整目录；使用不含相机的配置可只采集关节与动作。

交互命令需要按 Enter：

| 命令 | 操作 |
|---|---|
| `r` | 停止录制并保存；保存完成后再次输入可开始下一段 |
| `p` | 暂停/恢复跟随；暂停会结束当前录制 |
| `q` | 退出会话并保存 |

## 动作回放

```bash
onero-a1-replay-episode \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_cameras --root ./datasets/local/a1_cameras --episode 0
```

回放会先平滑对齐到首帧，再播放动作；默认至少 3 秒，可用 `--align-duration 5` 加长。
对齐不做避障，运行前确认运动路径。回放不打开相机；`--no-align` 跳过对齐但保留首帧保护，`--simulate` 可模拟回放。

## 注意事项

- 真机启动会使能机械臂；退出会失能，退出前应支撑机械臂。
- 录制期间保存原始图像和采样数据，**停止后才生成视频和数据集**。等待 `Episode writer finished ... status=saved` 再关闭终端，生成期间不能开始下一段。
- 已有数据集要求相机、关节特征、帧率及编码配置兼容；更换配置时使用新目录。
- 生成失败时会保留原始采集文件。恢复命令、编码设置及超时说明见[使用速查](docs/usage.md)。
- `with_gripper=true` 会启用夹爪控制与采集（0~1，1 为张开）：CAN 真机驱动实体夹爪，mock 端点用内存夹爪，因此 `--simulate` 与真机的特征完全一致；MuJoCo 不含夹爪（其末端属于 SDK 仿真）。实体协议由 `gripper_protocol` 显式选择（默认 `onero`，旧款 OmniPicker 用 `omnipicker`，`none` 表示不发送任何夹爪 CAN 帧），可视化工具模型由 `gripper_model` 独立选择（默认 `woan_picker`）。这两项只对从臂生效；末端是按压式手柄的主臂固定用 `onero`/`woan_picker` 只读读取，并须按实测填写 `handle_open_code`/`handle_closed_code`（不填则该主臂按普通夹爪驱动，即单臂主从配置的行为；填写后才启用只读手柄）。仿真、可视化及策略接口见[使用速查](docs/usage.md)。
- 配置和编码有免硬件用例：激活 venv 后先 `pip install -e ".[dev]"`（装 pytest/ruff，`scripts/setup.sh` 只装运行时依赖），再 `python -m pytest`；启动真机前可用 `python scripts/can_precheck.py` 只读确认各通道电机是否应答。详见[使用速查](docs/usage.md)。
