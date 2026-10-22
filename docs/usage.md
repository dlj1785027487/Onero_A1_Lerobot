# 使用速查

从零到各功能的完整操作顺序：**第 1 章部署环境 → 第 2 章接线 → 第 3 章查表 → 之后每章一条启动命令**。

四条约定，后文不再重复：

- 文中 `~/oneroa1lerobot` 是占位符，指**你自己的 a1-lerobot 代码仓库目录**（clone 或解压到任意位置都可以，例如 `~/a1-lerobot`）；下文所有 `configs/...`、`scripts/...`、`./datasets/...`、`outputs/...` 都是相对这个仓库根目录的路径。
- 所有命令都在**仓库根目录**、**已激活虚拟环境**的终端里执行（`cd <你的仓库目录> && source .venv/bin/activate`）。
- `configs/` 里的 9 份配置**已按第 2 章的接线固定好接口名和映射关系**，按本文接线后不需要改任何 JSON。
- 真要改（换接口名、换相机口、调速度），改配置里对应字段即可，本文所有命令仍然适用，见 §2.5 与附录 A。

## 1. 部署环境（一次性）

要求 Linux + Python 3.12+（只遥操 3.10+）。在你自己的仓库根目录执行：

```bash
cd ~/oneroa1lerobot        # 换成你自己的 a1-lerobot 仓库目录
A1_LEROBOT_VENV="$PWD/.venv" bash scripts/setup.sh
source .venv/bin/activate
```

- `A1_LEROBOT_VENV` 指定虚拟环境位置，上面把它建在仓库内的 `.venv`；想放到别处就改成你自己的路径（如 `$HOME/venvs/a1-lerobot`）。
- `armsdk`（`onero-sdk[dynamics,sim,python-can]>=0.3,<0.4`）默认从 PyPI 安装；要用本地源码时加 `A1_ARM_SDK_PATH=/path/to/armsdk`（你自己的 armsdk 源码目录）。
- 只遥操、不录数据可加 `--base`（不装 PyTorch / LeRobot / 录制依赖）。
- 脚本最后会做一次不碰 CAN 的自检（URDF、动力学、视频编解码；x86 另含 MuJoCo 模型）。

装完可选跑一遍免硬件用例（只自检需要，`setup.sh` 不装测试工具）：

```bash
pip install -e ".[dev]"
python -m pytest
```

**每次新开终端**都要重新 `source .venv/bin/activate`。退出用 `deactivate`，或 `source .venv/bin/deactivate`。

## 2. 接线与固定接口（一次性）

### 2.1 四条手臂 → CAN 接口（本文与全部配置的唯一约定）

| 物理臂 | 型号 | 配置里的位置 | 接口 |
|---|---|---|---|
| 左主臂（遥操手柄侧） | A1L | `teleop.arms.left` | `can1` |
| 右主臂（遥操手柄侧） | A1R | `teleop.arms.right` | `can2` |
| 左从臂（被跟随侧） | A1L | `robot.arms.left` | `can3` |
| 右从臂（被跟随侧） | A1R | `robot.arms.right` | `can4` |

左右由 `model`（A1L/A1R）和 `mounting`（`humanoid_left`/`humanoid_right`）决定；`can1`–`can4` 只是绑定名字，**必须按上表绑定**，否则跟随方向会错。

每条臂独占一个适配器和一个接口，速率 1 Mbit/s。

怎么确认哪条是主臂：**看末端**——主臂末端是按压手柄，从臂末端是两指夹爪；臂身标有 A1L/A1R。不要按接口编号猜。

接反的典型现象：启动遥操后 `can3`/`can4` 上的臂自己动（主臂被当成从臂使能），同时日志反复报 `Gripper fault: protocol=onero code=12 state=12`（手柄电机被当成夹爪驱动）。出现这两种现象立刻 Ctrl+C，按上表重新绑定。

### 2.2 把 USB-CAN 适配器绑成 can1–can4

先看适配器的稳定串口名（名字由适配器序列号决定，与插在哪个 USB 口无关）：

```bash
ls -l /dev/serial/by-id/
```

名字形如 `usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2096366D4343-if00`：前缀 `usb-`、厂商 `Openlight_Labs`、型号 `CANable2`、固件仓库与版本 `b158aa7_..._canable2.git`、**序列号 `2096366D4343`**、接口 `if00`。**只有序列号那段因适配器而异**，其余同款适配器完全相同。

绑定参数必须是 `/dev/serial/by-id/` 下的**路径**，不能只写序列号；写完整名字或用唯一序列号通配都可以（**通配符不要加引号**，让 shell 展开）：

```bash
sudo bash scripts/setup_can.sh can1 /dev/serial/by-id/*2096366D4343-if00
```

再按表格绑定（**四个 `<...>` 按上一条命令的实际名称替换，顺序对上表**）：

```bash
sudo bash scripts/setup_can.sh can1 /dev/serial/by-id/<左主臂适配器>-if00
sudo bash scripts/setup_can.sh can2 /dev/serial/by-id/<右主臂适配器>-if00
sudo bash scripts/setup_can.sh can3 /dev/serial/by-id/<左从臂适配器>-if00
sudo bash scripts/setup_can.sh can4 /dev/serial/by-id/<右从臂适配器>-if00
```

- 接口已存在时脚本会拒绝重复创建；只想调整发送队列（跟随循环要求 ≥1000 帧）：

```bash
sudo bash scripts/setup_can.sh --tx-queue can1 can2 can3 can4
```

- `by-id` 名字只由适配器序列号决定，**插哪个 USB 口都不变**（只有背后的 `ttyACMx` 编号会变），换口后照抄原来的绑定命令即可；不要用 `/dev/ttyACMx` 绑定，它每次插拔都可能变。
- 重插后先 `sudo bash scripts/cleanup_can.sh` 再绑：残留的 `slcand` 仍指向已失效的 `ttyACMx`，会让对应的 `canN` 直接消失（`ip -brief link show type can` 里少一条）。
- 手臂直接接板载 CAN 口（不是 USB 适配器）时用：`sudo bash scripts/setup_can.sh --native canX`。
- **重启或重新插拔后接口会消失**，重跑上面四条命令（或整段脚本）即可；脚本不设开机自启。

### 2.3 校验接线（只读，不使能任何电机）

```bash
python scripts/can_precheck.py                      # 按内置约定查 can1/can2/can3/can4
python scripts/can_precheck.py --watch 10 can1 can2 # 手转目标臂，看哪条通道读数在变
timeout 5 candump can3                              # 只收不发，看总线上有没有帧
```

判据：`DISABLED` 是未使能的正常状态；出现 `NO REPLY` / `ERROR(...)` 或整条通道打不开，先排查线序、供电和 `--tx-queue`，再往下走。

换过适配器或对调过 CAN 线，**必须**用 `--watch` 重新确认"哪条接口是哪条臂"，再回到 2.1 的表。

### 2.4 相机（仅数据采集与 rollout 需要）

配置里固定两路 USB 相机，按 USB 口（`by-path`）区分左右。下表的路径是**设备路径**，与仓库放在哪里无关，由相机插在哪个 USB 口决定：

| 相机 | 配置名 | 设备路径 |
|---|---|---|
| 左腕 | `left_wrist` | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0` |
| 右腕 | `right_wrist` | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0` |

- 把相机插到这两个 USB 口就与配置一致；分辨率 640×480、30 fps、固定曝光 200。
- 路径中的 `pci-0000:00:14.0` 是 USB 控制器地址，换机器时会变，以 `ls -l /dev/v4l/by-path/` 的实际输出为准。
- 相机接反不会报错，只会让策略"看着左腕画面动右臂"，接完用 §8 的 `print-observation` 看观测、或录一小段再 `viz.sh` 回看。
- 确认权限与设备：

```bash
id -nG | grep -q video || sudo usermod -aG video "$USER"   # 首次部署，重新登录生效
ls -l /dev/v4l/by-path/ | grep video-index0

# 确认分辨率/帧率
ffmpeg -f v4l2 -list_formats all -i /dev/video0
```

怎么判断相机插在哪个口：

```bash
ls -l /dev/v4l/by-path/ | grep video-index0                 # 名字里的 usb-0:2 / usb-0:3 就是 USB 口
udevadm info -q property -n /dev/video0 | grep '^ID_PATH'   # 反查某个 /dev/videoX 对应哪个口
```

最可靠的判定是**插拔对比**：记下当前列表，拔掉一个相机再看，消失的那行就是它。

```bash
ls /dev/v4l/by-path/ > /tmp/before.txt
# 拔掉一个相机后再执行
ls /dev/v4l/by-path/ > /tmp/after.txt
diff /tmp/before.txt /tmp/after.txt
```

一个 UVC 相机通常暴露 `video-index0` 和 `video-index1` 两个节点，配置里只用 `video-index0`。分不清左右腕时，遮住一个镜头抓一张图，全黑的那路就是被遮住的：

```bash
ffmpeg -f v4l2 -video_size 640x480 -i /dev/v4l/by-path/<该相机路径> -frames:v 1 /tmp/cam.jpg
```

### 2.5 什么情况下才需要改配置（第一次部署使用必须改相机路径）

只有两种：现场必须换 CAN 接口名（改对应文件里的 `channel`），或相机换了 USB 口/换了机器（改 `robot.cameras.*.device`）。改完仍然直接用本文命令，不用改命令行参数。

## 3. 配置与命令索引

11 份配置的接口名（§2.1）、夹爪参数、相机块都已统一：双臂配置是 `left→left` / `right→right`，五份单臂配置按文件名声明映射（同侧 `joint`，或跨臂 `joint` / `mirror`）。用途如下：

| 功能 | 配置文件 | 章节 |
|---|---|---|
| 双臂遥操（主从一一对应） | `configs/sdk_dual_joint_follow.json` | §4.1 |
| 单臂：右主臂 → 左从臂（逐关节） | `configs/sdk_right_leader_left_follow.json` | §4.2 |
| 单臂：左主臂 → 右从臂（逐关节） | `configs/sdk_left_leader_right_follow.json` | §4.3 |
| 单臂：左主臂 → 右从臂（镜像） | `configs/sdk_single_mirror.json` | §4.3 |
| 单臂：左主臂 → 左从臂（同侧，逐关节） | `configs/sdk_left_leader_left_follow.json` | §4.4 |
| 单臂：右主臂 → 右从臂（同侧，逐关节） | `configs/sdk_right_leader_right_follow.json` | §4.4 |
| 双臂采集（含两路相机） | `configs/sdk_dual_joint_follow_cameras.json` | §5.1 |
| 双臂采集（仅关节，无相机） | `configs/sdk_dual_joint_follow.json` | §5.2 |
| 动作回放 | `configs/sdk_dual_joint_follow_cameras.json` | §6 |
| 策略 rollout（真机） | `configs/native_dual_joint_follow_rollout.json` | §7.2 |
| 策略 rollout（无硬件干跑） | `configs/native_dual_joint_follow_rollout_mock.json` | §7.3 |
| 原生 `lerobot-teleoperate`（双臂） | `configs/native_dual_joint_follow_teleoperate.json` | §4.1 |
| 原生 `lerobot-record`（双臂） | `configs/native_dual_joint_follow_record.json` | §5.1 |

`--session-config` 用于本项目 4 个命令（teleoperate / record / replay / print-observation），`--config_path` 用于 LeRobot 原生 CLI 与 rollout，**两者不能互换**。

## 4. 遥操

**全量 / 增量**：所有配置默认**增量**（`teleop.reference: "relative"`）——从臂停在启动位置，只复现主臂的位移，启动时不会追位。需要**全量**（把主臂当前位姿整体交给从臂）时，在命令后加 `--reference absolute`，不用改 JSON：

```bash
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json --reference absolute
```

全量模式要求启动时从臂与主臂每个关节都在 `startup_max_delta_rad`（3.0 rad）以内，且启动瞬间从臂会移动到主臂姿态。`--reference` 对 `onero-a1-lerobot-record` 同样有效。

启动即使能从臂；退出（`q` 或 Ctrl+C）会失能，退出前托住手臂。

### 4.1 双臂（默认）

**本项目 CLI**

```bash
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json
```

**LeRobot 原生 CLI**

```bash
lerobot-teleoperate --config_path=configs/native_dual_joint_follow_teleoperate.json
```

原生写法的差异：

- 用 `--config_path` 而不是 `--session-config`，配置必须带 `robot.type=onero_sdk` / `teleop.type=onero_sdk_teleop`，`reference` 要逐臂写（不认会话级开关，`--reference` 参数对它无效）；
- 限时用 `--teleop_time_s`（本项目是 `--duration`），循环频率取 `fps`（30 Hz），本项目是 `control_hz`（200 Hz）；
- 没有跟随诊断（`--diagnostics-interval`）、没有 `--simulate/--viewer`、也没有 `r`/`p`/`q` 交互，只做跟随。
- **原生 teleoperate 配置带两路相机**：没接相机或 USB 口不对，会在启动阶段抛 `CameraError ... did not connect within 5s` 并退出（本项目 `sdk_dual_joint_follow.json` 不含相机）。暂时没有相机时用本项目 CLI，或删掉配置里的 `robot.cameras` 段。

### 4.2 单臂：右主臂 → 左从臂

只连接用到的两条臂（`can2` 右主臂 / `can3` 左从臂），另外两条保持不动：

```bash
onero-a1-lerobot-teleoperate --session-config configs/sdk_right_leader_left_follow.json
```

### 4.3 单臂：左主臂 → 右从臂

两种映射**二选一**（`can1` 左主臂 / `can4` 右从臂，另外两条保持不动）：

```bash
# 逐关节映射（joint：每个关节按 joint_signs 的符号传给从臂）
onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_right_follow.json

# 镜像映射（mirror：主臂增量取反；加 --reference absolute 时是绝对镜像）
onero-a1-lerobot-teleoperate --session-config configs/sdk_single_mirror.json
```

单臂没有对应的原生配置：LeRobot 原生 CLI 目前只提供双臂的 `native_dual_joint_follow_teleoperate.json` 与 `native_dual_joint_follow_record.json`。

### 4.4 单臂：同侧（左主臂 → 左从臂 / 右主臂 → 右从臂）

同侧跟随把主臂关节角**原样**（`joint_signs` 全 `+1`，`joint` 模式）传给同侧从臂，`can1`→`can3`（左）、`can2`→`can4`（右），另外两条臂保持不动：

```bash
# 左主臂（can1）→ 左从臂（can3）
onero-a1-lerobot-teleoperate --session-config configs/sdk_left_leader_left_follow.json

# 右主臂（can2）→ 右从臂（can4）
onero-a1-lerobot-teleoperate --session-config configs/sdk_right_leader_right_follow.json
```

左右**不能共用一个文件**：CLI 没有覆盖 `channel` 的参数，而 `teleop.arms` / `robot.arms` 的键（`left`/`right`）、`model`、`mounting` 与 `joint_signs` 都随侧别不同。两份配置除这些字段外完全相同，换侧时改的是 4 处：`robot.arms.<side>.channel`、`teleop.arms.<side>.channel`，以及两处 `arms` 的键名。

### 4.5 运行中可用参数

```bash
# 每秒打印主臂、映射后的目标、从臂实测与误差（不录制）
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json --diagnostics-interval 1

# 限时 60 秒（到点自动收尾）
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json --duration 60

# 无硬件：mock 臂，不打开 CAN 和相机
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json --simulate --duration 5

# 无硬件：MuJoCo 窗口（x86 环境）
onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json --simulate mujoco --viewer
```

交互命令需要按 Enter：`r` 开始/结束录制，`p` 暂停/恢复跟随，`q` 退出。

跟随类型由 `teleop.reference` 决定，默认全部 `"relative"`（增量）：从臂留在启动位置，只复现主臂位移。`"absolute"`（全量，启动时加 `--reference absolute` 或改配置）把主臂位姿整体交给从臂，要求初始差在 `robot.startup_max_delta_rad`（3.0 rad）以内；这段追位不受该阈值保护，从臂被挡住或通信丢帧会立即报 `Tracking error` / `SDK state is stale` 并急停。

## 5. 数据采集

### 5.1 带相机（两路腕部）

```bash
onero-a1-lerobot-record \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_cameras --task "双臂操作采集" \
  --root ./datasets/local/a1_cameras
```

启动即开始录制，Ctrl+C 停止并保存；也可以给 `--duration 60` 限制会话时长。

**LeRobot 原生 CLI**

```bash
lerobot-record --config_path=configs/native_dual_joint_follow_record.json
```

差异：数据集参数在配置的 `dataset` 段（`repo_id`、`root`、`num_episodes`、`episode_time_s`、`reset_time_s`，示例为 10 段 × 60 秒），命令行没有 `--repo-id/--task/--root`；按键是 `→` 提前结束本段、`←` 重录上一段、`Esc` 停止；`action` 列记录的是**主臂原始动作**，与本项目 CLI 记录的"映射后从臂目标"不同，两种数据集不能混用。

### 5.2 不带相机（只采关节与动作）

```bash
onero-a1-lerobot-record \
  --session-config configs/sdk_dual_joint_follow.json \
  --repo-id local/a1_joints --task "双臂关节采集" \
  --root ./datasets/local/a1_joints
```

### 5.3 交互与保存

| 命令 | 行为 |
|---|---|
| `r` | 开始录制；再次输入结束当前 episode 并保存（保存完成后才能开始下一段） |
| `p` | 暂停/恢复跟随；暂停会先结束当前录制 |
| `q` 或 Ctrl+C | 结束会话，失能手臂，等待录制收尾 |

- `--duration` 是**会话**时长，不是录制时长。
- 保存成功的标志：`Post-record conversion complete` 和 `Episode writer finished ... status=saved`。生成期间不要关闭终端，也不要开始下一段。
- `--no-record` 可临时关掉配置里的 `recording.enabled`；`onero-a1-lerobot-teleoperate` 加 `--repo-id/--task/--root` 后可按 `r` 随时录。
- 快捷脚本：`bash scripts/record.sh <与上面完全相同的参数>`。

### 5.4 产物、追加与恢复

- 数据集：`--root` 指定的目录，标准 LeRobot 格式。
- 原始采集：`<root 的父目录>/.a1-recordings/<数据集名>-<标识>/`（`session.json`、`samples.jsonl`、`frames/`）。
- 追加录制要求相机名、尺寸、fps、编码与已有数据集完全一致，否则换新目录。
- 生成失败会保留原始采集，用下面的命令离线恢复（不连硬件，必须给新目录）：

```bash
python -m onero_a1_lerobot.deferred_recording \
  ./datasets/local/.a1-recordings/a1_cameras-<标识> \
  --root ./datasets/local/a1_cameras_recovered --repo-id local/a1_cameras_recovered
```

## 6. 动作回放

```bash
onero-a1-replay-episode \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_cameras --root ./datasets/local/a1_cameras --episode 0
```

- 回放前先用至少 3 秒平滑对齐到第 0 帧；`--align-duration 5` 放慢，`--no-align` 跳过对齐（仍保留首帧距离保护）。
- 对齐不做避障，先清空运动路径；回放不打开相机。
- `--episode N` 选片段，`--fps` 覆盖数据集帧率，`--simulate` 用 mock 从臂。
- 快捷脚本：`bash scripts/replay.sh <与上面完全相同的参数>`。

**LeRobot 原生 CLI**

```bash
lerobot-replay --config_path=configs/native_dual_joint_follow_rollout.json \
  --dataset.repo_id=local/a1_cameras --dataset.root=./datasets/local/a1_cameras \
  --dataset.episode=0 --play_sounds=false
```

差异：会连接配置里的相机（没相机直接失败，本项目回放不打开相机）；直接按数据集 `action` 列下发，没有平滑对齐到第 0 帧；无硬件时把 `--config_path` 换成 `configs/native_dual_joint_follow_rollout_mock.json`。

## 7. 策略训练与 rollout

### 7.1 训练（LeRobot 原生命令）

```bash
lerobot-train \
  --dataset.repo_id=local/a1_cameras --dataset.root=./datasets/local/a1_cameras \
  --dataset.return_uint8=true \
  --policy.type=act --policy.device=cpu \
  --policy.repo_id=local/act_a1 --policy.push_to_hub=false \
  --output_dir=outputs/train/a1_act --job_name=a1_act \
  --steps=5000 --batch_size=8 --num_workers=4 --save_freq=1000 --wandb.enable=false
```

`--policy.push_to_hub` 默认 `true`，不给 `false` 会在训练结束时尝试上传并失败。产物在 `outputs/train/a1_act/checkpoints/last/pretrained_model`。

**参数逐项说明**（"默认"是 LeRobot 0.6.1 的原生缺失值，不是本示例里的值）：

| 参数 | 默认 | 作用 | 怎么选 |
|---|---|---|---|
| `--dataset.repo_id` | 必填 | 数据集标识；本地训练只用于记录与产物命名 | 与录制时 `--repo-id` 完全一致，如 `local/a1_cameras` |
| `--dataset.root` | 无 | 数据集本地目录；不给就按 `repo_id` 去 Hub 下载 | 本地采的数据必须给 `./datasets/local/...` |
| `--dataset.return_uint8` | `false` | 图像以 uint8 返回，不在 dataloader 里转 float32 | 本项目 640×480 双相机，内存/显存紧张，建议 `true` |
| `--dataset.episodes` | 全部 | 只取指定 episode，列表写法 `--dataset.episodes='[0,1]'` | 先用 1–2 个 episode 打通流水线，再全量训 |
| `--dataset.eval_split` | `0.0` | 留出多少比例做验证集 | 想用 `--eval_steps` 就得设 >0（如 `0.1`），否则训练不会算验证 loss |
| `--policy.type` | 必填 | 策略架构，可选 `act` / `diffusion` / `pi0` / `smolvla` / `vqbet` / `tdmpc` 等 | 纯 CPU 训练、千帧级数据 → `act`（最快、最省显存、小样本表现稳）；数据上万帧且有 GPU 再考虑 `diffusion` / `smolvla` |
| `--policy.device` | 自动探测 | `cpu` / `cuda` / `mps` | 有 CUDA 版 torch 且显存 ≥8 GB 用 `cuda`，Apple Silicon 用 `mps`，其余用 `cpu`（`torch.cuda.is_available()` 为 `False` 时只能 `cpu`） |
| `--policy.repo_id` | 无 | 策略在 Hub 上的仓库名 | `push_to_hub=false` 时它只是本地标签，随便起，如 `local/act_a1` |
| `--policy.push_to_hub` | `true` | 训练结束与每次存 checkpoint 时上传 Hub | 本地训练**必须** `false`，否则收尾时上传失败 |
| `--policy.chunk_size` | `100` | 一次预测多少个未来动作 | 200 Hz 下 100 帧 = 0.5 s，够用；显存/内存不够降到 `50` |
| `--policy.n_action_steps` | `100` | 每次推理实际下发几步（≤ `chunk_size`） | 一般与 `chunk_size` 相同；想更跟手可减半（推理更频繁） |
| `--policy.dim_model` | `512` | Transformer 隐藏宽度（默认约 52 M 参数，降到 `256` 约 29 M） | 显存 ≤4 GB 或纯 CPU 训练时降到 `256`，明显省时省内存 |
| `--steps` | `100000` | 训练迭代数（不是 epoch），1 step = 1 个 batch | 每 epoch 步数 = 数据集帧数 ÷ `batch_size`（示例数据约 1600 帧、batch 8 → 约 200 步/epoch，`5000` 步 ≈ 25 个 epoch）；总时长按单步耗时折算，见下方 |
| `--batch_size` | `8` | 每步样本数 | CPU 训练保持 `8`；显存 8 GB 可上 `32`，4 GB 保持 `8`–`16` |
| `--num_workers` | `4` | 数据加载进程数 | 取 CPU 核数一半左右（如 `4`–`6`）；视频解码跟不上就加大 |
| `--save_freq` | `20000` | 每多少步存一次 checkpoint（末步必存） | 短训练设 `1000`；长训练 `5000`–`20000`，省磁盘 |
| `--output_dir` | `outputs/train/<日期>/<时间>_<job_name>` | 产物目录 | 固定成 `outputs/train/a1_act` 便于 rollout 引用；**目录已存在且没加 `--resume` 会直接报错**，重跑请换目录或加 `--resume` |
| `--job_name` | 无 | 只影响目录名与日志名 | 起个能看懂的名字 |
| `--resume` | `false` | 从 `output_dir` 里最近的 checkpoint 续训 | 训练中断后续训用；不加又复用同一目录会报 `FileExistsError` |
| `--seed` | `1000` | 模型初始化与数据打乱的随机种子 | 想复现同一结果就固定；对比实验保持不同种子 |
| `--wandb.enable` | `false` | 是否上报 Weights & Biases | 没登录 wandb 就保持 `false`（示例里显式写出只是为了明确） |

**三种常见调整**（都在上面那条命令上改，不用重写整条）：

- **只想验证流水线能跑通**：`--steps=200 --save_freq=200`，够跑到产出第一个 checkpoint，用来确认数据、相机、依赖都没问题。
- **CPU 训练太慢**：`--policy.dim_model=256 --policy.chunk_size=50 --policy.n_action_steps=50`，参数从约 52 M 降到约 29 M；**同时把 `--output_dir` 换一个新目录**，否则会因目录已存在报错。
- **要跑长训练**：换到装了 CUDA 版 torch、显存 ≥8 GB 的机器，改 `--policy.device=cuda`。纯 CPU 训练一步通常要几秒到十几秒，`--steps=5000` 可能就是十几个小时，先用小步数试跑再折算。

### 7.2 真机 rollout（只连接两条从臂）

```bash
onero-a1-lerobot-rollout \
  --config_path=configs/native_dual_joint_follow_rollout.json \
  --strategy.type=base \
  --policy.path=outputs/train/a1_act/checkpoints/last/pretrained_model \
  --task="双臂操作采集" --duration=10
```

### 7.3 无硬件干跑（mock 臂 + mock 相机）

```bash
onero-a1-lerobot-rollout \
  --config_path=configs/native_dual_joint_follow_rollout_mock.json \
  --strategy.type=base \
  --policy.path=outputs/train/a1_act/checkpoints/last/pretrained_model \
  --task="双臂操作采集" --duration=3
```

- 这条命令会先抓取两路相机的首帧，所以必须已接好 §2.4 的相机；只想验证策略链路时用 §7.3。
- rollout 的相机、分辨率必须与训练数据一致，否则策略输入对不上。
- base 策略没有键盘热键：靠 `--duration` 到点或 Ctrl+C 结束；两种方式都会先平滑回到启动姿态再失能，连按两次 Ctrl+C 跳过回位。
- 启动前把从臂摆到接近录制起始姿态，偏差过大会被首帧保护拦下。
- 加 `--display_data=true` 可看实时画面（rerun）。

## 8. 观测与可视化

```bash
# 打印从臂反馈，手臂保持原位（可在真机或 mock 上跑）
onero-a1-print-observation --session-config configs/sdk_dual_joint_follow.json --count 5 --rate 1
onero-a1-print-observation --session-config configs/sdk_dual_joint_follow.json --simulate --count 5 --rate 1

# 离线看数据集（rerun）：--root 是包含 <repo-id> 子目录的数据集根目录
bash scripts/viz.sh --repo-id local/a1_cameras --root ./datasets --episode 0

# 保存为 rrd 而不直接弹窗
bash scripts/viz.sh --repo-id local/a1_cameras --root ./datasets --episode 0 --save 1 --output-dir ./outputs/viz
```

- `scripts/viz.sh` 的 `--root` 与 `record --root` 语义不同：viz 需要**父目录**（`HF_LEROBOT_HOME`），record 的 `--root` 是数据集本身的目录。上面的命令对应 `./datasets/local/a1_cameras`。
- 本项目的会话命令（teleoperate / record / replay）没有实时画面开关：验证相机请用 `onero-a1-print-observation` 看观测，或用 `--simulate mujoco --viewer` 看仿真；rollout 用 `--display_data=true` 打开 rerun 画面。

## 9. 无硬件自检（不接机械臂、不接相机）

```bash
python -m pytest   # 需先按 §1 装测试依赖：pip install -e ".[dev]"

onero-a1-lerobot-teleoperate --session-config configs/sdk_dual_joint_follow.json --simulate --duration 5
onero-a1-lerobot-record \
  --session-config configs/sdk_dual_joint_follow_cameras.json --simulate --duration 5 \
  --repo-id local/a1_cameras_mock --task "mock test" --root ./datasets/local/a1_cameras_mock
onero-a1-replay-episode \
  --session-config configs/sdk_dual_joint_follow_cameras.json --simulate \
  --repo-id local/a1_cameras_mock --root ./datasets/local/a1_cameras_mock --episode 0
```

`--simulate` 会把**所有**臂和相机换成 mock，与配置里写死的 `backend` 无关；`--simulate mujoco` 用 SDK 物理仿真（x86 环境，`--viewer` 弹窗口，mujoco 下没有夹爪）。

## 10. 常见故障

| 现象 | 处理 |
|---|---|
| `canN: ... No such device` / 接口消失 | 重启或重插后接口不会自动恢复，重跑 §2.2 的四条绑定命令 |
| `slcand` 报接口已存在 | 先 `sudo bash scripts/cleanup_can.sh`，再重新绑定 |
| `Hardware fault on joint N: code=0x0 (Cannot verify motor state before enable)` | 使能前读该关节状态的一帧丢了（SDK 只查一次、50 ms 超时）；启动程序会自动重试 4 次。仍反复出现：`python scripts/can_precheck.py canN -n 30` 看该口丢帧率，再查这条臂 CAN 线束（尤其腕部弯折处）与终端电阻 |
| 启动后 `can3`/`can4` 的臂自己动，或夹爪报 `Gripper fault ... code=12 state=12` | 主从臂接反（`can3`/`can4` 上是主臂）：按 §2.1 重新绑定，主臂 → `can1`/`can2` |
| 拖动手臂时 `canN` 读数不变、别的通道在变 | 接口与物理臂的对应关系错了：`python scripts/can_precheck.py --watch 10 can1 can2 can3 can4` 重新确认后重绑 |
| `SDK state is stale (age=...)` | CPU 推理慢于状态阈值，调大 `robot.stale_state_s` 与 `robot.max_control_gap_s`（rollout 配置已设 1.0） |
| `canN: Joint j: 3 consecutive missed frames` | CAN 丢帧：查线序与终端电阻，并 `sudo bash scripts/setup_can.sh --tx-queue canN` |
| `Gripper position must be finite and in [0.0, 1.0]` | 策略输出越界，适配层会裁剪并告警一次；仍报错说明值非有限 |
| `Record loop is running slower ... than the target FPS` 持续刷 | 推理跟不上：降 `--fps`（如 15），或减少相机分辨率/路数 |
| `V4L2 reported a damaged frame` 偶发 | 瞬时坏帧，自动重试；频繁出现查 USB 线材与带宽 |
| 相机打不开 | 设备路径失效（换过 USB 口）或不在 `video` 组：`ls -l /dev/v4l/by-path/`、`id -nG` |
| `lerobot-teleoperate` 启动报 `CameraError ... No such file or directory` | 原生遥操配置带两路相机：按 §2.4 接好，或删掉配置里的 `robot.cameras`（本项目 CLI 的遥操配置不含相机） |
| 首帧报 `first target is too far` | 全量跟随要求从臂先摆到接近主臂位姿，或调大 `robot.startup_max_delta_rad` |
| 回放报 `Cannot align left: first replay target JN=... outside [...] rad` | 数据集首帧超出该关节限位（录制时姿态越界），对齐前的检查直接拒绝。加 `--no-align` 跳过对齐（SDK 会把目标夹回限位内，偏差即越界量），或换一段数据 |
| 启动几秒后 `Tracking error ... on joint N` 或 `SDK state is stale` | 先确认是不是加了 `--reference absolute`（默认 relative 不会启动追位）；再从臂悬空重试，并查 `dmesg` / `ip -s link show canN` 排除该通道反馈中断 |
| 串口被占用 | `sudo bash scripts/cleanup_can.sh` 清理残留会话与 `slcand` |

CAN 链路彻底重建：

```bash
lsusb -d 16d0:117e; ls -l /dev/serial/by-id/; ip -brief link show type can; pgrep -a -x slcand
sudo bash scripts/cleanup_can.sh
sudo bash scripts/setup_can.sh can1 /dev/serial/by-id/<左主臂适配器>-if00   # 依次 can2/can3/can4
python scripts/can_precheck.py
```

## 附录 A 配置字段速查

`--session-config` 文件的顶层只能是 `robot` / `teleop` / `recording` 三段：

| 段 | 关键字段 | 说明 |
|---|---|---|
| `robot` | `arms.{left,right}` | 从臂：`model`(A1L/A1R)、`backend`(can/mock/mujoco)、`channel`、`mounting`、`with_gripper` |
| | `cameras.{left_wrist,right_wrist}` | `backend`(usb/mock)、`device`、`width/height/fps`、`auto_exposure`、`exposure_absolute` |
| | `control_hz`、`max_velocity`、`max_acceleration` | 跟随频率与限幅（默认 200 Hz / 3 rad/s / 80 rad/s²） |
| | `continuous_follow`、`clip_joint_limits`、`velocity_feedforward_scale` | 连续跟随、关节限位裁剪、速度前馈 |
| | `startup_max_delta_rad`、`max_tracking_error_rad` | 首帧保护（默认 0.15 rad；双臂全量配置 3.0）与跟踪误差保护（0.5 rad） |
| | `isolate_can_processes`、`control_cpu_affinity` | CAN 子进程隔离、控制线程绑核（`"auto"`） |
| `teleop` | `arms.{left,right}` | 主臂：`model`、`channel`、`mounting`、`handle_open_code`/`handle_closed_code`（按压手柄码，只有主臂能写） |
| | `mappings.<side>.{source,mode,scale,joint_signs}` | 从臂侧的映射：`mode`=`joint`（逐关节，需 7 个 ±1 的 `joint_signs`）或 `mirror`（固定取反） |
| | `reference` | 会话级跟随类型，覆盖所有 mapping：`absolute` 全量、`relative` 增量（默认）；启动命令加 `--reference` 可临时覆盖 |
| | `damping`、`isolate_can_processes` | 主臂阻尼与进程隔离 |
| `recording` | `enabled`、`fps`、`video_encoder` | 采集开关、帧率、编码器（`auto`/`libx264`/`h264_rkmpp`） |
| | `repo_id`、`task`、`root` | 可被命令行的 `--repo-id/--task/--root` 覆盖 |

`--config_path` 文件（`native_*`）是 LeRobot 的 draccus 配置：多出 `robot.type=onero_sdk`、`teleop.type=onero_sdk_teleop`，映射类型必须**逐臂**写 `reference`（不认会话级开关），录制配置另有 `dataset` 段。

## 附录 B 夹爪与主臂手柄

- `with_gripper=true`：从臂驱动并采集实体夹爪（0~1，1 为张开）；mock 后端用内存夹爪，`--simulate` 与真机特征一致；MuJoCo 没有夹爪。
- `gripper_protocol`：实际 CAN 协议（`onero` 默认 / `omnipicker` / `none` 不发夹爪帧）；`gripper_model`：只影响可视化工装模型（`woan_picker` 默认）。
- 主臂按压手柄必须按实测填 `handle_open_code`/`handle_closed_code`（成对、不相等）；填了就按只读手柄读取，不填则按普通夹爪驱动。
