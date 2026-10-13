# A1 CAN 主从框架

本项目以 LeRobot 的 `Robot` / `Teleoperator` 接口组织 A1L/A1R 控制，
使用 `armsdk` 公共 `Arm` API 访问机械臂。真机控制通过 CAN，USB 相机直接连接部署电脑；
mock 与 MuJoCo 用于无机械臂硬件验证。底层电机协议、模型、零位、动力学及 SDK 安全约束由 `armsdk` 维护。

## 模块职责

```text
遥操作 / 录制 CLI → SdkSession → OneroSdkTeleop → ArmMapper
                             → OneroSdk → JointSlewLimiter
                                          → SdkDevice / 进程隔离设备
                                            → armsdk.Arm
                                              → SocketCAN / SLCAN
                                              → SDK MuJoCo
                             → USB 相机后台采集 → RGB 最新帧缓存
                             → EpisodeRecorder → 有界 RGB 引用队列
                               → 搬运线程 → JPEG / 采样记录 IPC
                                 → 常驻 spawn 录制进程 → 原始文件落盘 → 停止后生成 H.264 MP4
```

| 模块 | 职责 |
|---|---|
| `src/onero_a1_lerobot/sdk_config.py` | 设备、映射及控制配置；注册 `onero_sdk` / `onero_sdk_teleop` |
| `src/onero_a1_lerobot/sdk_device_config.py` | CAN 工作进程使用的纯 SDK 设备配置，避免引入 LeRobot/PyTorch |
| `src/onero_a1_lerobot/features.py` | 每侧七个关节位置及可选夹爪特征，固定左右顺序 |
| `src/onero_a1_lerobot/arm_mapping.py` | 主从方向/参考映射，速度与加速度限制 |
| `src/onero_a1_lerobot/sdk_client.py` | SDK 依赖检查、连接、状态、伺服、拖动与关闭 |
| `src/onero_a1_lerobot/sdk_robot.py` | LeRobot 从臂接口，接受物理目标角 |
| `src/onero_a1_lerobot/sdk_teleoperator.py` | LeRobot 主臂接口，输出映射后的从臂目标 |
| `src/onero_a1_lerobot/sdk_session.py` | 连接协调、参考捕获、控制循环、暂停、故障及录制 |
| `src/onero_a1_lerobot/sdk_process_leader.py` | 将 CAN 主臂拖动放入独立进程 |
| `src/onero_a1_lerobot/sdk_process_follower.py` | 将 CAN 从臂事务放入独立进程 |
| `src/onero_a1_lerobot/camera_config.py` | USB/mock 相机路径、格式和时效配置 |
| `src/onero_a1_lerobot/v4l2_camera.py` | V4L2 MJPEG 格式与 mmap 缓冲区采集 |
| `src/onero_a1_lerobot/camera.py` | 后台采集、同帧 JPEG/RGB 缓存、重连 |
| `src/onero_a1_lerobot/episode_recording.py` | 录制状态、有界引用队列、搬运线程及录制后 LeRobot 写入 |
| `src/onero_a1_lerobot/recording_process.py` | 独立磁盘采集进程及 IPC，停止后触发视频生成 |
| `src/onero_a1_lerobot/recording_image.py` | 原始 JPEG/RGB 载荷与录制后解码 |
| `src/onero_a1_lerobot/deferred_recording.py` | 原始图像/采样记录落盘、停止后生成及失败恢复入口 |
| `src/onero_a1_lerobot/streaming_recording.py` | 录制结束后的多相机 H.264 编码及视频收尾 |
| `src/onero_a1_lerobot/ffmpeg_encoder.py` | 系统 FFmpeg/Rockchip 硬件编码检测、有界管道及输出帧数检查 |
| `src/onero_a1_lerobot/replay_alignment.py` | 回放首帧的限速平滑对齐、到位确认及控制配置恢复 |
| `src/onero_a1_lerobot/sdk_viewer.py` / `sdk_window.py` | SDK MuJoCo 场景展示及交互 |

单臂与双臂使用同一实现，只改变端点与映射配置。每条物理臂独占一个 CAN 通道；
重复通道、未知映射源或目标、无效关节数量在连接前拒绝。

## 硬件接口与模型

`SdkArmConfig` 明确指定 `A1L` 或 `A1R`、backend、channel 和 mounting。
`canN` 等接口通过 SDK 原生 SocketCAN 连接；`/dev/...` 串口通过
`PythonCanBus(interface="slcan", buffered_slcan=True)` 连接，CAN 速率及串口速率均为 1 Mbps。
主从双方不得由其他控制程序同时占用。

适配层从 SDK 模型读取七轴关节范围与速度上限，调用 `Arm.servoj()` 发送位置和可选速度前馈。
主臂使用 SDK `freedrive_enable()`，依靠 Pinocchio 重力补偿拖动。
安装方向影响重力补偿，必须对应真实装配。零位由 SDK 管理，适配器不重写零位。

模型和网格均使用 SDK 自带的 A1 资源。`with_gripper` 选择 SDK 重力补偿配置；
`gripper_protocol` 独立选择实体 CAN 协议，CAN 真机下据此创建夹爪驱动
（默认 `OneroGripper`，可选 `OmniPickerGripper` 或 `none`），`gripper_model` 独立选择
Link7 上的可视化工具模型；这两项只对从臂生效，主臂固定 `onero`/`woan_picker`（后续由按压式
手柄提供开合度），从臂则按实际装配选择。主臂夹爪开合度经
`{side}_arm.gripper.pos` 特征传给从臂并进入录制数据集。mock/mujoco 中仅选择重力补偿配置，
不提供夹爪动作。当前没有移动底盘或其他关节的特征。

`robot.cameras` 声明 USB 相机，当前双腕示例为 `left_wrist` / `right_wrist`，
默认 MJPEG 1600 × 1200、30 fps。V4L2 mmap 采集在后台完成，OpenCV 仅负责 JPEG 解码。
设备返回尺寸必须匹配配置，不通过静默缩放掩盖差异。新电脑须填入实际稳定设备路径，
示例 `/dev/videoLeft` / `/dev/videoRight` 不是自动生成的接口。

## 数据契约与映射

位置单位为 rad，特征名称为：

```text
left_arm.joint1-l.pos ... left_arm.joint7-l.pos
right_arm.joint1-r.pos ... right_arm.joint7-r.pos
```

无夹爪时单臂七维、双臂按左七维右七维排列。CAN 下启用夹爪时，
每侧七个关节之后附加 `{side}_arm.gripper.pos`（0~1，1 为张开）；两臂均启用时为 16 维。
`observation.state` 为从臂实测角度，`action` 为适配层限制后提交给 SDK 的目标角。
启用的夹爪开合度观测和目标分别包含在这两个向量中。
SDK 还会施加自己的约束，因此数据集动作不能解释成已经测量确认的最终电机指令。

配置相机后，Robot 观测以相机名增加 RGB HWC `uint8` 图像，数据集使用
`observation.images.<name>` 标准视频特征。`camera.<name>.time_s` 与
`camera.<name>.sequence` 作为独立 float64 字段，不并入 `observation.state`。
相机时间是主机取得帧的单调时钟时间，不是硬件曝光时间；两路相机与 CAN 不保证硬件同步。
录制可能复用同一最新缓存帧，序号用于识别重复图像，`control.time_s` 保留控制采样时间。

`joint` 映射使用显式七个 ±1 方向系数；`mirror` 对全部七个关节取 -1。
相对参考在会话准备时捕获：

```text
q_target = q_follower_start + signs * scale * (q_leader - q_leader_start)
```

首帧目标等于从臂起始姿态。暂停后恢复需要重新捕获参考，不追赶暂停期间的旧目标。
映射发生在 Teleoperator 一处；Robot、数据集回放和策略执行都接收从臂绝对位置。
关节映射不承诺任意姿态下的末端轨迹几何一致。

relative 映射必须使用 `SdkSession` 的准备流程，直接独立构造 Robot/Teleoperator
不会自动取得双方参考。回放与 policy rollout 不经过主臂映射，数据集和策略必须匹配
配置的名称、顺序、关节弧度单位和可选夹爪的 0~1 开合度特征。

## 生命周期与控制

```text
校验配置 / 依赖
    ├─ 相机后台连接 → 首帧 / 最新缓存
    ├─ 机械臂并行连接 → 先就绪从臂保持 → 全臂就绪 → 捕获参考 → 跟随
    └─ 录制器独立 spawn 启动 → 数据集初始化

相机与录制器就绪 → 接收录制帧 → 结束 episode / 保存 / finalize
暂停保持 → 显式恢复并重新捕获参考
退出拖动 / 请求失能 / 关闭连接 / 录制收尾
```

`send_action=false` 只抑制 Robot 的跟随命令；底层连接仍会使能机械臂并建立初始保持。
`--simulate` 将所有相机替换为 mock，`--simulate mujoco` 也只生成合成图像而非场景渲染。
需要真实 USB 加模拟臂时，在 JSON 中分别选择相机 usb、机械臂 mock，并且不加 `--simulate`。
正常退出调用 SDK `close(disable=True)`，
请求失能所有已连接机械臂；暂停则持续保持姿态。

在 `SdkSession` 中，相机连接是独立后台任务，机械臂连接与录制器 `spawn` 启动并行进行。
示例的四个隔离 CAN 端点并行初始化，先连接完成的从臂持续保位。
在启动相机后台任务和任何设备/录制子进程前，同步完成相机软件依赖导入；
这一阶段不打开相机，用于避免 OpenCV bootstrap 临时 `sys.path` 与 `spawn` 路径快照竞态。
全部机械臂就绪后刷新反馈、初始化限幅器、捕获主从参考并进入 following，
不等待相机首帧或录制器初始化。相机和录制器未就绪时只等待录制，不阻塞机械臂跟随。

CAN 子进程使用拆分后的纯设备配置与包的按需导出，避免四个 worker 重复导入 LeRobot/PyTorch，
保留 `spawn` 隔离和标准 LeRobot 接口。SLCAN 驱动约 2 秒的稳定等待仍保留。
`Startup:` 记录软件与设备各阶段耗时；本次优化的实际改善尚未测量，不承诺固定启动秒数。

相机 `connect_timeout_s` 默认 5 秒，`stale_frame_s` 默认 0.5 秒。
相机启动失败只使当前录制尝试失败，输入 `r` 可异步重试相机并请求新的 episode。
采集与重连后台运行，控制循环通过最新帧缓存获取图像，不等待 USB 读取。
直接调用原生 `OneroSdk.connect()` 时仍要求已配置相机可用，以保持策略观测接口的完整性；
这是独立 Robot 与会话协调器启动行为的区别。

控制 CPU 亲和性选择部分可用核心时，相机线程和录制子进程使用剩余后台核心。
剩余集合由进程启动前允许的 CPU 集合减去控制线程实际使用的集合计算，不硬编码核心编号。
没有空余核心时共享 CPU 并明确记录日志。进程和亲和性隔离不是硬实时保证，
250 ms 控制断流阈值保持不变，不为数据集初始化、图像写入或视频编码放宽。

控制循环读取主臂缓存反馈，通过映射、有限值校验、首帧距离检查和运动限制生成命令。
从臂 `servoj()` 发送命令并等待反馈，不在同周期重复主动查询。
进程隔离允许独立 CAN 事务并发，但每轮等待已提交事务完成，且不承诺双路命令原子到达。

`continuous_follow` 为移动目标生成连续的位置与速度参考。
`velocity_feedforward_scale` 只缩放送入 SDK 的速度前馈，不改变规划位置、速度及加速度约束。
`clip_joint_limits` 控制适配层的位置裁剪与边界制动，SDK 的独立限位始终保留。
控制频率是请求值，实际周期和抖动应由运行测量确认；SDK 主臂 freedrive 频率独立于会话频率。

机械臂控制故障锁定会话，停止跟随并尽量刷新仍可控设备的保持目标；不自动清除 SDK 急停或重新使能。
整帧时间戳不能证明每个关节反馈都是新数据，应用间隔检查也不能替代独立硬件看门狗。
通信断开、底层 I/O 阻塞或进程被强杀时，软件无法保证停止或失能帧送达。

## 录制、回放与命令入口

纯跟随默认不录制；提供完整数据集参数时启动空闲录制进程，请求录制时创建原始采集目录。
采集与机械臂控制独立管理就绪状态。相机后台同时缓存同一帧的 RGB 和原始 JPEG，策略读取 RGB，
会话录制只引用原始 JPEG。主控制循环不执行图像复制、IPC、磁盘写入或视频编码。

搬运线程将压缩图像和同组采样元数据经 IPC 交给常驻 `spawn` 录制进程。
子进程直接写图像文件，提交 `samples.jsonl` 后 ACK；采集阶段不导入 LeRobot/PyTorch，不启动编码器。
无 JPEG 的 mock/RGB 输入由搬运线程打包为原始 RGB 字节。不存在共享 RGB 传输槽。

停止录制后，先排空待写队列，再在子进程加载 LeRobot/PyTorch、读取采集文件，
使用现有 H.264 编码器生成视频和数据集。生成按采样组顺序进行，图像、状态和动作一一对应。
`recording.video_encoder` 可选 `auto`、`h264_rkmpp`、`libx264`，硬件选择只在生成阶段执行。
软件参数为 `ultrafast`、`zerolatency`、`bf=0`，硬件为 CQP、QP=23，GOP 均等于录制 fps。

原始文件位于数据集父目录的 `.a1-recordings/`，成功生成后清理，失败/超时保留以供离线恢复。
`session.json` 保存配置/特征，`samples.jsonl` 保存采样组，`frames/` 保存图像；文件锁避免
恢复命令读取仍在写入或生成的采集目录。恢复输出必须使用新数据集目录，防止重复追加。
录制进程跨 episode 复用，但必须等当前生成完成后才能开始下一段。

默认 300 组 / 256 MiB 限制待写载荷和预留 IPC 副本，JPEG 按压缩大小计费。
IPC 预留为最大单组载荷的 4 倍；内存不保存整段图像，原始文件持续写磁盘。
磁盘错误、持续慢写或相机异常会停止当前录制并尝试处理已保存部分，不停止 CAN 跟随。
采集吞吐日志与停止后的生成进度分开。`finalize_timeout_s` 默认 300 秒，限制连续无进度时间。
子进程通过同一 RPC 通道上报已转换帧数和阶段，父进程续期 RPC 和录制线程/进程退出等待。
进度属性独立于长时间持有的 RPC 锁，避免关闭线程因等锁触发固定总时限。总转换时间不设 300 秒上限。
依赖加载和最终数据集保存等阶段没有逐步进度时仍受单阶段闲置时限约束；实际停滞会终止录制子进程，
保留未完成源文件并报告失败。最终保存状态仅在收到完成回复后确定。
`--duration` 仍是会话运行时间，视频时长为保存样本数除以录制 fps。
已有数据集必须匹配全部特征、相机名称/尺寸及 fps，不能将单臂数据直接追加为双臂数据，
也不能向仅关节数据集追加视频。未配置相机时原有纯关节采集行为保持不变。

- `onero-a1-lerobot-teleoperate`：主从会话，必须 `--session-config`。
- `onero-a1-lerobot-record` / `onero-a1-record-episode`：相同会话，默认开始录制。
- `onero-a1-replay-episode`：必须 `--session-config`，仅连接从臂，读取动作并校验 schema，不打开 USB 或解码视频。
- `onero-a1-print-observation`：必须 `--session-config` 且 `send_action=true`，持续保位并打印从臂反馈。
- `onero-a1-lerobot-rollout`：原生 LeRobot 参数，设备类型为 `onero_sdk`，不接受 `--session-config`。

发布包为 `lerobot_robot_onero_a1`，功能模块位于 `onero_a1_lerobot`。
安装、会话示例、CAN 设置和模型仿真见[使用说明](usage_sdk_follow.md)，
USB 设备配置与视频录制见[相机接入指南](usage_usb_cameras.md)。

## 验证范围

既有无硬件检查覆盖配置校验、特征顺序、映射、限速、生命周期、录制状态机和参数路由。
mock 仅验证软件流程；SDK MuJoCo 验证模型运动，展示中的独立机械臂不构成跨臂碰撞仿真。
此前的数据集验证不覆盖本次原始文件落盘与停止后生成流程。
本轮未运行测试或真机验证，实际启动耗时、持续录制吞吐与故障行为由部署方验收。
`get_observation()` 可为推理提供图像，但完整视觉策略训练与真机部署尚未端到端验证。
实机仍需独立验证 USB 格式与吞吐、CAN 通道、安装方向、零位、控制周期、跟踪误差及故障停止行为。
