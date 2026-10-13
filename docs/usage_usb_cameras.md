# USB 双腕相机与视频录制

USB 相机直接插在运行本项目的 Linux 部署电脑上，与 A1 CAN 主从控制在同一会话中使用。
采集通过 V4L2 的 MJPEG + mmap 后台读取，不需要相机服务或 ROS 节点。
[`configs/sdk_dual_joint_follow_cameras.json`](../configs/sdk_dual_joint_follow_cameras.json)
在双臂跟随配置上增加 `left_wrist` 和 `right_wrist` 两路相机；未配置 `robot.cameras`
时，原来的关节跟随和录制流程不变。CAN 真机的 `with_gripper=true` 同时启用夹爪控制与采集，
实体协议由 `gripper_protocol` 显式选择（见[跟随用法](usage_sdk_follow.md)）。

## 1. 安装依赖

默认部署流程保持不变，在本项目根目录执行：

```bash
A1_LEROBOT_VENV="$PWD/.venv" bash scripts/setup.sh
source .venv/bin/activate
```

Python 3.12+ 环境默认安装 LeRobot 与视频编解码依赖。x86_64 同时安装仿真、训练和可视化依赖，
arm64 精简为控制与数据采集所需。采集需要的 OpenCV 在默认 LeRobot 环境中已有依赖；
本项目只用它解码 JPEG，不用 OpenCV 打开摄像头。

使用 `--base` 安装的环境，单独补充 USB 相机解码依赖：

```bash
python -m pip install -e '.[camera]'
```

`camera` extra 为 `opencv-python-headless>=4.9,<4.14`。要在 Python 3.12+ 环境中录制，
可以重新运行默认 setup，或安装相机与 LeRobot 依赖：

```bash
python -m pip install -e '.[camera,lerobot]'
```

## 2. 确认本机设备与格式

部署示例用 `/dev/videoLeft` / `/dev/videoRight` 作别名，参考配置里写的则是
`/dev/v4l/by-path/...` 路径；新的电脑不会自动具有这些路径。
先检查实际设备：

```bash
ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
id -nG
```

有唯一序列号时优先选稳定的 by-id 路径。同型号摄像头没有唯一序列号时，可以使用 by-path
区分 USB 端口；更换插口后应重新确认路径及左右对应关系。不要照搬其他机器的 USB 端口规则，
也不要只凭 `video0` / `video1` 的数字判断左右相机。

确认当前用户具有设备读写权限；设备由 `video` 组管理时，用户需要加入该组，重新登录后生效：

```bash
sudo usermod -aG video "$USER"
```

安装有 `v4l2-ctl` 的主机上，分别替换成左右相机的实际设备路径，检查格式列表：

```bash
v4l2-ctl -d /dev/v4l/by-id/实际左相机设备 --list-formats-ext
v4l2-ctl -d /dev/v4l/by-id/实际右相机设备 --list-formats-ext
```

示例要求 `MJPG`、1600 × 1200、30 fps。设备不支持时，选择它实际支持的 MJPEG 尺寸和帧率，
并修改 JSON 的 `width`、`height`、`fps`。实现会校验返回的格式与尺寸，不会静默缩放图片来满足配置。

## 3. 填写会话配置

复制或编辑 [`configs/sdk_dual_joint_follow_cameras.json`](../configs/sdk_dual_joint_follow_cameras.json)，
核对全部 CAN 通道和安装方向，并将 `robot.cameras` 中的设备路径替换为本机实际路径。
下面是相机配置片段，放在已有 `robot` 对象中：

```json
{
  "cameras": {
    "left_wrist": {
      "backend": "usb",
      "device": "/dev/videoLeft",
      "width": 1600,
      "height": 1200,
      "fps": 30,
      "connect_timeout_s": 5.0,
      "stale_frame_s": 0.5
    },
    "right_wrist": {
      "backend": "usb",
      "device": "/dev/videoRight",
      "width": 1600,
      "height": 1200,
      "fps": 30,
      "connect_timeout_s": 5.0,
      "stale_frame_s": 0.5
    }
  }
}
```

`connect_timeout_s` 默认为 5 秒，限制启动时等待首帧的时间；`stale_frame_s` 默认为 0.5 秒，
超过这个年龄的缓存帧不能用于录制。

会话中，相机连接、机械臂连接和录制器启动分别并行进行，不相互等待。
创建子进程前会先同步完成 OpenCV 的首次软件导入，不打开 USB 设备。
OpenCV 加载器会临时修改 Python 模块搜索路径；这一步避免子进程继承临时路径，
将 `cv2/typing` 错认成标准库 `typing`。导入失败也会恢复搜索路径，并将相机软件标记为不可用；
修复依赖后需重新启动会话，`r` 只重试设备连接，不重试失败的软件导入。
示例启用了 CAN 进程隔离，四条机械臂并行初始化；先就绪的从臂持续保位，
全部机械臂就绪后捕获主从参考并开始跟随。相机首帧或录制器尚未就绪时只等待录制，
不会推迟跟随。相机不可用、权限不足或格式不匹配只使当前录制尝试失败，不阻止机械臂运行。
处理设备问题后输入 `r`，会异步重试相机并请求新一段录制。

上述独立启动是 `SdkSession` 的行为。直接调用原生 `OneroSdk.connect()` 时，
仍要求配置的相机可用，以满足策略调用 `get_observation()` 的图像观测要求。
启动日志分别显示相机、机械臂和录制器的进度；`decoded RGB FPS` 是后台采集统计，
不能用它判断机械臂或录制器是否已经就绪。

`Startup:` 耗时包括子进程启动、软件导入及设备连接，不能直接等同于某一次 CAN 读写时间。
CAN 工作进程改用纯 SDK 设备配置和按需包导出，避免四个 worker 重复导入 LeRobot/PyTorch。
仍使用 `spawn`，SLCAN 驱动约 2 秒的稳定等待也保留；实际启动改善尚未测量，不承诺固定几秒完成。

相机采集和重连在后台运行。控制循环只取最新缓存，不等待 USB 读取；两路相机分别采集，
并不保证在同一时刻曝光。

## 4. 先验证模拟，再开始采集

全模拟录制保留两路图像特征，但不打开 USB 或 CAN：

```bash
onero-a1-lerobot-record \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --simulate --duration 5 \
  --repo-id local/a1_camera_mock --task "dual arm camera test" \
  --root ./datasets/local/a1_camera_mock
```

`--simulate` 会把所有相机换成 mock。使用 `--simulate mujoco` 时，机械臂有物理仿真，
相机仍生成合成图像，图像不是 MuJoCo 场景渲染结果。
换成 mock 时配置里的 V4L2 曝光项（`auto_exposure` / `exposure_absolute`）会被清空，
因为合成图像没有曝光控制；分辨率、帧率和相机名称保持不变，数据集 schema 不变。

需要验证真实 USB 相机、同时保持机械臂模拟时，在单独的 JSON 中将所有
`robot.arms` 和 `teleop.arms` 设为 `backend="mock"` 并删除 `channel`，
保留 `robot.cameras.*.backend="usb"`，运行时不加 `--simulate`。

确认真机配置和设备格式后开始跟随，提供录制参数但不立即录制：

```bash
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_wrist --task "dual arm wrist camera collection" \
  --root ./datasets/local/a1_wrist
```

机械臂全部就绪后开始 CAN 主从跟随，不等待相机或录制器。交互终端输入 `r` + Enter 请求录制，
待相机和录制器就绪后采样；再次输入结束并保存，保存时跟随继续。
使用 `onero-a1-lerobot-record` 默认在启动时请求录制，初始化期间机械臂仍可先开始跟随。
录制器和相机就绪后才接收采样，不会补录初始化期间的动作与图像；`--duration` 控制跟随会话的运行时间。
例如 `--duration 60` 不表示一定录满 60 秒，异步录制准备耗时仍占用会话运行时间。
输入 `q` + Enter 或 Ctrl+C 退出整个会话，会请求失能机械臂并等待录制收尾。

## 5. 数据内容与时间含义

图像单独作为标准 LeRobot 视频特征，不改变关节和夹爪的向量维度：

| 字段 | 内容 |
|---|---|
| `observation.state` | 每臂七个实测关节角（rad），启用夹爪时附加一个开合度（0~1） |
| `action` | 对应关节目标（rad）与可选夹爪目标（0~1） |
| `observation.images.left_wrist` | 左腕 RGB 视频 |
| `observation.images.right_wrist` | 右腕 RGB 视频 |
| `camera.left_wrist.time_s` / `camera.right_wrist.time_s` | 独立 float64 字段，主机取得该相机帧的单调时钟时间 |
| `camera.left_wrist.sequence` / `camera.right_wrist.sequence` | 独立 float64 字段，相机帧序号 |
| `control.time_s` | 主机控制采样时间 |

无夹爪时单臂/双臂为 7/14 维，两臂均启用夹爪时为 8/16 维，顺序为每侧七个关节后接该侧夹爪。
启用夹爪后的特征与原来仅关节的数据集不兼容，需使用匹配的新数据集或策略。

Robot 的图像观测是 RGB、HWC、`uint8` 数组，配置 1600 × 1200 时形状为 `(1200, 1600, 3)`。
相机时间和序号不并入 `observation.state`，不会把关节状态维度扩大。

相机时间是主机取帧时间，不是硬件曝光时间，也不代表相机与 CAN 硬件同步。
控制、相机和录制频率独立；录制可能连续使用同一缓存帧，比较 `sequence` 可以识别重复帧，
结合 `time_s` 和 `control.time_s` 可以检查取帧与采样之间的时间差。

录制期间只保存图像和采样记录，停止录制后才生成 H.264 MP4 和 LeRobot 数据集。
USB 相机直接保存取帧时已得到的 MJPEG/JPEG 字节，不重复编码 JPEG，也不传输完整 RGB；
mock 或不带 JPEG 的图像源保存原始 RGB 文件。策略接口仍返回 RGB 数组。
图像路径、左右相机时间/序号、关节、动作和控制时间按同一组写入 `samples.jsonl`，
一组图像和记录写完后才确认消费。视频生成按原采样顺序逐组读取，不把整段图像装进内存。

以下编码配置仅在停止录制后的生成阶段使用。
`recording.video_encoder` 默认 `auto`：检测到 Rockchip MPP 设备和系统 FFmpeg 的
`h264_rkmpp` 编码器时使用硬件编码，否则使用 PyAV/libx264 软件编码。
也可指定 `h264_rkmpp` 或 `libx264`；指定硬件编码但依赖或权限缺失时明确报错。
编码结束后核对视频帧数、保存统计和元数据，保持标准 LeRobot 数据集结构与图像统计。
进程跨 episode 复用，结束当前录制后仍按段保存，再次输入 `r` 创建新的 episode。
启用相机后应使用匹配完整图像特征的新数据集；不能向原来只有关节状态的数据集直接追加视频。
追加录制要求相机名称、图像尺寸、其他特征及录制 fps 匹配。
已有视频的编码参数也必须与当前录制器一致；不一致时拒绝追加，需使用新的 `repo_id` / `root`。
这避免将不同编码设置混入同一视频文件，而元数据仍保留首段设置。
Rockchip 的 RGB 输入可能生成记录为 `yuvj420p` 的 H.264 视频，不能用软件编码器默认的
`yuv420p` 直接判断硬件输出不兼容。硬件追加先检查后端及参数，再在提交 episode 前读取
新视频的实际编码、像素格式、尺寸、帧率和帧数，与已有数据集核对；两种像素格式不会被
直接视为相同，也不会为通过检查而改写旧视频元信息。报错会列出具体相机及不一致字段。

## 6. 队列、保存与恢复

`recording` 可显式配置以下参数：

```json
{
  "enabled": false,
  "fps": 30,
  "video_encoder": "auto",
  "queue_size": 300,
  "max_queue_bytes": 268435456,
  "finalize_timeout_s": 300.0
}
```

主控制循环只向有界队列提交不可变图像引用和采样信息，搬运线程经 IPC 交给常驻
`spawn` 录制进程。采集阶段不加载 LeRobot/PyTorch、不启动视频编码器。
图像复制、IPC 和磁盘写入均在后台，CPU 亲和性仍动态使用控制核之外的可用核心。

原始数据位于数据集父目录的 `.a1-recordings/<数据集名>-<随机标识>/`：

```text
session.json       录制配置、特征、生成状态
samples.jsonl      每组图像路径、关节、动作和时间戳
frames/*.jpg       USB 相机原始压缩帧（mock 为 *.rgb）
```

停止录制后，先写完队列，再加载 LeRobot/PyTorch，逐组读取原始文件并生成视频与数据集。
输入 `r` 停止录制时，主从跟随继续；输入 `q`、Ctrl+C 或到达 `--duration` 时，关闭机械臂后等待收尾。
等待日志 `Post-record conversion complete` 和 `Episode writer finished ... status=saved` 表示保存完成。
生成完成前不能开始下一段。`finalize_timeout_s` 默认 300 秒，表示连续无进度的最长等待时间，
不再限制整个生成过程的总时长。每组写盘、每帧转换及阶段切换都会更新进度；父进程的 RPC、
录制线程等待和进程关闭都遵循这一规则。持续转换时，即使总耗时超过 300 秒也不会提前报失败。
依赖加载、最终统计/元数据保存等不报告中间进度的单独阶段仍须在该时限内完成，必要时增加该值。
实际无进度超时会停止本次录制子进程并保留未完成的源文件；子进程异常退出也会报告错误。
采样速度不再受视频编码速度限制，结束后的生成可能较慢。

软件 H.264 使用 `ultrafast`、`zerolatency`、`bf=0`，每路 `encoder_threads=1`。
Rockchip 路径为每路相机启动一个系统 FFmpeg 进程，通过 MPP 编码 H.264，使用 CQP、QP=23，
每路输入队列大小为 1；优先直接交付 RGB，按编码器支持情况使用 RGB/NV12 输入格式。
两种路径的 GOP 都等于 `recording.fps`（约每秒一个关键帧）。
`Recording writer ready` 只表示原始文件写入器就绪；相机就绪仍独立判断。
编码器和数据集兼容性在停止录制后的生成阶段检查，失败时保留原始采集文件。
硬件失败会报告录制错误，不在录制中途静默切换后端。实际后端和参数写入图像特征元数据。
示例仍为两路 1600 × 1200、30 fps，实际能否持续写满目标帧率需要在部署机器上测量。

`queue_size` 默认 300 组，`max_queue_bytes` 默认 256 MiB，两者共同限制缓冲量。
字节预算计入排队/正在写入的图像载荷，以及为 IPC 临时副本预留的空间（最大一组载荷的 4 倍）。
USB JPEG 按压缩后的字节数计费，不再固定为每组 11.52 MB，也不再有共享 RGB 槽。
整段原图保存在磁盘上，内存中只保留有限待写组；mock 原始 RGB 比 JPEG 占用更多内存与磁盘带宽。
这不是整个进程的内存上限，相机解码缓存、系统缓存及生成阶段编解码器另占内存。
队列仍用于完整保存采样，不会静默覆盖历史图像；若磁盘持续写入过慢或写满，仍会报告错误。

当控制 CPU 亲和性只选择部分可用核心时，相机采集线程和录制子进程使用剩余后台核心。
后台核由“启动前进程允许使用的 CPU 集合减去控制线程实际使用的 CPU 集合”动态计算，
不固定任何核心编号，适用于不同机器以及容器或系统施加的 CPU 范围限制。
没有剩余核心时会记录共享 CPU 的情况。这不构成硬实时保证，控制断流阈值仍为 250 ms，
不会为录制启动或编码放宽。

运行期间发生相机断连、缓存帧过期、队列超限或磁盘写入错误时，会结束当前录制并报告错误，
尝试保存已经接收的部分数据，CAN 主从跟随继续。相机后台会尝试重连；恢复后需等保存结束，
再由用户明确输入 `r` 开始新 episode，不会自动续写故障前的那一段。
成功生成数据集后自动清理本次原始文件。编码失败、数据集不兼容或生成超时时保留源目录，
日志会给出位置；采集阶段写盘失败可能只留下部分完整样本。

退出采集进程后，可将保留数据重新生成为新的数据集（不打开相机或机械臂）：

```bash
python -m onero_a1_lerobot.deferred_recording \
  ./datasets/local/.a1-recordings/a1_cameras-实际标识 \
  --root ./datasets/local/a1_cameras_recovered --repo-id local/a1_cameras_recovered
```

恢复命令要求新的 `--root`，拒绝读取仍在采集/生成的数据；手动恢复成功后保留源文件。
若需在没有 Rockchip 编码器的电脑上恢复，可加 `--video-encoder libx264`。

录制器初始化期间取消或停止时，会记录
`Recording preparation completed after stop; no frames accepted`；没有接收帧的尝试以 `empty` 结束，
不报告 `0 frames saved`。若此前已有机械臂控制故障，仍保留录制错误及其原因。

### 查看实际写入速度

采集前 3 组输出 `Capture frame...`，表示图像 IPC 和写盘耗时，不包含视频编码。
每 5 秒的 `Capture throughput=... target... queued... image_budget...` 显示实际落盘速率与排队情况。
停止后才会出现 `Post-record video generation`、`Video group`、`Dataset frame` 和转换进度日志。
相机的 `decoded RGB FPS` 只反映采集解码速率，不能代替录制消费速率。

需实测采集落盘速率是否达到目标，不能用相机的解码 FPS 或录制结束后的编码 FPS 代替。

视频按固定帧率播放，时长为实际保存帧数 `N / recording.fps`。
例如只保存 22 帧、fps 为 30 时，视频约 0.73 秒；队列超限保存的部分数据不会因为
`--duration 60` 自动补成一分钟。原有的 22 帧数据保留不变；比较新编码路径时使用新的
`--repo-id` 和 `--root`，避免追加录制与原数据混在一起。

## 7. 回放与策略接口

回放可使用同一份含相机的 session 配置：

```bash
onero-a1-replay-episode \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_wrist --root ./datasets/local/a1_wrist --episode 0
```

回放只连接从臂，读取数据集动作，不打开 USB 相机，也不需要为回放解码视频。
默认先平滑对齐到录制首帧，再开始回放；首次目标距离与动作 schema 校验仍然生效。
对齐参数见[回放说明](usage_sdk_follow.md#5-回放与-lerobot-集成)。可以加 `--simulate` 验证模拟从臂回放。

`OneroSdk.get_observation()` 可为推理提供配置的 RGB 图像，视觉策略必须与相机名称、尺寸及关节特征匹配。
此前的数据集验证不覆盖本次原始图像落盘和录制后生成流程。
本轮未执行测试或真机验证，由部署方测量实际启动耗时、持续录制速率和故障行为；
完整视觉策略训练与真机部署尚未端到端验证。
