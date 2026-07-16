# Matrix SONIC 主链路视频录制

## 目标

`scripts/record_matrix_sonic_video.sh` 录制 Matrix 原生 UE 窗口，同时保持
`scripts/run_matrix_sonic.sh -> scripts/run_sim.sh` 这条 SONIC 主启动链路不变。
录制器不创建第二套仿真逻辑，也不会降低或缩放 UE 窗口分辨率。

长时间仿真和录制必须在 tmux 中运行：

```bash
ssh trna-zt
tmux new-window -t matrix-sonic-eval -n matrix-record
tmux attach -t matrix-sonic-eval
```

## 依赖

TRNA 的 Matrix 审计环境使用项目锁定的 `imageio-ffmpeg`：

```bash
cd /home/trna/matrix-eval
.venv-audit/bin/python -m pip install \
  -r research/sonic_integration/requirements-trna.txt
```

解析顺序是 `--ffmpeg`、`MATRIX_FFMPEG`、系统 `ffmpeg`、当前 Python 的
`imageio-ffmpeg`。不要把其他仓库虚拟环境里的 ffmpeg 路径写死到脚本中。
`xwininfo` 用来发现精确 UE window id；不依赖 `xdotool` 或桌面区域裁剪。

## 启动并录制

下面命令沿用正常 Matrix+SONIC 启动。录制器等 `active_lowcmd=true` 且有效控制
达到 8 秒后开始录制，因此默认会跳过 SONIC 初始化和临时弹性带阶段：

```bash
cd /home/trna/matrix-eval
MATRIX_UE_MAX_FPS=30 \
bash scripts/record_matrix_sonic_video.sh \
  --output outputs/videos/matrix_sonic_apartment.mp4 \
  --duration 15 \
  --fps 30 \
  --notes "ApartmentWorld planner walk" \
  -- \
  bash scripts/run_matrix_sonic.sh \
    --scene 21 \
    --urdf /home/trna/matrix-sonic-assets/g1_29dof/g1_29dof.urdf \
    --control-source planner \
    --walk-after 10 \
    --vx 0.25 \
    --max-seconds 70
```

`--max-seconds` 必须覆盖模型加载、8 秒 ready 等待和完整录制时长。录完后，录制器
会终止自己启动的 Matrix 栈，并让原启动脚本执行正常清理。传入
`--keep-running` 才会保留该仿真。

## 附着已有仿真

已有 Matrix+SONIC 正常运行时，只录窗口、不接管其生命周期：

```bash
bash scripts/record_matrix_sonic_video.sh \
  --attach \
  --output outputs/videos/matrix_sonic_attached.mp4 \
  --duration 15 \
  --fps 30
```

如果同时存在多个 Matrix UE 窗口，脚本会拒绝猜测。使用 `--window-id 0x...`
明确选择窗口。

## 编码器

`--encoder auto` 会实际编码一个 64x64 探针，优先选择可工作的
`h264_nvenc`，否则回退 `libx264`。当前 TRNA 的 RTX 5080 有 NVENC，但
`imageio-ffmpeg 0.6.0` 静态包没有编入 NVENC，因此默认走 CPU `libx264`。
安装并传入经过验证的 NVENC ffmpeg 后，可以使用：

```bash
MATRIX_FFMPEG=/path/to/nvenc-enabled-ffmpeg \
bash scripts/record_matrix_sonic_video.sh \
  --attach --encoder h264_nvenc --fps 60 --duration 15 \
  --output outputs/videos/matrix_sonic_60fps.mp4
```

提高到 60 FPS 不改变 1920x1080 分辨率，但必须重新测量 physics Hz、RTF、UE FPS
和重复帧，不能只看 MP4 标称帧率。

## 产物与门禁

对 `example.mp4`，脚本同时生成：

- `example.json`：Git 提交、完整启动命令、窗口 ID/尺寸、ffmpeg/编码器、视频
  SHA256、质量结果以及录制前后 SONIC 状态。
- `example.launch.log`：由录制器启动仿真时的主链路日志。
- `example.ffmpeg.log`：窗口采集和编码日志。
- `example.preview.jpg`：从原始 MP4 等间隔抽取的 5 帧预览条，仅用于快速检查。

MP4 必须满足：输出分辨率等于 UE 窗口、时长和帧数不少于请求值的 90%、首帧
不是全黑/全白、抽样帧不是完全静止。失败视频保留为 `example.rejected.mp4`，脚本
返回非零；`--allow-static` 和 `--allow-short` 只能用于明确的诊断场景。

TRNA 上的主链路验收结果已固化在
`research/sonic_integration/results/trna_matrix_sonic_video_20260715.json`。该次
ApartmentWorld 录制为 1920x1080、30 FPS、10 秒、300 帧；50 个抽样帧均不同，
同期 SONIC 物理约 200 Hz、RTF 约 1.0，未跌倒或触发稳定性重置。

## 数据录制边界

窗口 MP4 是操作员视觉证据，不替代训练数据。AI 数据应另外用 ROS2 bag 保存
`/image_raw/compressed`、机器人状态和控制指令，保持原始时间戳。

当前 adjacent Overworld V1 只有六场景连续 MuJoCo 物理，没有六地图 UE 视觉。
在 `/Game/Maps/Overworld` 真正打包前，禁止把任意单地图 UE 视频命名为 Overworld；
此时只能产出明确标注的 MuJoCo 物理诊断视频。
