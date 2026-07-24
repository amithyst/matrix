# TwinBot scene6 任务在 Matrix UE 中回放与录制

## 能力边界

这条链路把 TwinBot 已完成的 HouseWorld scene6 任务轨迹送入 Matrix UE，画面顺序为
“行走 → 靠桌 → Dex3 抓取 → 搬运 → 放置”。它使用两个明确分离的阶段：

- `physics_execution=offline_mujoco_persistent_world`：TwinBot 在同一个 MuJoCo
  world/data 实例中执行并记录任务。
- `render_mode=matrix_ue_trace_replay`：Matrix 以 25 FPS 将记录的
  `qpos/qvel/ctrl` 发到本机 UDP `127.0.0.1:9999`，由 HouseWorld UE 窗口显示。

它不是 live SONIC manipulation。当前抓取是“拇指与对向手指接触门禁后启用腕部—方块
weld”，操作阶段还固定了站姿 anchor；视频、状态和 summary 都保留
`contact_gated_wrist_cube_weld_and_anchored_stance` 标记。不能将其描述为纯摩擦抓取。

## 输入门禁

`scripts/replay_matrix_physics_trace.py` fail closed，只接受：

- schema `twinbot.physics_trace.mujoco.v0`、`physics_backend=mujoco`、
  `persistent_world_state=true`、`status=succeeded`；
- scene 6、`/Game/Maps/HouseWorld` 以及上述物理/渲染/辅助边界；
- 每帧严格为 G1 29-DOF + 双 Dex3 + 动态方块的 `57/55/43`；
- 所有数值有限、时间不倒退、JSON 无重复键，trace 与模型均为普通文件。

检查而不启动 UE：

```bash
python3 scripts/replay_matrix_physics_trace.py \
  --trace /path/to/physics-trace.json \
  --inspect
```

inspection 和最终 summary 均记录 trace/model SHA256、维度、源帧数及 25 FPS 时长。

## 只启动 Matrix 回放

```bash
cd /home/ununtu/matrix
bash scripts/run_matrix_scene6_trace_replay.sh \
  --trace /tmp/twinbot-matrix-scene6-task/physics-trace.json \
  --summary outputs/matrix-scene6-task.replay-summary.json
```

启动脚本会持有单实例文件锁，并从 trace 场景 XML 中解析唯一的 G1+Dex3 robot
include。`stage_matrix_trace_model.py` 将引用到的完整 mesh closure 复制到 Matrix 的
MuJoCo 与 UE `custom` 模型根，临时替换两边的 `current.xml`；退出时只在 active XML
仍等于本次 staged SHA256 时恢复原文件，避免覆盖并发修改。缓存 mesh 以内容哈希命名，
可供后续相同回放复用。它还在任何 active XML 改写前落盘 journal，并保存/恢复
`config.json`、MuJoCo config、MC launcher/参数，以及本次生成的 UE/MuJoCo custom
scene；原来不存在的文件会恢复为不存在，原权限位也会恢复。

单实例锁复用正常 Matrix/SONIC launcher 的 `/tmp/matrix-sonic-${UID}.lock`，所以不会
在 live Matrix 旁边改 active model。若 wrapper 被 SIGKILL 或机器掉电，journal 会留在
`outputs/runtime/matrix-scene6-stage.*`；下一次持锁启动先执行 hash-gated 恢复并生成
`recovered-*.json`，恢复失败则拒绝新回放。

`run_sim.sh` 的 external replay 模式具有以下生命周期约束：UE 成功启动并度过 startup
窗口、且本次 UE 日志出现 HouseWorld `LoadMap complete` 后才启动回放 child；它不启动 stock `robot_mujoco`、MC 或 SONIC；精确监督 replay
child 和 UE supervisor，任一异常退出都会结束另一侧并返回非零。不要手工同时设置
`MATRIX_EXTERNAL_REPLAY=1` 与 `MATRIX_SONIC=1`。

## 录制完整视频

```bash
cd /home/ununtu/matrix
bash scripts/record_matrix_scene6_task_video.sh \
  --trace /tmp/twinbot-matrix-scene6-task/physics-trace.json \
  --output outputs/videos/twinbot-matrix-scene6-task.mp4 \
  --display :1 \
  --xauthority /run/user/1000/gdm/Xauthority
```

录制脚本复用 `record_matrix_sonic_video.sh` 的 X11 窗口发现、H.264 编码、视频探针、
质量门禁和 preview 逻辑，但 readiness 使用本回放器的新鲜 status：
`--ready status` / `active_lowcmd=true`。视频固定 25 FPS；录制时长由
`pre-roll + trace_frame_count / 25 + tail` 计算。replay final hold 必须长于整个录制
窗口，因此录制器不会因 launcher 提前退出而截断末尾放置画面。

对输出 `task.mp4`，主要产物为：

- `task.mp4`、`task.preview.jpg`、`task.json`（现有录制器的质量与来源元数据）；
- `task.replay-status.json`（录制期间持续更新，ready 时
  `active_lowcmd=true`）；
- `task.replay-summary.json`（包数、帧数、维度、trace/model SHA256 与边界标签）；
- `task.restore.json`（两份 `current.xml` 及 `run_sim.sh` 运行时改写文件的事务恢复回执）；
- `task.verified.json`（交叉验证视频质量、完整 trace 发送、模型恢复及三方 SHA256
  后才生成的最终回执）；
- `task.launch.log`、`task.ffmpeg.log` 以及 Matrix 的
  `outputs/logs/matrix_trace_replay.log`。

最终验收至少检查 MP4 分辨率、25 FPS、时长/帧数、质量 `passed=true`，并将
trace、模型、视频三个 SHA256 一起保存。postflight 还要求 Matrix Git checkout clean、
UDP 9999 已释放，并且 UE supervisor、replayer、stock MuJoCo/MC 都无残留。运行时
staging state、恢复或 cleanup 任一失败时不得把视频标成完成品。
