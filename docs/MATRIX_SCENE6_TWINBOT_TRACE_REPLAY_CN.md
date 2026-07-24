# TwinBot scene6 任务在 Matrix UE 中回放与录制

## 能力边界

这条链路把 TwinBot 已完成的 HouseWorld scene6 任务轨迹送入 Matrix UE，画面顺序为
“行走 → 靠桌 → Dex3 抓取 → 搬运 → 放置”。它使用两个明确分离的阶段：

- `physics_execution=offline_mujoco_persistent_world`：TwinBot 在同一个 MuJoCo
  world/data 实例中执行并记录任务。
- `render_mode=matrix_ue_trace_replay`：Matrix 以 25 FPS 将记录的
  `qpos/qvel` 发到本机 UDP `127.0.0.1:9999`，由 HouseWorld UE 窗口显示。

精确 packaged UE 的 legacy 第三段虽然按 `nu` 标长，实际却写到 `mjData.act`，不是
`mjData.ctrl`；对本模型 `na=0`，直接转发 43 维 `ctrl` 会覆盖相邻工作区。external replay
因此保留 1260-byte ABI，但把第三段固定为 43 个零，并在 status/summary 中记录
`wire_third_vector_semantics=legacy_act_slot_zeroed`。postflight 缺少该标记就拒绝产物。
长期修复仍应同时重建 stock sender 与精确 UE receiver，使两端真正读写 `mjData.ctrl`。

它不是 live SONIC manipulation。当前抓取是“拇指与对向手指接触门禁后启用腕部—方块
weld”，操作阶段还固定了站姿 anchor；视频、状态和 summary 都保留
`contact_gated_wrist_cube_weld_and_anchored_stance` 标记。不能将其描述为纯摩擦抓取。

## 输入门禁

### 派生 Matrix UE 可见的任务方块模型

Matrix 0.1.2 packaged custom bridge 支持 primitive box，`geom group` 也不是显示过滤条件；
因此不能靠把 `group=2` 改成 `group=1` 修复。真实 shim-only 诊断保留原 v2 模型并成功加载
审计 material bridge，但最终桌面和全部任务阶段仍看不到红色方块，视觉门禁继续拒绝
`carry_lower` / `release_settle`。这份 current-run 失败证据触发 mesh fallback，而不是假设
UE 不支持 primitive。

不能改 TwinBot 原模型或把 mesh 写进 mieli 的工作区。先生成独立、自包含的 render-only
模型目录：

```bash
python3 scripts/prepare_matrix_scene6_render_model.py \
  --source-scene /artifact/model/scene6_house_task.xml \
  --source-robot-model /artifact/model/g1_29dof_dex3.scene6.xml \
  --output-dir /artifact/matrix-render-v3/model \
  --expected-source-scene-sha256 <sha256> \
  --expected-source-robot-model-sha256 <sha256> \
  --expected-source-mesh-closure-sha256 <sha256>
```

该工具保持原 visual box 的尺寸/质量贡献、collision box、freejoint、pose、inertial、joint
和 actuator 不变；仅把原 visual box 的 render alpha 置零，并在同一个 `pick_cube` body 上
增加一个 6 cm、`mass=0`、无碰撞的红色 mesh visual。这样只保留一个可见表面，不会与
primitive 共面闪烁。
生成的 binary STL 固定为 12 个三角面、684 bytes；原机器人 50 个 mesh 也以普通文件复制
进本地 `meshes/`，`compiler.meshdir` 只改为该相对目录。输出目录原子发布且拒绝覆盖，
`render-model-receipt.json` 记录源/派生 XML、51 文件 mesh closure、生成 STL 和不变量的
SHA256；生产调用还必须钉死原 50 文件 closure SHA256。该派生仅改变 Matrix UE 表现，
不修改 source trace 的任何 frame。

红色材质还依赖仓库内经过审计的 Matrix 0.1.2 material bridge：

```bash
bash scripts/build_matrix_ue_material_fix.sh
```

Scene6 replay wrapper 会 fail closed 地要求该普通文件存在，从注册的 G1 skin 解析 palette，
追加 cube RGB `0.95,0.18,0.05`，并使用 scope alpha `0.99609375`。缺 bridge、palette 或
scope 不匹配时不启动 UE；wrapper 还钉死 bridge SHA256，并要求 current-run UE 日志同时
出现匹配 skin/颜色数量和审计安装 marker。相同 binding 会进入 replay status/summary，
postflight v3 再交叉验证，避免产出“有方块但材质未绑定”的候选视频。

### 将 TwinBot v2 原始 trace 投影为 Matrix replay 输入

TwinBot 的 Scene6 v2 原始 trace 是物理权威，不能就地补字段。若原始文件尚未包含 Matrix
渲染声明，先用确定性 projection 生成新文件：

```bash
python3 scripts/prepare_matrix_scene6_replay_trace.py \
  --source-trace /artifact/physics-trace.source.json \
  --source-summary /artifact/summary.source.json \
  --validation /artifact/review/scene6_raise_arm_validation.json \
  --scene-model /artifact/matrix-render-v3/model/scene6_house_task.xml \
  --render-robot-model /artifact/matrix-render-v3/model/g1_29dof_dex3.scene6.xml \
  --output-trace /artifact/physics-trace.matrix-replay.json \
  --receipt /artifact/projection-receipt.json \
  --expected-source-trace-sha256 <sha256> \
  --expected-source-summary-sha256 <sha256> \
  --expected-validation-sha256 <sha256> \
  --expected-scene-model-sha256 <sha256> \
  --expected-render-robot-model-sha256 <sha256>
```

projection 会交叉验证 trace/summary/群附件 validation、2005 帧身份、57/55/43、
0.002s/25 FPS、辅助机制，以及
`world_ready → navigation → dock_arm_clearance → dock_with_pregrasp`。它只增加 Matrix
渲染元数据和持久模型绑定，不修改或重采样任何 frame；输入、frame 与输出 SHA256 均进入
receipt。输出或 receipt 已存在时拒绝覆盖，自验失败时不会留下可接受产物。

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

scene6 replay 默认关闭 SpringArm 平移/旋转延迟、保留碰撞，将臂长设为 180 cm，并选择
实际渲染 custom robot 的 `MujocoSim_Custom_C`，使行走与桌边操作保持在跟随构图内。
显式设置 `MATRIX_UE_EXTRA_EXEC_CMDS` 可覆盖这组录制机位命令。

需要避开身体对右手/方块的遮挡时，可显式使用已验签的 Spectator 居中 overlay：

```bash
bash scripts/run_matrix_scene6_trace_replay.sh \
  --trace /path/to/physics-trace.json \
  --camera-mode spectator-overlay \
  --camera-distance-cm 180 \
  --overlay-bundle /absolute/path/matrix-centered-camera-overlay-v3-bundle \
  --camera-ready-file /tmp/matrix-scene6-side-ready.json \
  --camera-ready-timeout 120 \
  --camera-settle 0.5
```

该模式不会改变 trace 或重新执行任务。launcher 只接受
`config/runtime/matrix-centered-camera-overlay-v3.json` 锁定的三个 PAK/IoStore 文件，
启动前原子安装、从本次 UE 日志验证 Found/Mounted，退出后再移除；缺文件、哈希不符、
挂载失败或清理失败都会拒绝回放。它选择 `Spectator_C`，让 pivot 持续跟随
`MujocoSim_Custom_C`，并保留 Spectator 自身的 orbit 旋转，适合录制前通过真实 UE
鼠标输入选右前/侧面角度。这个 overlay 仍不是具有最终 view 回读的完整 camera bridge；
未实际验证的 yaw 不能只靠 X11 指针位移声称准确。默认 `robot` 模式保持不变。

生产视频使用双终端确认，不能靠两秒 pre-roll 抢时间。终端 A 启动上述命令；等它打印
`Waiting for Matrix scene6 camera confirmation` 后，在真实 UE 窗口完成 orbit 构图。终端 B
再写入本次确认：

```bash
cd /home/ununtu/matrix
/usr/bin/python3 -I scripts/matrix_scene6_camera_receipt.py confirm \
  --output /tmp/matrix-scene6-side-ready.json \
  --mode spectator-overlay \
  --framing-label right-side
```

wrapper 持有 Matrix 单实例锁后会先删除同路径旧确认；当前 UE 完成 map、UDP 与 overlay
挂载验证后，`run_sim.sh` 才等待新文件。确认通过并稳定 `0.5s` 后，它在 Active overlay
仍存在时写 current-run camera receipt，再启动第一个 replay packet。省略
`--camera-ready-file` 只表示使用未人工确认的默认 orbit；这种产物不能声称是已确认侧视角。

单实例锁复用正常 Matrix/SONIC launcher 的 `/tmp/matrix-sonic-${UID}.lock`，所以不会
在 live Matrix 旁边改 active model。若 wrapper 被 SIGKILL 或机器掉电，journal 会留在
`outputs/runtime/matrix-scene6-stage.*`；下一次持锁启动先执行 hash-gated 恢复并生成
`recovered-*.json`，恢复失败则拒绝新回放。

`run_sim.sh` 的 external replay 模式具有以下生命周期约束：UE 成功启动并度过 startup
窗口、且本次 UE 日志同时确认 HouseWorld `LoadMap complete`、MuJoCo 模型加载成功和
UE 持有 UDP 9999 后才启动回放 child；external replay 会显式设置
`use_custom_urdf=true`，让 scene6 custom robot 使用已纳入事务恢复的
`model/custom/scene_terrain_custom.xml`。它不启动 stock
`robot_mujoco`、MC 或 SONIC；精确监督 replay
child 和 UE supervisor，任一异常退出都会结束另一侧并返回非零。不要手工同时设置
`MATRIX_EXTERNAL_REPLAY=1` 与 `MATRIX_SONIC=1`。

custom robot 的 `compiler meshdir` 会影响合并后所有 MuJoCo asset 的解析。scene composer
因此把 HouseWorld 原生 mesh/height-field 先复制到 custom 目录，再把生成场景中的文件
引用绑定到这些已核验的绝对副本，避免环境高度图被错误解析到机器人 mesh cache。

## 录制完整视频

```bash
cd /home/ununtu/matrix
bash scripts/record_matrix_scene6_task_video.sh \
  --trace /tmp/twinbot-matrix-scene6-task/physics-trace.json \
  --output outputs/videos/twinbot-matrix-scene6-task.mp4 \
  --display :1 \
  --xauthority /run/user/1000/gdm/Xauthority
```

录制 Spectator overlay 时再加：

```bash
  --camera-mode spectator-overlay \
  --camera-distance-cm 180 \
  --overlay-bundle /absolute/path/matrix-centered-camera-overlay-v3-bundle \
  --camera-ready-file outputs/videos/twinbot-matrix-scene6-task.camera-ready.json \
  --camera-ready-timeout 120 \
  --camera-settle 0.5
```

录制器会把 launcher 输出写入 `twinbot-matrix-scene6-task.launch.log`。看到其中的
`Waiting for Matrix scene6 camera confirmation` 后，在第二终端执行：

```bash
cd /home/ununtu/matrix
/usr/bin/python3 -I scripts/matrix_scene6_camera_receipt.py confirm \
  --output outputs/videos/twinbot-matrix-scene6-task.camera-ready.json \
  --mode spectator-overlay \
  --framing-label right-side
```

录制脚本复用 `record_matrix_sonic_video.sh` 的 X11 窗口发现、H.264 编码、视频探针、
质量门禁和 preview 逻辑，但 readiness 使用本回放器的新鲜 status：
`--ready status` / `active_lowcmd=true`。视频固定 25 FPS；录制时长由
`pre-roll + trace_frame_count / 25 + tail` 计算。replay final hold 必须长于整个录制
窗口，因此录制器不会因 launcher 提前退出而截断末尾放置画面。视频捕获结束后，
scene6 wrapper 会继续等待 launcher 自然完成 final hold、写完 summary/final status、
关闭 UE 并恢复运行时事务；等待超时或 launcher 非零退出都会 fail closed，不会生成
合格回执。

`task.json` 使用扩展 schema `matrix.scene6_video_metadata.v2`，其中
`matrix_scene6_camera` 是本次 `task.camera.json` 的原样副本：记录实际 UE ExecCmds、
camera mode/viewclass、规范化臂长、人工确认标签、固定 v3 contract/三文件哈希以及当前
UE 日志的精确 Found/Mounted 字节区间。replayer 在首包前把 camera receipt SHA256 写入
status/summary v2，postflight 再按该哈希、bundle、contract 和日志区间交叉验证。

对输出 `task.mp4`，主要产物为：

- `task.mp4`、`task.preview.jpg`、`task.json`（现有录制器的质量与来源元数据）；
- `task.camera.json`（`matrix.scene6_camera_receipt.v1` current-run 相机与挂载证据）；
- `task.camera-ready.json`（启用双终端 gate 时的人工作图确认输入，其内容也进入 camera
  receipt）；
- `task.visual-motion.json`（`matrix.scene6_visual_motion_receipt.v1`；按 trace transitions
  对 navigation、raise/dock、grasp、carry/lower、release/settle 五段抽帧；其中
  `dock_arm_clearance` 必须先于 `dock_with_pregrasp`，明确覆盖靠桌前抬臂。门禁分别检查全帧与
  中央机器人/桌面 ROI 的可见像素运动，并以 pre-roll 静止画面校准编码噪声）；
- `task.replay-status.json`（录制期间持续更新，ready 时
  `active_lowcmd=true`）；
- `task.replay-summary.json`（包数、帧数、维度、trace/model SHA256 与边界标签）；
- `task.restore.json`（两份 `current.xml` 及 `run_sim.sh` 运行时改写文件的事务恢复回执）；
- `task.verified.json`（`matrix.scene6_twinbot_video_postflight.v3`；交叉验证视频质量、
  完整 trace 发送、模型恢复、相机回执、视觉运动回执及相关 SHA256 后才生成）；
- `task.launch.log`、`task.ffmpeg.log` 以及 Matrix 的
  `outputs/logs/matrix_trace_replay.log`。

最终验收至少检查 MP4 分辨率、25 FPS、时长/帧数、质量 `passed=true`，且五个任务阶段
均存在超过编码噪声的可见运动，并将 trace、模型、视频三个 SHA256 一起保存。缺阶段、
静态画面、视觉回执缺失或 `passed=false` 都不得通过。postflight 还要求 Matrix Git checkout clean、
UDP 9999 已释放，并且 UE supervisor、replayer、stock MuJoCo/MC 都无残留。运行时
staging state、恢复或 cleanup 任一失败时不得把视频标成完成品。合格 replay 必须是
`completion=scheduled_replay_complete`，launcher 返回 0 且不是录制器强停；final
status 必须 `completed=true`、`passed=true`、`active_lowcmd=false`。
