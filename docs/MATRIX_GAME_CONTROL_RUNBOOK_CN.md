# Matrix 相机相对二游式控制运行手册

本文只说明 `--control-source game` 这条“本地键鼠输入 → 原生 SONIC 运动”的交互
链路。它与 planner 自动行走验收、Matrix 上游旧遥控器按键是三套不同语义。

当前发布策略：功能开发、实验和日常源码更新都在河源进行。TRNA 是第二顺位备份，ZZA
是第三顺位备份；无论普通改动还是大版本，都只有在项目负责人明确要求时才向对应备份机
同步源码或私有运行资产。

## 范围与验收边界

已经实现的行为：

- WASD 按相机水平朝向映射到世界坐标；
- W/A/S/D 四个方向都让机器人自动面向运动方向；
- 斜向输入归一化，并限制速度、加速度、减速度和转向速度；
- 键盘 WASD 使用按住修饰键的原生静走/普通走/跑步：Ctrl、无修饰键、Shift 分别映射
  SONIC mode 1、2、3；
- Q/E 不再参与机器人 yaw；
- 精确 UE PID 失焦、观察到的 V 安全状态切换、鼠标拖相机、输入过期、断连和
  provider 故障都会停机；
- 原生 LowCmd 必须 fresh 且启动弹性带完全释放，运动帧才会放行；
- 私有 socket 除了核验 UID，还会绑定受监督 provider 的精确 PID；
- 启动和每次安全停机后都必须先收到一帧回中输入；
- 原生 SONIC planner 是唯一运动指令发布者，不直接旋转 UE Actor。

Matrix 0.1.2 cooked 运行包目前**没有完成**以下能力：

- 无漂移读取操作员实际看到的跟随相机变换；
- 用右摇杆驱动这个可见相机；
- 证明 CARLA spectator 与画面里的跟随相机严格耦合；
- 回读 UE 真实自由相机/输入模式；当前 V 只做 best-effort 镜像，居中 overlay v3
  明确不会随 V 切换可见画面。

因此 `fixed` 仍是安全默认值，`x11-mirror` 只是待标定候选，并不是真实相机姿态。
不能把当前实现描述成“右摇杆相机已经完成”。

## 默认值和安全不变量

| 项目 | 默认值 / 不变量 |
|---|---|
| 输入采样 | 50 Hz |
| 本地输入协议 | 严格 `matrix-game-input/v2`（`ctrl`、`shift` 为必填字段） |
| SONIC 控制 | 50 Hz；原生物理保持 200 Hz |
| 键盘最高目标 | 2.50 m/s（`RUN` 下边界） |
| 模拟量最高速度 | 默认 0.30 m/s；最高可配置 0.80 m/s，保持在 `SLOW_WALK` |
| 键盘步态档 | Ctrl mode 1 / 0.10；无修饰 mode 2 / 0.80；Shift mode 3 / 2.50 m/s；冲突时 Ctrl 优先 |
| 原生步态区间 | mode 1：0.10-0.80；mode 2：0.80-2.50；mode 3：2.50-7.50 m/s |
| 加速度 / 减速度 | 1.20 / 2.40 m/s² |
| 最大朝向变化率 | 2.50 rad/s |
| 平移朝向门 | 15 度内启动；超过 30 度停止 |
| 左摇杆径向死区 | 0.15 |
| 输入 deadman 超时 | 0.15 s |
| 快照最大年龄 | 0.15 s |
| 松方向键 / 安全停机 | 当帧 mode 0 且指令归零，不走平滑减速 |
| 恢复运动 | 必须先收到一帧有焦点的回中输入 |

河源首轮标定不要提高超时或 0.30 m/s 模拟量上限；保留启动弹性带，并保留 launcher
默认的跌倒即失败、零数值重置等门禁。键盘目标更高是因为它会真正选择原生 `WALK`/
`RUN`。限幅爬升阶段在 0.80 m/s 前发布 mode 1，0.80-2.50 m/s 发布 mode 2，只有达到
2.50 m/s 才发布 mode 3；需要记录实际步态切换距离。
CLI 允许把模拟量上限配置到原生 `SLOW_WALK` 的 0.80 m/s，但河源 profile 和默认值仍为
0.30 m/s。从键盘切到已经偏转的摇杆时，同一帧会夹到实际配置上限并回到 mode 1。

## 河源启动前检查

保留 `/home/kaijie/matrix` 作为 clean main checkout；本功能使用独立实验 worktree 和
仓库内的 `heyuan` profile。启动前松开全部移动键。

feature branch 推送后先做一次 worktree 准备。Git worktree 不会自动带上 main checkout
里被忽略的 UE 安装包、`.venv-audit` 和 `.matrix/local.env`，因此必须 bootstrap：

```bash
git -C /home/kaijie/matrix fetch origin main
git -C /home/kaijie/matrix worktree add --detach \
  /home/kaijie/worktrees/matrix-game-control-exp \
  origin/main
cd /home/kaijie/worktrees/matrix-game-control-exp
MATRIX_SONIC_ROOT=/home/kaijie/worktrees/sonic-matrix-native-final \
  bash scripts/bootstrap_matrix_sonic.sh \
    --profile heyuan \
    --release-cache /home/kaijie/matrix-eval/releases \
    --runtime-root /home/kaijie/matrix-artifacts/matrix-sonic-native-v2-heyuan \
    --write-local-env
/usr/bin/python3 scripts/update_matrix_local_env.py \
  .matrix/local.env MATRIX_SONIC_ROOT \
  /home/kaijie/worktrees/sonic-matrix-native-final
```

```bash
MATRIX_EXPERIMENT_WORKTREE="${MATRIX_EXPERIMENT_WORKTREE:-/home/kaijie/worktrees/matrix-game-control-exp}"
cd "$MATRIX_EXPERIMENT_WORKTREE"
git status --short
git rev-parse HEAD
printf 'DISPLAY=%s\n' "$DISPLAY"
test -S /tmp/.X11-unix/X1001
```

河源 profile 使用当前 NoMachine X11 display `:1001`。必须前台显示运行，不能加
`--offscreen`。输入 provider 要求活动 X11 窗口属于受监督 UE 的精确 PID，同时默认检查标题
`(zsibot|matrix|unreal)`。如果 cooked 窗口标题不同，用 `MATRIX_GAME_FOCUS_TITLE`
精确配置；正式验收绝不能退回只看标题的焦点判断。

当前河源桌面/tmux 的 `PATH` 中存在一个同名的 `~/.local/bin/env` 初始化脚本，它不是
GNU `env`，会忽略后续启动参数。需要清理 Conda 污染时必须显式使用
`/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH ...`，不能只写裸 `env`。

当前锁定的 cooked 包实测不含 `/Game/Maps/ApartmentWorld`，因此 `--scene 21` 会在 UE
日志中报缺包，不能作为可玩验收。该资源重新 cook 之前，河源交互/ESC 面板验收使用包内
已确认存在的 `--scene 2`（`Town10World`）；物理侧仍由对应的 SONIC Town10 场景驱动。

从新的 UE 进程及其默认居中相机模式开始。cooked runtime 无法报告 provider 启动前
发生的 V 按键沿。

统一从主 launcher 启动。直接调用 `run_sim.sh` 或 `run_matrix_sonic.py` 只能调试，不能
形成 qualified 验收证据。

### 居中 overlay 启动前检查与生命周期

河源 profile 默认把 `MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE` 设为
`/home/kaijie/matrix-artifacts/matrix-centered-camera-custom-v1`。每次启动时，主 launcher
在持有宿主机锁的情况下，只清理已通过验签的旧 active 目录，并按
`config/runtime/matrix-centered-camera-overlay-v3.json` 验签 bundle。bundle 必须是绝对
路径下的真实目录，而且只能包含锁定的三个 `pakchunk99-MatrixCentered-Linux_P` 文件；
symlink、额外文件、路径绕转、size 不同或 SHA-256 不同都会 fail closed。

只有“SONIC game + 居中 + `custom`”会安装：`run_sim.sh` 在 UE 前原子安装私有副本，
选择 `Spectator_C`，并只在本次启动新增日志段等待 `LogPakFile: Found Pak file` 与
`LogPakFile: Mounted IoStore container`；新增 stem 行包含 `Failed` 会立刻失败，历史
日志不能通过门禁。active 副本在整个 UE 进程期间持久在线，只在受监督 UE 精确停止后
移除；若 `kill -KILL` 令
cleanup 无法执行，下一次取得宿主机锁后由 `purge-stale` 处理。launcher 会把资产的
110 cm SpringArm 覆盖为适合全身构图的 150 cm 默认值。
`MATRIX_GAME_CAMERA_DISTANCE_CM` 只接受 80-500 cm 内的普通十进制；只有在明确测试宽景
时才建议用 180 cm。planner/PICO/external、
非 SONIC、非 custom 或关闭居中的启动都不会安装。未配置 bundle 时，原有机器人
viewclass fallback 保持不变。

V 不会在视觉上把 overlay v3 切到自由相机；它只切换 input provider 的 best-effort
镜像安全状态。因此 V 安全测试仍需再按一次 V 并完成 neutral re-arm，虽然画面始终居中。
需要显式绕开河源默认值做恢复启动时，必须让 profile 继承空值：

```bash
MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE= \
  bash scripts/run_matrix_sonic.sh --profile heyuan --scene 2 \
    --control-source game
```

## 阶段一：固定坐标系功能测试

先用固定 SONIC 相机 yaw 验证按键和安全行为；此时不假装控制坐标会跟随可见相机：

```bash
/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash scripts/run_matrix_sonic.sh \
  --profile heyuan \
  --scene 2 \
  --control-source game \
  --game-input-source keyboard \
  --game-camera-yaw-source fixed \
  --game-initial-yaw 0 \
  --game-max-speed 0.30 \
  --game-input-timeout 0.15
```

在另一个终端观察状态：

```bash
watch -n 0.5 'jq "{control_source, physics_step_hz, rtf, fall_detected, instability_resets, root_xyz, root_displacement_xy_m, game_input}" outputs/matrix_sonic_status.json'
```

如果启动时已经按着移动键，第一帧应显示 `game_input.stop_reason` 为
`awaiting_neutral`。松开 WASD 后模式应变为 `idle`，再次按下才允许运动。

固定 SONIC yaw 为 0 时，验证归一化物理方向：

| 输入 | 预期 root 方向 | 预期朝向 |
|---|---|---|
| W | +X | +X |
| S | 转身后 -X | -X |
| A | +Y | +Y |
| D | -Y | -Y |

另外确认：

1. W+A、W+D 的速度不高于单独 W；
2. Ctrl+W、W、Shift+W 分别稳定在原生 mode/速度 1/0.10、2/0.80、3/2.50 m/s；
   Ctrl+Shift 冲突时使用 1/0.10；切档中每一帧 mode/速度仍落在原生合法区间；
3. A、D、S 都会让机器人朝运动方向转身；180 度反向时先转身，再逐渐建立平移；
4. 单独按 Q 或 E 不改变 root，也不改变 game-control heading；
5. 松开全部方向键、失焦或超时都当帧发布 mode 0；单独按 Ctrl/Shift 仍为 mode 0。

`fixed` 下画面相机可能和上表不一致，这是预期限制，所以这一阶段不算“相机相对控制
验收通过”。

## 阶段二：X11 镜像标定

`x11-mirror` 订阅 XInput2 `XI_RawMotion`；这是 SDL relative mouse mode 常用的输入层，
launcher 也请求 SDL raw 模式，但 packaged UE 是否等量消费尚未通过 live 黑盒证明。
它不再用 50 Hz `XQueryPointer` 绝对坐标差，因此 MouseLock
把指针送回中心时，不会把一次拖动的出程和回程在同一采样周期内相消。它仍不查询 UE
最终渲染相机，也不主动移动相机。最终送给 SONIC 的 yaw 为：

```text
wrap(sign × (initial_yaw + 累计 XI2 raw X × SDL scale × sensitivity) + offset)
```

先把画面相机放到可重复的基准姿态，再用保守默认值启动：

```bash
/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash scripts/run_matrix_sonic.sh \
  --profile heyuan \
  --scene 2 \
  --control-source game \
  --game-input-source keyboard \
  --game-camera-yaw-source x11-mirror \
  --game-look-button left \
  --game-initial-yaw 0 \
  --game-mouse-sensitivity 0.12 \
  --game-camera-yaw-sign -1 \
  --game-camera-yaw-offset 0 \
  --game-max-speed 0.30 \
  --game-input-timeout 0.15
```

按以下顺序标定：

1. **offset：**启动画面对准一个已知世界方向，调整 offset，直到 W 的运动方向与画面
   前向一致；
2. **sign：**把相机沿一个已知水平方向拖动；松开全部移动键和鼠标后再按 W。如果
   控制方向变化与画面相反，就翻转 `--game-camera-yaw-sign`；
3. **sensitivity：**让画面相机完成可辨认的 90 度转动。SONIC 坐标转得不够就增大
   每 XI2 raw unit 角度，转过头就减小。

每次拖动都是一次安全停机。正确操作顺序是：松开 WASD → 拖动相机 → 松开鼠标 →
给一帧回中输入 → 再按 W。若拖动期间一直按着 W，松开鼠标后也必须保持
`awaiting_neutral`，不能突然继续走。

四方向门禁用标定后的 SONIC yaw 和 root 位移核对：

| 标定后相机 yaw | W 必须朝向 |
|---|---|
| 0° | +X |
| +90° | +Y |
| ±180° | -X |
| -90° | -Y |

顺时针、逆时针连续绕转几圈后重复测试；把指针移到各个屏幕边缘后重复；按两次 V
走完镜像安全状态后再重复（v3 可见画面仍保持居中）。只要出现累计偏差、raw 输入与
UE 实际处理分叉、或 UE 自动回正造成不一致，
`x11-mirror` 就不能作为验收相机源，此时继续保留 `fixed` 默认值。

远程桌面拖动过快时，不要把系统 `xinput` 加速度当成 UE 修复：launcher 请求 SDL raw
relative mode，`x11-mirror` 读取 XI2 raw；packaged UE 是否等量消费仍须 live 验证。
系统指针曲线可能只改变 X11 绝对坐标，使可见相机和 `x11-mirror` 进一步分叉。
启动组合为 SONIC `game` + Remote 时，launcher 会自行保存当前指针曲线，只在本次运行
期间执行 `xset m 1/1 0`，并在 cleanup 中恢复；桌面设置应保留用户平时的值。请按 ESC
进入安全设置页，点击 Remote、用大号 -/+ 遍历 19 个精确档位：0.01x–0.10x 每次
0.01，随后 0.20x–1.00x 每次 0.10；0.10x 与 0.20x 直接相邻，键盘 -/+ 与面板点击
使用同一档位表。再点击“返回游戏并应用”（或按 Enter）；页面会等待安全中立帧后
自动重载完整运行链，F9 仅作为键盘兜底。
重启后确认面板的 `CURRENT APPLIED (SDL)` 与目标一致，并重新执行四方向与
多轮往返测试。F10/F12 仍属于外部 MouseLock，不是 Matrix 设置页的绑定。

Local 固定为 1.0x。Remote 0.4x 仍是其中一个档位，也是 SDL/UE 原生倍率；同一个所选
Remote 倍率会同时进入可见 SDL 路径与 `x11-mirror` raw 增益。示例基值
0.12 度/raw unit 乘后，状态会显示 `x11-mirror = 0.048 度/raw unit`。XI2 raw 是 SDL
相对鼠标模式常用的输入层，launcher 也请求 SDL raw 模式，但 packaged UE 是否等量消费
仍未经过 live 黑盒证明；它不是最终渲染相机 yaw 的回读，状态必须保持
`visible_follow_camera_verified=false`。配置文件缺失、损坏或被手工改成非预设档位时，
系统会安全回退到 Local 1.0x。

## 阶段三：安全与恢复矩阵

每一行都从低速行走状态开始验证：

| 测试 | 必须结果 |
|---|---|
| 按住 W 时启动 | `awaiting_neutral`；松开再按之前不得运动 |
| Alt-Tab / 聚焦其他窗口 | 立即归零并显示 `focus_lost`；恢复焦点后仍需回中 |
| 按住配置的鼠标拖动键 | 拖动期间立即归零；结束后仍需回中 |
| provider 启动后按 V | 观察到按键沿时立即归零并显示镜像 `free_camera`；overlay v3 不切换可见画面 |
| 停止发送输入包 | 0.15 s 为超时阈值；下一次 50 Hz tick 归零，标称最坏约 0.17 s 再加调度抖动 |
| 关闭输入 socket | 下一次控制轮询即归零；重连后需回中 |
| 结束受监督的 provider | 归零并清理整条启动链路拥有的子进程 |
| 重连时仍按着 W | 持续 `awaiting_neutral`，直到松开 W |
| LowCmd 尚未 fresh 或启动弹性带未归零时启动 | `sonic_not_ready`、零速度，但原生 deploy 仍收到 `start=True` |
| fresh LowCmd 掉为 stale，恢复时仍按着 W | 立即归零；恢复后仍为 `awaiting_neutral`，必须先松开 W |
| 按住 Q 或 E | 不产生 SONIC yaw 或平移指令 |

主 launcher 会创建权限 0700 的私有运行目录和唯一的本地 `SOCK_SEQPACKET`，socket
权限为 0600，并同时验证对端 UID 与受监督 provider 经过 exec 保持的精确 PID。不要
把它改成网络桥，也不要恢复旧 AndroidTwin UDP/DDS 链路。

0.15 s 只覆盖仍在运行的 runtime 对输入/provider 故障的检测；若整个 Python runtime
冻结，将退回 SONIC 自身更长的 watchdog，不能宣称也是 0.15 s。

## 阶段四：河源有界验收

只有 fixed 安全测试和 `x11-mirror` 四方向都通过后，才从干净 checkout 启动有界验收。
保留 runtime lock 给出的 displacement、lowcmd、跌倒、重置、physics 和 RTF 下限，
不要为了通过而放宽。
有界 game qualification 会拒绝 `fixed`、拒绝关闭受监督 provider，并要求至少有一帧
非零运动指令确实通过原生 planner 发布边界。同时固定使用仓库内 provider 脚本和
runtime 同一个已验证 Python；解释器 override 只能调试，有界验收会拒绝。

```bash
MEASURED_MOUSE_DEG_PER_RAW_UNIT=0.12  # 替换为河源实测值
MEASURED_CAMERA_YAW_SIGN=-1        # 替换为方向探针得到的 -1 或 1
MEASURED_CAMERA_YAW_OFFSET_DEG=0   # 替换为标定后的 offset

/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash scripts/run_matrix_sonic.sh \
  --profile heyuan \
  --scene 2 \
  --control-source game \
  --game-input-source keyboard \
  --game-camera-yaw-source x11-mirror \
  --game-look-button left \
  --game-initial-yaw 0 \
  --game-mouse-sensitivity "$MEASURED_MOUSE_DEG_PER_RAW_UNIT" \
  --game-camera-yaw-sign "$MEASURED_CAMERA_YAW_SIGN" \
  --game-camera-yaw-offset "$MEASURED_CAMERA_YAW_OFFSET_DEG" \
  --game-max-speed 0.30 \
  --game-input-timeout 0.15 \
  --max-seconds 120 \
  --min-active-seconds 60
```

在有界窗口里覆盖 W/A/S/D、键盘三档速度、斜向、180 度反转、Q/E、一次相机拖动并
完成回中复位，以及一次失焦/恢复。V 必须做完整循环：进入镜像安全状态、确认硬停、再次
按 V 清除、再完成 neutral re-arm；只切换一次会正确地停在 safe stop，无法通过边界。
最后保证有效位移达到 lock 要求。

最终 `outputs/matrix_sonic_status.json` 至少满足：

- `control_source: "game"`，实际操控期间 game input 已连接且持续应用；
- `passed: true`、`fall_detected: false`、`instability_resets: 0`；
- 物理频率不低于 195 Hz，RTF 不低于 0.95；
- 没有协议/重放错误，`game_input_at_boundary` 中没有无法解释的 safe stop；
- `game_input_at_boundary.moving_command_frames >= 1`，没有对端 PID 不匹配，且 provider 在验收
  边界仍保持连接；
- 最终位移达到 lock 下限。四方向属于过程证据，必须由河源视频、分方向 checkpoint 或
  周期状态/日志证明；单个最终净位移和最终 yaw 无法证明完整四向过程。

对 `x11-mirror` 而言，`passed: true` 只证明经过身份认证的输入链路和 SONIC 运动链路，
不能单独证明画面中的跟随相机使用了同一个镜像 yaw。状态中的
`game_control_configuration` 会完整记录来源、sign、offset、鼠标灵敏度和 CARLA 手柄
参数，并明确给出 `visible_follow_camera_verified: false` 与
`external_visual_evidence_required: true`。只有同时保留河源四方向截图或视频后，才能
描述为完整的相机相对操控验收通过。

游戏控制的 measured-heading 零点固定为启动时的 MuJoCo 初始快照，状态中明确记录
`heading_anchor_source: "initial_snapshot"`。第一次观察到 fresh LowCmd 时只记录
`root_yaw_first_fresh_lowcmd_rad`、wrap 后的 `root_yaw_startup_delta_rad` 以及对应的
step、仿真时间和 wall elapsed；它不会在运行中重锚或突变控制坐标。若这些 first-fresh
字段为 `null`，说明本次运行从未观察到 fresh LowCmd 上升沿。

检查最终状态和日志：

```bash
jq '{run_id, matrix_commit, sonic_commit, runtime_lock_sha256, control_source, passed, acceptance_failures, physics_step_hz, rtf, fall_detected, instability_resets, root_displacement_xy_m, heading_anchor_source, root_yaw_initial_rad, root_yaw_first_fresh_lowcmd_rad, root_yaw_startup_delta_rad, root_yaw_relative_rad, game_input_at_boundary, game_input, game_control_configuration}' outputs/matrix_sonic_status.json
jq . outputs/matrix_game_control_input.json
tail -n 200 outputs/logs/matrix_sonic_runtime.log
```

最终 `game_input` 对象正常情况下也会显示 runtime 主动执行的停机急停；qualified 真正
使用的是急停前保存的 `game_input_at_boundary`。连接、freshness 和 safe-stop 边界应以
后者为准。

证据需记录 Matrix commit、SONIC commit、完整标定参数、场景、X11 display、手柄型号
（若使用）、状态 JSON，以及用于确认画面的截图/视频。无界手动运行或绕过主 launcher
的运行只能用于调试，不能作为 qualified 证据。

## 手柄与后续相机 bridge 门禁

当相机来源为 `fixed` 或 `x11-mirror` 时，`auto` 有意降级为仅键盘，显式
`--game-input-source gamepad` 会失败。这是安全设计，不是环境配置错误。不能在可见相机
完全没动时，仅把右摇杆数值累加到内部 yaw 来绕过门禁。

完整右摇杆实现至少需要 UE runtime bridge 提供等价能力：

- `GetViewTransform`：读取最终渲染的实际跟随相机；
- `SetOrbitYaw`：驱动画面中的相机水平旋转；
- `SubscribeInputMode`：订阅焦点、自由相机和拖动状态。

输入 provider 已实现可选的 CARLA spectator `set_transform → get_transform` 右摇杆路径，
但它只有在 runtime 确实开放 CARLA RPC、并证明 spectator 就是最终渲染相机时才满足前两项。
当前 0.1.2 二进制/日志未发现 CARLA server，因此该路径保持 fail-closed 候选，不能作为
当前发布包的完成证据。

增加 bridge 后，键鼠和手柄都必须重新通过四方向、累计漂移、失焦、deadman、回中复位、
跌倒/重置、物理频率和 RTF 门禁。全部通过后，才能把手柄设为默认，或宣称“右摇杆
相机完整实现”。

## 常见问题

- **`awaiting_neutral` 一直不消失：**聚焦 Matrix，松开 WASD、让左摇杆回中、松开
  鼠标拖动键，并确认 V 镜像安全状态已关闭；
- **Matrix 看似激活却显示 `focus_lost`：**先检查 provider 状态里的
  `focus.expected_ue_pid`、`focus.actual_pid` 和真实 X11 标题；PID 正确后再调整
  `MATRIX_GAME_FOCUS_TITLE`；
- **W 始终差一个固定角度：**修正 camera-yaw offset；
- **拖动后方向变化相反：**翻转 camera-yaw sign；
- **多次拖动后误差越来越大：**重新测 sensitivity；若由光标 warp/回正造成不连续，
  应判定 `x11-mirror` 不通过，而不是掩盖漂移；
- **显式 gamepad 被拒绝：**在 `fixed` 和 `x11-mirror` 下必然如此；选择 `carla` 时仅在
  spectator RPC 写入/回读成功后放行这个候选，仍不代表可见跟随相机已经验证；
- **input provider 退出：**检查 `outputs/matrix_game_control_input.json` 和 runtime log，
  正式验收不能绕开 supervision。

## 停止与发布

从最外层 launcher 按 Ctrl-C 停止，并确认它拥有的 UE、SONIC、deploy、input provider、
DDS、ZMQ 和本地 socket 都已退出。保留与被测 commit 对应的河源证据，然后合并并更新
河源 clean main checkout。除非项目负责人明确要求，不向 TRNA 或 ZZA 同步源码或私有资产。
