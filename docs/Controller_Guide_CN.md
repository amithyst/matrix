# 遥控器与控制指南

Matrix 目前有多条控制链路，下面两组按键语义不能混用：

- `--control-source game` 是本文首先说明的、面向 SONIC 的相机相对第三人称控制；
- 文末保留 Matrix 原生/旧控制链路的按键表。旧链路里的站立、跳跃等动作键没有接入
  `game` 模式。

启动、相机标定、安全测试和河源验收步骤见
[Matrix 二游式控制运行手册](MATRIX_GAME_CONTROL_RUNBOOK_CN.md)。

## 相机相对 SONIC 控制（`--control-source game`）

### 键盘与鼠标

| 输入 | 行为 |
|---|---|
| **W / S** | 沿相机水平前向前进 / 后退 |
| **A / D** | 沿相机水平坐标系向左 / 向右移动 |
| **W+A**、**W+D** 等 | 斜向移动，最高速度不会高于单方向输入 |
| 按住 **Ctrl** + WASD | 原生 mode 1 `SLOW_WALK`，目标 0.10 m/s |
| 不带速度修饰键的 WASD | 原生 mode 2 `WALK`，目标 0.80 m/s |
| 按住 **Shift** + WASD | 原生 mode 3 `RUN`，目标 2.50 m/s |
| 鼠标拖动 | 沿用 Matrix 原生相机操作；按住配置的拖动键时机器人立即停止 |
| **V** | best-effort 安全镜像；观察到按键时强制归零。居中 overlay v3 不把 V 当成视觉相机模式切换 |
| **Q / E** | 仅保留，不参与 SONIC 运动计算，也不再让机器人原地转向 |

左右 Ctrl 等价，左右 Shift 也等价；任意 Ctrl 与任意 Shift 同时按下时，优先使用更慢的
Ctrl 静走档。

W、S、A、D 都采用“朝运动方向转身”：机器人先朝目标世界方向转向，大角度转身
尚未完成时会主动降低平移速度。鼠标只操纵相机；该链路不会直接旋转 UE 里的机器人
Actor。

只有 command heading 和实体 measured heading 都进入目标方向 15 度以内，才允许原生
平移启动；步态激活后用较宽的 30 度停止边界，既避免朝向噪声反复抖动，也会在实体明显
偏向时停止平移。

键盘三档现在选择 SONIC 的三种原生步态，不再是 `SLOW_WALK` 的三个速度别名：Ctrl
选择 mode 1、目标 0.10 m/s；无修饰键选择 mode 2、目标 0.80 m/s；Shift 选择 mode 3、
目标 2.50 m/s。三者分别位于 SONIC 声明的 0.10-0.80、0.80-2.50、2.50-7.50 m/s
原生区间下边界。Ctrl 与 Shift 同时按下时仍按更安全的 Ctrl 静走处理，Q/E 不会被占用。

加速和按住方向时的降档仍受限幅。上档爬升过程中，只有速度到达 0.80 m/s 才发布
mode 2，到达 2.50 m/s 才发布 mode 3；降档按同一边界反向经过，因此每一帧的 mode/速度
组合都在原生合法区间。松开全部移动方向会当帧请求 mode 0 `IDLE`，单独按修饰键不会
移动。左摇杆越过径向死区后仍连续映射到单独配置的模拟量上限（默认 0.30 m/s，最高可配
到原生上限 0.80 m/s），并保持 `SLOW_WALK`。从键盘档切到摇杆时，同一输出帧就会夹到
该实际配置上限。需要在河源实测步态切换。

默认用鼠标左键拖动 Matrix 相机。松开拖动键后，如果 W/A/S/D 一直没松开，机器人
不会突然继续走。先松开全部移动输入，再重新按下；这是 neutral re-arm（回中复位）
安全互锁。

### 持久在线居中 cooked overlay 与原生 fallback

河源 profile 默认配置宿主机私有 bundle：
`/home/kaijie/matrix-artifacts/matrix-centered-camera-custom-v1`。版本由仓库内
`config/runtime/matrix-centered-camera-overlay-v3.json` 决定；bundle 目录名里的 `v1`
只是历史命名。v3 契约用精确 size 和 SHA-256 锁定
`pakchunk99-MatrixCentered-Linux_P` 的 `.pak`、`.utoc`、`.ucas` 三个文件，作用域仅为
`MujocoSim_Custom` 与 `Spectator`，支持类固定为 `MujocoSim_Custom_C`。helper 在代码中
独立 pin 同一组 artifact tuple，因此换一个 contract 路径也不能授权其他字节。

只有同时满足“原生 SONIC + `--control-source game` + 开启居中模式 + `custom` 机器人 +
配置了 bundle”时才启用 overlay。主 launcher 取得宿主机锁后，会先原子清掉通过验签的
崩溃残留，再在 SONIC runtime audit 之前验签外部 bundle。`run_sim.sh` 在 UE 启动前把
私有副本原子安装到：

```text
src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive
```

随后 UE 使用 `viewclass Spectator_C`。overlay v3 持续把 Spectator pivot 移到 custom
机器人 `MainBody` 的位置，同时保留 Spectator 自身旋转；关闭相机/旋转 lag，保留
SpringArm 碰撞，pitch 限制为 -75/+55 度。资产默认臂长为 110 cm；launcher 默认覆盖
为 150 cm，使全身约占画高 60-63%。`MATRIX_GAME_CAMERA_DISTANCE_CM` 只接受 80-500 cm
闭区间内的普通十进制数；未来需要宽景时建议用 180 cm。只有本次启动新增日志段里的
`LogPakFile: Found Pak file` 与 `LogPakFile: Mounted IoStore container` 才能通过；新增
stem 行一旦含 `Failed` 就直接拒绝，历史日志不能冒充本次挂载。active 目录在整个
受监督 UE 生命周期内一直存在，只在 UE 精确停止之后原子移走。
若移除失败，launcher 会返回失败，不能把残留 overlay 的运行误报为成功。

这是“整段会话持续在线”的居中模式，不是按 V 在视觉上来回切换的模式。V 目前仍只做
best-effort 运动安全观察；v3 的可见相机会继续保持居中，不能拿 V 来验收自由相机画面。

未配置 bundle 时，所有现有模式保持原生 fallback。SONIC game fallback 会关闭
SpringArm 相机 lag 和旋转 lag、开启碰撞，并选择真正渲染出来的机器人 Actor：

| Matrix 机器人类型 | cooked UE view class |
|---|---|
| `custom`（SONIC G1 启动链） | `MujocoSim_Custom_C` |
| `go2` / `go2w` | `MujoCoSim_go2_C` / `MujoCoSim_go2w_C` |
| `xgb` / `xgw` / `xxg` / `zgws` | `MujoCoSim_Xgb_C` / `MujoCoSim_Xgw_C` / `MujoCoSim_Xxg_C` / `MujoCoSim_Zgws_C` |

`xxg` 映射记录的是 cooked 资产中已经存在的类；当前 0.1.2 launcher 仍会在 UE 启动前
拒绝 `xxg` 机器人类型。

原生 fallback 已在河源 live cooked runtime 实测：PlayerController ViewTarget 为
`MujocoSim_Custom_C`，live custom SpringArm 的父组件为 `MainBody`，相机 lag、旋转
lag、碰撞检测分别为 `False`、`False`、`True`。overlay v3 已通过离线 IoStore 验证和
Legacy/Zen 精确 round trip。移动机器人 orbit、墙面/地面碰撞恢复、远程桌面手感仍需
黑盒验收；不能只凭包体校验就宣称达到完整商业二游相机效果。

启动行为可以回滚，而且只在上述模式中生效：

- 设置 `MATRIX_GAME_CENTERED_CAMERA=0` 可关闭居中选择，并阻止安装 overlay；
- 在加载河源 profile 前显式继承空的 `MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE`，可关闭
  overlay 并退回原生路径；
- 未配置 bundle 时，可设置 `MATRIX_GAME_CAMERA_VIEW_CLASS=AnotherRobot_C` 覆盖目标短
  Blueprint 类名。
  值必须是以 `_C` 结尾的单个 token；空格、逗号和控制台分隔符都会被拒绝；
- 配置 bundle 后，view-class override 只能为空或严格等于 `Spectator_C`，其他值直接失败；
- `MATRIX_UE_EXTRA_EXEC_CMDS` 始终最后追加，因此操作者可以有意覆盖默认命令。

这些 `set Engine.SpringArmComponent ...` 是 UE 的 class-wide 控制台操作，并不只作用于
选中的机器人；应假定所有已加载的 SpringArmComponent 都可能受影响。如果某个场景
依赖 SpringArm lag，请先关闭该默认行为，并在验证更窄的替代命令后再通过
`MATRIX_UE_EXTRA_EXEC_CMDS` 添加。planner、PICO、external 以及非 SONIC 启动都不会
收到这组默认命令。

### ESC 本地/远程鼠标设置

在 `game` 模式中按 **ESC** 会立即让机器人安全归零，并显示画面中心十字、可见鼠标和
大号 MC 风格设置面板。X11 遮罩会拦截面板外的核心 ButtonPress/Release；但 cooked UE
也可能订阅 XI2 raw input，因此部署后仍须对着固定地标实测面板点击/拖动不会转镜头。
面板区分“当前已应用”和“下次启动”配置：

| 按键 | 行为 |
|---|---|
| **M** | 在 `Local` 与 `Remote` 间切换下次启动配置 |
| **- / +** | 遍历 Remote 预设：0.01x–0.10x 每次 0.01，随后 0.20x–1.00x 每次 0.10 |
| **鼠标** | 点击 `Local`/`Remote`、使用同一预设表的 `-`/`+`，或点击英文按钮 `RETURN TO GAME & APPLY`（返回游戏并应用） |
| **Enter** | “返回游戏并应用”的键盘等价操作；没有变更时直接返回 |
| **F9** | 键盘兜底：有已保存变更时安全重启完整 Matrix/SONIC 运行链 |
| **F10 / F12** | 保留给外部 MouseLock 的回中/开关，不由 Matrix 捕获 |
| **ESC** | 离开设置页；恢复运动前仍需一次回中复位 |

`Local` 固定为 1.0x；`Remote` 默认预设为 0.5x。Remote 一共有 19 个精确档位：
0.01x–0.10x 每次增加 0.01，接着直接进入 0.20x–1.00x、每次增加 0.10
（`0.10 +` 是 `0.20`，`0.20 -` 是 `0.10`）。键盘和面板点击遍历完全相同的档位表，
原有 0.40x 档继续保留。选择会立即原子保存到
`~/.config/matrix/mouse-control.json`，但当前 UE 进程的拖动速度不会在运行中改变。
点击英文 `RETURN TO GAME & APPLY` 或按 Enter 后，provider 会等待一帧已成功送达的
安全中立输入，再通过既有私有重启通道让最外层 launcher 重启**整条**运行链。旧进程
会停留在安全页面并显示加载状态；保存或请求失败时仍留在页面并显示错误。F9 复用同一
安全门槛作为兜底。不要单独重启 UE，重启期间保持所有控制输入松开。

真正影响可见 UE 相机的是启动时注入的 `SDL_MOUSE_RELATIVE_SPEED_SCALE`。例如目前使用
的 Remote 0.4x 是 SDL/UE 原生输入倍率，不是 X 指针加速度。`x11-mirror` 的默认基值
为 0.12 度/XI2 raw unit 时，状态里会显示 `0.12 x 0.4 = 0.048 度/raw`。镜像现在订阅
SDL 相对鼠标模式通常使用的 `XI_RawMotion`，launcher 也请求 SDL raw 模式；但 packaged
UE 是否等量消费这些增量尚未由 live 黑盒验证。它仍不是最终渲染相机 yaw 的回读，
也不能证明镜像方向与可见画面已经一致。仍须完成下文的四方向、多轮往返黑盒验收。缺失或损坏的
设置文件，以及手工写入非预设档位的配置，都会安全回退到 Local 1.0x。有效的 Remote
启动会把同一个所选倍率同时交给可见 SDL/UE 输入路径和 `x11-mirror` 的名义增益。

启动器同时请求并配置 SDL raw relative motion，禁用 warp、窗口缩放和 SDL 系统指针
缩放，通过 UE Input 配置关闭 `bEnableMouseSmoothing` 与 FOV 灵敏度缩放，在机器人
居中视角中关闭 SpringArm lag，并加入 `r.MotionBlurQuality 0`。前几项消除输入插值，
最后一项消除画面运动模糊造成的视觉拖尾；它们都不会改变所选倍率。

当启动组合为 SONIC + `game` 且当前已应用 `Remote` 时，`run_sim.sh` 还会先记录当前
X display 的 acceleration/threshold，在 UE 启动前临时执行 `xset m 1/1 0`，清理时再
精确恢复原值。这只把设置页使用的 X11 绝对指针流线性化；yaw 镜像已经改用 XI2 raw
motion，也不修改 MouseLock。没有 `DISPLAY`、找不到 `xset` 或 X server
调用失败都只会告警，不阻止启动。指针参数在 Matrix 运行期间对该 X display 全局生效；
正常退出和可处理信号会恢复，但 `SIGKILL` 或主机宕机无法执行 cleanup，此时需按日志
记录手动执行 `xset m <原 acceleration> <原 threshold>`，或重启桌面会话。

远程桌面若仍产生指针回中、窗口边缘或绝对坐标跳变，应先用十字和外部 MouseLock
完成可见会话的回中标定；已经实测的当前 MouseLock `pyautogui.moveTo`/XTEST absolute
recenter 不会累计成 XI2 raw yaw。其他合成 relative recenter 不在这一结论范围内。

### 手柄当前状态

目标设计是左摇杆负责相机相对移动，右摇杆只负责相机。但当前 Matrix 0.1.2 cooked
运行包没有经过验证的接口来读取或驱动可见 UE 跟随相机，因此：

- 输入适配器能读取 Linux 摇杆轴；在 `carla` 来源下，右摇杆会写入 spectator yaw/pitch，并
  立即回读绝对 yaw。写入或回读失败会停机，不会把未观测的内部累计角度当作相机真值；
- 相机 yaw 来源为 `fixed` 或 `x11-mirror` 时，`--game-input-source auto` 会安全降级为
  仅键盘；显式指定 `gamepad` 会被拒绝；
- 显式选择 `carla` 后，只要 spectator RPC 写入并回读成功就会开放左摇杆移动；这仍只是
  spectator transform 候选，不能证明画面里的跟随相机同步转动。现有 0.1.2 发行包没有
  发现 CARLA server。

在增加 UE runtime camera bridge 并通过运行手册中的黑盒验收前，不能声称右摇杆相机
或完整手柄控制已经实现。

### 安全行为

输入默认以 50 Hz 采样；键盘静走/普通走/跑步分别是原生 mode 1/2/3，目标为
0.10/0.80/2.50 m/s；手柄在原生 `SLOW_WALK` 内连续映射，单独配置的上限默认是
0.30 m/s、最高 0.80 m/s。输入超时阈值和快照最大年龄均为 0.15 s。松开全部方向键
或遇到以下任一情况，SONIC 指令会直接硬归零，不保留减速尾巴：

- 启动后尚未收到回中帧；
- 原生 LowCmd 尚未 fresh，或启动弹性带尚未完全释放；
- Matrix 窗口失去焦点；
- 适配器观察到镜像 V 安全状态，或正在按住鼠标相机拖动键；
- 输入超时、过期、断连或协议校验失败；
- 输入 provider 退出、本地 socket 关闭，或所选相机 yaw 来源不可用。

启动、失焦、拖动相机、镜像 V 状态切换、超时或重连后，需要松开 WASD 并让左摇杆回中。
系统收到一帧“窗口有焦点且移动输入为零”的快照后才会重新允许运动，防止仍按着 W
或仍推着摇杆时机器人突然启动。

输入适配器只通过用户私有的本地 socket 发送完整快照；runtime 同时核验对端 UID 和
受监督 adapter 进程的精确 PID。原生 SONIC planner 仍是唯一运动指令发布者；输入
适配器本身不会发布 DDS 或 planner 指令。

launcher 会同时核验活动窗口标题和受监督 UE 的精确 PID，因此标题含有 “matrix” 的
终端或 IDE 不能继续驱动机器人。0.15 s 是超时阈值，实际在下一次 50 Hz 控制 tick
执行硬停，标称最坏约 0.17 s 再加调度抖动；它只覆盖输入/provider 链。若整个 runtime
进程冻结，则退回 SONIC 自身更长的 watchdog。V 在 UE 提供类似 `SubscribeInputMode`
的真实模式回读前仍是 best-effort，尤其不能识别 provider 启动前发生的切换；居中
overlay v3 明确会在按 V 时继续保持可见画面居中。

## 相机 yaw 来源

| 来源 | 用途 | 限制 |
|---|---|---|
| `fixed` | 安全验证方向键和 deadman | 可见相机旋转后，控制坐标系不会跟随 |
| `x11-mirror` | 河源上的标定候选 | 只在配置的 raw button 按下区间积分有序 XI2 raw motion；absolute warp 不会抵消拖动。它仍不读取或驱动最终 UE 相机，UE 自动回正仍可能造成分叉 |
| `carla` | 可写且可回读的 spectator 候选 | 右摇杆写 yaw/pitch 旋转后立即回读；写入/yaw 回读失败直接停机。Matrix 0.1.2 cooked 包实际未发现 CARLA server，且没有可见相机耦合证明 |

默认使用 `fixed`，避免未经验证的相机估计悄悄把机器人带向错误方向。只有在河源标定
sensitivity、sign、offset，并用 0/90/180/-90 度和多轮往返拖动证明 W 始终沿可见
相机前向且不累积漂移后，才能把 `x11-mirror` 当作验收候选。

## Matrix 原生/旧控制映射

下表是上游 Matrix 原始遥控链路的按键，不是 `--control-source game` 的动作绑定。

### 手柄

| 操作 | 控制输入 |
|---|---|
| 站立 / 坐下 | 按住 **LB** + **Y** |
| 前进 / 后退 / 左移 / 右移 | **左摇杆** |
| 向左 / 向右旋转 | **右摇杆** |
| 向前跳（冲刺） | 按住 **RB** + **Y** |
| 原地跳 | 按住 **RB** + **X** |
| 翻筋斗 | 按住 **RB** + **B** |

上游推荐 Logitech Wireless Gamepad F710。

### 键盘

| 操作 | 控制输入 |
|---|---|
| 站立 | **U** |
| 坐下 | **Space** |
| 前进 / 后退 / 左移 / 右移 | **W / S / A / D** |
| 向左 / 向右旋转 | **Q / E** |
| 开始 | **Enter** |

未启用居中 overlay v3 的旧原生链路中，**V** 用于切换自由相机，按住鼠标左键会暂时进入自由相机操作。使用
`game` 模式时，以本文前半部分的行为和安全互锁为准。
