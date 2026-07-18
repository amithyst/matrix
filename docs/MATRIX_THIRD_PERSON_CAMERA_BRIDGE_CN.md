# Matrix 二游第三人称相机：能力审计与 UE Bridge 契约

## 结论

Matrix 当前锁定的 0.1.2 cooked UE 运行包，没有可验证的接口来完成真正的
二游第三人称相机。尤其不能从本仓库做到以下五件事：

- 把最终渲染相机的 orbit pivot 绑定到画面中的机器人 Actor；
- 在机器人平移时让最终相机以同一个相对 offset 同步跟随；
- 读写最终相机的 yaw、pitch 和实际 view transform；
- 从同一个渲染帧保存自由相机相对机器人的位姿，再可靠交给锁定模式；
- 在 UE 地图碰撞世界中对墙面、地面做 ray/sphere sweep，并据此缩短 camera arm。

因此本轮没有用 X11 指针估计、CARLA spectator 或合成按键伪装成视觉功能已经完成。
仓库新增的是：可测试的四模式/能力门禁，以及 UE 插件需要遵守的 relative-lock、orbit 与
碰撞参考算法。
它们位于 `scripts/matrix_third_person_camera.py`，在新的 cooked UE bridge 出现前不会改变
现有画面。

## 当前三类“相机状态”的真实含义

| 名称 | 当前真实能力 | 边界 |
|---|---|---|
| 原生跟随/锁定 | cooked UE 自己决定构图、pivot 和跟随 | Python 侧不能读最终 view，也不能保证机器人居中 |
| 原生自由相机 | 上游文档用 `V` 切换 | 没有权威模式回读；输入 provider 只能镜像观察到的 V 按键沿 |
| 鼠标拖动 | cooked UE 消费原生鼠标事件 | `game` 控制在拖动时让机器人安全停机，但不拥有相机 |

`GameControlCore` 的 `_free_camera` 只是运动安全状态：看到聚焦窗口中的 V 新按下后切换
布尔值，并让 SONIC 归零。它不是 UE 相机控制器。provider 启动前发生的 V 切换、UE 忽略
V、或 UE 自己改变模式，都无法被这个布尔值发现。

## 能力证据

1. 仓库的 `src/UeSim` 只有 Linux 启动脚本与占位文件，没有 `.uproject`、Blueprint、
   PlayerCameraManager、SpringArm 或 UE C++ 源码。
2. `config/runtime/matrix-sonic.lock.json` 固定的是 cooked UE ELF、`Content/Paks` 和二进制
   目录。它没有可重新编译的相机项目源。
3. `x11-mirror` 只积分 X11 根窗口指针的横向变化。它不设置相机，不回读最终 view，
   指针 warp、远程桌面绝对坐标和 UE 自动回正都会造成分叉。
4. 可选 CARLA 路径只对 spectator 执行 `set_transform -> get_transform`。0.1.2 实测没有
   发现 CARLA server；即使 server 存在，也尚未证明 spectator 是用户看到的跟随相机。
5. `config/config.json` 中的 `sensors.camera` 是机器人 RGB 传感器，不是桌面 viewport
   相机，修改它不会得到二游构图。
6. ZZA Town10 的既有黑盒记录明确指出，G1 长期靠近画面右侧，发送 V 前后也没有得到
   可确认的跟随构图变化。

## 目标模式与按键（提案，尚未接入 cooked UE）

保留原生 legacy 模式，并把“相对锁定”和“真正二游”拆成两个明确模式：

| 模式 | UI 名称 | 行为 |
|---|---|---|
| `native-follow` | 原生跟随 | 完全保留 0.1.2 当前行为，也是默认值 |
| `native-free` | 自由相机 | 保留 V 的原生语义；机器人安全归零 |
| `relative-lock` | 相对锁定 | 锁存 camera/look-at 相对机器人 pivot 的两个 offset，机器人平移时等量跟随，保留自由相机构图 |
| `orbit-follow` | 二游跟随 | 机器人 pivot 居中、yaw/pitch 轨道、同步跟随和防穿模 |

这里的“保留”是严格边界：`native-follow` 仍由原 0.1.2 cooked UE 实现，不用参考 Python
算法替换，也不宣称它已经变成二游镜头。新增行为只属于未来 bridge 管理的
`relative-lock` 与 `orbit-follow`；bridge 不存在时仍只有两个原生模式。

`native-follow` 就是用户所说的“当前/legacy 模式”，不会被新算法覆盖。`relative-lock` 是可执行、
可 confirmed 的 bridge 模式，不是无人消费的数学 helper：`RelativeLockController` 从最终 source
frame 捕获两个 offset，之后每个权威 pivot 都生成 camera/look-at 的等量平移。它不强制人物
居中，也不等同于 `orbit-follow`。`orbit-follow` 才负责人物居中、绕 pivot 旋转和碰撞收臂。

建议用 **C** 在 `native-follow` 与 `orbit-follow` 间切换；ESC 大面板同时提供两个大号
单选项；ESC UI 另提供 legacy/relative-lock/orbit 三个明确选项。**V** 始终只负责“当前
lock/follow 模式 <-> `native-free`”，从自由相机返回时恢复上一次 confirmed lock 模式。
自由相机活动时忽略 C，避免一个按键把安全停机悄悄变回跟随。

从 `native-free` 返回 `orbit-follow` 不是一次无条件布尔切换，不能把相机留在一个世界坐标
锚点，也不能直接复用上次的 `orbit-follow`。UE 应在鼠标/摇杆
回中且输入捕获已释放后，从**同一个最终渲染帧**读取 robot pivot、camera position 和
look-at，保存 `camera - pivot` 与 `look_at - pivot`。普通相对锁定可把两个 offset 随新
pivot 等量平移；真正的二游锁定以 camera offset 初始化 yaw/pitch/arm，并把最终 look-at
改为 robot pivot，使机器人回到构图中心。之后机器人位移多少，相机无碰撞时同帧位移多少。

**所有**切换都采用 requested/confirmed 两阶段，不只自由相机返回：初始 `native-follow`
按 C、ESC 直接选择 follow、V 进入/退出自由相机，都只能创建 `CameraModeRequest`，不能立即
改 `CameraModeController.mode`。`mode` 永远是 UE 最后确认的模式；UE 完成 sweep 与可选视向
平滑后，必须用更新的最终渲染帧回显同一个 session/request id 才能提交。能力缺失、旧帧重放、
未来时间戳、输入仍被捕获或最终帧不自洽时都不完成切换，SONIC 始终保持停机。
pending request 默认 500 ms 超时；超时只撤销请求并失效其 provenance，不修改 confirmed mode。

每次真实模式切换都必须：

1. 立即发送 unfocused、全中立快照，让现有 SONIC 控制硬停；
2. 等 UE bridge 回读的新模式和最终 view，而不是看到按键就假定成功；
3. 切换完成后仍要求移动键/左摇杆回中，再允许机器人移动；
4. bridge 掉线、状态过期或能力减少时立即安全停机；在 UE 新帧确认前保留最后 confirmed
   mode，不能为了日志好看直接把变量改成 `native-free`。随后另发相关联的 free 请求。

在 bridge 不存在时，UI 必须把“二游跟随”显示为“运行包不支持”，C 不改变模式。

## UE 侧最小实现

需要取得 Matrix UE 项目源并重新 cook 运行包。只修改 Python 或 PAK 外部配置不够。

建议实现一个 `UMatrixOrbitCameraComponent` 或自定义 `APlayerCameraManager`：

1. **权威 pivot**：在每个 UE tick 的机器人状态同步之后，读取画面中主机器人 Actor
   的 root/pelvis 位置，加一个可配置的胸口高度。不要用外部 MuJoCo 坐标猜 viewport
   Actor 的最终插值位置。
2. **同步跟随**：pivot 平移不做滞后；无碰撞时，相机与 pivot 的相对 offset 必须只由
   orbit yaw、pitch 和 arm 决定。可选相机旋转阻尼不能让机器人长期偏离构图中心。
3. **相对锁定**：`relative-lock` 保存 source frame 的 `camera - pivot` 与 `look_at - pivot`；
   机器人 pivot 平移多少，两者同帧平移多少。确认帧必须回读同样 offset（2 cm 容差），不能
   偷换为 legacy 或人物居中的 orbit 构图。
4. **模式请求**：每次按键/UI 切换都从输入已释放的最终 source frame 生成单调 request id，
   记录 session、source sequence/timestamp/render frame、pivot 和相对位姿。初始 C 也不能
   例外。交接时先做 collision sweep；状态过期或 frame 不一致都拒绝，不能混用前一帧
   pivot 和后一帧 camera。
5. **视向交接**：无遮挡时相机位置在交接首帧不跳；`LookAtHandoffController` 可在例如
   0.2 秒内用 smoothstep 时间曲线和单位方向 slerp 把自由视向转到 pivot；同向使用归一化
   lerp，近 180° 反向使用确定性正交轴，避免插值向量经过零。平滑完成前权威模式仍是
   request 的 source confirmed mode（自由返回时才是 `native-free`，初始 C 时仍是
   `native-follow`），
   SONIC 保持停机；不能把人物尚未居中的中间帧发布成 `orbit-follow`。若障碍要求立即收臂，
   防穿模优先于“位置无跳”。
6. **轨道**：水平 yaw 环绕 pivot，pitch 限制在例如 `[-25°, 65°]`。相机最终
   `LookAt` 始终是 pivot，而不是让相机绕自己的当前位置旋转。
7. **防穿模**：每帧从 pivot 到目标相机位置执行 `SphereTrace`/`SweepSingleByChannel`，
   使用 `ECC_Camera`、约 20 cm probe radius，并忽略主机器人和相机 rig 自身。墙面、
   地面以及需要挡镜头的动态物体都必须在该 channel 阻挡。
8. **arm 恢复**：新障碍出现时立即把 arm 收到首个 hit 前的安全距离，不能平滑穿墙；
   障碍消失后才以受限速度平滑恢复到目标距离。
9. **最终回读**：在 PlayerCameraManager 完成最终 view 计算后发布实际 camera transform、
   pivot、期望/实际 arm、collision hit、模式、焦点/捕获状态和渲染 frame id。
10. **运行时 bridge**：通过用户私有本地 IPC 暴露能力和状态；provider 校验协议版本、
   同一 UID、受监督 UE PID、递增序号与状态新鲜度。不要开放网络监听端口。
11. **打包**：插件/Blueprint 必须进入新的 cooked ELF/PAK，随后更新 release 包与 runtime
   lock 哈希；不能往被忽略目录塞未锁定 `.so` 绕过 verifier。

UE 内也可以使用 `USpringArmComponent` 的 Camera collision 作为起点，但仍需补充权威
最终 view/mode 回读，并验证其恢复过程不会穿墙。若 SpringArm 的默认恢复过快，应在
“放长”方向增加阻尼；“缩短”方向继续以当前碰撞上限为硬边界。

## Bridge 能力门禁

`orbit-follow` 只有在 UE 同时报告下列全部能力时才可选：

```json
{
  "protocol": "matrix-third-person-camera/v1",
  "authoritative_robot_pivot": true,
  "final_view_readback": true,
  "orbit_control": true,
  "sphere_sweep": true,
  "input_mode_readback": true,
  "relative_pose_handoff": true,
  "relative_lock_control": true
}
```

少任意一项都拒绝进入，而不是用最后一次 yaw、X11 指针或 spectator 结果补齐。
参考实现对 capability 字段采用精确 schema，未知或缺失字段都会失败。
`relative-lock` 不声称 orbit/collision，因此要求 authoritative pivot、final view、input mode
readback、relative pose handoff 与独立的 `relative_lock_control`；缺任一项同样不可选。
`native-follow` 继续是 legacy fallback。`orbit-follow` 不借 `relative_lock_control` 冒充自身能力。

## Bridge 最终帧接口

未来 UE bridge 每次状态发布至少遵守 `CameraBridgeFrame` 的精确 schema。例如：

```json
{
  "protocol": "matrix-third-person-camera/v1",
  "session_id": "matrix-camera-7f31",
  "sequence": 42,
  "produced_monotonic_ns": 7312456000000,
  "applied_request_id": 8,
  "render_frame_id": 9001,
  "mode": "orbit-follow",
  "robot_pivot_m": [1.0, 2.0, 1.15],
  "camera_position_m": [-1.9, 2.0, 1.15],
  "look_at_m": [1.0, 2.0, 1.15],
  "input_captured": false,
  "desired_arm_m": 3.2,
  "actual_arm_m": 2.9,
  "collision_limited": true
}
```

`session_id` 标识本次受监督 UE/consumer 会话；每个模式请求的 `request_id` 在该会话中单调
递增，最终帧必须用 `applied_request_id` 原样回显。`sequence` 必须严格递增，
`produced_monotonic_ns` 用于 100 ms 默认 deadman，
`render_frame_id` 必须对应实际提交给 PlayerCameraManager 的同一帧。
非 orbit 模式（包括 `relative-lock`）的三个 arm/collision 字段必须是 `null`，不得伪装成
bridge 已执行碰撞；
`orbit-follow` 必须同时给出期望臂长、最终臂长和一致的约束标志：只要
`actual_arm_m < desired_arm_m`，包括障碍已经离开但臂长仍在限速恢复时，
`collision_limited` 仍为 `true`。此外 `|camera - pivot|` 必须在 5 mm 内等于实际臂长，
`look_at` 必须在 5 mm 内等于 pivot；否则拒绝这帧，不能伪造权威 orbit 状态。
进程身份、消息新鲜度和
frame 连续性由 IPC consumer 额外校验。v1 的 `look_at_m` 是与最终 forward 一致的目标点，
并约定世界 Z-up、零 roll、无 lens shift；若实际镜头允许 roll 或偏轴投影，必须升级协议并
增加 up/projection 回读，不能继续声称 v1 是完整 view。Python 参考类本身不建立 socket，
也不驱动画面。

### 请求关联与空间绑定

仅仅“frame 自己几何自洽”仍不够。确认帧还必须满足：

- session 与 pending request 相同，`applied_request_id` 精确匹配；
- sequence、render frame id、produced monotonic timestamp 都严格晚于 source frame，且 produced
  timestamp 还必须晚于 request 创建时刻，排除“请求之前已经生成”的缓存帧；
- safe-stop 期间 pivot 与请求源相差不超过 25 cm，拒绝虽然自洽但出现在 1000 m 外的帧；
- orbit 期望臂长与 source relative pose 相差不超过 2 cm，相机 offset 方向误差不超过 1°；
- 任一检查失败都保留原 confirmed mode 和 pending request，不消费失败帧。
- pending 超过 500 ms 后主动取消并使 request provenance 失效；迟到完成帧不能复活它。

## 参考 orbit/collision 行为

`OrbitCameraController` 给未来 UE 自动化测试提供了平台无关的 conformance reference：

- pivot 等于机器人位置加固定高度；机器人平移多少，pivot 同步平移多少；
- `RelativeCameraPose` 从权威 confirmed source 最终帧捕获 source sequence/timestamp/pivot 与两个
  offset；pivot 平移后 camera/look-at 可等量平移，不留下世界坐标锚点；
- `RelativeLockController` 是 `relative-lock` 的可执行 conformance 状态：确认帧保持两个
  captured offset，后续 pivot 平移时 camera/look-at 严格等量平移；legacy 不受它影响；
- 生产交接只用 `relock_from_request` 生成带 request provenance 的 collision-safe orbit 目标；
  `relock_from_free` 只是无授权的几何参考，不能用于确认模式；
- `LookAtHandoffController` 必须持有仍处于 pending 的同一个 request，并只接受携带相同内部
  provenance 的 target state。任意构造、另一请求派生或已取消请求的 state 都被拒绝；
- 视向插值在同向、一般夹角和 180° 反向三种情况下都保持非零单位方向，不能用 look-at
  点的简单直线插值穿过 camera position；
- yaw/pitch 只改变 pivot-to-camera offset，`look_at` 恒等于 pivot；
- sphere sweep 是每帧必需输入，缺失、异常、起点穿透或越界 hit 都 fail closed；
- 障碍变近时实际 arm 在同一步立即缩短到 `hit distance - padding`；
- 障碍离开时实际 arm 按 `recovery_speed * dt` 平滑增长，且永不超过本帧已验证空间；恢复
  完成前 `collision_limited` 仍为真，与 `actual < desired` 的 frame schema 保持一致；
- sweep 失败是事务性的，最后一帧有效相机状态不会被半次 yaw/pitch 更新污染。

该 Python 控制器是数学与安全契约，不是通过 X11 驱动画面的替代实现。生产路径应在
UE 内按相同不变量完成 collision 和最终 view 计算，避免跨进程一帧延迟造成穿模。

## 验收清单

### UE 自动化

- 机器人沿 X/Y/Z 平移时，pivot 同帧跟随，无遮挡 offset 误差保持在厘米级阈值内；
- 连续 yaw 旋转多圈无累计漂移，pitch 在上下边界不抖动；
- 自由相机移动后锁定，无遮挡时首帧相机位置不跳；视向平滑期间仍报告 `native-free` 且
  机器人停机，正式 `orbit-follow` 第一帧人物已经居中；随后 camera/pivot 位移相等；
- 初始 native-follow 按 C、所有 V/UI 切换在 UE 回读前 confirmed mode 均保持不变；
- legacy、relative-lock、orbit-follow 三个 confirmed lock 模式互不冒充；relative-lock 的
  camera/look-at offset 在机器人平移前后保持不变；
- 错 session/request、旧 timestamp、回放 frame、远处自洽 frame 和错 relative pose 均拒绝；
- request 超时后 confirmed mode 不变，任意 state、另一 request state 和迟到 state 均拒绝；
- 用严格反向的 source/target view 做 0.5 插值，方向长度保持非零且所有分量有限；
- 正面靠墙、背靠墙、墙角、低矮天花板、斜坡和贴地俯视均不穿模；
- 突然出现障碍的第一帧就收短 arm，移开障碍后按配置平滑恢复；
- sweep 起点穿透、目标 Actor 销毁、地图切换或 bridge 断连都进入不可用状态；
- 最终 view readback 与真正提交给 PlayerCameraManager 的 view 同 frame id。

### Matrix/SONIC 集成

- `native-follow` 和既有 V 自由相机行为无回归；
- C/UI 每次切换都先硬停，按住 W 切换后不能自动继续走；
- orbit bridge 任一能力缺失或状态超过 deadman 阈值，机器人立即归零；
- 相机 yaw 用最终 view readback 驱动 WASD 坐标，而不是继续使用 `x11-mirror`；
- 0/90/180/-90 度下 W 与画面前向一致，连续绕转后无漂移；
- 物理频率、RTF、无跌倒/无数值重置和完整进程清理继续满足现有 qualified gate。

### 视觉黑盒

- 静止绕机器人一周时，机器人保持在目标构图中心；
- 机器人直线、斜向和转弯时相机同步跟随，不发生 pivot 留在原地的“绕空气”现象；
- 贴近建筑、进出门洞、上下坡时录制视频，逐帧确认墙面和地面不穿模；
- 记录模式 UI、bridge capability/state、最终 view、collision arm 与视频时间戳，避免只凭
  单张截图宣称完成。
