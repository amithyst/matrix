# Matrix 原生场景与 SONIC 集成审计

## 范围

本审计只评估 Matrix 0.1.2 自带的原生地图，不导入 AUE 当前的
`overworld` 场景。目标是判断 Matrix 能否承载 G1 29 自由度 SONIC，重点考察：

1. 视觉还原度
2. 物理还原度
3. SONIC 50 Hz 闭环能否稳定持续

审计基线为 Matrix `918fae3`，工作分支为
`research/sonic-integration-audit`。

## 原生场景数量

Matrix 0.1.2 发布清单包含 19 个可选地图包：18 个正式场景和 1 个
`CustomWorld` 模板。Base 包中还带有一个不在当前启动菜单中的
`EmptyWorld`，所以按发布资产计算共有 20 个地图资产。

| 启动 ID | 菜单/文档名 | UE 地图 | MuJoCo 场景 |
| --- | --- | --- | --- |
| 0 | CustomWorld | CustomWorld | scene_terrain_custom.xml |
| 1 | Warehouse | SceneWorld | scene_terrain_wh.xml |
| 2 | Town10World | Town10World | scene_terrain_t10.xml |
| 3 | YardWorld | YardWorld | scene_terrain_yard.xml |
| 4 | CrowdWorld | CrowdWorld | scene_terrain_crowd.xml |
| 5 | VeniceWorld | VeniceWorld | scene_terrain_venice.xml |
| 6 | HouseWorld | HouseWorld | scene_terrain_house.xml |
| 7 | RunningWorld | RunningWorld | scene_terrain_rw.xml |
| 8 | Town10Zombie | Town10Zombie | scene_terrain_zombie.xml |
| 9 | IROSFlatWorld | IROSFlatWorld | scene_terrain_flat.xml |
| 10 | IROSSlopedWorld | IROSSlopedWorld | scene_terrain_sloped.xml |
| 11 | IROSFlatWorld2025 | IROSFlatWorld2025 | scene_terrain_flat25.xml |
| 12 | IROSSloppedWorld2025 | IROSSloppedWorld2025 | scene_terrain_sloped25.xml |
| 13 | OfficeWorld | OfficeWorld | scene_terrain_office.xml |
| 14, 16, 17 | 3DGSWorld | 3DGSWorld | 3dgs.xml |
| 15 | MoonWorld | MoonWorld | scene_terrain_moon_dynamic.xml |
| 20 | Cali Room | CaliWorld | scene_terrain_cali.xml |
| 21 | Home | ApartmentWorld | scene_terrain_apart2.xml |
| 22 | Laboratory | MeetRoomWorld | scene_terrain_meet.xml |

`16` 和 `17` 都是 `3DGSWorld` 的别名，不是额外地图。完整包大小、SHA256、
预览图和物理代理统计见
`research/sonic_integration/native_scenes.json`。

## 当前静态结论

### 视觉

Matrix 使用 Unreal Engine 5/CARLA 渲染。原生预览中，`Town10World`、
`Warehouse`、`ApartmentWorld` 和 `3DGSWorld` 的材质、光照和场景密度明显高于
当前 AUE 小场景，适合做视觉基线。但预览图只能证明离线展示效果，不能证明
目标 GPU 上的持续帧率、传感器一致性或交互性。

### 物理

Matrix 的 UE 视觉与 MuJoCo 物理是两套场景表示。安装 0.1.2 Base 包后核对的
当前 XML 显示，大多数复杂视觉场景使用静态 box/cylinder 代理：

| 场景 | MuJoCo 环境 geom | 主要代理 | 动态物体 |
| --- | ---: | --- | ---: |
| 3DGSWorld | 11 | 1 plane + 10 box | 0 |
| Warehouse | 210 | 201 box + 8 cylinder | 0 |
| Town10World | 869 | 686 box + 173 cylinder + 9 mesh | 0 |
| VeniceWorld | 1276 | 1172 box + 103 cylinder | 0 |
| CrowdWorld | 131 | 130 cylinder | 0 |
| OfficeWorld | 146 | 145 box | 0 |
| MoonWorld | 256 | 256 动态 box | 256 |
| CaliWorld | 1 | 1 plane | 0 |
| ApartmentWorld | 74 | 68 box + 5 cylinder | 0 |
| MeetRoomWorld | 48 | 41 box + 6 cylinder | 0 |

因此，Matrix 的视觉复杂度不能直接等同于物理复杂度。特别是 3DGS 场景，当前
证据只支持粗碰撞导航，不支持桌面物体级的精细抓取和可交互性。尤其
`CaliWorld` 的视觉物体在 MuJoCo 中完全没有对应碰撞代理，只有地面。

### 频率

当前公开配置和启动脚本的默认频率是：

| 链路 | 默认值 |
| --- | ---: |
| UE 最大渲染帧率 | 30 FPS |
| 机器人视觉同步 | 10 Hz |
| RGB / Depth / LiDAR 发布 | 10 Hz |
| MuJoCo 模型 timestep | XML 未显式设置，MuJoCo 默认 2 ms / 500 Hz |
| SONIC 低层控制目标 | 50 Hz |

发布 XML 本身未覆盖 `timestep`，所以按 MuJoCo 3.3 默认值是 2 ms；仍需通过
运行消息率确认闭源二进制是否覆盖该值。MuJoCo 内部能跑得快，不代表 UE 同步和
相机也能达到 50 Hz。验收必须分别记录 physics tick、lowstate、lowcmd apply、
UE render 和传感器发布频率。

## G1/SONIC 集成状态

0.1.2 的 `robot_mujoco` 是 x86_64 闭源二进制。它链接 eCAL/Protobuf，并使用
`RobotCmd`、`RobotState` 和 `VisualData`。公开安装包中的 `RobotCmd` 固定为
四足语义字段：

```text
q_des_abad / hip / knee / foot
qd_des_abad / hip / knee / foot
kp_abad / hip / knee / foot
kd_abad / hip / knee / foot
tau_abad / hip / knee / foot
```

这不是 G1 29 自由度的通用关节数组，因此本分支没有修改或复用四足 `mc_ctrl`，
而是通过 `MATRIX_DISABLE_MC=1` 绕过它。物理由 AndroidTwin 的 `FusedSink`、
GR00T 原生 G1 MJCF 和 SONIC deploy 负责；Matrix 只负责原生地图、UE 渲染和
传感器。这样保留了已有 DDS、PD 力矩、安全限幅、planner/PICO 管理器，且没有
把 AUE `overworld` 搬入 Matrix。

Matrix UE 的 `9999/UDP` 状态协议已验证为通用变长数组，而不是四足固定槽位：

```text
little-endian double time
uint32 nq + nq * float64 qpos
uint32 nv + nv * float64 qvel
uint32 nu + nu * float64 ctrl
```

当前 G1 模型为 `nq=36, nv=35, nu=29`，单包 820 bytes。运行时使用 GR00T
`g1_29dof_with_hand.xml` 的惯量、碰撞、阻尼和摩擦，去除手指自由度后保留完整
29 关节 SONIC 顺序，再与所选 Matrix 原生地图的物理代理结构化组合。生成模型
固定为 `timestep=0.005`、关节 `damping=0.05`、`armature=0.01`、
`frictionloss>=0.1`。

SONIC deploy 的 INIT 姿态插值固定持续 3 秒。主脚本默认使用已有 AndroidTwin
临时弹性吊带保护该阶段：保持 4 秒、再用 3 秒淡出，之后完全由接触物理和 SONIC
控制；它不是持续悬挂。可用 `--no-startup-band` 做故障对照，但不应作为正常启动
配置。TRNA 对照中，关闭或只淡出 2 秒都会倒地，4+3 秒配置在吊带退出后稳定站立
并完成前进。

首轮视觉导入使用 SONIC 仓库的 canonical `g1_29dof.urdf`，不携带 AUE 场景。
该模型有 39 个 link、38 个 joint，其中 29 个可动关节；63 个 mesh 引用在源目录
中均可解析。URDF 只供 UE 构建可视模型，MuJoCo 物理不再使用转换器生成的近似
惯量，而使用上述 GR00T 原生 MJCF。

官方 `install_deps.sh` 没有安装 `python3-venv`、pip 或 `urdf2mjcf`，因此默认
安装完成后 custom URDF preflight 仍会失败。TRNA 审计环境使用：

```bash
sudo apt-get install -y python3-venv
python3 -m venv .venv-audit
.venv-audit/bin/python -m pip install \
  -r research/sonic_integration/requirements-trna.txt
```

其中转换器固定为 `urdf2mjcf==0.1.3`，Python MuJoCo 固定为 3.3.0，与 Matrix
系统运行库版本一致。

本次 TRNA 验收同时记录 AUE/AndroidTwin 与 GR00T 的 Git 提交，以及实际使用的
`FusedSink`、SONIC deploy 二进制、canonical MJCF、Unitree SDK 静态库和视觉
URDF 的 SHA256，见结果 JSON 的 `runtime_provenance`。GR00T 检出中存在两处未用于
planner smoke 的 manager/exporter 脚本改动，因此不能只依赖仓库提交号；后续 PICO
验收必须先固定这两处源码状态，再记录设备和管理器链路版本。

## 许可证边界

Matrix 代码仓库使用 BSD-3-Clause。这个许可证允许修改和再分发代码及其已授权
二进制，但不能自动证明每个 UE/CARLA 地图、第三方素材或 3DGS 扫描数据都允许
用于我们的开放世界游戏并再次发布。正式复用场景前，需要为每个地图补齐来源、
作者、许可证、商业使用和再分发许可；当前 release manifest 只有包名、大小和
校验值，没有逐资产许可证清单。

## 运行验收计划

TRNA 是首轮运行机：Ubuntu 22.04 x86_64、RTX 5080 Laptop 16 GB。安装和实验
固定在 tmux：

```bash
ssh trna-zt
tmux attach -t matrix-sonic-eval
```

代表性原生场景：

| 场景 | 用途 |
| --- | --- |
| IROSFlatWorld | 低视觉负载下的物理/控制频率上限 |
| CaliWorld | 小型原生室内基线 |
| 3DGSWorld | 高视觉还原、粗物理代理对照 |
| ApartmentWorld | 室内导航与遮挡 |
| Town10World | 大世界视觉和持续负载压力测试 |

每个场景至少采集 60 秒稳态窗口，记录 real-time factor、physics Hz、UE FPS、
视觉同步 Hz、传感器 Hz、GPU/CPU 占用和机器人是否出现接触发散。G1 依次验收
URDF/MJCF 装载、站立、前进和 PICO 遥操。

当前单一启动入口：

```bash
cd /home/trna/matrix-eval
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
MATRIX_UE_MAX_FPS=60 \
  bash scripts/run_matrix_sonic.sh \
    --scene 21 \
    --urdf /home/trna/matrix-sonic-assets/g1_29dof/g1_29dof.urdf \
    --control-source planner \
    --walk-after 10 \
    --vx 0.25 \
    --max-seconds 60
```

状态写入 `outputs/matrix_sonic_status.json`，物理日志写入
`outputs/logs/matrix_sonic_runtime.log`。其中 `ue_state_sync_hz`/兼容字段
`render_hz` 是发给 UE 的状态同步频率，不等同于 GPU 实际绘制 FPS；实际 UE FPS
仍需使用 `stat fps` 或外部采样确认。

2026-07-15 TRNA 纯物理行走探针结果：有效 lowcmd 33.48 秒，吊带按 4+3 秒完全
退出，`physics_step_hz≈200`、`ue_state_sync_hz≈50`、`rtf≈1.0`、
`instability_resets=0`。机器人在前进命令后从 `x≈0.02 m` 行至
`x≈1.75 m`，根高度保持约 `0.79 m`，随后在 Apartment 碰撞边界前保持站立。

最终集成提交 `45d022af88218185da11b5a2747c9c7c967117b1` 的完整 Matrix+UE
90 秒 smoke 正常退出并自动清理所有本次仿真子进程。退出前状态样本包含
66.941 秒有效 lowcmd：`physics_step_hz=200.307`、
`ue_state_sync_hz=50.077`、`rtf=1.0015`、`fall_detected=false`、
`instability_resets=0`、平面位移 1.76436 m。UE 日志帧计数器在 84 个稳态约一秒
窗口中的均值为 56.618 FPS、中位数为 56.331 FPS，范围 54.217--59.898 FPS，
上限设置为 60 FPS。退出后仿真使用的 5556、5860、5861 和 9999 端口均无残留
监听进程。

视觉截图保存在
`outputs/screenshots/matrix_sonic_apartment_walk_20260715.png`。ApartmentWorld 的
材质、家具和光照正常，机器人姿态与物理状态同步；但 G1 当前仍是灰色工程材质，
机器人本体的视觉还原度低于场景。结构化证据见
`research/sonic_integration/results/trna_apartment_sonic_20260715.json`。

审计分支保持 UE 默认上限 30 FPS，但允许在同一主启动链路中打开原生统计层：

```bash
MATRIX_UE_EXTRA_EXEC_CMDS="stat fps,stat unit" \
  bash scripts/run_sim.sh xgb 9 0 0 1
```

如需做渲染上限对照，可设置 `MATRIX_UE_MAX_FPS=60`；不设置时行为与上游一致。

## 复现清单校验

```bash
python3 scripts/validate_native_scene_inventory.py
python3 scripts/validate_native_scene_inventory.py \
  --manifest https://github.com/zsibot/matrix/releases/download/v0.1.2/manifest-0.1.2.json
python3 -m unittest tests/test_native_scene_inventory.py
```

已发现 0.1.2 的 `assets-0.1.2.tar.gz` 元数据不一致：release manifest 和 GitHub
asset digest 为
`c4af445e468bb919909176113a8b8e9b0ede5dabf80be7112a5e7e86085bb369`，但总
checksums 文件写的是
`3099ea010e400b4c16f5765fad6a465687d97e52a9ca49177800b3e67872b52b`，release
notes 又写成
`ec18164489775c9f5ac1f73da4790c8c5d18f701cddf48ec1a8e8d7b3861e8fc`。
安装时应以 manifest、GitHub digest 和实际文件三方一致为准，并把该问题视为
可复现性缺陷。
