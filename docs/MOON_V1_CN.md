# Matrix MoonWorld 箱庭 V1

## 目标

`moon-v1` 是 Matrix/SONIC 在 SOL-2080 箱庭世界中的第一版非地球场景。它复用 Matrix v0.1.2 官方
`MoonWorld` cooked PAK，不重做月面资产，也不反打包编辑上游内容。启动链路仍然走
`scripts/run_matrix_sonic.sh`，`scripts/run_matrix_sonic_moon_v1.sh` 只是固定
`--scene 15` 的薄包装。

## 锁定资产

runtime lock 纳入这些官方 MoonWorld 资产：

- `MoonWorld-0.1.2.tar.gz`
  - size: `633678813`
  - SHA256: `c4e3dee47ffa434712b0238d08b0b68067f1b1c9820e2ddb455f996f04e364b1`
- `pakchunk26-Linux.pak/.ucas/.utoc`
- `dynamicmaps/moonworld.bin`
- `scene_terrain_moon_dynamic.xml`

bootstrap 会从 `config/runtime/matrix-sonic.lock.json` 自动安装所有锁定地图包，不再写死只装
`Town10World`。

## 启动

MoonWorld 的 UE 视觉仍使用官方 `MoonWorld` cooked map 和 `dynamicmaps/moonworld.bin`。
SONIC 物理侧会在派生模型中应用 `moon-dynamic-ground-mocap-v3`：将官方
`scene_terrain_moon_dynamic.xml` 的 256 个月面碰撞块转换成 mocap body，保持 G1 的
robot-only 物理状态为 `nq/nv/nu=36/35/29`；启用 creative inventory 后物理状态会按
通用 item 自由度扩展，但发给 UE 的机器人 render ABI 仍投影为 `36/35/29`。运行时会在
每个 MuJoCo step 前按照机器人当前 XY 将这组 `16 × 16` 碰撞块滚动到脚下。块高度直接读取
锁定的 `dynamicmaps/moonworld.bin` 原始绝对高度，
量化、像素索引和边界裁剪均与官方 MoonWorld 算法一致。

V3 不再添加固定高度的无限支撑平面。出生高度、跌倒门禁、恢复门禁、存档安全审计都使用
`root_z - local_ground_z`，避免月面绝对高度随位置变化时出现悬空、误报跌倒或保存无支撑
坐标。默认中心点地面高度为 `-0.9296965m`，G1 默认 root clearance 为 `0.793m`，因此
无存档时默认 root z 约为 `-0.1366965m`。

Heyuan：

```bash
cd /home/kaijie/matrix
bash scripts/run_matrix_sonic_moon_v1.sh \
  --profile heyuan \
  --control-source game
```

无人工输入 smoke：

```bash
cd /home/kaijie/matrix
bash scripts/run_matrix_sonic_moon_v1.sh \
  --profile heyuan \
  --control-source planner \
  --walk-after 10 \
  --vx 0.20 \
  --max-seconds 45
```

## 截图/视频验收

优先复用主录制脚本，不走旁路启动：

```bash
cd /home/kaijie/matrix
bash scripts/record_matrix_sonic_video.sh \
  --output outputs/acceptance/moon-v1/moonworld-smoke.mp4 \
  --metadata outputs/acceptance/moon-v1/moonworld-smoke.json \
  --duration 12 \
  --fps 30 \
  --notes "MoonWorld box-world smoke" \
  -- \
  bash scripts/run_matrix_sonic_moon_v1.sh \
    --profile heyuan \
    --control-source planner \
    --walk-after 5 \
    --vx 0.15 \
    --max-seconds 25
```

验收材料至少保留：

- 命令行；
- Matrix commit；
- runtime lock SHA；
- `MoonWorld-0.1.2.tar.gz` SHA；
- screenshot/video 路径、尺寸、时长、SHA；
- UE 日志中 `/Game/Maps/MoonWorld` 和 chunk 26 的加载证据；
- SONIC status 中 physics Hz、RTF、fall/reset 状态。

## 当前边界

MoonWorld 视觉通过不等于月球低重力物理通过。Matrix 官方
`scene_terrain_moon_dynamic.xml` 不声明 `gravity=[0,0,-1.62]`，正式低重力 acceptance 需要
单独派生物理配置，并验证 SONIC 控制、接触、跌倒门禁和 benchmark observation。低重力
通过前，报告中必须写成 `Moon visual smoke` 或 `Earth-gravity MoonWorld smoke`，不能写成
完整月球动力学 benchmark。

V1 的 SONIC 物理验收也不宣称保留官方动态月面块的动力学自由度；这些 freejoint 地块在派生
物理模型中会被转换成确定性 mocap 地块。manifest 会记录
`scene_transform=moon-dynamic-ground-mocap-v3`、锁定高度图的 SHA/尺寸/分辨率以及
`scene_transform_contract.dynamic_ground.update_timing=before_each_mj_step`。V3 使用
独立 world revision，旧 static-v2 checkpoint 不会被继续加载。

高度图必须是普通、非符号链接文件，并通过锁定的尺寸、SHA256 和有限值校验。任一校验失败、
256 个地块映射不完整或运行时更新失败，MoonWorld 启动都会 fail closed。运行状态会记录
`local_ground_z_m`、`root_clearance_m`、最小 clearance 和动态地面 telemetry，便于定位
贴地与性能问题。

当前 V1 是箱庭入口，不宣称从城市地图无缝步行到月球。跨星体入口可以后续接到 ESC/ECS
传送命令，但需要每个目的地都有自己的场景启动路由、物理模型和验收记录。ESC 星体导航页
当前只作为同一 SOL-2080 宇宙下的箱庭目的地入口：月球支持直接启动和本箱庭内传送点；从
城市箱庭一键跨场景切换到 MoonWorld 仍需扩展 destination 的 scene/map 路由，不计入本 V1
已完成功能。
