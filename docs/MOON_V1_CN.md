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
SONIC 物理侧会在派生模型中应用 `moon-dynamic-ground-static-v1`：保留官方
`scene_terrain_moon_dynamic.xml` 的 256 个月面碰撞块，但移除这些地块自身的 freejoint，
让它们作为静态碰撞地形参与接触。这样 G1 的 SONIC body-only 状态维度保持可控，不把月面地块
的自由度混入机器人控制状态。

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
物理模型中被静态化，manifest 会记录 `scene_transform=moon-dynamic-ground-static-v1` 和
`staticized_freejoint_bodies`。

当前 V1 是箱庭入口，不宣称从城市地图无缝步行到月球。跨星体入口可以后续接到 ESC/ECS
传送命令，但需要每个目的地都有自己的场景启动路由、物理模型和验收记录。ESC 星体导航页
当前只作为同一 SOL-2080 宇宙下的箱庭目的地入口：月球支持直接启动和本箱庭内传送点；从
城市箱庭一键跨场景切换到 MoonWorld 仍需扩展 destination 的 scene/map 路由，不计入本 V1
已完成功能。
