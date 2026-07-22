# Matrix MoonWorld 箱庭 V1

## 目标

`moon-v1` 是 Matrix/SONIC 箱庭世界的第一版非地球场景。它复用 Matrix v0.1.2 官方
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

当前 V1 是箱庭入口，不宣称从城市地图无缝步行到月球。跨星体入口可以后续接到 ESC/ECS
传送命令，但需要每个目的地都有自己的场景启动路由、物理模型和验收记录。
