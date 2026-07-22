# ADR 0001: Matrix 先做箱庭世界

## 状态

Accepted, 2026-07-22.

## 背景

Matrix v0.1.2 提供了多个官方 cooked UE 地图和对应 MuJoCo 物理场景，但公开仓库没有匹配的
UE 可编辑工程、`.umap` 源文件或稳定的运行时多地图位移合成能力。直接宣称城市、月球、火星
组成无缝大世界，会把视觉、物理、坐标系和存档语义混在一起，验收不可控。

## 决策

Matrix/SONIC 近期按箱庭世界推进：

- 每个箱庭使用官方可锁定资产、独立启动入口和独立 acceptance。
- `MoonWorld` 作为第一版非地球箱庭，启动入口是 `scripts/run_matrix_sonic_moon_v1.sh`。
- 城市、月球等场景之间可以后续通过 ESC/ECS 命令传送，但每个目的地必须有明确 scene/map 路由。
- 在低重力物理模型完成前，月球只能标注为 `Moon visual smoke` 或 `Earth-gravity MoonWorld smoke`。
- MoonWorld V1 的 SONIC 物理模型保留官方月面碰撞块，但把这些地块的 freejoint 静态化，
  不宣称完成动态月壤 benchmark。
- 真正连续太阳系保留为长期方向，需要源 UE 工程、多坐标系运行时、天体时间系统和跨场景状态迁移。

## 影响

箱庭路线可以先得到可复现、可截图、可训练链路复用的场景验收，不阻塞未来开放世界设计。后续如果拿到
可编辑 UE 工程，可以把已验收箱庭逐步升级为同一宇宙坐标系下的连续世界。
