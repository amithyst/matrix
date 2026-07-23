# ADR 0003: 箱庭世界优先

- 状态：Accepted
- 日期：2026-07-22
- 适用：Matrix SONIC 遥操、ESC 导航、MoonWorld 接入、overworld 兼容命名

## 背景

项目早期把 SOL-2080 描述为“开放大世界”或“太阳系大世界”。这适合长期叙事，但不适合
当前 Matrix cooked PAK + MuJoCo 物理链路的近期工程交付。公开 Matrix runtime 没有可编辑
UE 工程源码，也不能可靠把多个 cooked UE 地图合成一个无缝视觉世界。继续把近期目标写成
无缝大世界，会让场景验收、AI 数据 provenance 和三机复现边界变得含糊。

## 决策

近期产品形态改为 **箱庭世界**：

- 每个可运行地图是一个边界清楚的小世界，例如城市、仓库、月球表面或 3DGS 室内场景；
- ESC 面板提供同一 SOL-2080 宇宙下的目的地和传送点，不做“切换存档”；
- 传送可以通过完整冷重启进入另一个箱庭，但 session、世界时钟、记录语义和 benchmark
  provenance 保持统一；
- 每个箱庭必须独立验收 UE 视觉、MuJoCo 碰撞、重力、出生点、相机、SONIC 控制和记录；
- 不宣称跨箱庭存在连续物理碰撞、连续可见地形或无加载飞行，除非后续 source-built
  renderer 真正实现并通过像素与物理验收。

内部兼容命名暂时保留 `overworld`。它表示当前地球侧主场景/归航入口和已有脚本路径，
不是对“无缝开放大世界已经完成”的承诺。等接口稳定后，再单独评估是否把用户可见名称
迁移为 `boxworld`、`hub` 或更贴合世界观的名称。

## 当前范围

第一阶段只推进：

1. 地球城市箱庭：继续使用当前 Matrix/SONIC 主链路和已锁定的 Town10/urban 资产；
2. 月球箱庭：复用官方 `MoonWorld` cooked PAK，补齐 chunk 安装、主链路启动、相机和
   G1/SONIC smoke；
3. ECS/ESC 传送：从“星体导航”收敛为“箱庭目的地导航”，仍走 typed command 与完整冷重启；
4. 持久状态：按 host profile 保存 `home`、teleport point、last exit 和 SOL-2080 时钟；
5. AI 数据：每帧记录 scene/world id、destination id、visual profile、gravity profile、
   runtime lock 和资产 SHA。

火星开发暂停。`mars.utopia` 只能作为 disabled/backlog 目录项存在；在找到官方/内部可运行
包，或启动我们自有 UE 内容工程前，不做可达入口、不做伪场景、不做截图冒充。

## 为什么不是现在做无缝太阳系

无缝太阳系需要 source-built renderer 或我们拥有可编辑工程：

- World Partition 或等价的大尺度 streaming；
- origin shifting/georeference；
- 跨星体资源加载与卸载；
- 动态天体光照、SkyAtmosphere、星空和曝光 readback；
- UE 视觉坐标与 MuJoCo 局部物理坐标的强一致桥。

这些都超出当前 cooked PAK 能可靠支持的范围。箱庭世界先把“每个场景都真实、可交互、
可复现、可用于训练”做扎实，再把成熟箱庭接入更大的 renderer。

## 验收口径

一个箱庭从 `planned` 提升到 `active` 前，至少需要：

1. PAK 或 UE 资产 bundle 有来源、大小、SHA256 和安装路径；
2. MuJoCo XML/heightfield/碰撞代理与视觉地图边界匹配；
3. 重力和接触参数明确，月球必须单独验收 `1.62 m/s^2`；
4. 主启动链路能进入该箱庭，不走旁路脚本；
5. G1 最终材质、第三人称相机和 ESC 面板状态正常；
6. SONIC 50 Hz 控制、MuJoCo 物理频率、RTF、跌倒门禁和记录链路通过 smoke；
7. 截图/视频/日志能反向追溯到 commit、runtime lock、资产 SHA 和启动命令。

## 相关文档

- [ADR 0001: SOL-2080 时间、坐标与光照真值](0001-sol-2080-celestial-frames.md)
- [ADR 0002: 行星 PAK 与可编辑 UE 工程边界](0002-planetary-pak-source-boundary.md)
- [Matrix SOL-2080 星体视觉 Profile](../MATRIX_CELESTIAL_VISUALS_CN.md)
- [Matrix Overworld 相邻场景 V1](../OVERWORLD_ADJACENT_V1_CN.md)
