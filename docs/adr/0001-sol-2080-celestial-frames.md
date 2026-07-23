# ADR 0001: SOL-2080 时间、坐标与光照真值

- 状态：Accepted for v2 scaffold
- 日期：2026-07-22
- 范围：导航坐标、星体运动、地表锚点、太阳光照状态

## 决策

SOL-2080 使用一套连续宇宙真值，但近期产品形态是箱庭世界，不把地球、月球和火星实现成
互不相关的存档，也不宣称当前 cooked runtime 已经支持无缝跨星体飞行。全局导航真值为
`sol_heliocentric_icrf`，单位米；每个可运行箱庭继续在星体表面的局部切平面中使用小坐标。
任意局部点在时刻 `t` 的全局位置为：

```text
p_sol(t) = r_body(t) + R_body_to_icrf(t) * p_body_fixed
```

传送目标保存 `body_id + latitude + longitude + altitude + heading + local pose`。传送时按到达
时刻重新求解，不把数亿米或数千亿米的全局坐标写进 MuJoCo root、UE Actor 或 SONIC
观测。月球在地图、碰撞、重力、入口点和运行路由验收前继续保持 `planned`；火星开发已暂停，
只保留 disabled/backlog 目录项。

权威时间使用整数 TAI 纳秒。当前场景纪元为 `2080-01-01T00:00:00Z`，并冻结
`TAI-UTC=37 s` 作为虚构世界的显示约定；现实世界 2080 年可能新增闰秒，因此这里的 UTC
标签不能声称是未来 IERS 真值。互动运行以 monotonic clock 推进，并原子保存到 profile 的
宇宙时钟文件；测试和数据回放可以注入固定时间。

## 开源方案审计

| 项目 | 许可证 | 采用方式 | 结论 |
|---|---|---|---|
| [NASA NAIF SPICE](https://naif.jpl.nasa.gov/naif/) / [SpiceyPy](https://github.com/AndrewAnnex/SpiceyPy) | NASA kernels/toolkit terms；SpiceyPy MIT | 最终高精度历表接口 | 目标方案；运行时必须离线固定 kernel、覆盖期和 SHA256 |
| [Skyfield](https://github.com/skyfielders/python-skyfield) | MIT | Python 历表与 planetary constants 参考 | 易用，但官方文档明确指出行星地固 frame 支持不完整，不作为唯一真值 |
| [Cesium Native](https://github.com/CesiumGS/cesium-native) / [Cesium for Unreal](https://github.com/CesiumGS/cesium-unreal) | Apache-2.0 | 椭球、地理坐标、原点重定位和 UE 集成参考 | 与 Matrix 的局部物理/远期无缝 renderer 分层一致；审计 revision `f71495e9` |
| [CARLA](https://github.com/carla-simulator/carla) | MIT | 当前 cooked Matrix 的 weather RPC 候选 | 采用 0.9.15 commit `d7b45c1e` 的原生 preset/14 字段协议；完整 readback 不等于最终相机验收 |
| [Bruneton atmosphere](https://github.com/ebruneton/precomputed_atmospheric_scattering) | BSD-3-Clause | Earth/Mars 物理大气散射参考 | 适合后续可控 UE shader/plugin，不直接塞进当前 cooked 包；审计 revision `34f14e74` |
| [OpenSpace](https://github.com/OpenSpace/OpenSpace) | MIT-style repository license | 星体 frame graph 和科学可视化参考 | 只借鉴架构，不复制运行时代码 |
| [Celestia](https://github.com/CelestiaProject/Celestia) | GPL-2.0 | 视觉与目录设计参考 | GPL 边界不适合直接复制到当前仓库 |

可审计的视觉数据优先考虑 NASA/USGS 单资产明确标注的公共领域数据、Natural Earth
公共领域地图，以及 Poly Haven / ambientCG 的 CC0 材质。任何纹理、DEM、3D Tiles 或
星表进入运行锁前，必须逐资产记录来源、许可证、版本、大小和 SHA256；不能因为来源是
NASA 或开源仓库就默认所有附件都可再分发。

## 当前实现

默认优先使用锁定的 `jpl-de440s-v1`：DE440s 只作为 32 MB 外部 runtime asset，jplephem
2.23 以 49 KB 的纯 Python wheel 直接挂载，不污染 SONIC 的严格 Python environment。
三机统一执行：

```bash
bash scripts/bootstrap_matrix_celestial.sh
```

launcher 检测到两个 SHA256 均通过的文件后自动使用 DE440s；缺失时才退到
`matrix-analytical-v1`。解析 fallback 使用 JPL 长周期平均轨道根数以及 IAU uniform
rotation 近似，覆盖 Sun/Earth/Moon/Mars。两种 provider 都可以确定性地产生：

- 星体中心位置、公转和自转 frame；
- 地理坐标到 body-fixed、局部 Matrix 坐标到 ICRF 的变换；
- 太阳高度、方位、距离、逆平方辐照度、视半径、星体遮挡和星空可见度；
- 跨冷重启持续的 TAI 场景时间；
- 版本化 CARLA visual profile，以及全部 weather 字段写入与 readback 校验。

`celestial-visual-profiles-v1.json` 锁定 CARLA 0.9.15 的 `ClearNoon`、
`WetCloudyNoon`、`SoftRainNoon` 和 `DustStorm` 参数；太阳角不使用 preset 常量，而由当前
历表状态覆盖。地球默认 `earth-wet-cloudy-v1`，并保留晴天/小雨 A/B。月球真空只定义
兼容 profile，不能替代尚未部署的月球箱庭和 UE 真空天空实现；火星尘暴 profile 暂时只作为
backlog 资产审计线索。每个 profile 都以
canonical JSON 计算 SHA256 并进入 ESC 状态，便于 AI 数据记录确切视觉条件。

CARLA RPC 在独立后台线程执行；50 Hz 输入/SONIC 控制线程只提交最新 profile，不等待 UE。
RPC 超时或 readback 不一致只把 `render_status` 降为 `unavailable`，不会阻塞遥操 deadman。

DE440s 在 `2080-01-01T00:00:00Z` 的锁定 Earth center 已加入回归测试。解析 fallback 与
DE440s 的同刻 center 误差约为 Earth 8,313 km、Moon 23,854 km、Mars 50,107 km，因此
fallback 只允许 ESC/预览继续工作，不能作为跨星体航行真值。即使使用 DE440s，当前
body-fixed rotation 仍是 IAU uniform 近似；月球物理天平动、Earth UT1/EOP、章动和
高阶 rotation 尚未接入，所以整体仍不用于航天器制导、掩食接触时刻、测绘或科学分析。

## SPICE 升级门

把当前 `jpl-de440s-v1` 提升为完整 `naif-spice-de440` 前仍必须完成：

1. 在已锁定 DE440s SPK 的基础上增加 PCK、Moon binary PCK/FK 和 LSK，并校验覆盖期。
2. 固定 SpiceyPy/CSPICE 版本及离线 wheel；三台开发机不得启动时联网取 kernel。
3. 对 Earth/Moon/Mars 做 provider 交叉测试、frame round-trip、surface anchor 和
   eclipse regression；记录解析 fallback 与 SPICE 的误差上限。
4. 明确未来 UTC 的冻结闰秒约定，不能把 LSK 的外推标签描述为真实 2080 UTC。
5. 通过 UE screenshot/readback 验收后，才把 `render_authority` 从 `state-only` 或
   `carla-weather` 提升为完整 SkyAtmosphere authority。

## 光影边界

当前原生 Matrix 地图材质、阴影、反射和曝光仍由 cooked UE 地图管理。启用
`carla-weather` 时更新太阳角、云、雨、积水、风、雾、湿度、散射和尘暴，并要求全部字段
读回一致；它仍不自动证明可见相机、SkyAtmosphere、月面真空天空或火星 CO2 大气正确。
CARLA weather 也没有本项目要求的 DirectionalLight lux、日食调光、自动曝光和星空旋转
readback，因此已计算的辐照度、食分与星空可见度仍只是状态真值。
完整方案需要可审计的 UE 插件
控制 DirectionalLight、SkyAtmosphere、SkyLight、VolumetricCloud、曝光和星空方向，并将
最终渲染参数写入每帧 AI 数据 provenance。

2026-07-22 在河源的 Matrix 0.1.2 cooked runtime 上未发现 `carla` Python 模块、监听中的
CARLA RPC 服务或可编辑 UE 工程。因此当前 `carla-weather` 是 fail-closed 的兼容入口，
不是已完成的可见画面功能。不能通过注入颜色、后处理滤镜或替换未知 cooked 资产来伪装验收。

## 视觉实施顺序

1. **当前 v2：**锁定历表、场景时间、太阳方向/辐照度/食分、星空可见度和版本化 CARLA
   visual profile 的完整 weather readback；这些状态可进入导航和 AI 数据，但不冒充最终像素真值。
2. **箱庭世界 v1：**把城市和 MoonWorld 作为边界清楚的可运行箱庭接入同一 ESC 目的地目录，
   保留 session、记录、world state 和 benchmark provenance，不宣称连续地形或无加载飞行。
3. **UE renderer v1：**取得可构建的 Matrix UE 工程后，以独立插件控制
   `DirectionalLight`、`SkyAtmosphere`、`SkyLight`、`VolumetricCloud`、自动曝光和星空旋转；
   Earth 使用 Bruneton/UE 的物理大气参数，Moon 使用真空黑天，Mars 使用稀薄 CO2/尘埃
   profile。每个设置都必须有 readback、截图和最终相机像素验收。
4. **无缝宇宙 v2：**按 Cesium georeference/origin-shift 模式接入地形和行星表面数据，渲染
   坐标与局部 MuJoCo/SONIC 物理坐标分层；纹理、DEM、星表和 3D Tiles 逐资产锁来源、许可、
   大小与 SHA256。
5. **科学 frame v3：**加入完整 SPICE PCK/FK/LSK、月球物理天平动和 Earth EOP；只有通过
   provider 交叉验证后才提升跨星体导航精度声明。
