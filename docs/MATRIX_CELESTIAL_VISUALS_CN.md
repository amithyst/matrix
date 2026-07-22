# Matrix SOL-2080 星体视觉 Profile

## 这版解决什么

天体系统负责回答“太阳和星球此刻在哪里”，视觉 profile 负责回答“渲染器应使用哪组可复现
参数”。两者分开后，场景不会再依赖某台机器上手调但没有记录的天气状态。

当前 profile 只控制 CARLA weather 可公开读写的 14 个字段：太阳高度/方位来自 DE440s 或
解析 fallback，其余云、雨、积水、风、雾、湿度、大气散射和尘暴来自 Git 管理的
`config/universe/celestial-visual-profiles-v1.json`。每个 profile 都会计算 SHA256，并随
ESC 星体状态发布。它不修改 MuJoCo 物理、SONIC 50 Hz 控制、碰撞、相机位姿或分辨率。

## Matrix 原生月球/火星资产核对

截至 2026-07-22，Matrix 官方资产与当前 SOL-2080 箱庭世界运行状态必须分开理解：

| 星体 | Matrix 官方资产 | Heyuan 当前安装 | SOL-2080 当前状态 |
|---|---|---|---|
| 月球 | **有**。官方地图 ID `15`、UE 地图 `/Game/Maps/MoonWorld`、MuJoCo 场景 `scene_terrain_moon_dynamic.xml`；v0.1.2 正式包 `MoonWorld-0.1.2.tar.gz` 为 `633678813` bytes，SHA256 `c4e3dee47ffa434712b0238d08b0b68067f1b1c9820e2ddb455f996f04e364b1`，对应 chunk `26` | runtime lock 已纳入 `MoonWorld`、`dynamicmaps/moonworld.bin` 和月球 XML；每台机器仍需重新 bootstrap 安装 chunk 26 | `moon-tranquility-outpost` 仍为 `planned/disabled`，等待 MoonWorld 主链路 live smoke、G1/SONIC 物理、出生点、截图和低重力边界验收后再提升 |
| 火星 | **没有找到**。官方地图文档、启动菜单、v0.1.0-v0.1.2 release/manifest 和 ZsiBot 公开源码均无 `MarsWorld` | 无 | 火星开发已暂停；`mars-utopia-outpost` 只是 backlog 目录项，不是 Matrix 原生地图，必须保持 `planned/disabled` |

官方证据：

- [Matrix 地图文档（固定 revision）](https://github.com/zsibot/matrix/blob/5b5559476ef963cccc39411cd2eb3b836df08343/docs/Robots_and_Maps.md)
- [Matrix v0.1.2 manifest](https://github.com/zsibot/matrix/releases/download/v0.1.2/manifest-0.1.2.json)
- [MoonWorld v0.1.2 官方包](https://github.com/zsibot/matrix/releases/download/v0.1.2/MoonWorld-0.1.2.tar.gz)
- [Matrix v0.1.2 release](https://github.com/zsibot/matrix/releases/tag/v0.1.2)

因此“Matrix 有月球地图”和“当前 ESC 可以传送到月球”不是同一件事。箱庭世界第一阶段应复用
官方 MoonWorld，不重做月面；但要先把 chunk 26 加入 runtime lock 并在隔离 profile 安装，
再完成 SONIC 物理、低重力、相机和持久化验收。具体命令见
[MoonWorld 箱庭 V1](MOON_V1_CN.md)。火星暂不推进，后续恢复时再另选有来源和
许可证的地形资产。

## Matrix 公开 UE 源码核对

ZsiBot 组织当前公开仓库中没有 `.uproject`、`.uplugin` 或 Matrix UE 游戏模块源码；
`zsibot/matrix` 发布的是启动脚本和 cooked UE PAK，`MATRiX_Python_SDK` 只公开 MuJoCo
加载及 UDP 状态同步协议，并不能编辑 MoonWorld、天空、灯光或材质。公开 fork 也不能替代
原始可编辑工程。

Matrix README 引用的
[MuJoCo-Unreal-Engine-Plugin](https://github.com/oneclicklabs/MuJoCo-Unreal-Engine-Plugin)
确实含 UE 工程和 C++ 插件源码，但 GitHub 当前未识别到许可证文件；在取得明确授权前，
不能直接复制其代码作为我们的可维护实现。

这不意味着所有新资产都要等待 Matrix 原工程。当前采用三层方案：官方 MoonWorld 直接
作为不可变 PAK 使用；后续自有天空/材质由我们自己的 UE 5.5.4 content-only 兼容工程
从 NASA/PDS 等有来源的原始资产 cook 成独立 PAK；只有 Cesium C++、动态 renderer bridge、
修改 Matrix 现有 Blueprint 或无缝跨星体 travel 才要求 source-built executable。近期产品
目标见 [ADR 0003](adr/0003-box-world-first.md)：先做可验证箱庭世界，再评估无缝宇宙。
完整选择边界和验收门禁见 [ADR 0002](adr/0002-planetary-pak-source-boundary.md)。

## 开源选型

| 方案 | 许可证 | 当前用途 | 接入结论 |
|---|---|---|---|
| CARLA 0.9.15 WeatherParameters | MIT | 云雾、降雨、湿地、散射、尘暴与太阳角 RPC | 已接入；完整字段写入后逐项 readback |
| Bruneton atmospheric scattering | BSD-3-Clause | Earth/Mars 物理大气散射 | 等可构建 UE 工程后做 renderer 插件 |
| Cesium for Unreal / Cesium Native | Apache-2.0 | 行星坐标、georeference、origin shifting | v2.28.0 已支持非 WGS84 椭球并自带 IAU2015 Moon/Mars ellipsoid；作为远期 source-built 无缝宇宙地形层，不进入当前 cooked 包 |
| NASA/USGS、Natural Earth、Poly Haven、ambientCG | 逐资产公共领域或 CC0 | 行星贴图、DEM、HDRI、PBR 材质候选 | 只有明确挂载点、来源、许可证和 SHA256 后才入资产锁 |

没有采用静态 HDRI 充当太阳真值。HDRI 内烘焙的太阳容易与 DirectionalLight、阴影和
2080 场景时间冲突，也会让 AI 数据中的光照 provenance 失真。没有复制 GPL 的 Celestia
渲染代码，也没有引入商业天空插件。

## 已锁定 Profile

| ID | 用途 | 来源/状态 |
|---|---|---|
| `earth-wet-cloudy-v1` | 地球 Overworld 默认，阴云和积水更适合城市画面 | CARLA 原生 `WetCloudyNoon` |
| `earth-clear-v1` | 晴天基准与视觉 benchmark 对照 | CARLA 原生 `ClearNoon` |
| `earth-soft-rain-v1` | 小雨、路面积水和感知压力测试 | CARLA 原生 `SoftRainNoon` |
| `moon-vacuum-v1` | 月球真空兼容参数 | 仅 renderer adapter；月球地图仍未部署 |
| `mars-dust-v1` | 火星尘暴兼容参数 | CARLA 原生 `DustStorm`；不是最终 CO2 大气模型 |

地球默认采用 wet-cloudy，但默认 bridge 仍是 `state-only`。这是有意的：没有 CARLA RPC
或 Python API 的机器不会伪装成“画面已生效”。

## 启动

从原有主启动链路选择 profile：

```bash
cd /home/kaijie/matrix
bash scripts/run_matrix_sonic_urban_v1.sh \
  --profile heyuan \
  --control-source game \
  --celestial-lighting-bridge carla-weather \
  --celestial-visual-profile earth-wet-cloudy-v1
```

晴天 A/B：

```bash
bash scripts/run_matrix_sonic_urban_v1.sh \
  --profile heyuan \
  --control-source game \
  --celestial-lighting-bridge carla-weather \
  --celestial-visual-profile earth-clear-v1
```

profile 文件会在 UE/SONIC 启动前严格校验。未知字段、重复 JSON key、NaN、越界值、错误
星体/大气组合或未知 profile 都会拒绝启动。运行时 RPC 在后台线程执行，不阻塞 50 Hz
控制；任何缺失字段、写入异常或 readback 不一致都会降级为 `state-only`。

## 验收边界

`render_status=applied` 只证明 CARLA 返回了完整一致的 weather 参数；
`visible_camera_verified` 在当前版本固定为 `false`。最终视觉验收还需要：

CARLA weather 没有本项目所需的 DirectionalLight lux、日食调光、自动曝光或星空旋转
readback；当前虽发布辐照度、食分和星空可见度真值，但不会伪称这些量已改变最终像素。

1. Matrix UE 可编辑工程或确认可用的 CARLA RPC 服务；
2. 主相机原分辨率下的 day/twilight/night 截图；
3. 云、阴影、积水、曝光和星空的像素检查；
4. 同时记录 profile ID、SHA256、场景时间和最终 renderer readback；
5. 确认没有改变 MuJoCo/SONIC 物理频率、碰撞和 benchmark 观测定义。

Heyuan 当前 Matrix 0.1.2 cooked bundle 未发现 CARLA Python 模块、RPC listener 或 UE 源
工程，因此只能验证配置、协议和 fail-closed 行为，不能宣称 wet-cloudy 最终相机画面已
通过。后续拿到 source-built UE 后，应先实现 DirectionalLight、SkyAtmosphere、SkyLight、
VolumetricCloud、曝光和星空的可读回插件，再锁 NASA/CC0 视觉资产。
