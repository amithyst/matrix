# ADR 0002: 行星 PAK 与可编辑 UE 工程边界

- 状态：Proposed，等待 MoonWorld live smoke 后转 Accepted
- 日期：2026-07-22
- 适用：Matrix 0.1.2、UE 5.5.4 Linux、SOL-2080 Moon/Mars world

## 背景

Matrix 把 MuJoCo 物理和 UE 视觉分开。官方 release 提供可直接运行的 cooked
IoStore 地图包，但没有公开 Matrix 的 `.uproject`、原始 `.umap`、Blueprint 图、材质工程
或游戏模块源码。与此同时，我们确实可以从 NASA/PDS、CARLA、Cesium 等公开来源取得
原始数据或有明确许可证的代码。

这里必须区分三件不同的东西：

1. **官方 cooked PAK**：运行成品，不是编辑源文件；
2. **我们自己的可编辑内容工程**：可以把公开原始资产做成新的 content-only PAK，不需要
   Matrix 原始工程；
3. **可重编译 renderer 工程**：修改 Matrix 游戏模块、现有 Blueprint 或加入 C++ 插件时
   才需要，优先取得 Matrix 原工程，否则替换为我们拥有源码的 renderer。

## 决策

采用三层混合架构，不把“公开下载”“开源代码”“可编辑 UE 原资产”混为一谈。

### L0：官方世界 PAK 原样复用

适用于已经满足视觉要求的 Matrix 原生地图。PAK 保持上游字节不变，按 URL、size、
SHA256 和 UE/platform 版本锁定，通过受监督的 `Saved/Paks` 临时目录挂载；基础
`Content/Paks` 不变，退出后删除 active 副本。

MoonWorld 第一版走这一层：

- `MoonWorld-0.1.2.tar.gz` 只包含 `pakchunk26-Linux.pak/.utoc/.ucas`；
- 主链已经把 `--scene 15` 映射到 `/Game/Maps/MoonWorld`；
- G1 visual 和 SONIC physics 仍由现有 custom robot/MuJoCo 链负责；
- 现有 centered-camera overlay 已证明 Matrix 0.1.2 会从 `Saved/Paks` 挂载受控 IoStore
  容器，因此 world bundle 应复用同一生命周期，不发明第二套 launcher。

L0 可以做：选择官方地图、加载现有材质/灯光、使用已有机器人类、通过现有
`-ExecCmds` 调整被 UE reflection 暴露的全局属性和 CVar。

L0 不可以可靠做：编辑 MoonWorld 内部 Actor、重连 Blueprint 图、替换地图引用、增加
Matrix executable 未编译的插件、提供新的权威 renderer readback。

### L1：自有 UE 5.5.4 content-only 兼容工程

为新的火星地图和我们拥有来源的视觉资产建立一个小型、可编辑、可复现的 UE 内容工程。
这不是 Matrix 私有原工程，也不包含 Epic Engine 源码。它只使用 Matrix executable 已经
具备的 Engine 标准类，面向 Linux，以 `/Game` mount point 和 UE 5.5.4 IoStore cook
独立 PAK。

第一版 MarsWorld 采用预烘焙区域，不在 runtime 依赖 Cesium C++：

- MOLA MEGDR/PDS 提供火星 DEM；区域影像使用 PDS/NASA 的明确产品；
- GDAL 将同一 DEM 确定性地产生 UE Landscape/mesh 输入和 MuJoCo heightfield/collider；
- UE 使用内置 DirectionalLight、SkyAtmosphere、SkyLight、PostProcessVolume 和材质；
- 每个原始资产记录产品 ID、URL、尺寸、SHA256、引用方式、许可/使用条款和处理命令；
- 禁止手搓纯色地表；最终地形必须有真实影像或经过来源锁定的 PBR 纹理；
- build 输出是版本化 PAK bundle，不把 Engine、缓存和大体积临时 cook 目录提交到 Git。

L1 也可制作一个仅含天空、灯光或装饰 Actor 的 additive overlay。是否能覆盖/包装某个
官方地图，必须以 exact 5.5.4 live mount 和像素证据为准；不得通过手改官方 `.uasset`
来冒充可复现源码。

### L2：source-built renderer

以下需求超出 content-only PAK，必须让相关 C++/Blueprint 成为可构建源：

- 把 Cesium for Unreal/Cesium Native 插件编进 executable，实时流式加载整颗行星；
- 动态接收 SOL-2080 历表并控制 DirectionalLight、SkyAtmosphere、曝光、星空及最终
  renderer readback；
- 同进程 Earth/Moon/Mars 无缝 travel、World Partition、origin shifting 和资源卸载；
- 修改 Matrix 已有 GameMode、PlayerCameraManager、机器人 Blueprint、传感器或 UI；
- 新增本地 IPC/C++ bridge，或要求像素、相机和传感器状态有权威逐帧回读。

L2 优先向 Matrix 维护方取得与 0.1.2 PAK 匹配的工程和构建说明。如果不可得，则建立
我们拥有源码的 renderer，并继续消费已经公开的 MuJoCo/UDP 状态协议；不能长期依赖
build-ID 偏移、反编译 Blueprint 或不可重现的二进制 patch。

## 选择表

| 需求 | 直接官方 PAK | 自有 content 工程 | source-built renderer |
|---|---:|---:|---:|
| 原样运行 MoonWorld | 是 | 否 | 否 |
| G1/SONIC 200/50 Hz 控制 | 是，物理链独立 | 是，物理链独立 | 是 |
| 设置 MuJoCo 月球重力 | 物理链独立 | 物理链独立 | 物理链独立 |
| 新建预烘焙 Mars 区域 | 否 | 是 | 否 |
| 新增静态天空、灯光和材质 | 有限 | 是 | 否 |
| 修改 MoonWorld 内部 Blueprint/Actor | 否 | 只能包装，不能改原件 | 是 |
| Cesium runtime/整星流式地形 | 否 | 否 | 是 |
| 动态天体光照与最终像素 readback | 否 | 静态/有限 | 是 |
| 无缝跨星体 travel/origin shifting | 否 | 否 | 是 |

## 许可证与资产边界

- `zsibot/matrix` 代码仓库声明 BSD-3-Clause；这不自动证明 release PAK 内每个第三方
  Fab/CARLA 资产都以 BSD 源资产形式发布。官方文档明确 shared 包含 Fab/CARLA 资源。
- UE Engine 源码对许可用户可见，但受 Epic EULA 管理，不是 OSI 意义的开源项目。
- 直接使用上游 MoonWorld PAK 比提取、修改和重新分发其中资产更清楚；本方案不反打包
  MoonWorld 来建立衍生源工程。
- NASA 内容通常不受美国版权保护，但仍需遵守标识、人物、第三方内容和不背书限制，
  并标注 NASA/PDS 来源。每个具体产品仍单独审计。
- `retoc`、UAssetGUI 等工具自身开源，并不授予其处理对象的资产权利；只用于诊断和
  round-trip 验证，不作为行星内容的权威创作流水线。

## 物理边界

PAK 只解决画面。Matrix 官方 `scene_terrain_moon_dynamic.xml` 没有声明月球重力，MuJoCo
会使用默认地球重力；其中 256 个动态地块通过 `gravcomp=1` 抵消自身重力。正式 Moon
验收必须在派生 G1 模型中显式写入 `gravity=[0,0,-1.62]`，并确认 SONIC policy、接触、
跌倒门禁和 benchmark observation 对低重力有效。不能因为画面是月球就把物理标成月球。

Mars 同样要求视觉 mesh/texture 与 MuJoCo heightfield/collider 来自同一个锁定 DEM，保存
坐标变换、垂直基准、分辨率、误差和对齐测试。

## 分阶段验收

### Phase A：Moon PAK-first

1. 把官方三件套下载到外部只读 bundle，校验 release SHA 和解包文件清单；
2. 新增 world-bundle contract，受监督地挂载到 `Saved/Paks/MatrixWorldActive`；
3. 用主入口 `run_matrix_sonic.sh --scene 15` 启动，不建立旁路脚本；
4. 日志必须确认本次 Found/Mounted chunk 26，地图加载无 missing package/material；
5. 先做 Earth-gravity visual smoke并明确标签，再做 1.62 m/s² physics acceptance；
6. 退出后 active bundle 清理，基础 PAK tree hash 不变。

### Phase B：Mars content-only PAK

1. provision 精确 UE 5.5.4 Linux Editor/commandlet；
2. 建立可 Git 管理的项目骨架、原始资产 manifest 和确定性 import/cook 脚本；
3. 先做一个 2-4 km 的 Utopia Planitia 区域，不虚构整颗火星已完成；
4. 验证独立 PAK 在 Matrix executable 中 mount/open、G1 出现、材质完整；
5. 用同源 DEM 的 MuJoCo 物理跑 SONIC 50 Hz、physics 200 Hz、RTF、碰撞和相机截图；
6. 通过后再注册 `mars.utopia`，此前目的地保持 disabled。

### Phase C：大世界 renderer

只有 L0/L1 无法满足连续航行、整星 streaming 和动态 renderer readback 时启动。先取得
Matrix source/build contract；不可得时再立项 clean renderer，避免同时维护两个未经验证的
半成品 UE 路径。

## 当前约束

已检查的 Spark、Heyuan、ZZA 当前均未发现 UnrealEditor、UnrealEditor-Cmd、RunUAT 或
UnrealPak。
Heyuan 正有另一条 Matrix creative-inventory live 运行，不能为了本 ADR 中断它。因此
Phase A 的 chunk 26 live smoke 和 Phase B 的 cook 环境 provision 都是下一执行阶段，本文
不宣称它们已经通过。

## 固定参考

- [Matrix v0.1.2 release](https://github.com/zsibot/matrix/releases/tag/v0.1.2)
- [Epic：Cooking Content](https://dev.epicgames.com/documentation/en-us/unreal-engine/cooking-content-in-unreal-engine)
- [Epic：Patching、Content Delivery 与 DLC](https://dev.epicgames.com/documentation/en-us/unreal-engine/patching-content-delivery-and-dlc-in-unreal-engine)
- [Epic：Cooking Content 与 Chunks](https://dev.epicgames.com/documentation/en-us/unreal-engine/cooking-content-and-creating-chunks-in-unreal-engine)
- [NASA CGI Moon Kit](https://svs.gsfc.nasa.gov/4720/)
- [PDS MGS MOLA MEGDR](https://pds-geosciences.wustl.edu/missions/mgs/megdr.html)
- [PDS LRO LOLA](https://pds-geosciences.wustl.edu/missions/lro/lola.htm)
- [NASA Images and Media Usage Guidelines](https://www.nasa.gov/nasa-brand-center/images-and-media/)
- [Cesium for Unreal](https://github.com/CesiumGS/cesium-unreal)
- [retoc](https://github.com/trumank/retoc)
