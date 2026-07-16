# Matrix 都市场景 V1

## 定位

`urban-v1` 使用 Matrix 0.1.2 原生 `Town10World`（场景 2）作为第一版城市基底。
它有完整的道路、建筑、路灯、绿化、车辆和行人视觉，MuJoCo 侧有 869 个原生
静态环境几何体。它比 ApartmentWorld 或自制厂房更接近真实都市，但还不是经过
2080 科幻美术改造的 Overworld。

固定契约在 `research/urban_v1/scene.json`。原生人物和车辆目前只是 UE 展示资产，
没有对应的动态 MuJoCo body，不能宣称可物理交互或可作为动态障碍训练真值。

## G1 材质

通用导入流水线 V15 会识别标准 G1，并使用可追溯的 `aue_g1_v1` 三层材质：黑色
软胶、深灰结构件和暖白外壳。颜色、粗糙度、金属度、部件匹配规则及 Matrix UE
当前只渲染 RGBA 的边界均记录在 `docs/G1_FINAL_MATERIALS_CN.md`。旧 V14 缓存会
自动重建；材质转换不改变 collision、质量、惯量、关节或 SONIC 控制。

TRNA 验收结果位于
`research/urban_v1/results/trna_aue_g1_material_v15_20260716.json`：36 个来源
visual 覆盖 41 个实际视觉 geom，黑色 12、深灰 6、暖白 23、未匹配 0；Town10
录制为 1920x1080、30 FPS、10 秒，SONIC 物理约 200 Hz、RTF 约 1.0，无跌倒或
稳定性重置。另保存了完整机器人近景，避免使用右侧裁切的默认跟随视角判断材质。

## 启动

```bash
cd /home/trna/matrix-eval
bash scripts/run_matrix_sonic_urban_v1.sh \
  --urdf /home/trna/matrix-sonic-assets/g1_29dof/g1_29dof.urdf \
  --control-source planner \
  --walk-after 10 \
  --vx 0.25
```

录制仍走同一条主链路：

```bash
bash scripts/record_matrix_sonic_video.sh \
  --output outputs/videos/matrix_sonic_urban_v1.mp4 \
  --duration 15 --fps 30 \
  -- \
  bash scripts/run_matrix_sonic_urban_v1.sh \
    --urdf /home/trna/matrix-sonic-assets/g1_29dof/g1_29dof.urdf \
    --control-source planner --walk-after 10 --vx 0.25 --max-seconds 75
```

## 2080 层的边界

当前发布物只有 UE 5.5.4 cooked map，没有可编辑源工程。V1 保留真实原生城市，不用
纯色方块伪装科幻资产。拿到匹配的 UE 源地图后，再做夜间照明、远程机器人基础设施、
零子战争痕迹、星舰通信设施和可交互任务物件，并为新增动态对象补 MuJoCo/SONIC
物理与数据标注。
