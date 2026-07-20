# G1 可见材质契约

## 当前默认配色

Matrix UE 0.1.2 的自定义机器人桥只把每个 geom 的 `FLinearColor` 传给动态材质。
旧 `aue_g1_v1` 虽然已经正确写入模型，但黑色、深灰和暖白全部是无彩色，在
Town10 的曝光和阴影下容易看成一整块纯灰。默认配置因此升级为
`config/materials/matrix_g1_v2.json`，专门为当前 RGBA-only 渲染链提供可辨识的
高对比外观：

| 表面 | 颜色 RGBA | 粗糙度 | 金属度 | 部件规则 |
| --- | --- | ---: | ---: | --- |
| 黑色软胶 | `0.018 0.024 0.035 1` | 0.62 | 0 | 骨盆、踝、腕、手部 |
| 石墨结构件 | `0.055 0.075 0.11 1` | 0.58 | 0 | 头、髋俯仰、腰部结构 |
| 冷白外壳 | `0.9 0.94 1 1` | 0.48 | 0 | 其余外壳 |
| Matrix 蓝点缀 | `0.015 0.2 0.95 1` | 0.42 | 0.08 | 骨盆轮廓、标识、髋偏航、膝、肩滚转、肘 |

规则按表格所述的 JSON 顺序匹配，蓝色点缀优先于包含范围更广的骨盆规则。配置
要求标准 G1 的九个关键 link 全部存在，普通自定义机器人不会误套用该材质。

`matrix_g1_v2` 以可追溯的 `aue_g1_v1` 为结构和部件分区基线，但新增蓝色表面并
主动提高明暗、色相对比，因此不再冒充 AUE 原始配色。旧配置继续保留，可用于历史
复现实验。

## 导入行为

通用 URDF 导入流水线 V18 默认使用 `auto`：

- 匹配标准 G1 link 签名时使用 `matrix_g1_v2`。
- 其他机器人继续忠实保留 URDF 的具名、全局或内联颜色。
- 每类表面生成确定性的 MJCF material，并把 `rgba`、`roughness` 和 `metallic`
  写入模型。
- UE 0.1.2 会把与 visual 重叠的 mesh collision 副本也画出来；V18 给副本写入与
  visual 相同的材质和 RGBA，避免默认白色碰撞材质遮住彩色表面，同时原样保留
  `contype`、`conaffinity`、`density`、尺寸和接触参数。
- collision 的几何与接触属性、质量、惯量、关节、执行器、传感器和 SONIC 控制
  参数均不修改。
- V17 及更旧缓存会自动重建；每次同步缓存后，V2 都会重新写入 MuJoCo 与 UE 的
  active XML，避免运行入口继续保留灰阶材质。

需要做 URDF 原色对照时，可临时禁用 G1 覆盖：

```bash
MATRIX_CUSTOM_MATERIAL_PROFILE=urdf \
  bash scripts/run_matrix_sonic_urban_v1.sh --urdf /path/to/g1_29dof.urdf
```

需要复现旧 AUE 灰阶配置时，显式指定配置文件运行材质工具，不要改写
`matrix_g1_v2.json`。

## 当前渲染边界

MuJoCo 3.3 能读取上述完整表面参数。Matrix 0.1.2 发布版 UE 当前只渲染 RGBA，
不会逐材质呈现粗糙度或金属度差异。

canonical G1 视觉网格是 STL，没有 UV；当前版本不会生成伪纹理。若后续取得
Matrix UE 插件源码或换用带 UV 的视觉网格，应让 bridge 直接消费同一 JSON 中的
PBR 参数，再升级为纹理材质。
