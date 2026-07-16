# G1 最终材质契约

## 来源

Matrix 不自行设计 G1 配色。`config/materials/aue_g1_v1.json` 固定引用
`aue-sim@13463d586ef01c9c4bde0907d9069f02d02724a0` 的
`src/aue/scenes/g1_visual_overlay.py::install_g1_material_overrides`，并保留以下三类
表面参数：

| 表面 | 颜色 RGBA | 粗糙度 | 金属度 | 部件规则 |
| --- | --- | ---: | ---: | --- |
| 黑色软胶 | `0.035 0.035 0.035 1` | 0.62 | 0 | 骨盆、踝、腕、手指 |
| 深灰结构件 | `0.16 0.16 0.16 1` | 0.58 | 0 | 头、标识、髋俯仰 |
| 暖白外壳 | `0.72 0.72 0.68 1` | 0.52 | 0 | 其余外壳 |

规则按表格顺序匹配，避免同一部件被后续规则覆盖。配置还要求标准 G1 的九个关键
link 全部存在；普通自定义机器人不会误套用此材质。

## 导入行为

通用 URDF 导入流水线 V15 默认使用 `auto`：

- 匹配标准 G1 link 签名时使用 `aue_g1_v1`。
- 其他机器人继续忠实保留 URDF 的具名、全局或内联颜色。
- 每类表面生成确定性的 MJCF material，并把 `rgba`、`roughness` 和 `metallic`
  写入模型。
- collision、质量、惯量、关节、执行器、传感器和 SONIC 控制参数均不修改。
- V14 及更旧缓存会自动重建。

需要做对照实验时，可临时禁用 G1 覆盖：

```bash
MATRIX_CUSTOM_MATERIAL_PROFILE=urdf \
  bash scripts/run_matrix_sonic_urban_v1.sh --urdf /path/to/g1_29dof.urdf
```

## 当前渲染边界

MuJoCo 3.3 能读取上述三类完整表面参数。Matrix 0.1.2 发布版的 UE 自定义机器人桥
目前只把每个 geom 的 `FLinearColor` 传给动态材质，因此 Town10 中可以看到准确的
黑色、深灰和暖白分区，但 UE 端暂时不会逐材质呈现粗糙度或金属度差异。

canonical G1 视觉网格是 STL，没有 UV；`aue-sim` 的来源材质本身也没有纹理贴图。
因此本版本不会生成伪纹理。若后续取得 Matrix UE 插件源码或换用带 UV 的视觉网格，
应直接消费同一 JSON 中已固化的 PBR 参数，而不是再创建另一套颜色规则。
