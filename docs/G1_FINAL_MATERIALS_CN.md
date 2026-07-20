# G1 可见材质与皮肤契约

## 皮肤注册表

G1 外观由 `config/materials/g1_skins.json` 数据驱动。当前注册两套皮肤：

| 皮肤 ID | 名称 | Profile | 用途 |
| --- | --- | --- | --- |
| `unitree-stock` | 原厂银灰 | `matrix_g1_stock_v1.json` | 默认，黑 / 白 / 石墨 / 银灰 |
| `matrix-blue` | Matrix 冰蓝 | `matrix_g1_v2.json` | 保留已验收的蓝 / 黑 / 白高对比外观 |

启动入口提供同一个选择：

```bash
# 默认原厂银灰
bash scripts/run_matrix_sonic_urban_v1.sh --skin unitree-stock

# 切回冰蓝
bash scripts/run_matrix_sonic_urban_v1.sh --skin matrix-blue

# 等价环境变量
MATRIX_G1_SKIN=matrix-blue bash scripts/run_matrix_sonic_urban_v1.sh
```

## 默认银灰配色

Matrix UE 0.1.2 的自定义机器人桥只消费每个 geom 的 `FLinearColor`，不会呈现
profile 中的 metallic / roughness。默认 `unitree-stock` 因此保留已经验证清楚的
黑、石墨和冷白层级，只把原先的大面积蓝色点缀换成中性银灰：

| 表面 | 线性 RGBA | 粗糙度 | 金属度 | 部件规则 |
| --- | --- | ---: | ---: | --- |
| 黑色软胶 | `0.018 0.024 0.035 1` | 0.62 | 0 | 骨盆、踝、腕、手部 |
| 石墨结构件 | `0.055 0.075 0.11 1` | 0.58 | 0 | 头、髋俯仰、腰部结构 |
| 冷白外壳 | `0.9 0.94 1 1` | 0.48 | 0 | 其余外壳 |
| 银灰点缀 | `0.42 0.42 0.42 1` | 0.38 | 0.35 | 骨盆轮廓、标识、髋偏航、膝、肩滚转、肘 |

Unitree 固定版本的 G1 URDF
`9926cc2f179ae3b86f4f74087bd32ef0c8b6fd90` 只定义 `dark=0.2` 与
`white=0.7`，官方产品图也以黑、白、银灰为主；并没有公开独立的油漆 RGB /
Pantone。这里的 `0.42` 是 Town10 强日照和 RGBA-only 渲染下的视觉补偿值，
显示端约为 sRGB `173 173 173`（`#ADADAD`），不是冒充厂商色度测量。

## 增加新皮肤

新增黄金等皮肤不需要修改 Python 或 UE shim：

1. 在 `config/materials/` 增加完整 profile JSON，保留标准 G1 matcher 和部件规则。
2. 在 `g1_skins.json` 注册稳定的皮肤 ID、名称和相对 profile 路径。
3. 用 `--skin <id>` 启动并做 Town10 实机截图验收。

启动器从注册 profile 生成当前皮肤的精确 RGB palette，并只在 UE active XML 中
给 G1 材质加入 alpha `0.99609375` 的不可见 scope tag；MuJoCo XML 仍保持 alpha 1。
该值是可精确表示的二进制分数；registry 与 preload 都拒绝大于 `0.999` 的 tag，
保证它不会落入 alpha 1 的匹配容差。
palette 和 tag 通过环境只传给受保护的 UE 进程。preload 最多接受 16 个合法、
有限、范围在 `[0, 1]` 的颜色，并要求 RGB 与 scope alpha 同时命中才识别为 G1；
调用原材质函数前会把 alpha 恢复为 1。只有这类 G1 runtime mesh 且 section 故障值
为 `MaterialIndex == -1` 时才映射到 slot 0，避免常见银灰色碰撞 Town10 场景材质。

## 导入与缓存行为

通用 URDF 导入流水线 V18 每次从缓存生成 active XML 后都会重新应用当前皮肤，
所以切换皮肤不需要更换 robot name、强制重导入或复制 mesh 缓存：

- 匹配标准 G1 link 签名时应用选中皮肤。
- 其他机器人继续忠实保留 URDF 的具名、全局或内联颜色。
- visual 与 Matrix UE 会重复绘制的 mesh collision 副本使用同一外观。
- collision 几何、接触参数、质量、惯量、关节、执行器、传感器和 SONIC 参数不变。

需要做 MuJoCo / 转换层的 URDF 原色对照时仍可禁用 G1 profile：

```bash
MATRIX_CUSTOM_MATERIAL_PROFILE=urdf \
  bash scripts/run_matrix_sonic_urban_v1.sh --urdf /path/to/g1_29dof.urdf
```

该模式不属于注册皮肤，UE shim 不会把未注册的 URDF 颜色当成 G1 scope；因此不能
把它当作 UE 最终可见材质验收。需要让一套 URDF 原色在 UE 中完整可见时，应将其
整理成独立 profile 并注册为皮肤，使 palette 与 scope tag 保持同源。

canonical G1 视觉网格是无 UV 的 STL，当前版本不会生成伪纹理。取得匹配的 UE
插件源码或带 UV 的视觉网格后，才能让 UE 直接消费同一 profile 的 PBR 参数。
