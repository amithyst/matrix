# Matrix Overworld 相邻场景 V1

## 目标

第一版把 `Town10World`、`Warehouse`、`YardWorld`、`ApartmentWorld`、
`OfficeWorld` 和 `MeetRoomWorld` 放进同一个连续坐标系。Town10 保持原位，另外
五个场景沿其东侧开放边界从南到北排列，场景包围盒不重叠，连接带宽度允许 G1
通过。场景之间保留约 2.1 米的连续地面连接带，既不重叠碰撞，也不用传送或重置
机器人。布局真值是 `research/overworld_v1/layout.json`。

## 验收状态

| 层 | 状态 | 证据 |
| --- | --- | --- |
| 六场景 MuJoCo 物理拼接 | 通过 | 1404 个环境 geom、1 张共享地面、无包围盒重叠 |
| SONIC 连续穿越 | 通过一条代表路线 | G1 从 Town10 连续走入 Warehouse，未倒地、未重置 |
| 六场景 UE 视觉拼接 | 阻塞 | 发布包只有 cooked IoStore 地图，没有可编辑源工程 |
| 动态人物/道具交互 | 未包含 | V1 明确只接收静态原生物理代理 |

TRNA 的结构化结果保存在
`research/overworld_v1/results/trna_adjacent_physics_20260715.json`。这份 V1 的准确
表述是“连续物理世界已经成立，六地图视觉世界尚未成立”。

## 当前边界

V1 已能组合六套 MuJoCo 静态物理代理，但不能宣称完成六套 UE 视觉地图合并。
Matrix 0.1.2 提供的是 UE 5.5.4 cooked IoStore 包，公开仓库没有 `.uproject`、
`.umap` 或可编辑源地图。运行时 `STREAMMAP` 探针证明它能给 Town10 增加一个
原点变换的 Apartment level，但不能给六个 cooked level 分别设置位置，而且再次
执行会替换前一个 streamed level。

视觉门禁写在布局的 `visual_contract` 中。解除门禁需要获得匹配的 UE 5.5 工程和
源地图，然后按相同 transform 制作 `/Game/Maps/Overworld`。在此之前，主 Matrix
UE 启动链路不会把单一 Town10 画面冒充成六场景 Overworld。

## 物理布局

| 场景 | 世界 XY 范围（米） | 连接方式 |
| --- | --- | --- |
| Town10 | `[-124.1,-146.6] - [125.9,103.4]` | 城市骨架 |
| Warehouse | `[128.0,-120.0] - [193.0,-59.2]` | 使用西侧原生开口 |
| Office | `[128.0,-55.0] - [157.99,-20.5]` | 使用西侧原生开口 |
| Yard | `[128.0,-16.0] - [155.48,25.88]` | 使用西侧原生开口 |
| Apartment | `[128.0,30.0] - [136.89,38.35]` | 使用西侧原生开口 |
| MeetRoom | `[128.0,42.0] - [144.1,47.5]` | 移除西墙代理 `phs_Cube37` |

所有输入地面 plane 被替换为一张统一地面，避免无限平面重复。环境共 1404 个
geom；场景名字和 mesh 资产在组合时按场景 key 加命名空间，防止碰撞。当前只接受
静态代理，输入场景出现 joint/freejoint 会直接失败。组合器先在临时目录生成完整
XML、manifest 和资产集，全部校验通过后才替换旧产物；坏布局不会删除上一版可用
世界。

## 运行

TRNA 上使用现有 `matrix-sonic-eval` tmux：

```bash
cd /home/trna/matrix-eval
bash scripts/run_matrix_sonic_overworld_v1.sh
```

默认从 Town10 到 Warehouse 的连接口内侧启动，等 SONIC lowcmd 生效 2 秒后以
`0.25 m/s` 向东行走，默认 smoke 为 70 秒；验收要求机器人根位置从 `x=124`
越过 Warehouse 西边界 `x=128`。该入口明确传入 `--no-render-sync`，状态写入
`outputs/matrix_overworld_v1_status.json`。物理模型和组合 manifest 写入
`outputs/runtime/matrix_overworld_v1/`。

2026-07-15 的最终 TRNA 运行正常退出：总墙钟 70 秒，MuJoCo 稳态
`200.027 Hz`、实时比 `1.0001`、`fall_detected=false`、
`instability_resets=0`。最后一个结构化状态样本中，G1 位于
`[129.52382, -105.32781, 0.78306]`，相对初始位置移动 5.5308 米，越过
Warehouse 西边界 1.52382 米。该测试只证明 Town10 到 Warehouse 这一条代表
路线；Office、Yard、Apartment 和 MeetRoom 的连接带已做布局/重叠校验，但还需
分别补行走 smoke，不能提前写成五条路线都已动态验收。

## 本地检查

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile scripts/compose_overworld_scene.py scripts/run_matrix_sonic.py
bash -n scripts/run_matrix_sonic_overworld_v1.sh
python3 -m json.tool research/overworld_v1/layout.json >/dev/null
python3 -m json.tool \
  research/overworld_v1/results/trna_adjacent_physics_20260715.json >/dev/null
```

## 视觉层下一步

需要 Matrix 对应版本的 UE 5.5 可编辑工程和六张源地图。拿到后按当前 layout 的
transform 创建 `/Game/Maps/Overworld`，使用 level instance 打包，再同时验收：
视觉位置与 MuJoCo 碰撞一致、六块地图可同时存在、SONIC 跨边界不断链、实际 UE
帧率和相机传感器输出。拿到源工程前不做粗糙自制平替，也不提交 Town10 单图截图
冒充 Overworld。
