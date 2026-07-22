# Matrix 永久场景清单 v1

`matrix-scene-manifest/v1` 是 Matrix 局内搭建的独立持久化边界。它保存稳定
entity ID、视觉/碰撞资产配对、统一坐标系、位姿、可见性、碰撞开关、物理模式
和标签。它不替代现有 `matrix_world_state.py`：后者保存 G1 出生/恢复状态，本
清单保存“世界里有什么”。

## 资产合同

视觉与碰撞是两类独立资产：

- visual backend：`3dgs`、`nurec`、`ue_cooked`；
- collision backend：`physx_usd`、`mujoco_mesh`；
- `file` locator 必须带大小和 SHA-256；`ue_package` 保存 `/Game/...` 包定位和
  内容摘要，不伪装成本地文件；
- `derived_from_asset_id` 可记录 MuJoCo collider 是由哪一份源 collision USD
  编译而来；
- 同时具有 visual 与 collision 的场景 entity 让二者共享同一个 canonical
  transform，避免画面和接触世界漂移；纯视觉对象、纯碰撞对象和 anchor 可只
  引用需要的一侧。

canonical frame 固定为右手、Z-up、米。UE 或其他坐标系必须在 importer 边界
显式转换，不能靠猜测轴向或单位。

`config/scenes/office-outside-ring.manifest.json` 已登记真实 benchmark 资产：

- NuRec USDZ：1,692,573,977 bytes，SHA-256
  `bc7957674b408d250520a20f916269b1d186c8a46dbe29c6f37700877f418e0c`；
- collision USD：14,941,306 bytes，SHA-256
  `61ab3ea47b5efe4b71ae084251ef3d008a0a68e7cabd83c80b2a90efe0a0bc33`；
- collision source selector：`/World/Scene/terrain`。

清单只热保存不可变引用和位姿，不会在每次保存时复制 1.7 GB 资产。永久性仍
依赖 VePFS/资产注册表的保留策略。恢复前必须用服务端配置的 allowlist roots
执行资产校验；客户端不能自行扩大白名单。验证通过可信根目录 fd 逐级
`O_NOFOLLOW` 打开并绑定最终 inode，拒绝中间目录 symlink 逃逸。恢复适配器必须在

```python
with open_verified_asset_references(...) as handles:
    # 用 handle.proc_path 加载，并保持 context 存活到引擎完成打开
```

的生命周期内消费 `/proc/self/fd/<fd>`，不能在校验后重新打开原始路径。
该路径只对持有 fd 的同一进程有效；跨进程加载必须显式传递 fd（例如
`SCM_RIGHTS`/`pass_fds`）或使用只读、content-addressed 的资产注册表。保持 fd
可以阻止 rename 换 inode，但不能阻止有写权限者原地修改同一 inode，因此运行
账号不得拥有资产库写权限。

## 写入和恢复

```bash
python3 scripts/matrix_scene_manifest.py validate-input \
  config/scenes/office-outside-ring.manifest.json

python3 scripts/matrix_scene_manifest.py write /absolute/path/scene.json \
  --input config/scenes/office-outside-ring.manifest.json \
  --expected-generation 0

python3 scripts/matrix_scene_manifest.py inspect /absolute/path/scene.json

python3 scripts/matrix_scene_manifest.py update /absolute/path/scene.json \
  --input /path/to/edited.manifest.json \
  --expected-generation 1 \
  --expected-store-digest <inspect-returned-digest>
```

每个 revision 都有不可复用的随机 `revision_id`。CAS 同时校验 generation 与
opaque store digest；任一过期都返回退出码 3，
文件不变。每次写入持独占 `flock`，使用 0600 临时文件、fsync、同目录原子
rename 和目录 fsync；上一代保存在 `.bak`。读主文件失败时只读回退备份并标记
`recovered_from_backup=true`，不会在共享读锁下偷偷修文件。主备均损坏时
fail closed。`.bak`、`.lock` 不能作为另一份主存档。
只有已确认的缺失或内容损坏会回退 `.bak`；权限、I/O 或安全打开错误直接
fail closed，禁止把“暂时读不到新版本”误判成“应覆盖旧版本”。
存档根目录必须由服务端创建和持有，不接受客户端选择的可写父目录或 symlink
父路径。

## 当前边界与后续接线

本提交只交付独立 schema/store/CLI/test，不修改三条并行运行时分支。它尚未
宣称 Matrix 已经端到端热保存：

1. creative inventory 的 spawn/delete/transform 事件尚未接到 manifest 事务；
2. 启动路径尚未按 manifest 重建 UE Actor 与 MuJoCo geom；
3. Matrix 当前没有 NuRec/3DGS runtime bridge；现有 `/Game/Maps/3DGSWorld` 是
   cooked UE 世界；
4. Matrix 的物理权威是 MuJoCo，不能直接加载 `physx_usd`。office 清单中的
   collision 是可信源资产，样例明确保存为 `physics_mode=none`、
   `collision_enabled=false`，不是“Matrix 已可碰撞”的声明；必须先离线编译成
   `mujoco_mesh`、记录 derived-from 与相同 transform，再通过 contact/raycast
   smoke；
5. MuJoCo 新拓扑仍需受控 reload；真正局内无重启增删需要预编译 entity slot
   pool。

运行时合并后的最小验收是：局内 commit、强杀重启精确恢复、视觉/碰撞 transform
一致、hash 错误拒绝恢复、主文件损坏回退上一 revision、真实碰撞/raycast 命中，
以及连续 20 次保存恢复不丢 entity。
