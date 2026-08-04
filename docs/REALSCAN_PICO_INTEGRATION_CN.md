# Matrix PICO + RealScan 机器人训练场

本集成固定采用以下运行时边界：

- 视觉：Matrix UE `ThreeDGaussians`，地图 `/Game/Maps/RobotTrainingGround`；
- 物理：SONIC/MuJoCo，场景 `scene_terrain_robot_training_ground.xml`；
- 遥操：Matrix 已有 `--control-source pico`；
- 禁止把 Isaac/NuRec/PhysX 作为 Matrix 运行时后端。

`model.nurec` 只作为离线高斯源。`scripts/convert_nurec_to_matrix_ply.py`
将锁定的 NuRec `sh-gaussians` 张量转换成 Matrix UE 插件使用的标准二进制
PLY。导入并烹饪完成后，运行时不再依赖 NuRec。

## 场景编号与导航

- Matrix scene ID：`18`
- UE map：`/Game/Maps/RobotTrainingGround`
- MuJoCo XML：`scene_terrain_robot_training_ground.xml`
- 星体导航命令：`/world realscan`
- 别名：`robot-training-ground`、`robot_training_ground`、`training`

在 cooked visual Pak 尚未安装并生成验证 receipt 以前，星体导航会把该入口显示为
`world_unavailable`。`scripts/run_sim.sh 18` 也会拒绝启动，避免回退显示旧书店
`3DGSWorld`。

## 离线资产流程

1. 从锁定的 USDZ 生成 Matrix 3DGS PLY：

   ```bash
   python scripts/convert_nurec_to_matrix_ply.py \
     --source-usdz "$ROBOT_TRAINING_GROUND_USDZ" \
     --output-ply "$MATRIX_RESCAN_ARTIFACT_ROOT/robot-training-ground_3m_bounded_opacity.ply" \
     --report "$MATRIX_RESCAN_ARTIFACT_ROOT/robot-training-ground_3m_bounded_opacity.report.json" \
     --max-points 3000000
   ```

2. 在 `jszr_mujoco_ue2` 中新建独立的 `RobotTrainingGround` 地图，将上述 PLY
   导入现有 `ThreeDGaussians` 插件。不要覆盖 `/Game/Maps/3DGSWorld`。
3. 烹饪一个独立的 Pak/UTOC/UCAS trio，再由工具根据真实文件和 UE Git
   provenance 自动创建 `receipt.json`：

   ```bash
   python scripts/create_realscan_scene_receipt.py \
     --bundle-dir "$ROBOT_TRAINING_GROUND_COOKED_BUNDLE" \
     --ue-repository "xvirobotics/jszr_mujoco_ue2" \
     --ue-commit "$(git -C "$JSZR_MUJOCO_UE2" rev-parse HEAD)"
   ```

   工具要求目录内恰好是一组同 stem 的 `.pak/.utoc/.ucas`，拒绝空文件、
   symlink、非小写完整 commit 和覆盖既有收据，并计算大小与 SHA256。生成格式为：

   ```json
   {
     "schema": "matrix-realscan-ue-package-receipt/v1",
     "map_name": "/Game/Maps/RobotTrainingGround",
     "source_usdz_sha256": "2b67231becf613036d4acdec796cffcad9ae3e2456dd311a96f8a00932df85cd",
     "source_ply_sha256": "911399630534fa9df8b143c2437fd89c68176ec5fe53bb1317e7d2fec03b472c",
     "ue_project": {
       "repository": "<authoritative jszr_mujoco_ue2 repository>",
       "commit": "<40 or 64 lowercase hex commit>"
     },
     "files": [
       {"name": "pakchunkNN-RobotTrainingGround-Linux.pak", "size_bytes": 1, "sha256": "<sha256>"},
       {"name": "pakchunkNN-RobotTrainingGround-Linux.utoc", "size_bytes": 1, "sha256": "<sha256>"},
       {"name": "pakchunkNN-RobotTrainingGround-Linux.ucas", "size_bytes": 1, "sha256": "<sha256>"}
     ]
   }
   ```

4. 安装到隔离 Matrix worktree：

   ```bash
   python scripts/install_realscan_scene.py \
     --project-root "$MATRIX_WORKTREE" \
     --visual-bundle-dir "$ROBOT_TRAINING_GROUND_COOKED_BUNDLE"
   ```

安装器会同时部署两份 MuJoCo runtime XML/高度图，验证 cooked Pak 全部哈希，再写入
`Saved/Paks/RobotTrainingGroundActive/receipt.json`。

## PICO 启动

不依赖 RealScan 的公共 PICO 入口默认启动 Town10，可先用于遥操链路验收：

```bash
bash scripts/run_matrix_pico.sh --profile trna
```

也可以在不启动 Matrix 的情况下检查参数解析与最终命令：

```bash
bash scripts/run_matrix_pico.sh --profile trna --scene 2 --dry-run
```

该入口固定 `--control-source pico`，拒绝调用方覆盖，并让 PICO 模式也携带当前
Matrix branch/commit/dirty 状态的 build provenance。scene 18 会额外执行 RealScan
安装验证，因此视觉包未就绪时即使 `--dry-run` 也会 fail closed。

默认设置 `MATRIX_PICO_AUTOSTART_MODE=planner`：进入后直接遥操，无需
`A+B+X+Y` 启动组合键；原生 manager 的 `A+X` PLANNER/POSE 切换、`A+B`
升档、`X+Y` 降档、左摇杆移动和右摇杆机器人转向保持不变。

场景安装验证通过后，从 tmux 或桌面 launcher 调用：

```bash
bash scripts/run_matrix_pico_realscan.sh --profile trna
```

PICO Python 与 wheel 仍由 Matrix runtime lock 校验；本脚本不引入新的 PICO SDK
链路。

## 当前 MuJoCo 代理范围

第一版为 0.25 m lower-floor navigation proxy：一个 `205 x 205` 高度场和 `453`
个合并边界 box，覆盖地面、坡面、墙/货架占地边界与不可通行孔洞。上层楼面和可抓取
动态物体需要后续拆成显式 MuJoCo body，不应伪装成已经完成。
