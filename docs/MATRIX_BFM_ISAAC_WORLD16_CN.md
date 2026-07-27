# Matrix × BFM/Isaac world16（tRNA）

这条链路把 Leo 的 BFM-SONIC world16 step079000 里程碑移植到当前 Matrix
仓库。它只支持 tRNA 上的 MoonWorld（scene 15），目前仍是独立运行时拓扑，不能与
旧 `run_matrix_sonic.sh` 的策略热切换或物理起身状态机混为一谈。

## 运行时拓扑

```text
键盘 -> Robo-PFNN 参考 -> world16 Teacher ONNX -> Isaac/PhysX G1
                                               -> Unix 状态流
                                               -> 50 Hz 插值中继
                                               -> Matrix UE 中的同一台 G1
```

Isaac/PhysX 是唯一物理权威。Matrix 关闭本地 MuJoCo、MC 和 SONIC，只显示收到的
root、29 个关节及速度；这不是两台并行受控机器人。

冻结时钟合同为：

- 物理模拟步长 `0.005 s`，即模拟时钟 200 Hz；
- 策略模拟频率 50 Hz；
- 每个策略动作保持 4 个 PhysX 子步；
- 默认地球重力约 `-9.81 m/s²`，没有月球低重力覆盖。

上述数值描述模拟时间，不保证墙钟实时。旧 GPU PhysX 路径在 tRNA 的 60 FPS
共卡基线只有 43.69 Hz 物理、10.92 Hz 策略、RTF 0.218；BFM 专用 30 FPS 后为
61.47 Hz、15.37 Hz、RTF 0.307。新 Matrix profile 把单环境 PhysX 放到 CPU，
Robo-PFNN 保持 `cuda:0`，Teacher ONNX 使用单线程顺序执行并关闭 worker spinning；
每个 20 ms 控制周期只写一次已证明无状态的 implicit PD target，再执行四个 5 ms
物理步并统一 `update(0.020)`。正式验收仍必须同时查看
`correctness_ok` 和 `realtime_ok`，不得用 `--correctness-only` 掩盖实时门失败。

## 启动

首次部署或依赖更新先执行：

```bash
cd <当前 Matrix checkout>
bash scripts/bootstrap_matrix_bfm_isaac.sh --profile trna
```

它把冻结的 Leo BFM runtime 安装到本项目自己的 `outputs/runtime`，不会修改
`/home/trna/matrix`；主机资产路径由 `config/hosts/trna.env` 提供。

交互进程必须放在 tmux 中：

```bash
cd <当前 Matrix checkout>
tmux new-session -s matrix-bfm-isaac
# 仅供人工功能诊断；实时资格以本轮 acceptance.json 为准。
bash scripts/run_matrix_bfm_isaac_guarded.sh interactive \
  --profile trna \
  --onscreen \
  --duration 600 \
  --correctness-only
```

正式资格验收必须去掉 `--correctness-only`；在命令返回 0 且
`correctness_ok=true`、`realtime_ok=true`、`manual_ok=true` 前，不得标记通过。

操作时先点击 Matrix 游戏窗口，然后使用：

- `W/S` 前后，`A/D` 转向，`Q/E` 横移；
- 按住 `Shift` 慢跑；
- 方向键旋转/俯仰相机；
- `Space` 停止，`R` 继续，`Esc` 正常结束并写最终证据。

共卡模式只对该 renderer 使用 `MATRIX_BFM_ISAAC_UE_MAX_FPS=30`。普通 Matrix
仍保留 tRNA profile 的 60 FPS；显式 A/B 可覆盖为 60/90/120。

自动 smoke 示例：

```bash
bash scripts/run_matrix_bfm_isaac_guarded.sh smoke \
  --profile trna \
  --schedule 'stand:2,walk:12,stand:2'
```

## 默认策略与默认拓扑的区别

`config/hosts/trna.env` 中：

```bash
MATRIX_INITIAL_LOCOMOTION_POLICY=bfm-sonic-teacher50k
```

只表示旧 Matrix/SONIC 主入口默认选中 BFM locomotion 槽，显式 `sonic` 仍可覆盖。
它不会把 `run_matrix_sonic.sh` 自动改成这里的 Isaac/world16 拓扑：旧槽使用
Teacher50k/Model12 路径，而本链路锁定 world16 step079000。若以后要让桌面图标默认
进入本链路，必须新增明确的 `bfm-isaac-world16` 运行时路由，并补齐策略面板、起身和
停止/恢复语义后再切换，不能复用旧 policy id 假装两者相同。

## 证据与清理

每轮证据写到：

```text
outputs/runs/matrix-bfm-isaac/<run-id>/
```

正常结束应包含 `runtime-report.json`、`trajectory.npz`、`relay-status.json`、
`acceptance.json` 和 `finalizer-status.json`。交互模式的动作覆盖需要人工复核；当前
自动 schedule 还不覆盖横移、停止和真实跌倒恢复。

不要用 `pkill`、`killall`，也不要修改或清理 `/home/trna/matrix`。正常在 tmux 中按
`Esc` 或给顶层 launcher 发一次 `Ctrl+C`；资源守卫只清理本实例账本中的进程。

## 当前资格状态

- 旧 GPU PhysX 对照：30 FPS 共卡 Moon `0.307 RTF`，关闭 Matrix UE 后 Moon
  `0.417 RTF`，flat `0.577 RTF`；once-write 后 Moon 也只有 `0.444 RTF`。
- CPU PhysX + CUDA Robo-PFNN + 单线程 Teacher ORT 的两轮无渲染 Moon 短测均为
  500/500、0 fall，RTF 分别为 `1.303`、`1.294`，control p95 分别为
  `19.38 ms`、`18.70 ms`；两轮的 34 个非计时语义数组与对照逐字节一致。
- 这已经证明 200/50 Hz 合同和 Moon 重力没有配置错误，原瓶颈是单环境 GPU PhysX
  同步开销以及 ORT 默认 24 线程自旋竞争。但 Matrix UE 30 FPS 共驻的正式 smoke、
  完整七 gait correctness 和 finalizer 尚未跑完，因此当前仍不能通知用户最终手测。
- tRNA 的 runtime Python 通过 editable install 引用 `/home/trna/IsaacLab`。verifier
  已锁定 Python/Isaac Sim/IsaacLab 版本、checkout commit、两处兼容补丁 hash 和精确
  dirty allowlist；最终合并前仍建议把补丁迁入项目自有的冻结 checkout，消除共享依赖。
- runtime lock schema 4 还锁定 Matrix port 关键文件、干净 Git checkout、启动姿态以及
  G1 URDF 引用的 36 个 mesh；acceptance 会记录 Matrix commit 和 lock SHA。
- 本 Isaac 链路没有旧 Matrix/SONIC 中的 `amp-flat-v3` 或 KungFu 物理起身策略；当前
  recovery 是 Leo runtime 自己的重置/搬运逻辑。
- 只有功能、视觉、实时性、人工动作覆盖和正常 finalizer 全部签字后，才能视为完整
  资格通过。
