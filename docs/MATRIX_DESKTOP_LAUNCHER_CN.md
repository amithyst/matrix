# Matrix SONIC 桌面一键启动

这套入口不启动 Web 服务，也不开放网络端口。桌面图标只负责把正式 Matrix
SONIC 游戏链路放入可恢复的 tmux 会话；游戏内配置、世界导航和版本信息仍在 ESC
战术终端中完成。

## tRNA 当前验收主线（BFM/Isaac world16）

tRNA 桌面只保留一个 `matrix-sonic.desktop`。当前发布入口不使用下面的 legacy
native/MuJoCo wrapper，而是通过稳定发布指针启动已验收的 BFM/Isaac world16 链：

```bash
ln -sfn /home/trna/<qualified-matrix-release> /home/trna/matrix-mainline
bash /home/trna/matrix-mainline/scripts/install_matrix_bfm_isaac_desktop_launcher.sh \
  --active-root /home/trna/matrix-mainline \
  --profile trna
```

安装器会原子覆盖同名桌面入口，因此不会产生第二个 Matrix 图标。双击执行：

```bash
bash scripts/run_matrix_bfm_isaac_guarded.sh interactive \
  --profile trna \
  --onscreen \
  --duration 7200 \
  --correctness-only
```

实际进程位于 `matrix-bfm-isaac-mainline-<uid>` tmux 会话。重复双击幂等；右键菜单
提供状态查询和安全停止。停止通过本次运行的受限 keyboard socket 发送 `SPACE` /
`ESCAPE` finalizer 请求并等待自然退出；不会向整套进程发送 `Ctrl+C`，超时会保留
现场而不会 `pkill`。桌面 wrapper 会清除视频、材质和 Python 路径覆盖，使用冻结验收
配置，并把启动/锁冲突/停止结果及每次 guard 控制台输出写入：

```text
~/.local/state/matrix-bfm-isaac/mainline-desktop-launcher.log
~/.local/state/matrix-bfm-isaac/desktop_<UTC>_<pid>.console.log
```

`matrix-mainline` 只在一个版本完成验证后由发布流程原子切换；双击不会执行 `git pull`。
本链固定使用已锁定的 world16 step079000 profile，不能把 native 入口中的策略热切换、
KungFu/AMP 起身或 `MATRIX_INITIAL_LOCOMOTION_POLICY` 语义套到本链上。

如果启动失败或 finalizer 未通过，下一次双击不会覆盖失败现场。检查上面的 console log
和 evidence 目录后，可显式清理已经死亡的 tmux 会话：

```bash
bash /home/trna/matrix-mainline/scripts/launch_matrix_bfm_isaac_desktop.sh \
  dismiss --profile trna
```

下面的安装器和 wrapper 属于 legacy native/MuJoCo 多场景链，保留给 Heyuan/ZZA、
回归和对照，不应再作为 tRNA 桌面的主入口。

## 安装桌面图标

从当前 Matrix 仓库执行：

```bash
bash scripts/install_matrix_desktop_launcher.sh --profile heyuan
```

安装器通过 `xdg-user-dir DESKTOP` 查找当前用户桌面，并生成本机专用的
`matrix-sonic.desktop`。生成文件包含当前仓库的绝对路径，但不会进入 Git；仓库中只保存
可跨机器同步的模板和安装器。

默认安装的是 MoonWorld 图标；移动策略由所选 host profile 决定：

```bash
bash scripts/install_matrix_desktop_launcher.sh --profile heyuan --scene 15
```

其他原生场景也可传入 `--scene ID`，安装器会生成带场景编号的独立图标，不会覆盖
默认 MoonWorld 入口。

TRNA 或 ZZA 使用相同命令，只替换 profile：

```bash
bash scripts/install_matrix_desktop_launcher.sh --profile trna
bash scripts/install_matrix_desktop_launcher.sh --profile zza
```

## 双击后的行为

默认图标启动 MoonWorld，但桌面层不提前写入移动策略：

```bash
bash scripts/run_matrix_sonic_moon_v1.sh \
  --profile <profile> \
  --control-source game \
  --game-fall-recovery auto
```

`moon-v1` 固定 UE MoonWorld（scene 15）和月球动态物理场景。`auto` 在 TRNA/Heyuan
交互运行中解析为物理倒地爬起；移动策略槽和倒地恢复槽彼此独立，因此恢复策略不会阻止
BFM SONIC 就绪或接收键盘控制。

tRNA 的 `config/hosts/trna.env` 是默认策略的唯一权威，值为
`bfm-sonic-teacher50k`；Heyuan/ZZA 没有该默认值，因而继续使用原生 SONIC。
需要在 tRNA 对照旧 SONIC 时可显式覆盖：

```bash
bash scripts/launch_matrix_sonic_desktop.sh start \
  --profile trna --scene 15 --initial-locomotion-policy sonic
```

命令行参数优先于显式 `MATRIX_INITIAL_LOCOMOTION_POLICY`，显式环境值又优先于
host profile 默认值。桌面和 `moon-v1` wrapper 都不会把 tRNA 的 BFM 默认污染到
Heyuan/ZZA。

非 15 场景使用同一桌面 wrapper，但会切到通用 `run_matrix_sonic.sh --scene <ID>`
链路。同一主机仍只允许一套 Matrix SONIC 运行实例；需要换场景时应先用桌面图标右键
菜单的 Stop，或执行下面的 `stop` 命令。

桌面启动器会显式清除继承的 `LD_LIBRARY_PATH` 和 `PYTHONPATH`，并在
`matrix-sonic-desktop-<uid>` tmux 会话中运行主链。重复双击不会创建第二套仿真。

运维命令：

```bash
bash scripts/launch_matrix_sonic_desktop.sh status --profile heyuan --scene 15
bash scripts/launch_matrix_sonic_desktop.sh attach --profile heyuan --scene 15
bash scripts/launch_matrix_sonic_desktop.sh stop --profile heyuan --scene 15
```

`stop` 会先向主 launcher 发送 `Ctrl+C` 并等待清理；只有清理超时才强制关闭 tmux
会话并返回失败。不要用 `pkill` 或直接杀 UE 进程代替这个入口。

## ESC 运行信息

打开 ESC 后选择“运行信息”。该页显示：

- 本次启动使用的 profile、场景和控制源；
- 启动时的 Git 分支、HEAD、提交标题、作者和时间；
- HEAD 相对第一父提交的文件数、增删行数和文件列表；
- 启动时工作树是否存在未提交修改。

这里展示的是本次仿真真实使用的启动快照。运行过程中即使另一个终端切换分支，面板也
不会把磁盘上的新分支冒充为正在运行的代码。跨世界重启会重新生成快照，因此
Earth -> Moon -> Earth 时 scene 会按 `2 -> 15 -> 2` 更新。

## 更新与迁移

仓库路径发生变化、重新 clone，或切换机器后，重新运行安装器即可刷新桌面图标。代码和
模板通过 Git 同步；生成的 `.desktop`、runtime 资产、日志和本机配置不进入仓库。
