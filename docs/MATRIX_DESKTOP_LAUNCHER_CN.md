# Matrix SONIC 桌面一键启动

这套入口不启动 Web 服务，也不开放网络端口。桌面图标只负责把正式 Matrix
SONIC 游戏链路放入可恢复的 tmux 会话；游戏内配置、世界导航和版本信息仍在 ESC
战术终端中完成。

## 安装桌面图标

从当前 Matrix 仓库执行：

```bash
bash scripts/install_matrix_desktop_launcher.sh --profile heyuan
```

安装器通过 `xdg-user-dir DESKTOP` 查找当前用户桌面，并生成本机专用的
`matrix-sonic.desktop`。生成文件包含当前仓库的绝对路径，但不会进入 Git；仓库中只保存
可跨机器同步的模板和安装器。

TRNA 或 ZZA 使用相同命令，只替换 profile：

```bash
bash scripts/install_matrix_desktop_launcher.sh --profile trna
bash scripts/install_matrix_desktop_launcher.sh --profile zza
```

## 双击后的行为

图标固定启动月球场景，并将移动策略固定为 BFM SONIC Teacher50k：

```bash
bash scripts/run_matrix_sonic_moon_v1.sh \
  --profile <profile> \
  --control-source game \
  --initial-locomotion-policy bfm-sonic-teacher50k \
  --game-fall-recovery auto
```

`moon-v1` 固定 UE MoonWorld（scene 15）和月球动态物理场景。`auto` 在 TRNA/Heyuan
交互运行中解析为物理倒地爬起；移动策略槽和倒地恢复槽彼此独立，因此恢复策略不会阻止
BFM SONIC 就绪或接收键盘控制。

桌面启动器会显式清除继承的 `LD_LIBRARY_PATH` 和 `PYTHONPATH`，并在
`matrix-sonic-desktop-<uid>` tmux 会话中运行主链。重复双击不会创建第二套仿真。

运维命令：

```bash
bash scripts/launch_matrix_sonic_desktop.sh status --profile heyuan
bash scripts/launch_matrix_sonic_desktop.sh attach --profile heyuan
bash scripts/launch_matrix_sonic_desktop.sh stop --profile heyuan
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
