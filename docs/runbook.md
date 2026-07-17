# Matrix + SONIC multi-host runbook

## Authority

- Source: `https://github.com/amithyst/matrix`
- Stable branch: `main`
- Runtime lock: `config/runtime/matrix-sonic.lock.json`
- Host defaults: `config/hosts/heyuan.env`, `config/hosts/trna.env`,
  `config/hosts/zza.env`
- Local non-secret overrides: `.matrix/local.env` (ignored by Git and loaded
  before profile defaults, so runtime-derived dependency paths stay coherent)

Git stores source, scripts, profiles, tests, and checksums. It does not store the
7.76 GB Matrix release packages, the internal SONIC models, TensorRT libraries,
generated engines, logs, or recordings. Those files are copied from a controlled
artifact source and verified against the tracked lock.

When `--release-cache` is supplied, bootstrap verifies every archive against the
runtime lock, reuses it with a hard link (or a local reflink/copy across file
systems), generates the installer manifest from the lock, and disables network
downloads. A cache hit therefore does not consume another 7.76 GB of network
traffic.

Launch and bootstrap scripts call `/usr/bin/env` explicitly. This prevents a
host-local executable named `env` earlier on `PATH` from turning NUMA re-entry
or offline package installation into a false successful no-op.

## Checkout policy

Use `~/matrix` as the active checkout on every host. Keep historical
`matrix-eval` directories read-only as evidence or package caches.

```bash
git clone https://github.com/amithyst/matrix ~/matrix
cd ~/matrix
git fetch origin --prune
git switch <shared-feature-branch>
git pull --ff-only
```

Do not maintain machine-specific implementation branches. Heyuan, TRNA, and
ZZA test the same commit. A feature branch is merged only after its required
host gates have been recorded.

## Runtime bundle layout

The path supplied to `--artifact-source` has this layout:

```text
aue-sim/
aue-sim/scripts/g1_sonic_sim_udp_dds_bridge.cpp
GR00T-WholeBodyControl/
inference/TensorRT/lib/
inference/onnxruntime/lib/
g1-visual/g1_29dof.urdf
bridge/g1_sonic_sim_udp_dds_bridge_accepted
ros2-humble-prefix/           # required by isolated Heyuan and ZZA profiles
matrix-native-deps/           # isolated native libraries
python-wheelhouse/            # CPython 3.10 x86_64 wheels plus SHA256SUMS
```

The accepted bridge is built on the older TRNA Ubuntu 22.04 ABI baseline and is
locked to at most `GLIBC_2.34` and `GLIBCXX_3.4.29`. Build shared native tools on
the oldest supported host; verifier runs `ldd` on both the deploy binary and the
bridge, so a newer-host-only binary is rejected before launch.

The artifact source may be a local directory or an rsync-compatible SSH source.
Because the GitHub repository is public and the SONIC model is internal, do not
upload this bundle to the public GitHub release.

## Bootstrap

Heyuan example using the accepted `matrix-eval` directory as a temporary cache:

```bash
cd ~/matrix
bash scripts/bootstrap_matrix_sonic.sh \
  --profile heyuan \
  --release-cache /home/kaijie/matrix-eval/releases \
  --runtime-root /home/kaijie/matrix-artifacts/matrix-sonic-v1-heyuan \
  --write-local-env
```

Use `--artifact-source` instead of `--runtime-root` when the bundle must first be
copied from a local or SSH source. TRNA uses the same command with
`--profile trna` and its local cache paths.
Bootstrap is idempotent: archives and runtime files are SHA-checked, Python 3.10
is installed from the offline wheelhouse, and the full native dependency closure
is tested. Do not bypass TLS errors with `--trusted-host`; refresh the wheelhouse
on a trusted network instead.

ZZA keeps large private artifacts on its data volume while the active checkout
remains `/home/ununtu/matrix`:

```bash
cd /home/ununtu/matrix
bash scripts/bootstrap_matrix_sonic.sh \
  --profile zza \
  --release-cache /data/user_data/matrix-release-cache/0.1.2 \
  --runtime-root /data/user_data/matrix-artifacts/matrix-sonic-v1-zza \
  --write-local-env
```

ZZA's profile prepends `$HOME/.local/bin` for user-installed operator tools.
Install `jq` there from a pinned upstream release when system package access is
unavailable; verify its published checksum before bootstrap. Do not copy a sudo
password or another host's credentials to satisfy this prerequisite.

## Launch

Town10 main path:

```bash
bash scripts/run_matrix_sonic_urban_v1.sh \
  --profile heyuan \
  --control-source planner \
  --walk-after 10 \
  --vx 0.25
```

For a bounded acceptance run, add:

```bash
--max-seconds 90 --min-active-seconds 60
```

The launcher fails on a detected fall, numerical reset, insufficient active
lowcmd duration, missing artifact, SHA mismatch, wrong TensorRT ABI, or failed
ROS2 RMW load. It also serializes launches per checkout and restores tracked
configuration files on exit.

## Host-specific behavior

- Heyuan uses GPU0 and CPU set `0-63,128-191`, the NUMA node local to that GPU.
- TRNA does not apply a CPU set by default.
- ZZA uses GPU0, X11 display `:1`, and its single 24-core NUMA node without an
  explicit CPU set. Its ROS2 RMW closure is isolated in the runtime bundle;
  `MATRIX_CUDA_ROOT` points at a dedicated user-managed CUDA 12 runtime instead
  of prepending a complete Conda library directory.
- Override a value in `.matrix/local.env`; do not edit the tracked host profile
  for a one-off experiment.

## Current Heyuan evidence

The accepted full-sensor Town10 run used the locked TensorRT 10.13.3 runtime:

- SONIC physics: 200.006 Hz
- real-time factor: 1.0
- active lowcmd: 91.602 s
- displacement: 10.909 m
- fall/reset: none
- UE FPS with RGB, depth, and Mid360 enabled: mean 8.178, median 8.147
- UE FPS with those sensors disabled: about 14.3

Sensor disabling is an operator-render profile tradeoff, not the AI-data default.
It must not be presented as a no-cost optimization.

## Current ZZA evidence

ZZA reproduced the locked Town10 SONIC path on commit `8719bed` with an RTX
4090. The 120-second bounded acceptance produced:

- SONIC physics: 200.003 Hz aggregate
- real-time factor: 1.0 aggregate
- active lowcmd: 90.339 s
- displacement: 10.771 m
- fall/reset: none
- UE state synchronization: about 50 Hz
- exit cleanup: no Matrix child processes or listeners remained

The full verifier passed TensorRT 10.13.3, ONNX Runtime 1.16.3, the dedicated
CUDA 12.1 runtime, ROS2 RMW, and UDP/DDS dependency closure. Two 1920x1080
frames confirm Town10 and the white/dark G1 visual rendered while the robot
walked. The cooked camera still leaves G1 near the right edge, and UE emitted a
non-fatal Vulkan texture-layout ensure; treat both as visual-runtime follow-up
items rather than physics acceptance failures.

Tracked metrics are in
`research/urban_v1/results/zza_town10_sonic_20260717.json`. The private logs,
verifier report, and screenshots are in
`/data/user_data/matrix-artifacts/evidence/zza-town10-20260717` on ZZA. Inspect
the retained operator panes with:

```bash
ssh zza-ubuntu -t 'tmux attach -t matrix-zza-acceptance'
```

## Sync and delivery

### Three-host collaboration contract

Heyuan, TRNA, and ZZA are peers of one repository, not separate forks:

1. Create one short-lived feature branch on whichever host starts the change.
2. Push the branch, then use `fetch`, `switch`, and `pull --ff-only` on the
   other required hosts so every result names the same commit.
3. Keep host paths and display/NUMA defaults in `config/hosts/*.env`; keep local
   runtime locations in ignored `.matrix/local.env`.
4. Synchronize private runtime bundles and release archives through bootstrap
   plus the tracked lock. Never synchronize an active Git worktree with rsync.
5. Choose gates by impact: PICO/device changes require TRNA device evidence;
   generic launcher/runtime changes require the affected host profiles; shared
   physics or packaging changes require more than one host before merge.
6. Merge through one PR only after source checks, artifact verification,
   runtime acceptance, cleanup, and evidence links are recorded.

Before switching hosts:

```bash
git status --short
git push -u origin HEAD
```

On the other host:

```bash
git fetch origin --prune
git switch <same-branch>
git pull --ff-only
export MATRIX_PROJECT_ROOT="$PWD"
if [[ -f .matrix/local.env ]]; then source .matrix/local.env; fi
source config/hosts/<host>.env
python3 scripts/verify_matrix_sonic_runtime.py \
  --runtime-root "$MATRIX_RUNTIME_ROOT" \
  --profile <host> --fast
```

Never synchronize a working tree with `rsync`. Git synchronizes source; the
bootstrap synchronizes ignored artifacts.
