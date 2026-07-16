# Matrix + SONIC two-host runbook

## Authority

- Source: `https://github.com/amithyst/matrix`
- Stable branch: `main`
- Runtime lock: `config/runtime/matrix-sonic.lock.json`
- Host defaults: `config/hosts/heyuan.env`, `config/hosts/trna.env`
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

Use `~/matrix` as the active checkout on both hosts. Keep historical
`matrix-eval` directories read-only as evidence or package caches.

```bash
git clone https://github.com/amithyst/matrix ~/matrix
cd ~/matrix
git fetch origin --prune
git switch <shared-feature-branch>
git pull --ff-only
```

Do not maintain `heyuan` and `trna` implementation branches. Both machines test
the same commit. A feature branch is merged only after both required host gates
have been recorded.

## Runtime bundle layout

The path supplied to `--artifact-source` has this layout:

```text
aue-sim/
GR00T-WholeBodyControl/
inference/TensorRT/lib/
inference/onnxruntime/lib/
g1-visual/g1_29dof.urdf
bridge/g1_sonic_sim_udp_dds_bridge_accepted
ros2-humble-prefix/           # required by the isolated Heyuan profile
matrix-native-deps/           # isolated native libraries
python-wheelhouse/            # CPython 3.10 x86_64 wheels plus SHA256SUMS
```

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

## Sync and delivery

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
python3 scripts/verify_matrix_sonic_runtime.py \
  --runtime-root outputs/runtime/matrix-sonic-v1 \
  --profile <host> --fast
```

Never synchronize a working tree with `rsync`. Git synchronizes source; the
bootstrap synchronizes ignored artifacts.
