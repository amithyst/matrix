# Matrix camera-relative game-control runbook

This runbook covers the interactive `--control-source game` path from local
keyboard/mouse input to native SONIC locomotion. It is deliberately separate
from planner-driven qualification and from the upstream Matrix remote-control
mappings.

Current rollout policy: develop and validate this feature on Heyuan. TRNA is
the second-priority backup and ZZA is the third-priority backup. Do not sync
ordinary changes or milestone releases to either backup unless the owner
explicitly requests that host.

## Scope and acceptance boundary

Implemented behavior:

- WASD is projected into the camera's horizontal frame;
- the robot turns toward every requested movement direction, including A, D,
  and S;
- diagonals are normalized, with speed, acceleration, deceleration, and turn
  rate limited;
- keyboard WASD uses hold-to-modify slow/walk/run profiles: Ctrl, no modifier,
  and Shift respectively; all remain native SONIC `SLOW_WALK`;
- Q/E is excluded from locomotion yaw;
- exact UE-PID focus loss, observed V safety-state toggles, camera drag, stale
  input, disconnect, and provider failure stop the robot;
- native LowCmd must be fresh and the startup elastic band fully released
  before any movement frame can pass;
- the private socket authenticates the exact supervised provider PID as well
  as its UID;
- a neutral frame is required at startup and after every safety stop;
- the native SONIC planner is the only command publisher. No UE actor is
  rotated directly.

Not yet established in the cooked Matrix 0.1.2 runtime:

- an API that reads the visible follow-camera transform without drift;
- an API that makes the right stick rotate that visible camera;
- verified coupling between a CARLA spectator and the camera shown to the
  operator;
- authoritative UE free-camera/input-mode readback. V mirroring is best-effort;
  centered-overlay v3 deliberately does not switch the visible camera on V.

Therefore `fixed` is the safe default. `x11-mirror` is a calibration candidate,
not an authoritative view transform. Full right-stick camera control must not
be claimed from this implementation.

## Defaults and safety invariants

| Item | Default / invariant |
|---|---|
| Input sampling | 50 Hz |
| Local input protocol | strict `matrix-game-input/v2` (`ctrl` and `shift` are required fields) |
| SONIC control | 50 Hz; native physics remains 200 Hz |
| Maximum speed | 0.30 m/s |
| Keyboard speed profiles | Ctrl slow 0.10; unmodified walk 0.20; Shift run 0.30 m/s; Ctrl wins a conflict |
| Native gait | `SLOW_WALK`; 0.10 m/s floor with start/stop hysteresis |
| Acceleration / deceleration | 1.20 / 2.40 m/s² |
| Maximum heading rate | 2.50 rad/s |
| Translation heading gate | start within 15 degrees; stop beyond 30 degrees |
| Left-stick radial deadzone | 0.15 |
| Input timeout | 0.15 s |
| Maximum snapshot age | 0.15 s |
| Safe stop | Immediate zero command, without smoothing |
| Re-arm | One focused neutral frame before movement |

Do not increase the timeout or speed during the first Heyuan calibration. Keep the
startup elastic band enabled and keep `--fail-on-fall`/zero-reset acceptance
gates at their launcher defaults. The internal ramp is bounded, but native
gait entry/exit has a 0.10 m/s floor step. The hidden intent ramp runs before
entry and published acceleration is limited again after that first native floor
frame; record physical start/stop distance.

## Preflight on Heyuan

Keep `/home/kaijie/matrix` as the clean main checkout. Use the tracked Heyuan
profile from a dedicated experiment worktree, and keep all movement keys
released before launch.

Prepare that worktree once after the feature branch is pushed. Bootstrap is
required because Git worktrees do not carry ignored UE packages,
`.venv-audit`, or `.matrix/local.env` from the main checkout:

```bash
git -C /home/kaijie/matrix fetch origin feature/camera-relative-game-control
git -C /home/kaijie/matrix worktree add --detach \
  /home/kaijie/worktrees/matrix-game-control-exp \
  origin/feature/camera-relative-game-control
cd /home/kaijie/worktrees/matrix-game-control-exp
MATRIX_SONIC_ROOT=/home/kaijie/worktrees/sonic-matrix-native-final \
  bash scripts/bootstrap_matrix_sonic.sh \
    --profile heyuan \
    --release-cache /home/kaijie/matrix-eval/releases \
    --runtime-root /home/kaijie/matrix-artifacts/matrix-sonic-native-v2-heyuan \
    --write-local-env
/usr/bin/python3 scripts/update_matrix_local_env.py \
  .matrix/local.env MATRIX_SONIC_ROOT \
  /home/kaijie/worktrees/sonic-matrix-native-final
```

```bash
MATRIX_EXPERIMENT_WORKTREE="${MATRIX_EXPERIMENT_WORKTREE:-/home/kaijie/worktrees/matrix-game-control-exp}"
cd "$MATRIX_EXPERIMENT_WORKTREE"
git status --short
git rev-parse HEAD
printf 'DISPLAY=%s\n' "$DISPLAY"
test -S /tmp/.X11-unix/X1001
```

The Heyuan profile uses the active NoMachine X11 display `:1001`. Run
interactively, not with `--offscreen`. The input provider requires the active window to belong to the
exact supervised UE PID and also checks title `(zsibot|matrix|unreal)` by
default. A different cooked-window title must be supplied through
`MATRIX_GAME_FOCUS_TITLE`; title matching alone is not an acceptance-grade
focus check.

The current Heyuan desktop/tmux `PATH` contains a same-named
`~/.local/bin/env` initialization script which is not GNU `env` and ignores the
launcher arguments. When removing polluted Conda variables, call
`/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH ...` explicitly rather than bare
`env`.

The currently locked cooked package does not contain
`/Game/Maps/ApartmentWorld`; `--scene 21` therefore fails map loading and is not
a playable acceptance target. Until that asset is recooked, use the packaged
`--scene 2` (`Town10World`) for Heyuan interactive and ESC-panel acceptance.

Start from a fresh UE process in its default centred mode. The cooked runtime
cannot report a V edge made before the input provider starts.

Use the canonical launcher. Directly running `run_sim.sh` or
`run_matrix_sonic.py` is only a debugging escape hatch and cannot produce
qualified evidence.

### Centered-overlay preflight and lifetime

The Heyuan profile defaults `MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE` to
`/home/kaijie/matrix-artifacts/matrix-centered-camera-custom-v1`. Before every
launch, the host-locked top-level launcher purges only a previously verified
stale active directory and verifies that bundle against
`config/runtime/matrix-centered-camera-overlay-v3.json`. The bundle must be a
real absolute directory containing exactly the three pinned
`pakchunk99-MatrixCentered-Linux_P` files; symlinks, extra entries, path
indirection, size differences, and SHA-256 differences fail closed.

For SONIC game + centred + `custom`, `run_sim.sh` installs the verified files
atomically immediately before UE, selects `Spectator_C`, and waits for both
`LogPakFile: Found Pak file` and `LogPakFile: Mounted IoStore container` in the
new log bytes after the launch boundary. A new stem line containing `Failed`
fails immediately; historical log bytes cannot pass the gate. The active copy
remains online for the whole UE process and is removed only after the exact
supervised UE stops. A
kill that prevents cleanup is handled by the next host-locked `purge-stale`.
The launcher overrides the asset's 110 cm SpringArm with a 150 cm full-body
default. `MATRIX_GAME_CAMERA_DISTANCE_CM` is fail-closed to plain decimals in
80-500 cm; use 180 cm only when deliberately testing a wider view.
Planner/PICO/external, non-SONIC, non-custom, or disabled-centred launches never
install it. With no configured bundle, the existing native robot-viewclass
fallback remains unchanged.

V does not visually switch overlay v3 to free camera. It only toggles the
input provider's best-effort mirrored safety state, so a V safety test still
needs a second V press and neutral re-arm even though the view stays centred.
For an explicit recovery launch without the Heyuan default, preserve an empty
value while the profile loads:

```bash
MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE= \
  bash scripts/run_matrix_sonic.sh --profile heyuan --scene 2 \
    --control-source game
```

## Stage 1: fixed-frame functional test

Start with a fixed SONIC camera yaw. This proves input mapping and safety without
pretending to follow the visible camera:

```bash
/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash scripts/run_matrix_sonic.sh \
  --profile heyuan \
  --scene 2 \
  --control-source game \
  --game-input-source keyboard \
  --game-camera-yaw-source fixed \
  --game-initial-yaw 0 \
  --game-max-speed 0.30 \
  --game-input-timeout 0.15
```

On another terminal, inspect live state:

```bash
watch -n 0.5 'jq "{control_source, physics_step_hz, rtf, fall_detected, instability_resets, root_xyz, root_displacement_xy_m, game_input}" outputs/matrix_sonic_status.json'
```

The first focused snapshot must report `game_input.stop_reason` as
`awaiting_neutral` if a movement key was already held. Release WASD; the mode
should become `idle`. Motion is permitted only on a new press.

At fixed SONIC yaw zero, verify the normalized physics directions:

| Input | Expected root direction | Expected facing |
|---|---|---|
| W | +X | +X |
| S | -X after turning | -X |
| A | +Y | +Y |
| D | -Y | -Y |

Also verify:

1. W+A and W+D are not faster than W.
2. Ctrl+W, W, and Shift+W settle at 0.10, 0.20, and 0.30 m/s respectively;
   holding Ctrl+Shift uses the safer 0.10 m/s profile, and modifier transitions
   obey acceleration/deceleration limits.
3. A, D, and S rotate the robot toward movement; a reversal turns before it
   develops full translation speed.
4. Q and E alone do not move the root or change the game-control heading.
5. Releasing movement decelerates normally, while a safety event hard-stops.

The visible camera may not align with this table in `fixed` mode. That mismatch
is expected and is why this stage is not camera-relative acceptance.

## Stage 2: X11 mirror calibration

`x11-mirror` observes the same root-window mouse delta that the native UI is
expected to consume. It does not query UE and does not move the camera itself.
Its SONIC command yaw is:

```text
wrap(sign × (initial_yaw + accumulated_mouse_dx × sensitivity) + offset)
```

Start from a repeatable visible camera pose and conservative defaults:

```bash
/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash scripts/run_matrix_sonic.sh \
  --profile heyuan \
  --scene 2 \
  --control-source game \
  --game-input-source keyboard \
  --game-camera-yaw-source x11-mirror \
  --game-look-button left \
  --game-initial-yaw 0 \
  --game-mouse-sensitivity 0.12 \
  --game-camera-yaw-sign -1 \
  --game-camera-yaw-offset 0 \
  --game-max-speed 0.30 \
  --game-input-timeout 0.15
```

Calibrate in this order:

1. **Offset:** with the startup view at a known world direction, adjust offset
   until W produces that direction.
2. **Sign:** drag horizontally in one known visual direction. After releasing
   all movement keys and the mouse, press W. If the movement frame changed in
   the opposite direction, flip `--game-camera-yaw-sign`.
3. **Sensitivity:** make a visually measured 90-degree yaw rotation. Increase the
   degrees-per-pixel value if the SONIC frame turns too little; decrease it if
   the frame turns too far.

Every drag is a safety stop. The correct sequence is: release WASD, drag the
camera, release the mouse, provide a neutral frame, then press W. Holding W
through a drag must leave the robot stopped with `awaiting_neutral` after the
mouse is released.

For the four-axis gate, align the calibrated SONIC yaw and check root delta:

| Calibrated camera yaw | W must move toward |
|---|---|
| 0° | +X |
| +90° | +Y |
| ±180° | -X |
| -90° | -Y |

Repeat the sequence after several clockwise/counter-clockwise rotations, after
moving the pointer near each screen edge, and after two V presses that exercise
the mirrored safety state (the v3 view itself remains centred). Any cumulative
mismatch, jump after pointer warp, or automatic camera
recenter invalidates `x11-mirror` as an acceptance source. Keep `fixed` as the
default in that case.

If remote-desktop dragging is too fast, do not tune system `xinput`
acceleration. UE/SDL consumes raw relative motion, so a pointer curve may alter
only the X11 absolute coordinates and make the visible camera diverge further
from `x11-mirror`. For SONIC `game` + Remote, the launcher itself snapshots the
current X pointer curve, uses `xset m 1/1 0` only for the lifetime of the run,
and restores it during cleanup; leave the desktop setting at the user's normal
value. Press ESC, click Remote, and use the large -/+ controls to traverse the
19 exact presets: 0.01x–0.10x in 0.01 steps, then 0.20x–1.00x in 0.10 steps.
The 0.10x and 0.20x presets are adjacent, and keyboard -/+ traverses the same
table as panel clicks. Click `Return to Game & Apply` (or press Enter). The
panel waits for the neutral safety gate and reloads the complete runtime; F9
remains the keyboard fallback. After restart,
verify that `CURRENT APPLIED (SDL)` is the intended value, then repeat the
four-axis and multi-turn tests. F10/F12 remain external MouseLock bindings, not
Matrix settings-page actions.

Local is fixed at 1.0x. Remote 0.4x remains one of the presets and is the native
SDL/UE multiplier. The same selected Remote multiplier feeds the visible SDL
path and the nominal `x11-mirror` gain. With the example base mirror gain of
0.12 deg/px, 0.4x makes the reported `x11-mirror` value 0.048 deg/px, but that
number is only nominal arithmetic over X11 root-pointer pixels, not a measured
visible-camera sensitivity. A missing, corrupt, or manually edited off-table
settings file fails safe to Local 1.0x.

## Stage 3: safety and recovery matrix

Run every row with the robot already moving slowly:

| Test | Required result |
|---|---|
| Launch while W is held | `awaiting_neutral`; no motion until release and re-press |
| Alt-Tab / focus another window | Immediate zero; `focus_lost`; neutral required after refocus |
| Hold the configured mouse look button | Immediate zero during drag; neutral required afterward |
| Press V after provider start | An observed edge gives immediate zero and mirrored `free_camera`; overlay v3 does not visually switch |
| Stop input packets | 0.15 s threshold; zero on the next 50 Hz tick (nominal worst case about 0.17 s plus scheduler jitter) |
| Close the input socket | Zero on the next control poll; reconnect requires neutral |
| Terminate the supervised provider | Zero/teardown; the whole launch must clean up its owned children |
| Reconnect while W is held | Remain `awaiting_neutral` until W is released |
| Start before LowCmd is fresh or while the startup band is nonzero | `sonic_not_ready`, zero speed, while native deploy still receives `start=True` |
| Make fresh LowCmd stale, then recover while W remains held | Immediate zero; remain `awaiting_neutral` after recovery until W is released |
| Hold Q or E | No SONIC yaw or translation command |

The launcher creates a private mode-0700 runtime directory and a unique local
`SOCK_SEQPACKET` endpoint; the socket is mode 0600 and checks both the peer UID
and the exact PID preserved by the supervised provider's exec boundary. Do not
replace it with a network bridge or restore the old AndroidTwin UDP/DDS path.

The 0.15 s threshold covers a live runtime detecting input/provider failure.
A freeze of the entire Python runtime falls back to SONIC's own longer
watchdog; it is not a 0.15 s path.

## Stage 4: bounded Heyuan acceptance

Only after fixed-mode safety and all four `x11-mirror` headings pass, run a
bounded session from a clean checkout. Keep the lock-derived acceptance floors;
do not weaken displacement, lowcmd, fall, reset, physics, or RTF thresholds.
Bounded game qualification rejects `fixed`, rejects any request to disable the
supervised provider, and requires at least one non-zero movement frame to have
actually crossed the native planner boundary. It also pins the bundled provider
script and the same verified Python interpreter used by the runtime; interpreter
overrides are diagnostic-only and rejected here.

```bash
MEASURED_MOUSE_DEG_PER_PIXEL=0.12  # replace with the Heyuan measurement
MEASURED_CAMERA_YAW_SIGN=-1        # replace with -1 or 1 from the sign probe
MEASURED_CAMERA_YAW_OFFSET_DEG=0   # replace with the calibrated offset

/usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash scripts/run_matrix_sonic.sh \
  --profile heyuan \
  --scene 2 \
  --control-source game \
  --game-input-source keyboard \
  --game-camera-yaw-source x11-mirror \
  --game-look-button left \
  --game-initial-yaw 0 \
  --game-mouse-sensitivity "$MEASURED_MOUSE_DEG_PER_PIXEL" \
  --game-camera-yaw-sign "$MEASURED_CAMERA_YAW_SIGN" \
  --game-camera-yaw-offset "$MEASURED_CAMERA_YAW_OFFSET_DEG" \
  --game-max-speed 0.30 \
  --game-input-timeout 0.15 \
  --max-seconds 120 \
  --min-active-seconds 60
```

During the bounded window, exercise W/A/S/D, all three keyboard speed profiles,
diagonals, a 180-degree reversal, Q/E, a camera drag/re-arm cycle, and one
focus-loss/recovery cycle. For V, enter the mirrored safety state, verify the hard stop, press
V again to clear it, then complete neutral re-arm; a single toggle would correctly
leave the boundary in safe stop. End with sufficient commanded displacement to
satisfy the locked profile.

The final `outputs/matrix_sonic_status.json` must show, at minimum:

- `control_source: "game"` and a connected/applied game-input stream during
  active control;
- `passed: true`, `fall_detected: false`, and `instability_resets: 0`;
- physics at least 195 Hz and RTF at least 0.95;
- no protocol/replay errors and no unexplained safe stop in
  `game_input_at_boundary`;
- `game_input_at_boundary.moving_command_frames >= 1`, no peer-PID mismatch, and the
  supervised provider still connected at the acceptance boundary;
- final displacement satisfies the locked floor. Four-direction behavior is a
  process claim and must be shown by the Heyuan video, per-direction checkpoints,
  or periodic status/log evidence; one final displacement and yaw cannot prove
  that sequence.

For `x11-mirror`, `passed: true` proves only the authenticated runtime input and
motion path. It does not by itself prove that the rendered follow camera used
the mirrored yaw. `game_control_configuration` therefore records every source,
sign, offset, sensitivity, and CARLA look parameter, while explicitly reporting
`visible_follow_camera_verified: false` and
`external_visual_evidence_required: true`. Preserve a Heyuan screenshot or video
showing the four view-relative directions before describing the full
camera-relative behavior as accepted.

The measured-heading zero remains the initial MuJoCo snapshot for the whole
game-control run and is reported as `heading_anchor_source:
"initial_snapshot"`. The first observed fresh-LowCmd edge only records
`root_yaw_first_fresh_lowcmd_rad`, the wrapped
`root_yaw_startup_delta_rad`, and its step, simulation time, and wall elapsed.
It never re-anchors the command frame. Null first-fresh fields mean that no
fresh-LowCmd rising edge was observed during the run.

Inspect the final record and logs with:

```bash
jq '{run_id, matrix_commit, sonic_commit, runtime_lock_sha256, control_source, passed, acceptance_failures, physics_step_hz, rtf, fall_detected, instability_resets, root_displacement_xy_m, heading_anchor_source, root_yaw_initial_rad, root_yaw_first_fresh_lowcmd_rad, root_yaw_startup_delta_rad, root_yaw_relative_rad, game_input_at_boundary, game_input, game_control_configuration}' outputs/matrix_sonic_status.json
jq . outputs/matrix_game_control_input.json
tail -n 200 outputs/logs/matrix_sonic_runtime.log
```

The final `game_input` object is expected to show the runtime's deliberate
shutdown emergency stop. Qualification uses the pre-stop
`game_input_at_boundary` snapshot; inspect that object for connection,
freshness, and safe-stop acceptance.

Record the Matrix commit, SONIC commit, exact calibration values, scene, X11
display, controller hardware if any, status JSON, and any capture used for
visual confirmation. A manual unbounded run or a run launched through a
low-level escape hatch is useful diagnostics, but not qualified evidence.

## Gamepad and future camera bridge gate

For `fixed` and `x11-mirror`, `auto` intentionally becomes keyboard-only and
explicit `--game-input-source gamepad` fails. This is a safety result, not a
setup error. Do not work around it by feeding right-stick values into the yaw
integrator while the visible camera remains unchanged.

A complete right-stick implementation requires a runtime bridge with, at
minimum, equivalents of:

- `GetViewTransform` for the actual rendered follow camera;
- `SetOrbitYaw` for visible camera rotation;
- `SubscribeInputMode` for focus/free-camera/drag state.

The input provider implements an optional CARLA spectator
`set_transform -> get_transform` right-stick path. It satisfies the first two
capabilities only when the runtime actually exposes CARLA RPC and a probe proves
that the spectator is the final rendered camera. No CARLA server was discovered
in the current 0.1.2 binary/logs, so this remains a fail-closed candidate rather
than completion evidence for the packaged runtime.

After adding that bridge, repeat the same four-heading, drift, focus, deadman,
neutral-rearm, fall/reset, physics-rate, and RTF gates with both keyboard/mouse
and gamepad. Only then may gamepad become the default or the feature be
described as complete right-stick camera control.

## Troubleshooting

- **`awaiting_neutral` never clears:** focus Matrix, release WASD, center the
  left stick, release the look button, and make sure the mirrored V safety state is off.
- **`focus_lost` while Matrix looks active:** inspect `focus.expected_ue_pid`,
  `focus.actual_pid`, and the X11 title in provider status; adjust
  `MATRIX_GAME_FOCUS_TITLE` only after the PID is correct.
- **W is consistently offset:** correct camera-yaw offset.
- **Yaw changes in the opposite direction:** flip camera-yaw sign.
- **Error grows after repeated drags:** recalibrate sensitivity; if pointer
  warp/recenter causes discontinuities, reject `x11-mirror` rather than hiding
  the drift.
- **Explicit gamepad is rejected:** always expected for `fixed` and
  `x11-mirror`. Selecting `carla` admits the spectator RPC candidate only after
  write/read-back succeeds; that still does not verify the visible follow
  camera.
- **Input provider exits:** inspect `outputs/matrix_game_control_input.json` and
  the runtime log; do not bypass supervision for acceptance.

## Stop and rollout

Stop with Ctrl-C at the top-level launcher and confirm that its UE, SONIC,
deploy, input-provider, DDS, ZMQ, and local-socket children are gone. Preserve
the Heyuan evidence with the tested commit, then merge and update the clean
Heyuan main checkout. Do not synchronize source or private artifacts to TRNA or
ZZA unless the owner explicitly requests that backup host.
