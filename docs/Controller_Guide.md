# Controller Guide

Matrix has more than one control path. The mappings in this document are not
interchangeable:

- `--control-source game` is the camera-relative, third-person SONIC control
  path described first below.
- The native/legacy Matrix mappings are retained at the end of this document
  for launch modes that still use them. Their posture and action buttons are
  not implemented by `game` mode.

For setup, camera calibration, safety tests, and Heyuan acceptance, follow the
[Matrix game-control runbook](MATRIX_GAME_CONTROL_RUNBOOK.md).

## Camera-relative SONIC control (`--control-source game`)

### Keyboard and mouse

| Input | Behavior |
|---|---|
| **W / S** | Move toward / away from the camera's horizontal forward direction |
| **A / D** | Move left / right in the camera's horizontal frame |
| **W+A**, **W+D**, etc. | Diagonal movement at the same maximum speed as a cardinal direction |
| Hold **Ctrl** or **Alt** + WASD | Native mode 1 `SLOW_WALK`, 0.10 m/s base target |
| WASD without a speed modifier | Native mode 2 `WALK`, 0.80 m/s target |
| Hold **Shift** + WASD | Native mode 3 `RUN`, 2.50 m/s target |
| Double-tap the same **W/A/S/D** key | Boost the active slow/walk/run tier to 0.20/1.00/2.75 m/s until release |
| Mouse drag | Native Matrix camera operation; the robot stops while the configured look button is held |
| **V** | Best-effort safety mirror; an observed press forces zero. Centered-overlay v3 does not use V as a visual camera-mode switch |
| **Q / E** | Reserved and ignored by SONIC locomotion; they do not rotate the robot |

Left and right Ctrl are equivalent, as are left and right Alt/Shift. Ctrl and
Alt both select the slow tier; if a slow modifier and Shift are held together,
the slower profile wins. Changing tier clears a pending or active double-tap boost.

All four movement directions use orient-to-movement. The robot turns toward the
requested world direction and reduces translation while a large turn is still
in progress. Mouse look changes only the camera; the control path never rotates
the UE robot actor directly.

Native translation starts only after both commanded and measured heading are
within 15 degrees of the requested direction. Once moving, a wider 30-degree
stop edge prevents noise from chattering the gait while still stopping a
materially misaligned body.

The keyboard tiers select SONIC's native locomotion modes, not three aliases of
`SLOW_WALK`: Ctrl/Alt selects mode 1 at 0.10 m/s, unmodified WASD selects mode 2
at 0.80 m/s, and Shift selects mode 3 at 2.50 m/s. A same-direction double tap
raises those targets to 0.20, 1.00, and 2.75 m/s respectively. The base targets are the lower
boundaries of SONIC's documented 0.10-0.80, 0.80-2.50, and 2.50-7.50 m/s gait
intervals. The slow tier wins modifier conflicts. Q/E is not reused. All six
base/boost speeds can be adjusted in the ESC motion panel and are persisted in
the host-scoped motion-control config.

Acceleration and modifier downshifts remain rate limited. The published native
mode follows the current ramp, so an upshift reaches mode 2 only at 0.80 m/s
and mode 3 only at 2.50 m/s; a downshift crosses the same boundaries in reverse.
Every published mode/speed pair therefore stays inside a native gait interval.
Releasing all movement directions immediately requests mode 0 `IDLE`; modifier
keys alone never move the robot. Above the radial deadzone, left-stick travel
still maps continuously from 0.10 m/s to the separately configured analog
maximum (0.30 m/s by default, configurable up to the native 0.80 m/s ceiling)
and remains `SLOW_WALK`. Switching from a keyboard gait to the stick clamps the
same output frame to that configured cap. Measure actual gait transitions on
Heyuan.

The default Matrix look button is the left mouse button. Releasing a camera
drag does not immediately resume a movement key that remained held. Release all
movement input once, then press it again; this is the neutral re-arm interlock.

### Persistent robot-centred cooked overlay and native fallback

The Heyuan profile configures the host-local bundle
`/home/kaijie/matrix-artifacts/matrix-centered-camera-custom-v1`. Its tracked
v3 contract is
`config/runtime/matrix-centered-camera-overlay-v3.json`; the bundle directory
name is historical and does not determine its version. The contract pins the
three `pakchunk99-MatrixCentered-Linux_P` `.pak`/`.utoc`/`.ucas` files by exact
size and SHA-256 and scopes them to `MujocoSim_Custom` plus `Spectator` for
`MujocoSim_Custom_C`. The helper independently pins the same artifact tuple in
code, so selecting a different contract file cannot authorize other bytes.

The overlay is selected only for native SONIC, `--control-source game`, enabled
centred mode, the `custom` robot, and a configured bundle. The top-level
launcher owns the host lock, atomically purges a verified crash residue, and
verifies the external bundle before the SONIC runtime audit. Immediately before
UE starts, `run_sim.sh` atomically installs a private copy at:

```text
src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive
```

UE then starts with `viewclass Spectator_C`. Overlay v3 continuously moves the
Spectator pivot to the custom robot's `MainBody` position while preserving the
Spectator rotation. It disables camera/rotation lag, keeps SpringArm collision,
and clamps pitch to -75/+55 degrees. The asset default is 110 cm; the launcher
overrides it to 150 cm so the full robot occupies about 60-63% of frame height.
`MATRIX_GAME_CAMERA_DISTANCE_CM` accepts only plain decimal values in the
inclusive 80-500 cm range; 180 cm is the suggested future wide view. Launch is
fail-closed unless the current launch's new log segment reports both
`LogPakFile: Found Pak file` and `LogPakFile: Mounted IoStore container` for the
pinned stem; any new stem line containing `Failed` is rejected. Historical log
lines cannot satisfy this gate. The active directory remains installed for the
entire supervised UE lifetime and is atomically retired only after UE stops.
Failure to remove it makes the launcher fail rather than reporting success.

This is a persistent centred session mode, not a visual mode that V switches
in and out of. V remains a best-effort locomotion safety observation, but v3
keeps the visible camera centred; do not use V to assess free-camera visuals.

Without a configured bundle, all existing modes retain the native fallback. In
SONIC game mode the fallback disables SpringArm camera lag and rotation lag,
enables SpringArm collision, and selects the rendered robot actor:

| Matrix robot type | Cooked UE view class |
|---|---|
| `custom` (the SONIC G1 launch path) | `MujocoSim_Custom_C` |
| `go2` / `go2w` | `MujoCoSim_go2_C` / `MujoCoSim_go2w_C` |
| `xgb` / `xgw` / `xxg` / `zgws` | `MujoCoSim_Xgb_C` / `MujoCoSim_Xgw_C` / `MujoCoSim_Xxg_C` / `MujoCoSim_Zgws_C` |

The `xxg` mapping records the class present in the cooked asset; the current
0.1.2 launcher still rejects robot type `xxg` before UE startup.

The native fallback was tested against the live Heyuan cooked runtime: its
PlayerController target was `MujocoSim_Custom_C`, and the live custom SpringArm
reported `MainBody` as parent, lag/rotation lag `False`, and collision test
`True`. Overlay v3 has passed offline IoStore verification and exact Legacy/Zen
round-trip checks. Moving-robot orbit, wall/ground collision recovery, and
remote-desktop feel still require black-box acceptance; do not claim complete
commercial-game camera parity from package verification alone.

The startup behavior is reversible and narrowly gated:

- set `MATRIX_GAME_CENTERED_CAMERA=0` to disable centred selection and prevent
  overlay installation;
- inherit an explicitly empty `MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE` before
  loading the Heyuan profile to disable the overlay and use native fallback;
- without a bundle, set `MATRIX_GAME_CAMERA_VIEW_CLASS=AnotherRobot_C` to select
  a different short Blueprint class. The value must be one token ending in `_C`; whitespace,
  commas, and console separators are rejected;
- with the bundle configured, the view-class override must be unset or exactly
  `Spectator_C`; every other value is rejected;
- `MATRIX_UE_EXTRA_EXEC_CMDS` is appended last, so an operator can deliberately
  supersede the defaults.

The `set Engine.SpringArmComponent ...` commands are class-wide UE console
operations. They are not scoped to the selected robot: assume every loaded
SpringArmComponent can be affected. Use the disable switch if another scene
depends on SpringArm lag, and validate any narrower replacement before adding
it through `MATRIX_UE_EXTRA_EXEC_CMDS`. Planner, PICO, external, and non-SONIC
launches do not receive these defaults.

### ESC local/remote mouse settings

In `game` mode, **ESC** immediately hard-zeros locomotion and shows the centre
crosshair, a visible pointer, and a large MC-style settings panel. An X11 modal
shield intercepts core ButtonPress/Release outside the panel. Because cooked UE
may also subscribe to XI2 raw input, confirm on the deployed desktop that panel
clicks/drags do not rotate a fixed landmark. The panel distinguishes the
configuration applied to the current process from the next-launch value:

| Key | Behavior |
|---|---|
| **M** | Toggle the next-launch profile between `Local` and `Remote` |
| **- / +** | Traverse the Remote presets: 0.01x–0.10x in 0.01 steps, then 0.20x–1.00x in 0.10 steps |
| **Mouse** | Click `Local`/`Remote`, the same preset-table `-`/`+`, or the large `Return to Game & Apply` button |
| **Enter** | Keyboard equivalent of `Return to Game & Apply`; returns directly when nothing changed |
| **F9** | Keyboard fallback: safely restart the complete Matrix/SONIC topology when a saved change is pending |
| **F10 / F12** | Reserved for the external MouseLock center/toggle actions; Matrix does not intercept them |
| **ESC** | Leave the panel; locomotion still requires a neutral re-arm |

`Local` is always 1.0x; the default saved Remote preset is 0.5x. Remote has 19
exact presets: 0.01x through 0.10x in 0.01 increments, followed directly by
0.20x through 1.00x in 0.10 increments (`0.10 +` becomes `0.20`, and
`0.20 -` becomes `0.10`). Keyboard and panel clicks traverse this same table,
including the existing 0.40x preset. Changes are atomically persisted to
`~/.config/matrix/mouse-control.json`, but they do not mutate the current UE
process. Apply/Enter waits for a successfully delivered neutral frame, then
asks the existing private restart channel to reload the **whole** runtime. The
old generation remains in the safe panel and displays reload progress; a
save/request failure leaves the panel open with an error. F9 applies the same
gate as a fallback. Do not restart UE alone, and keep all controls released
during the reload.

The visible UE camera receives `SDL_MOUSE_RELATIVE_SPEED_SCALE` at process
startup. For example, the currently used Remote 0.4x setting is a native
SDL/UE input multiplier; it is not an X pointer-acceleration value. With the
default `x11-mirror` base of 0.12 degrees per XI2 raw unit, status reports
`0.12 x 0.4 = 0.048 deg/raw`. The mirror subscribes to `XI_RawMotion`, which
SDL relative mouse mode commonly uses, and the launcher requests SDL raw mode.
Live black-box evidence has not yet shown that the packaged UE build consumes
those deltas one-for-one. This is not final rendered-camera yaw readback and
does not prove that the mirror and visible camera agree.
The four-axis, multi-turn black-box acceptance below is still required. A
missing, corrupt, or manually edited off-table settings file safely falls back
to Local 1.0x. On a valid Remote launch, the same selected multiplier is sent
to the visible SDL/UE path and used for the nominal `x11-mirror` gain.

The launcher also requests and configures SDL raw relative motion without warp emulation,
viewport scaling, or SDL system-pointer scaling, disables UE
`bEnableMouseSmoothing` and FOV sensitivity scaling, disables SpringArm lag in
the robot-centred view, and adds `r.MotionBlurQuality 0`. The first settings
remove input interpolation; the motion-blur command removes visual streaking.
Neither command changes the selected gain.

For an interactive SONIC `game` launch with the applied profile `Remote`,
`run_sim.sh` additionally snapshots the current X display's acceleration and
threshold, temporarily runs `xset m 1/1 0` before UE starts, and restores the
exact pair at cleanup. This linearizes the absolute X11 stream used by the
panel; the yaw mirror now uses XI2 raw motion. It does not modify MouseLock. A
missing `DISPLAY`, missing `xset`, or X-server failure
only produces a warning and does not block launch. Pointer control is global to
that X display while Matrix is running. Normal exit and handled signals restore
it, but an uncatchable `SIGKILL` or host crash cannot run cleanup; in that case
restore the values printed by the warning/log with `xset m <accel> <threshold>`
or restart the desktop session.

Pointer recentering, window-edge effects, and absolute-coordinate jumps still
require crosshair/MouseLock calibration for the visible remote session.
The tested current MouseLock `pyautogui.moveTo`/XTEST absolute recenter is not
accumulated as XI2 raw yaw motion. Other synthetic relative recenter paths are
outside that claim.

### Gamepad status

The intended mapping is left stick for camera-relative movement and right stick
for camera-only look. In the current cooked Matrix 0.1.2 runtime, however, no
verified bridge exposes or drives the visible UE follow-camera transform.
Consequently:

- the adapter can read Linux joystick axes. With the `carla` source, the right
  stick writes spectator yaw/pitch and immediately reads back absolute yaw. A write
  or read-back failure stops locomotion; an unobserved integrated angle is never
  treated as camera truth;
- with `fixed` or `x11-mirror` camera yaw, `--game-input-source auto` safely
  degrades to keyboard input and an explicit `gamepad` request is rejected;
- explicitly selecting `carla` permits left-stick locomotion once the spectator
  RPC write/read-back succeeds. This is only a spectator-transform candidate,
  not proof that the visible follow camera moved with it. The packaged 0.1.2
  runtime has no discovered CARLA server.

Do not report full gamepad or right-stick camera control as implemented until a
runtime camera bridge has passed the runbook's black-box acceptance checks.

### Safety behavior

Game input is sampled at 50 Hz. Keyboard slow/walk/run targets are native modes
1/2/3 with 0.10/0.80/2.50 m/s base and 0.20/1.00/2.75 m/s double-tap boost;
gamepad speed remains continuous in native
`SLOW_WALK` up to its separate configured cap (0.30 m/s by default, at most
0.80 m/s). The input timeout threshold and maximum snapshot age are 0.15 s.
Releasing all directions or any condition below hard-zeroes the SONIC command
without a deceleration tail:

- startup has not yet received a neutral frame;
- native LowCmd is not fresh or the startup elastic band has not fully released;
- the Matrix window loses focus;
- the adapter observed its mirrored V safety state or the mouse look button is held;
- input times out, becomes stale, disconnects, or is rejected;
- the provider exits, its local socket closes, or an observed camera yaw is
  unavailable.

After startup, focus loss, camera drag, mirrored V-state toggling, timeout, or
reconnection, release WASD and center the left stick. One focused neutral frame
must be accepted before motion can resume. This prevents a held key or stick
from causing an unexpected restart.

The input adapter sends snapshots over a user-private local socket. The runtime
checks both the peer UID and the exact PID of its supervised adapter process.
The native SONIC planner remains the only locomotion command publisher; the
input adapter does not publish DDS or planner commands itself.

The launcher binds focus to the supervised UE PID as well as the title, so a
terminal or IDE whose title contains “matrix” cannot keep driving the robot.
The 0.15 s timeout is evaluated on the next 50 Hz control tick (nominal worst
case about 0.17 s plus scheduler jitter). It covers the input/provider chain;
a freeze of the entire runtime process falls back to SONIC's own longer
watchdog. V remains best-effort until a UE `SubscribeInputMode`-style bridge
reports the actual free-camera state, including toggles before provider start;
centered-overlay v3 intentionally keeps the visible view centred on V.

## Camera-yaw sources

| Source | Use | Limitation |
|---|---|---|
| `fixed` | Safe axis and deadman testing | The command frame does not follow visible camera rotation |
| `x11-mirror` | Heyuan calibration candidate | Integrates ordered XI2 raw motion only while the configured raw button is held; absolute warps do not cancel the drag. It neither reads nor drives the final UE camera, so UE-side auto-rotation can still diverge |
| `carla` | Writable spectator candidate with read-back | Right-stick yaw/pitch rotation is immediately read back; write/yaw failure stops. The Matrix 0.1.2 cooked package exposes no discovered CARLA server and has no visible-camera coupling proof |

`fixed` is the default so an unverified camera estimate cannot silently steer
the robot. `x11-mirror` becomes acceptable only after sensitivity, sign, and
offset are calibrated on Heyuan and repeated 0/90/180/-90-degree tests show that
W always follows the visible camera direction without accumulated drift.

## Native/legacy Matrix mappings

These are upstream Matrix mappings for control paths that support the original
remote controller. They are not action bindings for `--control-source game`.

### Gamepad

| Action | Controller input |
|---|---|
| Stand / Sit | Hold **LB** + **Y** |
| Move Forward / Back / Left / Right | **Left Stick** |
| Rotate Left / Right | **Right Stick** |
| Jump Forward | Hold **RB** + **Y** |
| Jump in Place | Hold **RB** + **X** |
| Somersault | Hold **RB** + **B** |

Logitech Wireless Gamepad F710 is the upstream recommended controller.

### Keyboard

| Action | Keyboard input |
|---|---|
| Stand | **U** |
| Sit | **Space** |
| Move Forward / Back / Left / Right | **W / S / A / D** |
| Rotate Left / Right | **Q / E** |
| Start | **Enter** |

In the legacy native path without centered-overlay v3, **V** toggles free camera and holding the left mouse button
temporarily enters free-camera operation. In `game` mode, use the behavior and
safety interlocks documented above.
