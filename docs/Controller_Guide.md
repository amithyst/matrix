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
| Hold **Ctrl** + WASD | Precise slow walk (0.10 m/s with the default speed cap) |
| WASD without a speed modifier | Ordinary walk (0.20 m/s by default) |
| Hold **Shift** + WASD | Run (0.30 m/s default maximum) |
| Mouse drag | Native Matrix camera operation; the robot stops while the configured look button is held |
| **V** | Mirror the native free-camera toggle; an observed press forces zero, but cooked UE provides no authoritative mode readback |
| **Q / E** | Reserved and ignored by SONIC locomotion; they do not rotate the robot |

All four movement directions use orient-to-movement. The robot turns toward the
requested world direction and reduces translation while a large turn is still
in progress. Mouse look changes only the camera; the control path never rotates
the UE robot actor directly.

Native translation starts only after both commanded and measured heading are
within 15 degrees of the requested direction. Once moving, a wider 30-degree
stop edge prevents noise from chattering the gait while still stopping a
materially misaligned body.

All three keyboard speed profiles stay in native SONIC `SLOW_WALK`; “run” is an
operator speed profile, not a switch to another SONIC gait. Ctrl selects the
0.10 m/s native floor, unmodified WASD selects the midpoint between that floor
and the configured maximum, and Shift selects the configured maximum. Ctrl wins
if both modifiers are held. Q/E is not reused. Acceleration limits apply when a
modifier changes while moving. Native gait entry/exit necessarily has a
0.10 m/s floor step; after entry, the published ramp again follows the configured
limit. Above the radial deadzone, left-stick travel
still maps continuously from 0.10 m/s to the configured maximum instead of
being quantized into the keyboard tiers. Measure actual start/stop distance on
Heyuan.

The default Matrix look button is the left mouse button. Releasing a camera
drag does not immediately resume a movement key that remained held. Release all
movement input once, then press it again; this is the neutral re-arm interlock.

### Native robot-centred startup view

For `MATRIX_SONIC=1` with `--control-source game`, the launcher now defaults to
the cooked package's native robot camera instead of leaving the initial view on
the free `Spectator_C`. At UE startup it disables SpringArm camera lag and
rotation lag, enables SpringArm collision, and runs `viewclass` for the actual
rendered robot actor:

| Matrix robot type | Cooked UE view class |
|---|---|
| `custom` (the SONIC G1 launch path) | `MujocoSim_Custom_C` |
| `go2` / `go2w` | `MujoCoSim_go2_C` / `MujoCoSim_go2w_C` |
| `xgb` / `xgw` / `xxg` / `zgws` | `MujoCoSim_Xgb_C` / `MujoCoSim_Xgw_C` / `MujoCoSim_Xxg_C` / `MujoCoSim_Zgws_C` |

The `xxg` mapping records the class present in the cooked asset; the current
0.1.2 launcher still rejects robot type `xxg` before UE startup.

The robot-centred view selection has been tested against the live Heyuan cooked
runtime: a visible frame put the robot at the centre, the PlayerController view
target read back as `MujocoSim_Custom_C`, and the live custom SpringArm read
back `MainBody` as its parent, camera lag `False`, rotation lag `False`, and
collision test `True`. This is a real cooked UE camera path, not the non-runtime
Python camera contract.

This result does **not** yet prove a moving-robot translation-follow acceptance,
that every mouse drag is a true orbit about the robot, that pitch/wall/ground
collision recovery matches a commercial third-person game, or that V has
authoritative mode readback. Those still need black-box acceptance while the
robot moves and beside obstacles. Do not call this a complete Genshin-style
camera bridge.

The startup behavior is reversible and narrowly gated:

- set `MATRIX_GAME_CENTERED_CAMERA=0` to disable it;
- set `MATRIX_GAME_CAMERA_VIEW_CLASS=AnotherRobot_C` to select a different
  short Blueprint class. The value must be one token ending in `_C`; whitespace,
  commas, and console separators are rejected;
- `MATRIX_UE_EXTRA_EXEC_CMDS` is appended last, so an operator can deliberately
  supersede the defaults.

The three `set Engine.SpringArmComponent ...` commands are class-wide UE console
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
| **- / +** | Adjust the Remote scale in 0.1 steps from 0.2x through 1.0x |
| **Mouse** | Click `Local`/`Remote`, `-`/`+`, or the large `Return to Game & Apply` button |
| **Enter** | Keyboard equivalent of `Return to Game & Apply`; returns directly when nothing changed |
| **F9** | Keyboard fallback: safely restart the complete Matrix/SONIC topology when a saved change is pending |
| **F10 / F12** | Reserved for the external MouseLock center/toggle actions; Matrix does not intercept them |
| **ESC** | Leave the panel; locomotion still requires a neutral re-arm |

`Local` is always 1.0x; the default saved Remote preset is 0.5x. Changes are
atomically persisted to `~/.config/matrix/mouse-control.json`, but they do not
mutate the current UE process. Apply/Enter waits for a successfully delivered
neutral frame, then asks the existing private restart channel to reload the
**whole** runtime. The old generation remains in the safe panel and displays
reload progress; a save/request failure leaves the panel open with an error.
F9 applies the same gate as a fallback. Do not restart UE alone, and keep all
controls released during the reload.

The visible UE camera receives `SDL_MOUSE_RELATIVE_SPEED_SCALE` at process
startup. For example, the currently used Remote 0.4x setting is a native
SDL/UE input multiplier; it is not an X pointer-acceleration value. With the
default `x11-mirror` base of 0.12 degrees per pixel, status reports
`0.12 x 0.4 = 0.048 deg/px`. That 0.048 value is only the mirror's nominal
arithmetic gain over polled X11 root-pointer pixels. It is not a measured UE
camera sensitivity and does not prove that the mirror and visible camera agree.
The four-axis, multi-turn black-box acceptance below is still required. A
missing or corrupt settings file safely falls back to Local 1.0x.

The launcher also pins SDL to raw relative motion without warp emulation,
viewport scaling, or SDL system-pointer scaling, disables UE
`bEnableMouseSmoothing` and FOV sensitivity scaling, disables SpringArm lag in
the robot-centred view, and adds `r.MotionBlurQuality 0`. The first settings
remove input interpolation; the motion-blur command removes visual streaking.
Neither command changes the selected gain.

For an interactive SONIC `game` launch with the applied profile `Remote`,
`run_sim.sh` additionally snapshots the current X display's acceleration and
threshold, temporarily runs `xset m 1/1 0` before UE starts, and restores the
exact pair at cleanup. This linearizes the absolute X11 stream used by the
panel and `x11-mirror`; it does not replace SDL's raw-relative path and it does
not modify MouseLock. A missing `DISPLAY`, missing `xset`, or X-server failure
only produces a warning and does not block launch. Pointer control is global to
that X display while Matrix is running. Normal exit and handled signals restore
it, but an uncatchable `SIGKILL` or host crash cannot run cleanup; in that case
restore the values printed by the warning/log with `xset m <accel> <threshold>`
or restart the desktop session.

Pointer recentering, window-edge effects, and absolute-coordinate jumps still
require crosshair/MouseLock calibration; the speed scale deliberately does not
reinterpret such jumps as valid movement.

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

Game input is sampled at 50 Hz. Keyboard slow/walk/run defaults are
0.10/0.20/0.30 m/s in native `SLOW_WALK`; gamepad speed remains continuous in
the same range. The input timeout threshold and maximum snapshot age are
0.15 s. The SONIC command is hard-zeroed, without a deceleration tail, when any
of the following occurs:

- startup has not yet received a neutral frame;
- native LowCmd is not fresh or the startup elastic band has not fully released;
- the Matrix window loses focus;
- the adapter observed native V/free-camera mode or the mouse look button is held;
- input times out, becomes stale, disconnects, or is rejected;
- the provider exits, its local socket closes, or an observed camera yaw is
  unavailable.

After startup, focus loss, camera drag, free-camera toggling, timeout, or
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
reports the actual free-camera state, including toggles before provider start.

## Camera-yaw sources

| Source | Use | Limitation |
|---|---|---|
| `fixed` | Safe axis and deadman testing | The command frame does not follow visible camera rotation |
| `x11-mirror` | Heyuan calibration candidate | Integrates polled mouse deltas; press/release order inside one 20 ms sample is ambiguous, and it neither reads nor drives the actual UE camera. Pointer warps, window edges, and recentering can drift |
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

In the native path, **V** toggles free camera and holding the left mouse button
temporarily enters free-camera operation. In `game` mode, use the behavior and
safety interlocks documented above.
