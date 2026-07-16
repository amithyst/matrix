#!/usr/bin/env python3
"""Run AndroidTwin/SONIC MuJoCo physics and mirror it into a Matrix UE map."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
from types import SimpleNamespace


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--aue-root", type=Path, required=True)
    parser.add_argument("--gear-sonic-root", type=Path, required=True)
    parser.add_argument("--unitree-sdk-root", type=Path, required=True)
    parser.add_argument(
        "--control-source", choices=("planner", "pico", "external"), default="planner"
    )
    parser.add_argument("--planner-bind", default="tcp://0.0.0.0:5556")
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=9999)
    parser.add_argument(
        "--no-render-sync",
        action="store_true",
        help="Run physics/control without publishing state to a Matrix UE process",
    )
    parser.add_argument("--physics-hz", type=float, default=200.0)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--max-seconds", type=float, default=0.0)
    parser.add_argument("--walk-after", type=float, default=-1.0)
    parser.add_argument("--vx", type=float, default=0.30)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--status-file", type=Path)
    parser.add_argument("--print-every", type=float, default=2.0)
    parser.add_argument(
        "--startup-band",
        action="store_true",
        help="Hold the root through SONIC's three-second INIT ramp, then fade out",
    )
    parser.add_argument("--startup-band-hold", type=float, default=4.0)
    parser.add_argument("--startup-band-fade", type=float, default=3.0)
    parser.add_argument("--spawn-x", type=float)
    parser.add_argument("--spawn-y", type=float)
    parser.add_argument("--spawn-z", type=float)
    parser.add_argument("--spawn-yaw", type=float)
    return parser.parse_args()


def _atomic_json(path: Path | None, payload: dict[str, object]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary_path = Path(stream.name)
    os.replace(temporary_path, path)


def _base_yaw(qpos) -> float:
    w, x, y, z = [float(value) for value in qpos[3:7]]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _root_up_z(qpos) -> float:
    """World-Z component of the floating base's local up axis."""
    _, x, y, _ = [float(value) for value in qpos[3:7]]
    return 1.0 - 2.0 * (x * x + y * y)


def _apply_spawn_pose(
    qpos,
    *,
    x: float | None,
    y: float | None,
    z: float | None,
    yaw: float | None,
) -> None:
    """Apply an optional world-frame root pose to a free-base MuJoCo qpos."""
    if len(qpos) < 7:
        raise ValueError("spawn pose requires a free-base qpos with at least 7 values")
    values = [value for value in (x, y, z, yaw) if value is not None]
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("spawn pose values must be finite")
    if x is not None:
        qpos[0] = x
    if y is not None:
        qpos[1] = y
    if z is not None:
        qpos[2] = z
    if yaw is not None:
        qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def _configure_reused_runtime(args: argparse.Namespace) -> None:
    aue_src = args.aue_root.resolve() / "src"
    if not (aue_src / "androidtwin/control/sonic_sim/fused_sink.py").is_file():
        raise SystemExit(f"AndroidTwin FusedSink is missing below: {aue_src}")
    deploy_binary = (
        args.gear_sonic_root.resolve()
        / "gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
    )
    sdk_library = (
        args.unitree_sdk_root.resolve() / "lib/x86_64/libunitree_sdk2.a"
    )
    if not deploy_binary.is_file():
        raise SystemExit(f"SONIC deploy binary is missing: {deploy_binary}")
    if not sdk_library.is_file():
        raise SystemExit(f"Unitree SDK library is missing: {sdk_library}")

    os.environ["ANDROIDTWIN_GEAR_SONIC_ROOT"] = str(args.gear_sonic_root.resolve())
    os.environ["ANDROIDTWIN_GR00T_ROOT"] = str(args.gear_sonic_root.resolve())
    os.environ["ANDROIDTWIN_GEAR_SONIC_DEPLOY_ROOT"] = str(
        args.gear_sonic_root.resolve() / "gear_sonic_deploy"
    )
    os.environ["ANDROIDTWIN_UNITREE_SDK2_ROOT"] = str(args.unitree_sdk_root.resolve())
    sys.path.insert(0, str(aue_src))


def main() -> int:
    args = _parse_args()
    _configure_reused_runtime(args)

    try:
        import mujoco
        import numpy as np
        from androidtwin.control.sonic_sim import FusedSink, SonicPlannerSender
        from androidtwin.control.sonic_sim.planner_sender import nav_to_planner_command
        from matrix_render_protocol import MatrixRenderPublisher, packet_size
    except ImportError as exc:
        raise SystemExit(
            f"Matrix SONIC runtime dependency is missing: {exc}. "
            "Install research/sonic_integration/requirements-trna.txt."
        ) from exc

    model_path = args.model.resolve()
    if not model_path.is_file():
        raise SystemExit(f"composed Matrix model is missing: {model_path}")
    if args.control_hz <= 0.0:
        raise SystemExit("--control-hz must be positive")
    if args.startup_band_hold < 0.0 or args.startup_band_fade < 0.0:
        raise SystemExit("startup band hold/fade durations must be non-negative")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    if args.physics_hz <= 0.0:
        raise SystemExit("--physics-hz must be positive")
    model.opt.timestep = 1.0 / args.physics_hz
    data = mujoco.MjData(model)
    try:
        _apply_spawn_pose(
            data.qpos,
            x=args.spawn_x,
            y=args.spawn_y,
            z=args.spawn_z,
            yaw=args.spawn_yaw,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    mujoco.mj_forward(model, data)
    initial_root_xy = np.asarray(data.qpos[:2], dtype=np.float64).copy()
    physics_hz = 1.0 / float(model.opt.timestep)
    substeps_float = physics_hz / args.control_hz
    substeps = int(round(substeps_float))
    if substeps <= 0 or not math.isclose(substeps_float, substeps, rel_tol=0.0, abs_tol=1e-6):
        raise SystemExit(
            f"control_hz={args.control_hz} must divide physics_hz={physics_hz} exactly"
        )
    if model.nu < 29:
        raise SystemExit(f"SONIC requires at least 29 actuators, model has {model.nu}")

    env = SimpleNamespace(model=model, data=data)
    sink = None
    planner = None
    renderer = (
        None
        if args.no_render_sync
        else MatrixRenderPublisher(args.render_host, args.render_port)
    )
    running = True

    def request_stop(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        sink = FusedSink(
            env,
            network_interface="lo",
            real_mode=False,
            spawn_pico_manager=args.control_source == "pico",
            zmq_manager_port=int(args.planner_bind.rsplit(":", 1)[-1]),
            enable_elastic_band=False,
            enable_startup_elastic_band=args.startup_band,
            startup_elastic_hold_s=args.startup_band_hold,
            startup_elastic_fade_s=args.startup_band_fade,
            accept_body_cmds=True,
        )
        if args.control_source == "planner":
            planner = SonicPlannerSender(bind_endpoint=args.planner_bind, auto_start=True)

        expected_packet_size = packet_size(nq=model.nq, nv=model.nv, nu=model.nu)
        render_target = (
            "disabled"
            if renderer is None
            else f"{args.render_host}:{args.render_port}"
        )
        print(
            "matrix-sonic-runtime "
            f"model={model_path} nq={model.nq} nv={model.nv} nu={model.nu} "
            f"ngeom={model.ngeom} physics_hz={physics_hz:.1f} "
            f"control_hz={args.control_hz:.1f} substeps={substeps} "
            f"render={render_target} packet_bytes={expected_packet_size} "
            f"control_source={args.control_source}",
            flush=True,
        )

        started_wall = time.perf_counter()
        next_frame_wall = started_wall
        next_print = started_wall
        last_print_wall = started_wall
        last_render_count = 0
        last_physics_steps = 0
        control_frames = 0
        active_frames = 0
        physics_steps = 0
        instability_resets = 0
        unstable = False
        fall_detected = False
        min_root_z = float(data.qpos[2])
        active_started_wall = None
        walking = False

        while running:
            frame_wall = time.perf_counter()
            elapsed_wall = frame_wall - started_wall
            if args.max_seconds > 0.0 and elapsed_wall >= args.max_seconds:
                break

            if planner is not None:
                active_elapsed = (
                    frame_wall - active_started_wall
                    if active_started_wall is not None
                    else 0.0
                )
                walking = (
                    active_started_wall is not None
                    and args.walk_after >= 0.0
                    and active_elapsed >= args.walk_after
                )
                nav = np.asarray(
                    [args.vx if walking else 0.0, args.vy if walking else 0.0, args.yaw_rate if walking else 0.0],
                    dtype=np.float32,
                )
                command = nav_to_planner_command(
                    nav,
                    base_yaw=_base_yaw(data.qpos),
                    height_cmd=0.78,
                    idle_height=0.78,
                    native_walk_speed=True,
                    planner_local_frame=True,
                )
                planner.send(command, start=True)

            fallback_ctrl = np.zeros(model.nu, dtype=np.float64)
            sink.write(env, fallback_ctrl)
            if sink.has_body_cmd:
                if active_started_wall is None:
                    active_started_wall = time.perf_counter()
                for _ in range(substeps):
                    sink.apply_substep_forces(env)
                    previous_sim_time = float(data.time)
                    mujoco.mj_step(model, data)
                    physics_steps += 1
                    if float(data.time) + 1e-9 < previous_sim_time:
                        instability_resets += 1
                        unstable = True
                        running = False
                        print(
                            "matrix-sonic-runtime ERROR MuJoCo time moved backwards; "
                            "the model was reset after numerical instability",
                            flush=True,
                        )
                        break
                sink.clear_substep_forces(env)
                active_frames += 1

                root_z = float(data.qpos[2])
                root_up_z = _root_up_z(data.qpos)
                min_root_z = min(min_root_z, root_z)
                fall_detected = fall_detected or root_z < 0.5 or root_up_z < 0.5

            if renderer is not None:
                renderer.send(data.time, data.qpos, data.qvel, data.ctrl)
            control_frames += 1

            now = time.perf_counter()
            if now >= next_print:
                window_wall = max(now - last_print_wall, 1e-9)
                render_count = renderer.packet_count if renderer is not None else 0
                window_render = render_count - last_render_count
                window_physics_steps = physics_steps - last_physics_steps
                status = {
                    "active_elapsed_s": round(
                        now - active_started_wall, 3
                    ) if active_started_wall is not None else 0.0,
                    "active_lowcmd": bool(sink.has_body_cmd),
                    "control_frames": control_frames,
                    "control_hz": args.control_hz,
                    "elapsed_wall_s": round(now - started_wall, 3),
                    "model": str(model_path),
                    "ngeom": int(model.ngeom),
                    "nu": int(model.nu),
                    "fall_detected": fall_detected,
                    "min_root_z": round(min_root_z, 5),
                    "physics_hz_target": physics_hz,
                    "physics_step_hz": round(window_physics_steps / window_wall, 3),
                    "render_hz": round(window_render / window_wall, 3),
                    "render_packet_bytes": expected_packet_size,
                    "render_sync_enabled": renderer is not None,
                    "ue_state_sync_hz": round(window_render / window_wall, 3),
                    "root_xyz": [round(float(value), 5) for value in data.qpos[:3]],
                    "root_displacement_xy_m": round(
                        float(np.linalg.norm(np.asarray(data.qpos[:2]) - initial_root_xy)), 5
                    ),
                    "root_up_z": round(_root_up_z(data.qpos), 5),
                    "rtf": round(
                        (window_physics_steps * float(model.opt.timestep)) / window_wall,
                        4,
                    ),
                    "sim_time_s": round(float(data.time), 4),
                    "instability_resets": instability_resets,
                    "startup_band_enabled": bool(args.startup_band),
                    "startup_band_hold_s": args.startup_band_hold,
                    "startup_band_fade_s": args.startup_band_fade,
                    "walking_commanded": walking,
                }
                print(f"matrix-sonic-runtime status={json.dumps(status, sort_keys=True)}", flush=True)
                _atomic_json(args.status_file, status)
                last_print_wall = now
                last_render_count = render_count
                last_physics_steps = physics_steps
                next_print = now + max(args.print_every, 0.1)

            next_frame_wall += 1.0 / args.control_hz
            sleep_s = next_frame_wall - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            elif sleep_s < -(2.0 / args.control_hz):
                next_frame_wall = time.perf_counter()

        print(
            "matrix-sonic-runtime stopped "
            f"wall_s={time.perf_counter() - started_wall:.2f} "
            f"sim_s={data.time:.2f} frames={control_frames} active_frames={active_frames}",
            flush=True,
        )
        return 1 if unstable else 0
    finally:
        if planner is not None:
            planner.close()
        if sink is not None:
            sink.close()
        if renderer is not None:
            renderer.close()


if __name__ == "__main__":
    raise SystemExit(main())
