#!/usr/bin/env python3

# Copyright (c) 2026
#
# This script bridges live CARLA RGB camera streams into the Alpamayo 1
# inference interface. It captures a short synchronized history from a CARLA
# ego vehicle, converts it into the tensor layout expected by Alpamayo, and
# prints the resulting chain-of-causation and predicted trajectory.

from __future__ import annotations

import argparse
import queue
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import random
import socket
import subprocess
import sys
import time
import tomllib
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parent
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import carla
import numpy as np
import torch

from alpamayo_r1 import helper
from alpamayo_r1.models.alpamayo_r1 import AlpamayoR1


DEFAULT_SERVER_CONFIG = REPO_ROOT / "carla_server_profiles.toml"

CAMERA_SPECS = (
    {
        "name": "camera_cross_left_120fov",
        "index": 0,
        "fov": 120.0,
        "transform": carla.Transform(
            carla.Location(x=1.7, y=-0.35, z=1.7),
            carla.Rotation(pitch=-2.0, yaw=-90.0, roll=0.0),
        ),
    },
    {
        "name": "camera_front_wide_120fov",
        "index": 1,
        "fov": 120.0,
        "transform": carla.Transform(
            carla.Location(x=1.9, y=0.0, z=1.7),
            carla.Rotation(pitch=-2.0, yaw=0.0, roll=0.0),
        ),
    },
    {
        "name": "camera_cross_right_120fov",
        "index": 2,
        "fov": 120.0,
        "transform": carla.Transform(
            carla.Location(x=1.7, y=0.35, z=1.7),
            carla.Rotation(pitch=-2.0, yaw=90.0, roll=0.0),
        ),
    },
    {
        "name": "camera_front_tele_30fov",
        "index": 6,
        "fov": 30.0,
        "transform": carla.Transform(
            carla.Location(x=1.9, y=0.0, z=1.7),
            carla.Rotation(pitch=-1.0, yaw=0.0, roll=0.0),
        ),
    },
)


@dataclass
class CameraSample:
    frame: int
    timestamp: float
    image_rgb: np.ndarray


@dataclass
class ServerProfile:
    name: str
    host: str = "127.0.0.1"
    port: int = 2000
    tm_port: int = 8000
    connect_timeout_seconds: float = 10.0
    startup_timeout_seconds: float = 60.0
    launch_if_unreachable: bool = False
    terminate_on_exit: bool = False
    working_directory: str | None = None
    launch_command: list[str] | None = None
    description: str | None = None

    @classmethod
    def from_table(cls, name: str, table: dict[str, Any]) -> "ServerProfile":
        launch_command = table.get("launch_command")
        if launch_command is not None and not isinstance(launch_command, list):
            raise ValueError(f"profile `{name}` launch_command must be a TOML array of strings")
        return cls(
            name=name,
            host=table.get("host", "127.0.0.1"),
            port=int(table.get("port", 2000)),
            tm_port=int(table.get("tm_port", 8000)),
            connect_timeout_seconds=float(table.get("connect_timeout_seconds", 10.0)),
            startup_timeout_seconds=float(table.get("startup_timeout_seconds", 60.0)),
            launch_if_unreachable=bool(table.get("launch_if_unreachable", False)),
            terminate_on_exit=bool(table.get("terminate_on_exit", False)),
            working_directory=table.get("working_directory"),
            launch_command=launch_command,
            description=table.get("description"),
        )


class CameraStream:
    def __init__(
        self,
        world: carla.World,
        parent: carla.Actor,
        spec: dict[str, object],
        width: int,
        height: int,
        sensor_tick: float,
    ) -> None:
        self.name = str(spec["name"])
        self.index = int(spec["index"])
        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(width))
        blueprint.set_attribute("image_size_y", str(height))
        blueprint.set_attribute("fov", str(spec["fov"]))
        # In synchronous mode we want a frame every world tick. CARLA cameras
        # can skip ticks when sensor_tick is set exactly equal to the fixed
        # delta, so we leave it at 0.0 to publish every simulation step.
        blueprint.set_attribute("sensor_tick", "0.0")

        self._queue: queue.Queue[CameraSample] = queue.Queue()
        self.sensor = world.spawn_actor(
            blueprint,
            spec["transform"],
            attach_to=parent,
            attachment_type=carla.AttachmentType.Rigid,
        )
        self.sensor.listen(self._on_image)

    def _on_image(self, image: carla.Image) -> None:
        raw = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        rgb = raw[:, :, :3][:, :, ::-1].copy()
        self._queue.put(CameraSample(frame=image.frame, timestamp=image.timestamp, image_rgb=rgb))

    def get_for_frame(self, frame: int, timeout: float = 5.0) -> CameraSample:
        while True:
            try:
                sample = self._queue.get(timeout=timeout)
            except queue.Empty as exc:
                raise RuntimeError(
                    f"Timed out waiting for synchronized frame {frame} from `{self.name}`."
                ) from exc
            if sample.frame < frame:
                continue
            if sample.frame != frame:
                raise RuntimeError(
                    f"{self.name} returned frame {sample.frame}, expected synchronized frame {frame}"
                )
            return sample

    def destroy(self) -> None:
        self.sensor.stop()
        self.sensor.destroy()


def log(message: str) -> None:
    print(f"[carla_alpamayo_bridge] {message}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture CARLA camera frames and run one Alpamayo inference pass."
    )
    parser.add_argument(
        "--server-config",
        default=str(DEFAULT_SERVER_CONFIG),
        help="TOML file containing CARLA server connection and launch profiles",
    )
    parser.add_argument(
        "--server-profile",
        default=None,
        help="Profile name inside the server config to use",
    )
    parser.add_argument("--host", default=None, help="Override CARLA host")
    parser.add_argument("--port", type=int, default=None, help="Override CARLA RPC port")
    parser.add_argument("--tm-port", type=int, default=None, help="Override Traffic Manager port")
    parser.add_argument(
        "--model-id",
        default="nvidia/Alpamayo-R1-10B",
        help="Hugging Face model id for Alpamayo weights",
    )
    parser.add_argument(
        "--vehicle-role-name",
        default="hero",
        help="Reuse an existing vehicle with this CARLA role_name if present",
    )
    parser.add_argument(
        "--vehicle-filter",
        default="vehicle.tesla.model3",
        help="Blueprint filter used when spawning a new ego vehicle",
    )
    parser.add_argument(
        "--no-spawn-if-missing",
        action="store_true",
        help="Fail instead of spawning a new ego vehicle when no matching role_name actor exists",
    )
    parser.add_argument(
        "--no-autopilot",
        action="store_true",
        help="Leave the selected vehicle uncontrolled instead of enabling autopilot",
    )
    parser.add_argument(
        "--image-width",
        type=int,
        default=512,
        help="Camera width. 512x320 matches Alpamayo's processor minimum pixel count.",
    )
    parser.add_argument("--image-height", type=int, default=320, help="Camera height")
    parser.add_argument(
        "--fixed-delta-seconds",
        type=float,
        default=0.1,
        help="Synchronous simulation delta. Alpamayo expects 10 Hz trajectories by default.",
    )
    parser.add_argument("--history-steps", type=int, default=16, help="Ego history steps")
    parser.add_argument("--frames-per-camera", type=int, default=4, help="Frames per camera")
    parser.add_argument(
        "--warmup-ticks",
        type=int,
        default=20,
        help="Ticks to collect before running inference",
    )
    parser.add_argument(
        "--num-traj-samples",
        type=int,
        default=1,
        help="Trajectory samples to request from Alpamayo",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.6,
        help="Sampling temperature for the VLM rollout",
    )
    parser.add_argument(
        "--top-p",
        type=float,
        default=0.98,
        help="Top-p sampling parameter for the VLM rollout",
    )
    return parser.parse_args()


def load_server_profiles(config_path: Path) -> tuple[str | None, dict[str, ServerProfile]]:
    if not config_path.exists():
        return None, {}

    with config_path.open("rb") as handle:
        config = tomllib.load(handle)

    active_profile = config.get("active_profile")
    profile_tables = config.get("profiles", {})
    profiles = {
        name: ServerProfile.from_table(name, table)
        for name, table in profile_tables.items()
    }
    return active_profile, profiles


def resolve_server_profile(args: argparse.Namespace) -> tuple[ServerProfile, Path]:
    config_path = Path(args.server_config).expanduser().resolve()
    active_profile, profiles = load_server_profiles(config_path)

    profile_name = args.server_profile or active_profile or "inline_defaults"
    profile = profiles.get(profile_name, ServerProfile(name=profile_name))

    if args.host is not None:
        profile.host = args.host
    if args.port is not None:
        profile.port = args.port
    if args.tm_port is not None:
        profile.tm_port = args.tm_port

    return profile, config_path


def try_make_client(profile: ServerProfile) -> carla.Client | None:
    try:
        with socket.create_connection(
            (profile.host, profile.port),
            timeout=min(2.0, profile.connect_timeout_seconds),
        ):
            pass
    except OSError:
        return None

    try:
        client = carla.Client(profile.host, profile.port)
        client.set_timeout(profile.connect_timeout_seconds)
        client.get_world()
        return client
    except RuntimeError:
        return None


def wait_for_server(profile: ServerProfile, timeout_seconds: float) -> carla.Client | None:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        client = try_make_client(profile)
        if client is not None:
            return client
        time.sleep(1.0)
    return None


def maybe_launch_server(profile: ServerProfile) -> subprocess.Popen[str] | None:
    if not profile.launch_if_unreachable:
        return None
    if not profile.launch_command:
        raise RuntimeError(
            f"Profile `{profile.name}` is marked launch_if_unreachable=true but has no launch_command."
        )

    cwd = None
    if profile.working_directory:
        cwd = Path(profile.working_directory).expanduser()

    log(f"Launching CARLA server via profile `{profile.name}`...")
    return subprocess.Popen(profile.launch_command, cwd=cwd)


def ensure_server(profile: ServerProfile) -> tuple[carla.Client, subprocess.Popen[str] | None]:
    client = wait_for_server(profile, timeout_seconds=1.0)
    if client is not None:
        log(
            f"Connected to existing CARLA server `{profile.name}` at "
            f"{profile.host}:{profile.port} (tm_port={profile.tm_port})."
        )
        return client, None

    launched_process = maybe_launch_server(profile)
    if launched_process is None:
        raise RuntimeError(
            f"CARLA server `{profile.name}` is not reachable at {profile.host}:{profile.port}. "
            f"No launch command is configured for this profile."
        )

    client = wait_for_server(profile, timeout_seconds=profile.startup_timeout_seconds)
    if client is None:
        raise RuntimeError(
            f"CARLA server `{profile.name}` did not become ready within "
            f"{profile.startup_timeout_seconds} seconds."
        )
    log(
        f"Connected to launched CARLA server `{profile.name}` at "
        f"{profile.host}:{profile.port} (tm_port={profile.tm_port})."
    )
    return client, launched_process


def log_initial_health_check(client: carla.Client, world: carla.World, profile: ServerProfile) -> None:
    try:
        server_version = client.get_server_version()
    except RuntimeError:
        server_version = "unknown"
    try:
        client_version = client.get_client_version()
    except RuntimeError:
        client_version = "unknown"

    snapshot = world.get_snapshot()
    map_name = world.get_map().name
    log(
        "Health check [rpc]: "
        f"profile={profile.name} host={profile.host} port={profile.port} "
        f"server_version={server_version} client_version={client_version} "
        f"map={map_name} frame={snapshot.frame}"
    )


def run_sync_tick_health_check(world: carla.World) -> None:
    pre_snapshot = world.get_snapshot()
    tick_frame = world.tick()
    post_snapshot = world.get_snapshot()
    if tick_frame != post_snapshot.frame:
        raise RuntimeError(
            "CARLA tick health check failed: returned frame does not match the latest snapshot frame."
        )
    if tick_frame <= pre_snapshot.frame:
        raise RuntimeError(
            "CARLA tick health check failed: world frame did not advance after synchronous tick."
        )
    log(
        "Health check [tick]: "
        f"pre_frame={pre_snapshot.frame} tick_frame={tick_frame} post_frame={post_snapshot.frame} "
        f"delta_seconds={post_snapshot.timestamp.delta_seconds:.3f}"
    )


def prime_sensor_streams(world: carla.World, sensors: list[CameraStream]) -> None:
    log(f"Priming {len(sensors)} camera streams before capture...")
    frame = world.tick()
    for sensor in sensors:
        sensor.get_for_frame(frame, timeout=10.0)
    log(f"Sensor priming complete at frame {frame}.")


def load_model(model_id: str) -> tuple[AlpamayoR1, object]:
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required to run Alpamayo inference.")

    log(f"Loading Alpamayo model `{model_id}` on CUDA...")
    model = AlpamayoR1.from_pretrained(model_id, dtype=torch.bfloat16).to("cuda")
    processor = helper.get_processor(model.tokenizer)
    return model, processor


def find_existing_vehicle(world: carla.World, role_name: str) -> carla.Vehicle | None:
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def spawn_vehicle(world: carla.World, role_name: str, blueprint_filter: str) -> carla.Vehicle:
    blueprint_library = world.get_blueprint_library().filter(blueprint_filter)
    if not blueprint_library:
        raise RuntimeError(f"No CARLA blueprints matched filter `{blueprint_filter}`")

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points available in the current CARLA map.")

    blueprint = random.choice(blueprint_library)
    blueprint.set_attribute("role_name", role_name)
    if blueprint.has_attribute("color"):
        color = random.choice(blueprint.get_attribute("color").recommended_values)
        blueprint.set_attribute("color", color)

    random.shuffle(spawn_points)
    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            log(f"Spawned ego vehicle `{vehicle.type_id}` at {spawn_point.location}.")
            return vehicle

    raise RuntimeError("Failed to spawn a CARLA ego vehicle.")


def select_vehicle(
    world: carla.World,
    role_name: str,
    blueprint_filter: str,
    spawn_if_missing: bool,
) -> tuple[carla.Vehicle, bool]:
    vehicle = find_existing_vehicle(world, role_name)
    if vehicle is not None:
        log(f"Using existing vehicle `{vehicle.type_id}` with role_name `{role_name}`.")
        return vehicle, False

    if not spawn_if_missing:
        raise RuntimeError(
            f"No existing vehicle with role_name `{role_name}` found. "
            "Omit --no-spawn-if-missing to let the script create one."
        )

    return spawn_vehicle(world, role_name, blueprint_filter), True


def append_history(
    history_transforms: deque[carla.Transform],
    vehicle: carla.Vehicle,
) -> None:
    history_transforms.append(vehicle.get_transform())


def localize_history(transforms: Iterable[carla.Transform]) -> tuple[np.ndarray, np.ndarray]:
    transform_list = list(transforms)
    if not transform_list:
        raise ValueError("No transforms available to localize.")

    t0 = transform_list[-1]
    t0_inv = np.asarray(t0.get_inverse_matrix(), dtype=np.float32)

    xyz_list = []
    rot_list = []
    for transform in transform_list:
        matrix = np.asarray(transform.get_matrix(), dtype=np.float32)
        local_matrix = t0_inv @ matrix
        xyz_list.append(local_matrix[:3, 3])
        rot_list.append(local_matrix[:3, :3])

    return np.stack(xyz_list, axis=0), np.stack(rot_list, axis=0)


def build_model_inputs(
    history_transforms: deque[carla.Transform],
    camera_buffers: dict[str, deque[CameraSample]],
    camera_streams: list[CameraStream],
    processor: object,
) -> dict[str, torch.Tensor]:
    sorted_streams = sorted(camera_streams, key=lambda stream: stream.index)
    image_frames = []
    timestamps = []

    for stream in sorted_streams:
        samples = list(camera_buffers[stream.name])
        if len(samples) != camera_buffers[stream.name].maxlen:
            raise RuntimeError(f"Camera `{stream.name}` does not have enough buffered frames.")
        frame_tensor = torch.stack(
            [torch.from_numpy(sample.image_rgb).permute(2, 0, 1) for sample in samples], dim=0
        )
        image_frames.append(frame_tensor)
        timestamps.append(torch.tensor([sample.timestamp for sample in samples], dtype=torch.float32))

    image_frames_tensor = torch.stack(image_frames, dim=0)
    timestamp_tensor = torch.stack(timestamps, dim=0)
    relative_timestamps = timestamp_tensor - timestamp_tensor.min()

    ego_history_xyz, ego_history_rot = localize_history(history_transforms)
    ego_history_xyz_tensor = torch.from_numpy(ego_history_xyz).float().unsqueeze(0).unsqueeze(0)
    ego_history_rot_tensor = torch.from_numpy(ego_history_rot).float().unsqueeze(0).unsqueeze(0)

    messages = helper.create_message(image_frames_tensor.flatten(0, 1))
    tokenized = processor.apply_chat_template(
        messages,
        tokenize=True,
        add_generation_prompt=False,
        continue_final_message=True,
        return_dict=True,
        return_tensors="pt",
    )

    model_inputs = {
        "tokenized_data": tokenized,
        "ego_history_xyz": ego_history_xyz_tensor,
        "ego_history_rot": ego_history_rot_tensor,
        "camera_indices": torch.tensor([stream.index for stream in sorted_streams], dtype=torch.int64),
        "relative_timestamps": relative_timestamps,
        "absolute_timestamps": timestamp_tensor,
    }
    return helper.to_device(model_inputs, "cuda")


def run_inference(
    model: AlpamayoR1,
    processor: object,
    history_transforms: deque[carla.Transform],
    camera_buffers: dict[str, deque[CameraSample]],
    camera_streams: list[CameraStream],
    args: argparse.Namespace,
) -> None:
    model_inputs = build_model_inputs(history_transforms, camera_buffers, camera_streams, processor)
    torch.cuda.manual_seed_all(42)

    with torch.autocast("cuda", dtype=torch.bfloat16):
        pred_xyz, pred_rot, extra = model.sample_trajectories_from_data_with_vlm_rollout(
            data=model_inputs,
            top_p=args.top_p,
            temperature=args.temperature,
            num_traj_samples=args.num_traj_samples,
            max_generation_length=256,
            return_extra=True,
        )

    cot = extra["cot"][0]
    print("\nChain-of-Causation:")
    print(cot)

    pred_xyz_np = pred_xyz.detach().float().cpu().numpy()[0, 0, 0]
    if pred_xyz_np.ndim == 2 and pred_xyz_np.shape[0] == 3:
        pred_xyz_np = pred_xyz_np.T
    print("\nPredicted trajectory shape:", pred_xyz_np.shape)
    print("First 8 waypoints in local ego frame [x, y, z]:")
    print(np.round(pred_xyz_np[:8], 3))

    pred_rot_np = pred_rot.detach().float().cpu().numpy()[0, 0, 0]
    print("\nPredicted rotation tensor shape:", pred_rot_np.shape)


def main() -> int:
    args = parse_args()
    profile, config_path = resolve_server_profile(args)
    log(
        f"Using CARLA server profile `{profile.name}` from `{config_path}` "
        f"targeting {profile.host}:{profile.port} (tm_port={profile.tm_port})."
    )
    if profile.description:
        log(profile.description)

    model, processor = load_model(args.model_id)
    client, launched_process = ensure_server(profile)

    world = client.get_world()
    log_initial_health_check(client, world, profile)
    traffic_manager = client.get_trafficmanager(profile.tm_port)

    original_settings = world.get_settings()
    vehicle: carla.Vehicle | None = None
    sensors: list[CameraStream] = []
    spawned_vehicle = False
    autopilot_enabled_by_script = False

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.fixed_delta_seconds
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(True)
        run_sync_tick_health_check(world)

        vehicle, spawned_vehicle = select_vehicle(
            world,
            args.vehicle_role_name,
            args.vehicle_filter,
            not args.no_spawn_if_missing,
        )

        if not args.no_autopilot:
            vehicle.set_autopilot(True, traffic_manager.get_port())
            autopilot_enabled_by_script = True
            log("Autopilot enabled for ego vehicle.")

        sensors = [
            CameraStream(
                world=world,
                parent=vehicle,
                spec=spec,
                width=args.image_width,
                height=args.image_height,
                sensor_tick=args.fixed_delta_seconds,
            )
            for spec in CAMERA_SPECS
        ]
        prime_sensor_streams(world, sensors)

        history_transforms: deque[carla.Transform] = deque(maxlen=args.history_steps)
        camera_buffers: dict[str, deque[CameraSample]] = {
            sensor.name: deque(maxlen=args.frames_per_camera) for sensor in sensors
        }

        required_ticks = max(args.warmup_ticks, args.history_steps, args.frames_per_camera)
        log(
            f"Collecting synchronized CARLA data for {required_ticks} ticks "
            f"({args.history_steps} history steps, {args.frames_per_camera} frames per camera)..."
        )

        for tick_idx in range(required_ticks):
            frame = world.tick()
            append_history(history_transforms, vehicle)
            for sensor in sensors:
                sample = sensor.get_for_frame(frame)
                camera_buffers[sensor.name].append(sample)
            if (tick_idx + 1) % 5 == 0 or tick_idx + 1 == required_ticks:
                log(f"captured tick {tick_idx + 1}/{required_ticks}")

        run_inference(model, processor, history_transforms, camera_buffers, sensors, args)
        return 0

    finally:
        for sensor in sensors:
            try:
                sensor.destroy()
            except RuntimeError:
                pass

        if vehicle is not None and autopilot_enabled_by_script:
            try:
                vehicle.set_autopilot(False)
            except RuntimeError:
                pass

        if spawned_vehicle and vehicle is not None:
            try:
                vehicle.destroy()
            except RuntimeError:
                pass

        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)

        if launched_process is not None and profile.terminate_on_exit:
            log(f"Stopping launched CARLA server process for profile `{profile.name}`...")
            launched_process.terminate()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        raise SystemExit(130)
