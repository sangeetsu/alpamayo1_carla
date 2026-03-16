# CARLA Alpamayo Bridge Instructions

## Files

The current bridge implementation lives in this repo:

- `/home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py`
- `/home/sangeetsu/alpamayo1_carla/carla_server_profiles.example.toml`
- `/home/sangeetsu/alpamayo1_carla/carla_server_profiles.toml` (local, gitignored)
- `/home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge_instructions.md`
- `/home/sangeetsu/alpamayo1_carla/TODO.md`

The bridge script is kept in the Alpamayo repo so the model-facing integration logic stays with the model code rather than inside CARLA's example tree.

## Current Architecture

The bridge has four layers:

1. Server profile layer
   - Reads named CARLA targets from `carla_server_profiles.toml`
   - Supports host, RPC port, Traffic Manager port, timeouts, optional launch command, and working directory

2. Server lifecycle layer
   - Tries to connect to the selected server profile
   - Launches the server only if the profile allows it
   - Waits for the server to become reachable before doing any inference work

3. Health-check layer
   - Runs an RPC health check after connection
   - Logs server version, client version, map name, and snapshot frame
   - Runs a synchronous tick health check and verifies the frame advances correctly before inference capture starts

4. Capture and inference layer
   - Reuses a `hero` vehicle if present, otherwise spawns one by default
   - Attaches four RGB cameras approximating Alpamayo's expected rig
   - Primes the camera streams once before the real capture loop
   - Captures synchronized history and image frames
   - Converts that into Alpamayo tensors
   - Runs one Alpamayo rollout and prints the chain-of-causation and predicted trajectory

## Server Profiles

The default profile file is:

`/home/sangeetsu/alpamayo1_carla/carla_server_profiles.toml`

This local file is gitignored so internal hosts, usernames, and launch commands are not committed.
Create it by copying:

`/home/sangeetsu/alpamayo1_carla/carla_server_profiles.example.toml`

### `local_package`

This is the current default and the one that has been tested.

- Host: `127.0.0.1`
- RPC port: `2000`
- Traffic Manager port: `8010`
- Working directory: `/home/sangeetsu/CARLA/CARLA_0.9.16`
- Launch command:

```bash
./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
```

This launches the packaged CARLA 0.9.16 server headlessly.

### `lab_docker`

This is an example remote profile.

Recommended usage:

- set the real remote host and exposed CARLA port
- keep `launch_if_unreachable = false`
- manage Docker lifecycle outside the bridge script

That is the better operational pattern for a lab machine.

## Why profiles are better than just host/port

A plain host/port flag is not enough once you have:

- a local packaged CARLA
- a remote Dockerized CARLA
- different Traffic Manager ports
- different startup timeouts
- optional local launch behavior

The profile file gives you a stable place to store those environment-specific details without turning the script into a pile of ad hoc flags.

## Health Checks

The bridge currently logs two server health checks before capture:

### RPC health check

This verifies the server is reachable and logs:

- profile name
- host
- RPC port
- server version
- client version
- map name
- current frame

Example:

```text
[carla_alpamayo_bridge] Health check [rpc]: profile=local_package host=127.0.0.1 port=2000 server_version=0.9.16 client_version=0.9.16 map=Carla/Maps/Town10HD_Opt frame=8
```

### Tick health check

This verifies the world can advance synchronously before inference starts.

It logs:

- pre-tick frame
- returned tick frame
- post-tick frame
- `delta_seconds`

Example:

```text
[carla_alpamayo_bridge] Health check [tick]: pre_frame=28 tick_frame=29 post_frame=29 delta_seconds=0.100
```

## Camera and Capture Behavior

The current bridge attaches four RGB cameras:

- `camera_cross_left_120fov`
- `camera_front_wide_120fov`
- `camera_cross_right_120fov`
- `camera_front_tele_30fov`

Important implementation details:

- The rig is an approximation of Alpamayo's training layout, not an exact physical reproduction
- Cameras use `sensor_tick = 0.0` so they publish every synchronous world tick
- The script primes the sensors once before the actual capture loop
- Capture defaults:
  - `16` ego-history steps
  - `4` frames per camera
  - `0.1` second fixed delta
  - `20` warmup ticks

## Environment

Use the `alpamayo_carla` conda environment:

```bash
source /home/sangeetsu/miniconda3/etc/profile.d/conda.sh
conda activate alpamayo_carla
```

The env already contains:

- Alpamayo inference dependencies
- CARLA 0.9.16 Python API wheel for Python 3.12
- `pygame`
- `shapely`
- `opencv-python`

You also need:

1. Hugging Face access to `nvidia/Alpamayo-R1-10B`
2. CUDA available in the environment
3. A CARLA server that is either already reachable or launchable through the selected profile

## Basic Usage

Before the first run on a fresh checkout:

```bash
cp /home/sangeetsu/alpamayo1_carla/carla_server_profiles.example.toml /home/sangeetsu/alpamayo1_carla/carla_server_profiles.toml
```

Run with the default tested local profile:

```bash
python /home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py
```

Select a profile explicitly:

```bash
python /home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py --server-profile local_package
python /home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py --server-profile lab_docker
```

Override a profile ad hoc:

```bash
python /home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py --server-profile lab_docker --host 10.0.0.25 --port 3000 --tm-port 9000
```

Use a different profile file:

```bash
python /home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py --server-config /path/to/your_profiles.toml --server-profile my_server
```

## Useful Flags

```bash
python /home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py --help
```

Common flags:

- `--server-config /path/to/file.toml`
- `--server-profile local_package`
- `--host 127.0.0.1`
- `--port 2000`
- `--tm-port 8010`
- `--vehicle-role-name hero`
- `--vehicle-filter vehicle.tesla.model3`
- `--no-spawn-if-missing`
- `--no-autopilot`
- `--image-width 512`
- `--image-height 320`
- `--fixed-delta-seconds 0.1`
- `--history-steps 16`
- `--frames-per-camera 4`
- `--warmup-ticks 20`
- `--num-traj-samples 1`

## Verified Local Test

The current implementation has been tested successfully against the packaged local server at:

`/home/sangeetsu/CARLA/CARLA_0.9.16/CarlaUE4.sh`

using the `local_package` profile in headless mode.

Observed successful flow:

- launched CARLA headlessly
- passed RPC health check
- passed synchronous tick health check
- spawned a local ego vehicle
- primed the sensor rig
- captured all synchronized ticks
- ran Alpamayo inference successfully

Example inference result from the verified run:

- CoT: `Adapt speed for the right curve ahead.`
- Predicted trajectory shape: `(64, 3)`

## Remote / Lab Server Guidance

For a remote Dockerized CARLA server, the better pattern is:

- use a dedicated profile in `carla_server_profiles.toml`
- connect to the published host/port
- leave `launch_if_unreachable = false`
- manage the Docker container lifecycle outside the bridge script

Examples of better lifecycle management:

- `docker compose up -d`
- systemd
- tmux or screen
- an explicit SSH wrapper script

The bridge can support remote launch commands if you choose to add one, but that is intentionally not the default.

## TODO Tracking

Planned follow-ups are recorded in:

`/home/sangeetsu/alpamayo1_carla/TODO.md`

Current items:

- SSH-based remote-launch profile example for the lab server
- Docker Compose example for CARLA server lifecycle
- stronger health-check work

## Official CARLA Docs Consulted

- CARLA 0.9.16 quick start:
  - https://carla.readthedocs.io/en/0.9.16/start_quickstart/
- CARLA 0.9.16 Docker docs:
  - https://carla.readthedocs.io/en/0.9.16/build_docker/
- CARLA 0.9.16 Python API docs:
  - https://carla.readthedocs.io/en/0.9.16/python_api/
