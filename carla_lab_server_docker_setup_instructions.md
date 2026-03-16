# CARLA Lab Server Docker Setup Instructions

This document is for a human colleague or an AI coding assistant working on the lab server.

Goal:

- run CARLA 0.9.16 on the lab server as a Docker container
- keep it headless
- make it remotely startable and stoppable
- make it usable by this repo's bridge script

This guide is written for the CARLA 0.9.16 Docker flow documented by the official CARLA docs:

- https://carla.readthedocs.io/en/0.9.16/build_docker/

## Recommended Architecture

Use this split:

1. Lab server
   - runs the CARLA Docker container
   - owns the GPU
   - exposes the CARLA RPC port
   - exposes the Traffic Manager port you want to use

2. Workstation / client machine
   - runs `carla_alpamayo_bridge.py`
   - connects to the lab server through a named server profile
   - does not manage the CARLA process directly unless you explicitly choose to add an SSH launch command

Recommended default:

- keep CARLA lifecycle management on the lab server
- start and stop with `docker compose`
- connect from this repo using a remote profile

## Why this approach

This is better than trying to make the bridge script own all remote Docker behavior because:

- GPU scheduling and container startup are server-ops concerns
- Docker daemon access and SSH access are security-sensitive
- CARLA is often longer-lived than a single inference script
- the bridge remains focused on connection, health checking, capture, and inference

## Server Prerequisites

The lab server should be:

- Linux
- NVIDIA GPU available
- NVIDIA driver installed and working
- Docker installed
- NVIDIA Container Toolkit installed

The CARLA docs for 0.9.16 explicitly require Docker plus NVIDIA Container Toolkit for the containerized flow.

## Step 1: Verify GPU and Docker on the lab server

Run on the lab server:

```bash
nvidia-smi
docker --version
docker run --rm hello-world
```

If Docker requires sudo and that is not desired for routine operation, add the intended operator user to the Docker group and re-login.

## Step 2: Install NVIDIA Container Toolkit if needed

Follow the current NVIDIA Container Toolkit installation instructions for the lab server OS.

After installation, verify GPU visibility inside containers on the lab server.

Example sanity check:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

If this fails, do not proceed to CARLA until containerized GPU access works.

## Step 3: Pull the official CARLA 0.9.16 image

On the lab server:

```bash
docker pull carlasim/carla:0.9.16
```

This is the image version referenced by the official CARLA 0.9.16 Docker docs.

## Step 4: Create a dedicated directory on the lab server

Recommended:

```bash
mkdir -p ~/carla-0.9.16-docker
cd ~/carla-0.9.16-docker
```

## Step 5: Create a Docker Compose file

Create `docker-compose.yml` on the lab server with a service like this:

```yaml
services:
  carla:
    image: carlasim/carla:0.9.16
    container_name: carla-0.9.16
    restart: unless-stopped
    network_mode: host
    gpus: all
    environment:
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: all
    command:
      - bash
      - CarlaUE4.sh
      - -RenderOffScreen
      - -nosound
      - -carla-rpc-port=2000
```

Why this shape:

- `network_mode: host`
  - matches the official CARLA 0.9.16 Docker guidance
  - avoids container-to-host port translation weirdness
- `-RenderOffScreen`
  - runs headlessly
- `-nosound`
  - avoids irrelevant audio stack issues
- `-carla-rpc-port=2000`
  - makes the CARLA RPC endpoint predictable for this repo

## Step 6: Start the container on the lab server

From the same directory:

```bash
docker compose up -d
```

Check status:

```bash
docker compose ps
docker logs --tail=200 carla-0.9.16
```

## Step 7: Verify CARLA is reachable on the lab server

On the lab server:

```bash
ss -ltnp | rg 2000
```

You should see the CARLA process bound on the expected port.

If you plan to use Traffic Manager at port `8010`, make sure that port is also available for CARLA Traffic Manager usage from clients.

## Step 8: Network and firewall requirements

From the workstation that will run `carla_alpamayo_bridge.py`, the lab server must be reachable on:

- CARLA RPC port: `2000`
- Traffic Manager port: `8010` or whatever you standardize on

Recommended:

- do not expose these ports broadly to the public Internet
- restrict access by:
  - internal lab network
  - VPN
  - firewall allowlist for trusted client IPs

If you cannot open the ports directly, use SSH tunneling instead.

## Step 9: Recommended remote control commands

On the lab server, these are the clean operational commands:

Start:

```bash
cd ~/carla-0.9.16-docker
docker compose up -d
```

Stop:

```bash
cd ~/carla-0.9.16-docker
docker compose stop
```

Hard stop and remove:

```bash
cd ~/carla-0.9.16-docker
docker compose down
```

Logs:

```bash
cd ~/carla-0.9.16-docker
docker compose logs -f
```

## Step 10: Remote start from your workstation

Recommended manual remote start pattern:

```bash
ssh your_user@your_lab_server 'cd ~/carla-0.9.16-docker && docker compose up -d'
```

Recommended manual remote stop pattern:

```bash
ssh your_user@your_lab_server 'cd ~/carla-0.9.16-docker && docker compose stop'
```

This is the safest default because it keeps remote process lifecycle explicit.

## Step 11: Configure this repo to use the lab server

On the workstation in this repo, copy `carla_server_profiles.example.toml` to `carla_server_profiles.toml`, then update the local `carla_server_profiles.toml`.

Example remote profile:

```toml
[profiles.lab_docker]
description = "CARLA 0.9.16 Docker container running on the lab server."
host = "your.lab.server.hostname"
port = 2000
tm_port = 8010
connect_timeout_seconds = 10.0
startup_timeout_seconds = 90.0
launch_if_unreachable = false
terminate_on_exit = false
```

Then run:

```bash
conda activate alpamayo_carla
python /home/sangeetsu/alpamayo1_carla/carla_alpamayo_bridge.py --server-profile lab_docker
```

## Optional: Add SSH launch commands to the bridge profile

This is optional, not the default recommendation.

If you want the bridge script itself to trigger remote startup, you can define a profile like:

```toml
[profiles.lab_docker]
description = "CARLA 0.9.16 Docker container running on the lab server."
host = "your.lab.server.hostname"
port = 2000
tm_port = 8010
connect_timeout_seconds = 10.0
startup_timeout_seconds = 90.0
launch_if_unreachable = true
terminate_on_exit = false
launch_command = [
  "ssh",
  "your_user@your_lab_server",
  "cd ~/carla-0.9.16-docker && docker compose up -d"
]
```

Caveat:

- this requires SSH auth to be working non-interactively
- this couples bridge execution to remote ops behavior
- this is less robust than a separate explicit remote start command

## Operational Checks for the Colleague or AI Assistant

Before handing the setup back, verify all of the following on the lab server:

1. `docker compose up -d` starts the CARLA container successfully
2. `docker compose ps` shows the service as running
3. `docker logs carla-0.9.16` does not show immediate fatal startup errors
4. `nvidia-smi` on the host shows the containerized CARLA process using the GPU
5. The workstation can reach `host:2000`
6. If using Traffic Manager remotely, the chosen TM port is reachable and not already occupied

## What to send back after setup

Ask the colleague or AI assistant to provide:

- the lab server hostname or IP
- the final CARLA RPC port
- the final Traffic Manager port
- the exact `docker-compose.yml`
- the exact remote start command
- whether SSH key-based access is already configured
- any firewall or VPN constraints the client machine must satisfy

## Notes

- This repo's bridge has already been validated locally against CARLA 0.9.16 in headless mode.
- For remote usage, the most important failure modes are:
  - host firewall
  - wrong exposed port
  - broken NVIDIA container runtime
  - occupied Traffic Manager port
  - SSH command requiring interactive auth

## Sources

Official CARLA 0.9.16 docs used:

- CARLA in Docker:
  - https://carla.readthedocs.io/en/0.9.16/build_docker/
- CARLA Python API:
  - https://carla.readthedocs.io/en/0.9.16/python_api/
