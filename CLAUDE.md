# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SpaceMouse Compact → UR3 robot arm teleoperation system. macOS host reads the SpaceMouse via HID and sends 6DOF velocity commands to the UR3, either directly (ur_rtde) or through a Docker-based ROS2 stack (MoveIt Servo).

## Architecture

```
Mode 1 - Direct (recommended, no ROS):
  macOS: SpaceMouse (HID) → spacemouse_reader.py → ur3_controller.py → UR3 (RTDE TCP/IP)

Mode 2 - ROS (for MoveIt/multi-sensor integration):
  macOS: SpaceMouse → spacemouse_reader.py → roslibpy (WebSocket:9090)
  Docker: rosbridge → twist_relay_node → MoveIt Servo → ur_robot_driver → UR3
```

Key constraints:
- macOS Docker doesn't support USB passthrough → SpaceMouse always read on macOS host
- 3DxWare driver must be killed before running (`killall 3DconnexionHelper 3DxNLServer`) — it exclusively locks the HID device
- SpaceMouse is read via `hidapi` directly (not `pyspacemouse`, which has macOS compatibility issues with `easyhid`)

## Environment Setup

```bash
# Python environment (conda)
conda activate sm-ur3    # Python 3.12, at /opt/anaconda3/envs/sm-ur3/

# System dependencies
brew install hidapi

# Python dependencies
pip install -r requirements.txt
```

## Commands

```bash
# Direct mode (no Docker/ROS needed)
./scripts/start_direct.sh                 # Full startup (kills 3DxWare, launches teleop)
./scripts/start_direct.sh --dry-run       # Test SpaceMouse without robot connection

# ROS mode (requires Docker Desktop)
./scripts/start_ros.sh                    # Builds Docker, starts rosbridge + SpaceMouse bridge

# Docker only
docker compose -f docker/docker-compose.yml up -d --build
docker compose -f docker/docker-compose.yml logs -f
docker compose -f docker/docker-compose.yml down
```

## Key Configuration

All tunable parameters in `config/teleop_config.yaml`:
- `robot.ip` — UR3 IP address on LAN (default 192.168.1.2)
- `spacemouse.axis_map` — Mapping between SpaceMouse and robot base frame axes (with sign flips)
- `safety.max_linear_speed` / `max_angular_speed` — Velocity limits (start low)
- `safety.workspace_bounds` — Soft workspace limits (z min prevents table collision)
- `control.deadman_button` — SpaceMouse button that must be held for motion

## Safety Design

- **Dead-man switch**: Robot only moves while SpaceMouse button 0 is held
- **Workspace bounds**: Velocity components moving TCP outside bounds are zeroed (soft limits, not hard-stop)
- **Velocity clamping**: By vector magnitude (preserves direction) not per-axis
- **Signal handlers**: `ur3_controller.py` catches SIGINT/SIGTERM and calls `speedStop()` before exit
- **ROS timeout watchdog**: `twist_relay_node.py` sends zero velocity if no input received within 150ms

## File Roles

- `src/spacemouse_reader.py` — HID reading via `hidapi`, deadzone filtering, axis mapping, velocity scaling. Opens device by `usage_page=1, usage=8` to select the correct HID interface.
- `src/ur3_controller.py` — ur_rtde wrapper with workspace enforcement and safe shutdown
- `src/teleop_direct.py` — Main loop for direct mode (125Hz, `--dry-run` for SpaceMouse-only testing)
- `src/teleop_ros_bridge.py` — Main loop for ROS bridge mode (publishes TwistStamped via roslibpy)
- `ros2_ws/src/sm_teleop/` — ROS2 package with twist relay node and launch file
- `docker/` — Dockerfile (ROS2 Humble + ur_robot_driver + MoveIt Servo + rosbridge)

## UR3 Robot Notes

- RTDE ports: 30001-30004 (primary/secondary), 50001-50003 (RTDE)
- Joint names: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
- The UR3 requires the External Control URCap for the ROS driver; direct mode uses RTDE which doesn't need it
- `speedL` is used for velocity control (maps naturally to SpaceMouse 6DOF input)

## macOS-Specific

- SpaceMouse requires Input Monitoring permission: System Settings > Privacy & Security > Input Monitoring (grant to Terminal/IDE)
- 3DxWare driver (`3DconnexionHelper`, `3DxNLServer`, etc.) exclusively locks the SpaceMouse HID device — must be killed before running
- `ur-rtde` on Apple Silicon needs `cmake` and `boost` from Homebrew: `brew install cmake boost`
