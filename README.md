# SpaceMouse → UR3 Teleoperation

Real-time teleoperation of a UR3 robot arm using a 3Dconnexion SpaceMouse Compact.

## Architecture

```
SpaceMouse (USB HID)  →  macOS Python  →  UR3 (TCP/IP RTDE)
     6DOF input          scaling/safety      speedL velocity control
```

Two control modes:
- **Direct mode** (recommended): macOS controls UR3 directly via RTDE protocol, < 8ms latency
- **ROS mode**: Docker-based ROS2 Humble + MoveIt Servo, for collision detection / multi-sensor integration

## Quick Start

### 1. Environment Setup

```bash
# System dependency
brew install hidapi

# Python environment
conda create -n sm-ur3 python=3.12
conda activate sm-ur3
pip install -r requirements.txt
```

### 2. macOS Permissions

- System Settings > Privacy & Security > Input Monitoring → add your terminal app

### 3. Test SpaceMouse

```bash
./scripts/start_direct.sh --dry-run
```

Move the SpaceMouse to see values change. Hold left button (dead-man switch) to see `btn=ON`.

### 4. Connect to UR3

Set `robot.ip` in `config/teleop_config.yaml` to your UR3's IP address, then:

```bash
./scripts/start_direct.sh
```

Hold the SpaceMouse left button and move to control the robot. Release to stop immediately.

### 5. ROS Mode (optional)

Requires Docker Desktop:

```bash
./scripts/start_ros.sh
```

## Safety

| Mechanism | Description |
|-----------|-------------|
| Dead-man switch | Must hold button 0 to move, release stops immediately |
| Velocity clamping | By vector magnitude (preserves direction), default 0.15 m/s |
| Workspace bounds | Velocity zeroed in directions that would exceed bounds |
| Signal handlers | Ctrl+C triggers `speedStop()` before exit |

## Configuration

All parameters in `config/teleop_config.yaml`:

```yaml
robot:
  ip: "192.168.1.2"         # UR3 IP address on LAN

safety:
  max_linear_speed: 0.15     # m/s — start low on first use
  max_angular_speed: 0.30    # rad/s
  workspace_bounds:
    min: [-0.5, -0.5, 0.02]  # z=0.02 prevents table collision
    max: [0.5, 0.5, 0.5]

spacemouse:
  deadzone: 0.05             # Filters small noise when idle
  axis_map: ...              # SpaceMouse axis → robot axis mapping
```

## Project Structure

```
src/
  spacemouse_reader.py    # SpaceMouse HID reading, deadzone, axis mapping
  ur3_controller.py       # UR3 RTDE control, safety enforcement, shutdown
  teleop_direct.py        # Direct mode main loop
  teleop_ros_bridge.py    # ROS bridge mode main loop
config/
  teleop_config.yaml      # All tunable parameters
scripts/
  start_direct.sh         # One-command direct mode launch
  start_ros.sh            # One-command ROS mode launch
docker/                   # ROS2 Docker environment (optional)
ros2_ws/                  # ROS2 workspace (optional)
```

## Notes

- 3DxWare driver exclusively locks the SpaceMouse — start scripts kill it automatically
- On first UR3 connection, set `max_linear_speed` low (e.g. 0.05)
- Direct mode uses RTDE protocol, no External Control URCap needed
