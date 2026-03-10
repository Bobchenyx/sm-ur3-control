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
- 3DxWare driver exclusively locks the raw HID device — scripts detect this and prompt user to kill it (virtual device uses vendor-specific protocol, cannot be used as alternative)
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

## Verification Status / 验证状态

各组件验证情况，标注验证方式和日期。

### ✅ Verified / 已验证

| Component | How Verified | Date |
|-----------|-------------|------|
| `src/spacemouse_reader.py` | dry-run 实测，6轴数据 + 按钮均正常响应 | 2026-03-10 |
| `src/teleop_direct.py --dry-run` | 实际运行，确认 SpaceMouse 数据读取、deadzone 过滤、axis mapping、dead-man switch 均工作 | 2026-03-10 |
| `docker/Dockerfile` | `docker compose build` 成功，所有 apt 包安装通过，colcon build 通过 | 2026-03-10 |
| `ros2_ws/src/sm_teleop/` (colcon build) | Docker 内 colcon build 成功，`twist_relay_node` 可执行文件正确安装到 `lib/sm_teleop/` | 2026-03-10 |
| Docker ROS2 launch (fake hardware) | `ros2 launch sm_teleop ur3_teleop.launch.py use_fake_hardware:=true` — 所有 11 个节点启动成功：ros2_control, move_group ("You can start planning now!"), servo_node_main, rosbridge (port 9090), twist_relay_node | 2026-03-10 |
| macOS → rosbridge 连通性 | 从 macOS 用 roslibpy 连接 Docker 内 rosbridge，确认 57 个 topic 可见，包括 `/servo_node/delta_twist_cmds` 和 `/spacemouse/twist` | 2026-03-10 |
| conda 环境 `sm-ur3` | Python 3.12, hidapi/numpy/pyyaml/roslibpy 均安装验证通过 | 2026-03-10 |

### ❌ Not Yet Verified / 未验证

| Component | What Needs Testing | Blocker |
|-----------|-------------------|---------|
| `src/ur3_controller.py` | 连接真实 UR3，验证 RTDE 连接、`speedL` 指令、workspace enforcement、emergency stop | 需要 UR3 机械臂在线 + `ur-rtde` pip 安装 |
| `src/teleop_direct.py` (live mode) | 完整直连遥操作：SpaceMouse 输入 → UR3 运动 | 需要 UR3 在线 |
| `src/teleop_ros_bridge.py` | macOS 发送 TwistStamped → Docker rosbridge → twist_relay → servo → UR3 运动 | 需要 UR3 在线 |
| `scripts/start_ros.sh` | 完整一键启动流程：Docker build + wait for rosbridge + SpaceMouse bridge | 需要 UR3 在线做端到端测试 |
| `scripts/start_direct.sh` (live) | 完整一键启动直连模式 | 需要 UR3 在线 |
| Docker ROS2 launch (real hardware) | `use_fake_hardware:=false` with real UR3 | 需要 UR3 在线 + External Control URCap |
| Axis mapping correctness | SpaceMouse 轴方向与 UR3 实际运动方向的对应关系 | 需要实际操作验证，可能需要调整 config 中的 sign |
| Dead-man switch (live) | 松开按钮后 UR3 是否立即停止 | 需要 UR3 在线 |
| Workspace bounds enforcement | TCP 接近边界时速度是否正确归零 | 需要 UR3 在线 |
| Emergency stop (SIGINT) | Ctrl+C 后 UR3 是否执行 `speedStop()` | 需要 UR3 在线 |

### 已知问题 / Known Issues

- `pyspacemouse` 库在 macOS 上不可用（`easyhid` 找不到 hidapi 动态库 + 无法选择正确 HID 接口），已改用 `hid` 库直接读取
- 3DxWare 驱动会独占 SpaceMouse HID 设备，必须先 kill 相关进程
- MoveIt Servo 的 kinematics.yaml 未传给 servo_node（使用 inverse Jacobian 替代，功能正常但无 IK solver）
- `move_group` 的 `use_sim_time` 参数必须是 bool 类型，不能传字符串

## UR3 Robot Notes

- RTDE ports: 30001-30004 (primary/secondary), 50001-50003 (RTDE)
- Joint names: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
- The UR3 requires the External Control URCap for the ROS driver; direct mode uses RTDE which doesn't need it
- `speedL` is used for velocity control (maps naturally to SpaceMouse 6DOF input)

## macOS-Specific

- SpaceMouse requires Input Monitoring permission: System Settings > Privacy & Security > Input Monitoring (grant to Terminal/IDE)
- 3DxWare driver (`3DconnexionHelper`, `3DxNLServer`, etc.) exclusively locks the SpaceMouse HID device — must be killed before running
- `ur-rtde` on Apple Silicon needs `cmake` and `boost` from Homebrew: `brew install cmake boost`
