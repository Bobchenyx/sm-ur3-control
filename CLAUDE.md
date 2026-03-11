# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SpaceMouse Compact → UR3 robot arm teleoperation system. macOS host reads the SpaceMouse via HID and sends 6DOF velocity commands to the UR3, either directly (ur_rtde) or through a Docker-based ROS2 stack (MoveIt Servo).

## Architecture

```
Mode 1 - Direct (recommended, no ROS):
  macOS: SpaceMouse (HID) → spacemouse_reader.py → ur3_controller.py → UR3 (RTDE servoL)

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

# ur-rtde build dependencies (if pip install ur-rtde fails)
conda install -c conda-forge cmake=3.31 boost=1.84
PATH="/opt/anaconda3/envs/sm-ur3/bin:$PATH" CMAKE_PREFIX_PATH="/opt/anaconda3/envs/sm-ur3" pip install ur-rtde
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
- `robot.ip` — UR3 IP address on LAN (default 192.168.0.2)
- `spacemouse.axis_map` — Mapping between SpaceMouse and robot base frame axes (with sign flips)
- `safety.max_linear_speed` / `max_angular_speed` — Velocity limits (start low)
- `safety.workspace_bounds` — Soft workspace limits (z min prevents table collision)
- `control.deadman_button` — SpaceMouse button that must be held for motion

## Safety Design

- **Dead-man switch**: Robot only moves while SpaceMouse button 0 is held
- **Workspace bounds**: Velocity components moving TCP outside bounds are zeroed (soft limits, not hard-stop)
- **Velocity clamping**: By vector magnitude (preserves direction) not per-axis
- **Signal handlers**: `ur3_controller.py` catches SIGINT/SIGTERM and calls `servoStop()` before exit
- **ROS timeout watchdog**: `twist_relay_node.py` sends zero velocity if no input received within 150ms

## File Roles

- `src/spacemouse_reader.py` — HID reading via `hidapi`, deadzone filtering, axis mapping, velocity scaling. Opens device by `usage_page=1, usage=8` to select the correct HID interface.
- `src/ur3_controller.py` — ur_rtde wrapper using `servoL` for real-time control, with workspace enforcement and safe shutdown
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
| Docker ROS2 launch (fake hardware) | `ros2 launch sm_teleop ur3_teleop.launch.py use_fake_hardware:=true` — 所有 11 个节点启动成功 | 2026-03-10 |
| macOS → rosbridge 连通性 | 从 macOS 用 roslibpy 连接 Docker 内 rosbridge，确认 57 个 topic 可见 | 2026-03-10 |
| conda 环境 `sm-ur3` | Python 3.12, hidapi/numpy/pyyaml/roslibpy/ur-rtde 均安装验证通过 | 2026-03-11 |
| `ur-rtde` RTDE 连接 | `RTDEReceiveInterface` + `RTDEControlInterface` 均连接成功，能读 TCP 位姿和关节角 | 2026-03-11 |
| `src/ur3_controller.py` (servoL) | 连接真实 UR3 (192.168.0.2)，servoL 控制 Z 轴平移正常 | 2026-03-11 |
| `moveJ` 安全姿态复位 | 通过 `moveJ` 将 UR3 移至远离奇异点的姿态 [0,-50,-90,-40,-90,0]°，正常工作 | 2026-03-11 |
| Dead-man switch (live) | 松开 SpaceMouse 按钮后 UR3 立即停止运动，多次验证一致 | 2026-03-11 |
| Emergency stop (SIGINT) | Ctrl+C 后正常执行 `servoStop()` + 断开连接，UR3 安全停止 | 2026-03-11 |
| `src/teleop_direct.py` (live, 平移) | SpaceMouse 平移输入 → UR3 Z 轴运动正常，X 轴运动正常 | 2026-03-11 |
| `speedL` → 保护停止问题复现 | `speedL` 在遥操作中方向切换时触发 "position deviates from path: Base"，已确认并改用 `servoL` | 2026-03-11 |

### ❌ Not Yet Verified / 未验证

| Component | What Needs Testing | Blocker |
|-----------|-------------------|---------|
| `src/teleop_direct.py` (full 6DOF live) | 完整 6DOF 遥操作：平移 + 旋转同时控制 | 需要调整 rotation_scale 参数后验证 |
| Axis mapping correctness | SpaceMouse 轴方向与 UR3 实际运动方向的对应关系 | 需要更多实际操作验证，可能需要调整 sign |
| Workspace bounds enforcement | TCP 接近边界时速度是否正确归零 | 需要 UR3 在线，向边界移动测试 |
| `scripts/start_direct.sh` (live) | 完整一键启动直连模式（脚本流程，非手动命令） | 直接运行脚本验证 |
| `src/teleop_ros_bridge.py` | macOS 发送 TwistStamped → Docker rosbridge → twist_relay → servo → UR3 运动 | 需要 UR3 在线 |
| `scripts/start_ros.sh` | 完整一键启动流程 | 需要 UR3 在线做端到端测试 |
| Docker ROS2 launch (real hardware) | `use_fake_hardware:=false` with real UR3 | 需要 UR3 在线 + External Control URCap |

### 已知问题 / Known Issues

- `pyspacemouse` 库在 macOS 上不可用（`easyhid` 找不到 hidapi 动态库 + 无法选择正确 HID 接口），已改用 `hid` 库直接读取
- 3DxWare 驱动会独占 SpaceMouse HID 设备，必须先 kill 相关进程
- `speedL` 不适合遥操作：其内部轨迹规划器在方向切换时触发 "position deviates from path: Base" 保护停止，已改用 `servoL`
- `ur-rtde` 在 macOS (Apple Silicon) + CMake 4.x + Boost 1.90 下编译失败，需要 conda 安装 cmake=3.31 + boost=1.84
- MoveIt Servo 的 kinematics.yaml 未传给 servo_node（使用 inverse Jacobian 替代，功能正常但无 IK solver）
- `move_group` 的 `use_sim_time` 参数必须是 bool 类型，不能传字符串

## UR3 Robot Notes

- Robot IP: 192.168.0.2 (subnet 255.255.255.0)
- RTDE ports: 30001-30004 (primary/secondary), 50001-50003 (RTDE)
- Joint names: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
- The UR3 requires the External Control URCap for the ROS driver; direct mode uses RTDE which doesn't need it
- Use `servoL` for real-time teleoperation control (not `speedL` — see Known Issues)
- Wrist singularity: avoid `wrist_2_joint ≈ 0°` — use safe pose [0,-50,-90,-40,-90,0]° as home position
- Safe pose home: `moveJ([0, -50, -90, -40, -90, 0]°, speed=0.3, accel=0.3)`

## macOS-Specific

- SpaceMouse requires Input Monitoring permission: System Settings > Privacy & Security > Input Monitoring (grant to Terminal/IDE)
- 3DxWare driver (`3DconnexionHelper`, `3DxNLServer`, etc.) exclusively locks the SpaceMouse HID device — must be killed before running
- `ur-rtde` on Apple Silicon needs `cmake` and `boost` via conda (not Homebrew — Boost 1.90 missing `boost_system` cmake config)
