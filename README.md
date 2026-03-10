# SpaceMouse → UR3 Teleoperation

用 3Dconnexion SpaceMouse Compact 实时遥操作 UR3 机械臂。

## 架构

```
SpaceMouse (USB HID)  →  macOS Python  →  UR3 (TCP/IP RTDE)
     6DOF输入            缩放/安全限制       speedL 速度控制
```

支持两种模式：
- **直连模式**（推荐）：macOS 直接通过 RTDE 协议控制 UR3，延迟 < 8ms
- **ROS 模式**：通过 Docker 中的 ROS2 + MoveIt Servo 控制，适合需要碰撞检测/多传感器集成的场景

## 快速开始

### 1. 环境准备

```bash
# 系统依赖
brew install hidapi

# Python 环境
conda create -n sm-ur3 python=3.12
conda activate sm-ur3
pip install -r requirements.txt
```

### 2. macOS 权限设置

- 系统设置 > 隐私与安全性 > 输入监控 → 添加你的终端应用

### 3. 测试 SpaceMouse

```bash
./scripts/start_direct.sh --dry-run
```

移动 SpaceMouse 应能看到数值变化。按住左按钮（dead-man switch）时 `btn=ON`。

### 4. 连接 UR3

修改 `config/teleop_config.yaml` 中的 `robot.ip` 为 UR3 的实际 IP，然后：

```bash
./scripts/start_direct.sh
```

按住 SpaceMouse 左按钮同时移动即可控制机械臂。松开按钮立即停止。

## 安全机制

| 机制 | 说明 |
|------|------|
| Dead-man switch | 必须按住按钮0才能控制，松开立即停止 |
| 速度限制 | 按向量幅值限制（保持方向），默认 0.15 m/s |
| 工作空间边界 | TCP 接近边界时对应方向速度归零，防止撞桌面 |
| 信号处理 | Ctrl+C 时自动发送 `speedStop()` |

## 配置

所有参数在 `config/teleop_config.yaml`：

```yaml
robot:
  ip: "192.168.1.2"         # UR3 的 IP 地址

safety:
  max_linear_speed: 0.15     # m/s，初次使用建议设低
  max_angular_speed: 0.30    # rad/s
  workspace_bounds:
    min: [-0.5, -0.5, 0.02]  # z=0.02 防止撞桌面
    max: [0.5, 0.5, 0.5]

spacemouse:
  deadzone: 0.05             # 死区，过滤微小抖动
  axis_map: ...              # SpaceMouse 轴 → 机器人轴映射
```

## 项目结构

```
src/
  spacemouse_reader.py    # SpaceMouse HID 读取、死区过滤、轴映射
  ur3_controller.py       # UR3 RTDE 控制、安全保护、停止处理
  teleop_direct.py        # 直连模式主程序
  teleop_ros_bridge.py    # ROS 桥接模式主程序
config/
  teleop_config.yaml      # 所有可调参数
scripts/
  start_direct.sh         # 一键启动直连模式
  start_ros.sh            # 一键启动 ROS 模式
docker/                   # ROS2 Docker 环境（可选）
ros2_ws/                  # ROS2 工作空间（可选）
```

## 注意事项

- 3DxWare 驱动会独占 SpaceMouse，启动脚本会自动关闭它
- 首次连接 UR3 前，确保 `max_linear_speed` 设置较低（如 0.05）
- UR3 直连模式使用 RTDE 协议，不需要安装 External Control URCap
