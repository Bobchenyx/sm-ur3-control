"""UR3 robot controller via RTDE with safety enforcement."""

import signal
import sys
import time

import numpy as np

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None
    print("[UR3] Warning: ur-rtde not installed. Install with: pip install ur-rtde")


class UR3Controller:
    """Safe velocity-based control of UR3 via RTDE."""

    def __init__(self, config: dict):
        robot_cfg = config["robot"]
        safety_cfg = config["safety"]
        control_cfg = config["control"]

        self.ip = robot_cfg["ip"]
        self.frequency = robot_cfg["rtde_frequency"]
        self.acceleration = control_cfg["acceleration"]
        self.dt = 1.0 / control_cfg["loop_rate"]

        self.max_linear = safety_cfg["max_linear_speed"]
        self.max_angular = safety_cfg["max_angular_speed"]
        self.ws_min = np.array(safety_cfg["workspace_bounds"]["min"])
        self.ws_max = np.array(safety_cfg["workspace_bounds"]["max"])

        self._rtde_c = None
        self._rtde_r = None
        self._connected = False
        self._original_sigint = None
        self._original_sigterm = None

    def connect(self) -> bool:
        """Connect to the UR3 via RTDE. Returns True on success."""
        if rtde_control is None:
            print("[UR3] ur-rtde library not available")
            return False

        try:
            print(f"[UR3] Connecting to {self.ip}...")
            self._rtde_r = rtde_receive.RTDEReceiveInterface(self.ip)
            self._rtde_c = rtde_control.RTDEControlInterface(self.ip)
            self._connected = True
            self._install_signal_handlers()
            print(f"[UR3] Connected. TCP pose: {self._format_pose(self.get_tcp_pose())}")
            return True
        except Exception as e:
            print(f"[UR3] Connection failed: {e}")
            print(f"  Check that the robot is powered on and reachable at {self.ip}")
            return False

    def get_tcp_pose(self) -> list:
        """Get current TCP pose [x, y, z, rx, ry, rz]."""
        if not self._connected:
            return [0.0] * 6
        return self._rtde_r.getActualTCPPose()

    def get_joint_positions(self) -> list:
        """Get current joint positions [q0..q5] in radians."""
        if not self._connected:
            return [0.0] * 6
        return self._rtde_r.getActualQ()

    def send_velocity(self, linear: list, angular: list):
        """Send Cartesian velocity command with safety enforcement.

        Args:
            linear: [vx, vy, vz] in m/s, robot base frame
            angular: [wrx, wry, wrz] in rad/s, robot base frame
        """
        if not self._connected:
            return

        linear = np.array(linear, dtype=float)
        angular = np.array(angular, dtype=float)

        # Clamp velocity magnitudes
        linear = self._clamp_vector(linear, self.max_linear)
        angular = self._clamp_vector(angular, self.max_angular)

        # Workspace enforcement: zero out components moving out of bounds
        linear = self._enforce_workspace(linear)

        speed_vector = list(linear) + list(angular)
        self._rtde_c.speedL(speed_vector, self.acceleration, self.dt)

    def stop(self):
        """Immediately stop robot motion."""
        if self._connected and self._rtde_c is not None:
            try:
                self._rtde_c.speedStop()
            except Exception:
                pass

    def disconnect(self):
        """Stop robot and disconnect RTDE interfaces."""
        self.stop()
        if self._rtde_c is not None:
            try:
                self._rtde_c.stopScript()
            except Exception:
                pass
        self._connected = False
        self._restore_signal_handlers()
        print("[UR3] Disconnected")

    def is_connected(self) -> bool:
        return self._connected

    def _enforce_workspace(self, linear: np.ndarray) -> np.ndarray:
        """Zero out velocity components that would move TCP outside workspace."""
        pose = self.get_tcp_pose()
        pos = np.array(pose[:3])

        for i in range(3):
            if pos[i] <= self.ws_min[i] and linear[i] < 0:
                linear[i] = 0.0
            elif pos[i] >= self.ws_max[i] and linear[i] > 0:
                linear[i] = 0.0

        return linear

    @staticmethod
    def _clamp_vector(v: np.ndarray, max_mag: float) -> np.ndarray:
        mag = np.linalg.norm(v)
        if mag > max_mag:
            return v * (max_mag / mag)
        return v

    @staticmethod
    def _format_pose(pose: list) -> str:
        pos = [f"{v:.4f}" for v in pose[:3]]
        rot = [f"{v:.3f}" for v in pose[3:]]
        return f"pos=[{', '.join(pos)}] rot=[{', '.join(rot)}]"

    def _install_signal_handlers(self):
        """Install signal handlers to stop robot on SIGINT/SIGTERM."""
        self._original_sigint = signal.getsignal(signal.SIGINT)
        self._original_sigterm = signal.getsignal(signal.SIGTERM)

        def _handler(signum, frame):
            print("\n[UR3] Signal received, stopping robot...")
            self.stop()
            self.disconnect()
            sys.exit(0)

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)

    def _restore_signal_handlers(self):
        if self._original_sigint is not None:
            signal.signal(signal.SIGINT, self._original_sigint)
        if self._original_sigterm is not None:
            signal.signal(signal.SIGTERM, self._original_sigterm)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
