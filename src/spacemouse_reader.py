"""Read SpaceMouse Compact on macOS via HID and provide scaled 6DOF velocity."""

import struct
import time
from dataclasses import dataclass

import hid
import numpy as np

# 3Dconnexion SpaceMouse Compact
VENDOR_ID = 0x256F
PRODUCT_ID = 0xC635  # SpaceMouse Compact


@dataclass
class SpaceMouseState:
    """6DOF velocity state from SpaceMouse after scaling and deadzone."""
    vx: float = 0.0  # linear velocity, robot frame
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0  # angular velocity, robot frame
    wy: float = 0.0
    wz: float = 0.0
    buttons: list = None
    timestamp: float = 0.0

    def __post_init__(self):
        if self.buttons is None:
            self.buttons = []

    @property
    def linear(self) -> list:
        return [self.vx, self.vy, self.vz]

    @property
    def angular(self) -> list:
        return [self.wx, self.wy, self.wz]

    @property
    def is_zero(self) -> bool:
        return all(abs(v) < 1e-9 for v in self.linear + self.angular)


class SpaceMouseReader:
    """Reads SpaceMouse Compact via hidapi and outputs scaled, deadzone-filtered 6DOF velocity."""

    # SpaceMouse Compact raw axis range: approx -350 to +350
    RAW_SCALE = 350.0

    def __init__(self, config: dict):
        sm_cfg = config["spacemouse"]
        safety_cfg = config["safety"]

        self.deadzone = sm_cfg["deadzone"]
        self.translation_scale = np.array(sm_cfg["translation_scale"])
        self.rotation_scale = np.array(sm_cfg["rotation_scale"])
        self.axis_map = sm_cfg["axis_map"]
        self.max_linear = safety_cfg["max_linear_speed"]
        self.max_angular = safety_cfg["max_angular_speed"]

        self._device = None
        self._connected = False
        # Current raw state (updated incrementally by HID reports)
        self._raw = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._buttons = [0, 0]

    def open(self) -> bool:
        """Open the SpaceMouse device. Returns True on success."""
        try:
            # Find the correct HID interface (usage_page=1, usage=8 for multi-axis controller)
            devices = hid.enumerate(VENDOR_ID, PRODUCT_ID)
            if not devices:
                print("[SpaceMouse] No SpaceMouse Compact found. Check USB connection.")
                return False

            # Prefer usage_page=1 usage=8 (Generic Desktop / Multi-axis Controller)
            # which is the interface that sends motion reports
            target = None
            for d in devices:
                if d["usage_page"] == 1 and d["usage"] == 8:
                    target = d
                    break
            if target is None:
                target = devices[0]  # fallback to first interface

            self._device = hid.device()
            self._device.open_path(target["path"])
            self._device.set_nonblocking(True)
            product = self._device.get_product_string()
            self._connected = True
            print(f"[SpaceMouse] Connected: {product}")
            return True
        except IOError as e:
            print(f"[SpaceMouse] Failed to open device: {e}")
            # Check if 3DxWare is locking the device
            if self._is_3dxware_running():
                print("  3DxWare driver is running and exclusively locks the SpaceMouse.")
                print("  Please quit 3DxWare first, or run:")
                print("    killall 3DconnexionHelper 3DxNLServer 3DxRadialMenu 3DxVirtualNumpad")
            else:
                print("  Check USB connection and Input Monitoring permission")
                print("  macOS: System Settings > Privacy & Security > Input Monitoring")
            self._device = None
            return False

    def read(self) -> SpaceMouseState:
        """Read current SpaceMouse state, apply deadzone/scaling/clamping."""
        if not self._connected:
            return SpaceMouseState(timestamp=time.time())

        # Read all pending HID reports (non-blocking)
        while True:
            data = self._device.read(64)
            if not data:
                break
            self._parse_hid_report(data)

        # Normalize raw values to [-1, 1]
        raw_axes = {k: v / self.RAW_SCALE for k, v in self._raw.items()}

        # Apply deadzone
        for k in raw_axes:
            if abs(raw_axes[k]) < self.deadzone:
                raw_axes[k] = 0.0

        # Map SpaceMouse axes → robot axes with sign and scaling
        am = self.axis_map
        vx = raw_axes[am["x"]["sm_axis"]] * am["x"]["sign"] * self.translation_scale[0]
        vy = raw_axes[am["y"]["sm_axis"]] * am["y"]["sign"] * self.translation_scale[1]
        vz = raw_axes[am["z"]["sm_axis"]] * am["z"]["sign"] * self.translation_scale[2]
        wx = raw_axes[am["rx"]["sm_axis"]] * am["rx"]["sign"] * self.rotation_scale[0]
        wy = raw_axes[am["ry"]["sm_axis"]] * am["ry"]["sign"] * self.rotation_scale[1]
        wz = raw_axes[am["rz"]["sm_axis"]] * am["rz"]["sign"] * self.rotation_scale[2]

        # Clamp by vector magnitude (preserves direction)
        linear = np.array([vx, vy, vz])
        angular = np.array([wx, wy, wz])
        linear = self._clamp_vector(linear, self.max_linear)
        angular = self._clamp_vector(angular, self.max_angular)

        return SpaceMouseState(
            vx=linear[0], vy=linear[1], vz=linear[2],
            wx=angular[0], wy=angular[1], wz=angular[2],
            buttons=list(self._buttons),
            timestamp=time.time(),
        )

    def _parse_hid_report(self, data: list):
        """Parse SpaceMouse Compact HID report.

        Report types:
          1: translation (x, y, z) - 3 x int16 little-endian
          2: rotation (roll, pitch, yaw) - 3 x int16 little-endian
          3: buttons - bitmask
        """
        report_id = data[0]
        if report_id == 1 and len(data) >= 7:
            x, y, z = struct.unpack("<hhh", bytes(data[1:7]))
            self._raw["x"] = float(x)
            self._raw["y"] = float(y)
            self._raw["z"] = float(z)
        elif report_id == 2 and len(data) >= 7:
            roll, pitch, yaw = struct.unpack("<hhh", bytes(data[1:7]))
            self._raw["roll"] = float(roll)
            self._raw["pitch"] = float(pitch)
            self._raw["yaw"] = float(yaw)
        elif report_id == 3 and len(data) >= 2:
            btn_byte = data[1]
            self._buttons = [btn_byte & 1, (btn_byte >> 1) & 1]

    def close(self):
        """Close the SpaceMouse device."""
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None
        self._connected = False
        print("[SpaceMouse] Disconnected")

    @staticmethod
    def _clamp_vector(v: np.ndarray, max_mag: float) -> np.ndarray:
        """Clamp vector magnitude while preserving direction."""
        mag = np.linalg.norm(v)
        if mag > max_mag:
            return v * (max_mag / mag)
        return v

    @staticmethod
    def _is_3dxware_running() -> bool:
        """Check if 3DxWare driver processes are running."""
        import subprocess
        try:
            result = subprocess.run(
                ["pgrep", "-f", "3Dconnexion"],
                capture_output=True, timeout=2,
            )
            return result.returncode == 0
        except Exception:
            return False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()
