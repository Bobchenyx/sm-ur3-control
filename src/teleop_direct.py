#!/usr/bin/env python3
"""Direct teleoperation: SpaceMouse → UR3 via ur_rtde (no ROS required)."""

import argparse
import time

import yaml

from spacemouse_reader import SpaceMouseReader
from ur3_controller import UR3Controller


def main():
    parser = argparse.ArgumentParser(description="SpaceMouse → UR3 direct teleoperation")
    parser.add_argument("--config", default="config/teleop_config.yaml", help="Config file path")
    parser.add_argument("--dry-run", action="store_true", help="Read SpaceMouse only, don't connect to robot")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    loop_rate = config["control"]["loop_rate"]
    deadman_btn = config["control"]["deadman_button"]
    dt = 1.0 / loop_rate
    print_interval = loop_rate * 2  # Print TCP pose every 2 seconds

    mouse = SpaceMouseReader(config)
    if not mouse.open():
        return

    controller = None
    if not args.dry_run:
        controller = UR3Controller(config)
        if not controller.connect():
            mouse.close()
            return

    print("\n--- Teleoperation Active ---")
    print(f"  Mode: {'DRY RUN' if args.dry_run else 'LIVE'}")
    print(f"  Loop rate: {loop_rate} Hz")
    print(f"  Dead-man switch: button {deadman_btn} (hold to move)")
    print(f"  Max linear: {config['safety']['max_linear_speed']} m/s")
    print(f"  Max angular: {config['safety']['max_angular_speed']} rad/s")
    print("  Press Ctrl+C to stop\n")

    running = True
    iteration = 0

    try:
        while running:
            t_start = time.perf_counter()

            state = mouse.read()

            # Dead-man switch: only move while button is held
            deadman_active = len(state.buttons) > deadman_btn and state.buttons[deadman_btn]

            if deadman_active and not state.is_zero:
                linear = state.linear
                angular = state.angular
            else:
                linear = [0.0, 0.0, 0.0]
                angular = [0.0, 0.0, 0.0]

            if args.dry_run:
                # Print every 0.5s — show RAW values (before dead-man filter) for debugging
                if iteration % (loop_rate // 2) == 0:
                    dm = "ON " if deadman_active else "OFF"
                    raw = state  # show actual SpaceMouse output, not dead-man filtered
                    print(f"[DRY] btn={dm} raw_lin=[{raw.vx:+.4f}, {raw.vy:+.4f}, {raw.vz:+.4f}] "
                          f"raw_ang=[{raw.wx:+.3f}, {raw.wy:+.3f}, {raw.wz:+.3f}]",
                          flush=True)
            else:
                controller.send_velocity(linear, angular)
                if iteration % print_interval == 0:
                    pose = controller.get_tcp_pose()
                    print(f"[TCP] x={pose[0]:+.4f} y={pose[1]:+.4f} z={pose[2]:+.4f} | "
                          f"rx={pose[3]:+.3f} ry={pose[4]:+.3f} rz={pose[5]:+.3f}")

            iteration += 1

            # Maintain loop rate
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\n[Teleop] Shutting down...")
    finally:
        if controller is not None:
            controller.disconnect()
        mouse.close()
        print("[Teleop] Done.")


if __name__ == "__main__":
    main()
