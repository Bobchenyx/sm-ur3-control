#!/usr/bin/env python3
"""ROS bridge teleoperation: SpaceMouse (macOS) → rosbridge → MoveIt Servo (Docker)."""

import argparse
import time

import roslibpy
import yaml

from spacemouse_reader import SpaceMouseReader


def main():
    parser = argparse.ArgumentParser(description="SpaceMouse → ROS bridge teleoperation")
    parser.add_argument("--config", default="config/teleop_config.yaml", help="Config file path")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    ros_cfg = config["ros_bridge"]
    loop_rate = config["control"]["loop_rate"]
    deadman_btn = config["control"]["deadman_button"]
    dt = 1.0 / loop_rate

    # Connect to SpaceMouse
    mouse = SpaceMouseReader(config)
    if not mouse.open():
        return

    # Connect to rosbridge
    print(f"[ROS] Connecting to rosbridge at {ros_cfg['host']}:{ros_cfg['port']}...")
    client = roslibpy.Ros(host=ros_cfg["host"], port=ros_cfg["port"])
    try:
        client.run()
    except Exception as e:
        print(f"[ROS] Failed to connect to rosbridge: {e}")
        print("  Make sure Docker is running: ./scripts/start_ros.sh")
        mouse.close()
        return

    print("[ROS] Connected to rosbridge")

    twist_topic = roslibpy.Topic(
        client,
        ros_cfg["twist_topic"],
        "geometry_msgs/msg/TwistStamped",
    )

    frame_id = ros_cfg["frame_id"]
    print_interval = loop_rate * 2

    print("\n--- ROS Bridge Teleoperation Active ---")
    print(f"  Publishing to: {ros_cfg['twist_topic']}")
    print(f"  Frame: {frame_id}")
    print(f"  Dead-man switch: button {deadman_btn}")
    print("  Press Ctrl+C to stop\n")

    iteration = 0
    try:
        while client.is_connected:
            t_start = time.perf_counter()

            state = mouse.read()
            deadman_active = len(state.buttons) > deadman_btn and state.buttons[deadman_btn]

            if deadman_active and not state.is_zero:
                linear = state.linear
                angular = state.angular
            else:
                linear = [0.0, 0.0, 0.0]
                angular = [0.0, 0.0, 0.0]

            now = time.time()
            sec = int(now)
            nsec = int((now - sec) * 1e9)

            msg = roslibpy.Message({
                "header": {
                    "stamp": {"sec": sec, "nanosec": nsec},
                    "frame_id": frame_id,
                },
                "twist": {
                    "linear": {"x": linear[0], "y": linear[1], "z": linear[2]},
                    "angular": {"x": angular[0], "y": angular[1], "z": angular[2]},
                },
            })
            twist_topic.publish(msg)

            if iteration % print_interval == 0:
                dm = "ON " if deadman_active else "OFF"
                print(f"[PUB] btn={dm} lin=[{linear[0]:+.4f}, {linear[1]:+.4f}, {linear[2]:+.4f}]")

            iteration += 1
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\n[ROS Bridge] Shutting down...")
    finally:
        # Send zero twist before disconnecting
        zero_msg = roslibpy.Message({
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame_id},
            "twist": {
                "linear": {"x": 0, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": 0},
            },
        })
        try:
            twist_topic.publish(zero_msg)
            time.sleep(0.1)
            twist_topic.unadvertise()
            client.terminate()
        except Exception:
            pass
        mouse.close()
        print("[ROS Bridge] Done.")


if __name__ == "__main__":
    main()
