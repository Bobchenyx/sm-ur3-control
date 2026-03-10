"""Twist relay node: safety watchdog between rosbridge and MoveIt Servo.

Subscribes to twist from rosbridge (macOS SpaceMouse client).
Republishes to MoveIt Servo with proper timestamps.
Zeros velocity if no input received within timeout (safety).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

TIMEOUT_SEC = 0.15  # Zero velocity if no input for this long


class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay_node")

        self.declare_parameter("input_topic", "/spacemouse/twist")
        self.declare_parameter("output_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("timeout_sec", TIMEOUT_SEC)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate", 125.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.timeout_sec = self.get_parameter("timeout_sec").value
        self.frame_id = self.get_parameter("frame_id").value
        rate = self.get_parameter("publish_rate").value

        self._last_twist = TwistStamped()
        self._last_received_time = self.get_clock().now()
        self._active = False

        self.subscription = self.create_subscription(
            TwistStamped, input_topic, self._twist_callback, 10
        )
        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._publish_callback)

        self.get_logger().info(
            f"Twist relay: {input_topic} -> {output_topic} @ {rate} Hz"
        )

    def _twist_callback(self, msg: TwistStamped):
        self._last_twist = msg
        self._last_received_time = self.get_clock().now()
        self._active = True

    def _publish_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_received_time).nanoseconds / 1e9

        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id

        if self._active and elapsed < self.timeout_sec:
            # Forward last received twist
            out.twist = self._last_twist.twist
        else:
            if self._active:
                self.get_logger().warn("SpaceMouse input timeout, sending zero velocity")
                self._active = False
            # Zero velocity (default TwistStamped)

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
