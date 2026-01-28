#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelScaler(Node):
    def __init__(self):
        super().__init__('cmd_vel_scaler')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/mecanum_controller/reference')
        self.declare_parameter('output_stamped', True)
        self.declare_parameter('scale_linear', 0.001)
        self.declare_parameter('scale_angular', 0.001)
        self.declare_parameter('max_input', 2000.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.scale_linear = float(self.get_parameter('scale_linear').value)
        self.scale_angular = float(self.get_parameter('scale_angular').value)
        self.max_input = float(self.get_parameter('max_input').value)

        self.output_stamped = bool(self.get_parameter('output_stamped').value)

        if self.output_stamped:
            self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        else:
            self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self._on_cmd_vel,
            10,
        )

    def _clamp(self, value: float) -> float:
        if value > self.max_input:
            return self.max_input
        if value < -self.max_input:
            return -self.max_input
        return value

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self.output_stamped:
            scaled = TwistStamped()
            scaled.header.stamp = self.get_clock().now().to_msg()
            scaled.twist.linear.x = self._clamp(msg.linear.x) * self.scale_linear
            scaled.twist.linear.y = self._clamp(msg.linear.y) * self.scale_linear
            scaled.twist.linear.z = self._clamp(msg.linear.z) * self.scale_linear
            scaled.twist.angular.x = self._clamp(msg.angular.x) * self.scale_angular
            scaled.twist.angular.y = self._clamp(msg.angular.y) * self.scale_angular
            scaled.twist.angular.z = self._clamp(msg.angular.z) * self.scale_angular
        else:
            scaled = Twist()
            scaled.linear.x = self._clamp(msg.linear.x) * self.scale_linear
            scaled.linear.y = self._clamp(msg.linear.y) * self.scale_linear
            scaled.linear.z = self._clamp(msg.linear.z) * self.scale_linear
            scaled.angular.x = self._clamp(msg.angular.x) * self.scale_angular
            scaled.angular.y = self._clamp(msg.angular.y) * self.scale_angular
            scaled.angular.z = self._clamp(msg.angular.z) * self.scale_angular
        self.publisher.publish(scaled)


def main() -> None:
    rclpy.init()
    node = CmdVelScaler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            # Launch may already have shut down the context on SIGINT.
            rclpy.shutdown()


if __name__ == '__main__':
    main()
