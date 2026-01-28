#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class SlipMonitor(Node):
    def __init__(self) -> None:
        super().__init__('slip_monitor')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('sum_lw', 0.35)
        self.declare_parameter(
            'joint_names',
            [
                'front_left_wheel_joint',
                'front_right_wheel_joint',
                'rear_left_wheel_joint',
                'rear_right_wheel_joint',
            ],
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('odom_topic', '/mecanum_controller/odometry')
        self.declare_parameter('output_topic', '/mecanum_slip')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('min_speed', 0.02)
        self.declare_parameter('vy_sign', 1.0)
        self.declare_parameter('wz_sign', 1.0)

        self._joint_states_topic = self.get_parameter('joint_states_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._vy_sign = float(self.get_parameter('vy_sign').value)
        self._wz_sign = float(self.get_parameter('wz_sign').value)

        self._joint_names = list(self.get_parameter('joint_names').value)
        self._joint_indices = None
        self._last_joint_state = None
        self._last_odom = None

        self._pub = self.create_publisher(Vector3, self._output_topic, 10)
        self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 20)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)

        period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.1
        self.create_timer(period, self._compute_slip)

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg
        if self._joint_indices is not None:
            return
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        indices = []
        for name in self._joint_names:
            if name not in name_to_index:
                self.get_logger().warn(f'Joint name not found: {name}')
                return
            indices.append(name_to_index[name])
        self._joint_indices = indices

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _compute_slip(self) -> None:
        if self._last_joint_state is None or self._last_odom is None:
            return
        if self._joint_indices is None:
            return

        velocities = self._last_joint_state.velocity
        if len(velocities) <= max(self._joint_indices):
            return

        w_fl, w_fr, w_rl, w_rr = (velocities[i] for i in self._joint_indices)

        wheel_radius = float(self.get_parameter('wheel_radius').value)
        sum_lw = float(self.get_parameter('sum_lw').value)
        if sum_lw <= 0.0 or wheel_radius <= 0.0:
            return

        vx = wheel_radius * 0.25 * (w_fl + w_fr + w_rl + w_rr)
        vy = self._vy_sign * wheel_radius * 0.25 * (-w_fl + w_fr + w_rl - w_rr)
        wz = self._wz_sign * wheel_radius * 0.25 * (-w_fl + w_fr - w_rl + w_rr) / sum_lw

        odom_twist = self._last_odom.twist.twist
        odom_vx = odom_twist.linear.x
        odom_vy = odom_twist.linear.y
        odom_wz = odom_twist.angular.z

        pred_lin = math.hypot(vx, vy)
        odom_lin = math.hypot(odom_vx, odom_vy)
        if pred_lin < self._min_speed:
            slip_ratio = 0.0
        else:
            slip_ratio = (pred_lin - odom_lin) / pred_lin

        slip = Vector3()
        slip.x = slip_ratio
        slip.y = pred_lin - odom_lin
        slip.z = wz - odom_wz
        self._pub.publish(slip)


def main() -> None:
    rclpy.init()
    node = SlipMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
