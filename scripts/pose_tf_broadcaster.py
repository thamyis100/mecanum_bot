#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PoseTfBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_tf_broadcaster')
        self.pose_topic = self.declare_parameter('pose_topic', '/model/mecanum_bot/pose').value
        self.pose_msg_type = self.declare_parameter('pose_msg_type', 'pose').value
        self.pose_index = int(self.declare_parameter('pose_index', 0).value)
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.pose_msg_type == 'pose_array':
            self.sub = self.create_subscription(PoseArray, self.pose_topic, self.pose_array_cb, 10)
        elif self.pose_msg_type == 'pose_stamped':
            self.sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_stamped_cb, 10)
        else:
            self.sub = self.create_subscription(Pose, self.pose_topic, self.pose_cb, 10)

    def pose_cb(self, msg: Pose) -> None:
        self._publish_tf(self.get_clock().now().to_msg(), msg)

    def pose_stamped_cb(self, msg: PoseStamped) -> None:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        self._publish_tf(stamp, msg.pose)

    def pose_array_cb(self, msg: PoseArray) -> None:
        if not msg.poses:
            return
        idx = self.pose_index
        if idx < 0 or idx >= len(msg.poses):
            idx = 0
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        self._publish_tf(stamp, msg.poses[idx])

    def _publish_tf(self, stamp, pose: Pose) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
