#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener


class BinDetector(Node):
    def __init__(self):
        super().__init__('bin_detector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_tag_id = 0          # AprilTag ID for the bin
        self.standoff = 1.0             # meters in front of tag
        self.nav_pose_saved = None      # PoseStamped in spot_nav/map

        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detections_cb,
            10
        )

        self.pub_nav = self.create_publisher(PoseStamped, '/bin_nav_pose', 10)
        self.pub_detected = self.create_publisher(Bool, '/bin_detected', 10)

        # Publish whether bin has been detected
        self.create_timer(0.5, self.detected_timer_cb)

        self.get_logger().info('BinDetector started (simple yaw-based version)')

    def detected_timer_cb(self):
        self.pub_detected.publish(Bool(data=self.nav_pose_saved is not None))

    def _quat_to_yaw(self, q):
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    def detections_cb(self, msg: AprilTagDetectionArray):
        # Already have a nav pose
        if self.nav_pose_saved is not None:
            return

        # Look for our target tag ID
        bin_detection = None
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                bin_detection = detection
                break

        if bin_detection is None:
            return

        try:
            # Tag origin in map frame
            tag_pose_tag = PoseStamped()
            tag_pose_tag.header.stamp = self.get_clock().now().to_msg()
            tag_pose_tag.header.frame_id = 'bin_tag_link'
            tag_pose_tag.pose.orientation.w = 1.0  # identity

            tag_pose_map = self.tf_buffer.transform(
                tag_pose_tag,
                'spot_nav/map',
                timeout=Duration(seconds=0.5)
            )

            tag_x = tag_pose_map.pose.position.x
            tag_y = tag_pose_map.pose.position.y
            tag_q = tag_pose_map.pose.orientation

            # Tag yaw in map
            tag_yaw = self._quat_to_yaw(tag_q)

            # We want the robot to face the tag → +X toward tag
            base_yaw = tag_yaw + math.pi

            # Position: standoff meters in front of tag along base_yaw
            dx = math.cos(base_yaw)
            dy = math.sin(base_yaw)

            goal_x = tag_x + dx * self.standoff
            goal_y = tag_y + dy * self.standoff
            goal_z = 0.0

            # Yaw-only quaternion
            qz = math.sin(base_yaw / 2.0)
            qw = math.cos(base_yaw / 2.0)

            nav_pose = PoseStamped()
            nav_pose.header.stamp = self.get_clock().now().to_msg()
            nav_pose.header.frame_id = 'spot_nav/map'
            nav_pose.pose.position.x = goal_x
            nav_pose.pose.position.y = goal_y
            nav_pose.pose.position.z = goal_z
            nav_pose.pose.orientation.x = 0.0
            nav_pose.pose.orientation.y = 0.0
            nav_pose.pose.orientation.z = qz
            nav_pose.pose.orientation.w = qw

            self.nav_pose_saved = nav_pose
            self.pub_nav.publish(nav_pose)
            self.pub_detected.publish(Bool(data=True))

            self.get_logger().info(
                f'Bin nav pose saved at ({goal_x:.2f}, {goal_y:.2f}), '
                f'yaw={base_yaw:.2f} rad (robot facing tag)'
            )

        except Exception as e:
            self.get_logger().warn(f'BinDetector TF error: {type(e).__name__}: {e}')


def main():
    rclpy.init()
    node = BinDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
