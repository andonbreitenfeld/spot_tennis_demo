#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


class BinDetector(Node):
    def __init__(self):
        super().__init__('bin_detector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

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

        # Periodically publish whether bin has been detected
        self.create_timer(0.5, self.detected_timer_cb)
        # Periodically broadcast TF for nav goal (if we have one)
        self.create_timer(0.1, self.broadcast_nav_tf)

        self.get_logger().info('BinDetector started (axis-based, detect once, TF nav goal)')

    def detected_timer_cb(self):
        self.pub_detected.publish(Bool(data=self.nav_pose_saved is not None))

    def broadcast_nav_tf(self):
        """Broadcast bin_nav_goal frame so you can see the nav goal in TF/RViz."""
        if self.nav_pose_saved is None:
            return

        pose = self.nav_pose_saved

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'spot_nav/map'
        t.child_frame_id = 'bin_nav_goal'

        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z

        t.transform.rotation = pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def detections_cb(self, msg: AprilTagDetectionArray):
        # Already have a nav pose → nothing more to do
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
            # Get transform: bin_tag_link -> spot_nav/map (latest available)
            tf_tag_map = self.tf_buffer.lookup_transform(
                'spot_nav/map',     # target frame
                'bin_tag_link',     # source frame
                Time(),             # latest
                timeout=Duration(seconds=0.5)
            )

            tag_x = tf_tag_map.transform.translation.x
            tag_y = tf_tag_map.transform.translation.y
            tag_q = tf_tag_map.transform.rotation

            # --- Tag +Z axis in map frame ---
            qx, qy, qz, qw = tag_q.x, tag_q.y, tag_q.z, tag_q.w

            # R * (0,0,1) → tag's +Z expressed in map (x,y,z)
            zmx = 2.0 * (qx * qz + qy * qw)
            zmy = 2.0 * (qy * qz - qx * qw)
            # zmz = 1.0 - 2.0 * (qx*qx + qy*qy)  # not needed

            # Project +Z onto XY plane
            norm_xy = math.hypot(zmx, zmy)
            if norm_xy < 1e-6:
                self.get_logger().warn('Tag Z axis too vertical; cannot compute nav direction')
                return

            zmx /= norm_xy
            zmy /= norm_xy

            # Stand in front of the tag along +Z_tag
            stand_x = tag_x + zmx * self.standoff
            stand_y = tag_y + zmy * self.standoff

            # Face back toward the tag → +X along -Z_tag
            face_x = -zmx
            face_y = -zmy

            base_yaw = math.atan2(face_y, face_x)
            qz_nav = math.sin(base_yaw / 2.0)
            qw_nav = math.cos(base_yaw / 2.0)

            nav_pose = PoseStamped()
            nav_pose.header.stamp = self.get_clock().now().to_msg()
            nav_pose.header.frame_id = 'spot_nav/map'
            nav_pose.pose.position.x = stand_x
            nav_pose.pose.position.y = stand_y
            nav_pose.pose.position.z = 0.0
            nav_pose.pose.orientation.x = 0.0
            nav_pose.pose.orientation.y = 0.0
            nav_pose.pose.orientation.z = qz_nav
            nav_pose.pose.orientation.w = qw_nav

            self.nav_pose_saved = nav_pose
            self.pub_nav.publish(nav_pose)
            self.pub_detected.publish(Bool(data=True))

            self.get_logger().info(
                f'Bin nav pose saved at ({stand_x:.2f}, {stand_y:.2f}), '
                f'yaw={base_yaw:.2f} rad (stand on +Z_tag, face tag)'
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
