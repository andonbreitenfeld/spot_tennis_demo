#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener

class BinDetector(Node):
    def __init__(self):
        super().__init__('bin_detector')

        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.target_tag_id = 0
        self.standoff = 1.2
        self.nav_pose_saved = None

        self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detections_cb,
            10
        )

        self.pub_nav = self.create_publisher(PoseStamped, '/bin_nav_pose', 10)
        self.pub_detected = self.create_publisher(Bool, '/bin_detected', 10)

        # Publish whether bin has been detected
        self.create_timer(0.5, self.detected_timer_cb)

        self.get_logger().info('BinDetector started')

    def detected_timer_cb(self):
        self.pub_detected.publish(Bool(data=self.nav_pose_saved is not None))

    def detections_cb(self, msg: AprilTagDetectionArray):
        if self.nav_pose_saved is not None:
            return

        # Look for our target tag
        detection = None
        for det in msg.detections:
            if det.id == self.target_tag_id:
                detection = det
                break

        if detection is None:
            return

        try:
            # Transform bin_tag_link to spot_nav/map
            tf_map_tag = self.tf.lookup_transform(
                'spot_nav/map',
                'bin_tag_link',
                Time(),
                timeout=Duration(seconds=0.5)
            )

            tag_x = tf_map_tag.transform.translation.x
            tag_y = tf_map_tag.transform.translation.y
            q = tf_map_tag.transform.rotation

            qx, qy, qz, qw = q.x, q.y, q.z, q.w

            # Tag +Z axis expressed in map frame (zmx, zmy)
            zmx = 2.0 * (qx * qz + qy * qw)
            zmy = 2.0 * (qy * qz - qx * qw)

            norm_xy = math.hypot(zmx, zmy)
            if norm_xy < 1e-6:
                self.get_logger().warn('Tag Z axis vertical')
                return

            zmx /= norm_xy
            zmy /= norm_xy

            # Stand in front of the tag along +Z_tag
            stand_x = tag_x + zmx * self.standoff
            stand_y = tag_y + zmy * self.standoff

            # Face the tag: +X aligned with -Z_tag
            face_x = -zmx
            face_y = -zmy
            yaw = math.atan2(face_y, face_x)

            qz_nav = math.sin(yaw / 2.0)
            qw_nav = math.cos(yaw / 2.0)

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
                f'Bin nav pose saved at ({stand_x:.2f}, {stand_y:.2f}), yaw={yaw:.2f}'
            )

        except Exception as e:
            self.get_logger().warn(f'BinDetector TF error: {type(e).__name__}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BinDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

