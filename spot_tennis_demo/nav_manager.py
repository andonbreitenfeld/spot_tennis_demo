#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

import numpy as np

# Fix for numpy deprecations
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'complex'):
    np.complex = complex
if not hasattr(np, 'bool'):
    np.bool = bool

import tf_transformations


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        self.ball = None
        self.bin = None

        self.target_frame = 'spot_nav/map'
        self.base_frame = 'base_link'
        self.hand_frame = 'front_rail'

        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.create_subscription(PoseStamped, '/ball_nav_pose', self.ball_cb, 10)
        self.create_subscription(PoseStamped, '/bin_nav_pose',  self.bin_cb, 10)

        # Publishers for base poses (for behavior tree)
        self.ball_base_pub = self.create_publisher(PoseStamped, '/ball_base_pose', 10)
        self.bin_base_pub  = self.create_publisher(PoseStamped, '/bin_base_pose', 10)

    def ball_cb(self, msg):
        self.ball = msg
        base_pose = self._compute_base_pose(msg)
        if base_pose is not None:
            self.ball_base_pub.publish(base_pose)

    def bin_cb(self, msg):
        self.bin = msg
        base_pose = self._compute_base_pose(msg)
        if base_pose is not None:
            self.bin_base_pub.publish(base_pose)

    def _msg_to_mat(self, msg):
        # Handles both PoseStamped and TransformStamped
        if isinstance(msg, PoseStamped):
            t = msg.pose.position
            q = msg.pose.orientation
        else:
            t = msg.transform.translation
            q = msg.transform.rotation
        return tf_transformations.compose_matrix(
            translate=[t.x, t.y, t.z],
            angles=tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        )

    def _mat_to_pose(self, M, frame):
        p = PoseStamped()
        p.header.frame_id = frame
        tr = tf_transformations.translation_from_matrix(M)
        qr = tf_transformations.quaternion_from_matrix(M)
        p.pose.position.x = tr[0]
        p.pose.position.y = tr[1]
        p.pose.position.z = tr[2]
        p.pose.orientation.x = qr[0]
        p.pose.orientation.y = qr[1]
        p.pose.orientation.z = qr[2]
        p.pose.orientation.w = qr[3]
        return p

    def _compute_base_pose(self, hand_pose):
        if hand_pose is None:
            return None

        # Turn hand goal pose into base_link goal pose
        try:
            if hand_pose.header.frame_id != self.target_frame:
                # Transform from hand_pose to base_link frame
                tf_map_handframe = self.tf.lookup_transform(
                    self.target_frame,
                    hand_pose.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.5)
                )

                M_map_handframe = self._msg_to_mat(tf_map_handframe)
                M_handframe_hand = self._msg_to_mat(hand_pose)

                M_map_hand = M_map_handframe @ M_handframe_hand

            else:
                M_map_hand = self._msg_to_mat(hand_pose)

        except Exception as e:
            self.get_logger().warn(f"TF hand frame -> {self.target_frame} failed: {e}")
            return None

        try:
            tf_base_hand = self.tf.lookup_transform(
                self.base_frame,
                self.hand_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )

        except Exception as e:
            self.get_logger().warn(f"TF {self.base_frame} -> {self.hand_frame} failed: {e}")
            return None

        M_base_hand = self._msg_to_mat(tf_base_hand)

        # M_map_base = M_map_hand * inv(M_base_hand)
        M_map_base = M_map_hand @ tf_transformations.inverse_matrix(M_base_hand)
        base_goal = self._mat_to_pose(M_map_base, self.target_frame)
        base_goal.header.stamp = self.get_clock().now().to_msg()

        return base_goal


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
