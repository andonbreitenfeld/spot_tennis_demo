#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        self.ball_hand_pose = None
        self.bin_hand_pose = None

        self.target_frame = 'spot_nav/map'   # Nav2 frame
        self.base_frame = 'base_link'
        self.hand_frame = 'front_rail'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, '/ball_nav_pose', self.ball_cb, 10)
        self.create_subscription(PoseStamped, '/bin_nav_pose', self.bin_cb, 10)

        self.create_service(Trigger, '/nav_to_ball', self.nav_to_ball)
        self.create_service(Trigger, '/nav_to_bin', self.nav_to_bin)

        self.nav_client = ActionClient(self, NavigateToPose, '/spot_nav/navigate_to_pose')

        self.get_logger().info('Nav Manager started ')

    def ball_cb(self, msg: PoseStamped):
        self.ball_hand_pose = msg

    def bin_cb(self, msg: PoseStamped):
        self.bin_hand_pose = msg

    def nav_to_ball(self, request, response):
        return self._nav_to_hand_target(self.ball_hand_pose, response)

    def nav_to_bin(self, request, response):
        return self._nav_to_hand_target(self.bin_hand_pose, response)

    def _quat_to_yaw(self, q):
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    def _compute_base_goal_from_hand(self, hand_x, hand_y, hand_yaw, tf_base_hand):
        # For given hand goal pose, where does base need to be
        tx = tf_base_hand.transform.translation.x
        ty = tf_base_hand.transform.translation.y
        tq = tf_base_hand.transform.rotation

        rel_yaw = self._quat_to_yaw(tq)
        base_yaw = hand_yaw - rel_yaw # hand_yaw = base_yaw + rel_yaw

        # Distance in map frame
        dx_map = tx * math.cos(base_yaw) - ty * math.sin(base_yaw)
        dy_map = tx * math.sin(base_yaw) + ty * math.cos(base_yaw)

        # Nav2 Goal Position
        base_x = hand_x - dx_map
        base_y = hand_y - dy_map

        return base_x, base_y, base_yaw

    def _nav_to_hand_target(self, hand_pose: PoseStamped, response):
        # This takes a goal pose for the hand and computes where the base_link should go for Nav2
        self.get_logger().info("nav_to_* service called")
        if hand_pose is None:
            response.success = False
            response.message = "No target pose received"
            return response

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = "Nav2 unavailable"
            return response

        # Ensure hand goal pose is in target frame (map)
        try:
            if hand_pose.header.frame_id != self.target_frame:
                hand = self.tf_buffer.transform(
                    hand_pose,
                    self.target_frame,
                    timeout=Duration(seconds=0.5)
                )
            else:
                hand = hand_pose

        except Exception as e:
            response.success = False
            response.message = "Transform hand goal pose to map failed"
            self.get_logger().error(f"TF error (hand to map): {e}")
            return response

        hand_x = hand.pose.position.x
        hand_y = hand.pose.position.y
        hand_yaw = self._quat_to_yaw(hand.pose.orientation)

        # Base_link -> Hand Transform
        try:
            tf_base_hand = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.hand_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )

        except Exception as e:
            response.success = False
            response.message = "Lookup base to hand transform failed"
            self.get_logger().error(f"TF error (base to hand): {e}")
            return response

        base_x, base_y, base_yaw = self._compute_base_goal_from_hand(
            hand_x, hand_y, hand_yaw, tf_base_hand
        )

        base_qz = math.sin(base_yaw / 2.0)
        base_qw = math.cos(base_yaw / 2.0)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.target_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = base_x
        goal_pose.pose.position.y = base_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = base_qz
        goal_pose.pose.orientation.w = base_qw

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.nav_client.send_goal_async(goal)
        response.success = True
        response.message = "Navigating"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
