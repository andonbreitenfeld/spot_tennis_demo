#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        # Original variables kept
        self.ball_hand_pose = None
        self.bin_hand_pose = None

        # Frames unchanged
        self.target_frame = 'spot_nav/map'
        self.base_frame = 'base_link'
        self.hand_frame = 'front_rail'

        # TF setup stays unchanged
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # INPUT: YOLO and bin targets (unchanged)
        self.create_subscription(PoseStamped, '/ball_nav_pose', self.ball_cb, 10)
        self.create_subscription(PoseStamped, '/bin_nav_pose', self.bin_cb, 10)

        # ----------------------------
        # CHANGE #1:
        # nav_to_ball and nav_to_bin NO LONGER send Nav2 actions directly.
        # Instead, they compute the base_link goal and PUBLISH it.
        # BT's GoalUpdater reads it and NavigateToPose executes it.
        # ----------------------------
        self.create_service(Trigger, '/nav_to_ball', self.nav_to_ball)
        self.create_service(Trigger, '/nav_to_bin', self.nav_to_bin)

        # ----------------------------
        # CHANGE #2:
        # New publishers that expose computed nav goals to BT.
        # These replace the internal actionclient navigation.
        # ----------------------------
        self.ball_goal_pub = self.create_publisher(
            PoseStamped, '/ball_goal', 10
        )
        self.bin_goal_pub = self.create_publisher(
            PoseStamped, '/bin_goal', 10
        )

        # ----------------------------
        # CHANGE #3:
        # REMOVE Nav2 ActionClient entirely.
        # BT now handles NavigateToPose.
        # ----------------------------
        # self.nav_client = ActionClient(self, NavigateToPose, '/spot_nav/navigate_to_pose')
        # (Deleted)

        self.get_logger().info('Nav Manager started (BT-compatible)')


    # ===== Original callbacks unchanged =====
    def ball_cb(self, msg: PoseStamped):
        self.ball_hand_pose = msg

    def bin_cb(self, msg: PoseStamped):
        self.bin_hand_pose = msg


    # ================================================
    # CHANGE #4:
    # nav_to_ball/nav_to_bin NO LONGER run Nav2.
    # They compute a base goal and publish it for BT to use.
    # ================================================
    def nav_to_ball(self, request, response):
        goal = self.compute_base_goal(self.ball_hand_pose)

        if goal is None:
            response.success = False
            response.message = "No valid ball pose"
            return response

        # Publish goal for BT
        self.ball_goal_pub.publish(goal)

        response.success = True
        response.message = "ball_goal published (BT will navigate)"
        return response

    def nav_to_bin(self, request, response):
        goal = self.compute_base_goal(self.bin_hand_pose)

        if goal is None:
            response.success = False
            response.message = "No valid bin pose"
            return response

        # Publish goal for BT
        self.bin_goal_pub.publish(goal)

        response.success = True
        response.message = "bin_goal published (BT will navigate)"
        return response


    # ===== TF + math unchanged =====
    def _quat_to_yaw(self, q):
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)


    def compute_base_goal(self, hand_pose: PoseStamped):
        """
        CHANGE #5:
        This function remains completely unchanged.
        We simply return a PoseStamped for BT instead of
        sending a goal directly to Nav2.
        """

        if hand_pose is None:
            return None

        # Transform hand → map frame
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
            self.get_logger().error(f"TF hand->map failed: {e}")
            return None

        hand_x = hand.pose.position.x
        hand_y = hand.pose.position.y
        hand_yaw = self._quat_to_yaw(hand.pose.orientation)

        # Base → hand TF
        try:
            tf_base_hand = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.hand_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().error(f"TF base->hand failed: {e}")
            return None

        # Compute where base_link needs to go
        tx = tf_base_hand.transform.translation.x
        ty = tf_base_hand.transform.translation.y
        tq = tf_base_hand.transform.rotation

        rel_yaw = self._quat_to_yaw(tq)
        base_yaw = hand_yaw - rel_yaw

        dx_map = tx * math.cos(base_yaw) - ty * math.sin(base_yaw)
        dy_map = tx * math.sin(base_yaw) + ty * math.cos(base_yaw)

        base_x = hand_x - dx_map
        base_y = hand_y - dy_map

        # Final PoseStamped returned to BT
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.target_frame
        goal.pose.position.x = base_x
        goal.pose.position.y = base_y
        goal.pose.position.z = 0.0

        goal.pose.orientation.z = math.sin(base_yaw / 2.0)
        goal.pose.orientation.w = math.cos(base_yaw / 2.0)

        return goal


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
