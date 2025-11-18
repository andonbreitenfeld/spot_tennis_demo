#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ball_pose = None
        self.bin_pose = None

        # Subscribe to Ball Pose
        self.create_subscription(
            PoseStamped,
            '/ball_stable_pose',
            self.ball_cb,
            10,
        )

        # Subscribe to Bin Pose
        self.create_subscription(
            PoseStamped,
            '/bin_pose',
            self.bin_cb,
            10,
        )

        # Service: Navigate to Ball
        self.create_service(Trigger, '/nav_to_ball', self.nav_to_ball)
        
        # Service: Navigate to Bin
        self.create_service(Trigger, '/nav_to_bin', self.nav_to_bin)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, '/spot_nav/navigate_to_pose')

        self.target_frame = 'spot_nav/map'
        self.base_frame = 'base_link'
        self.ball_standoff = 1.0  # 1 m
        self.bin_standoff = 0.5   # 0.5 m

    def ball_cb(self, msg):
        self.ball_pose = msg

    def bin_cb(self, msg):
        self.bin_pose = msg

    def nav_to_ball(self, request, response):
        # Check for No Ball Pose
        if self.ball_pose is None:
            response.success = False
            response.message = "No ball detected"
            self.get_logger().warn("No ball pose received")
            return response

        # Check Nav2 Server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = "Nav2 unavailable"
            self.get_logger().warn("Nav2 unavailable")
            return response

        ball_x = self.ball_pose.pose.position.x
        ball_y = self.ball_pose.pose.position.y

        goal_pose = self._compute_approach_pose(ball_x, ball_y, self.ball_standoff)
        
        if goal_pose:
            self.nav_client.send_goal_async(goal_pose)
            response.success = True
            response.message = "Navigating to ball"
            self.get_logger().info(
                f"[NAV2 BALL] Goal=({goal_pose.pose.pose.position.x:.2f}, {goal_pose.pose.pose.position.y:.2f}) "
                f"Ball=({ball_x:.2f}, {ball_y:.2f})"
            )
        else:
            response.success = False
            response.message = "Nav failed"
        
        return response

    def nav_to_bin(self, request, response):
        # Check for No Bin Pose
        if self.bin_pose is None:
            response.success = False
            response.message = "No bin detected"
            self.get_logger().warn("No bin pose received")
            return response

        # Check Nav2 Server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = "Nav2 unavailable"
            self.get_logger().warn("Nav2 unavailable")
            return response

        bin_x = self.bin_pose.pose.position.x
        bin_y = self.bin_pose.pose.position.y

        goal_pose = self._compute_approach_pose(bin_x, bin_y, self.bin_standoff)
        
        if goal_pose:
            self.nav_client.send_goal_async(goal_pose)
            response.success = True
            response.message = "Navigating to bin"
            self.get_logger().info(
                f"[NAV2 BIN] Goal=({goal_pose.pose.pose.position.x:.2f}, {goal_pose.pose.pose.position.y:.2f}) "
                f"Bin=({bin_x:.2f}, {bin_y:.2f})"
            )
        else:
            response.success = False
            response.message = "Nav failed"
        
        return response

    def _compute_approach_pose(self, target_x, target_y, standoff):
        # Robot Transform: Base Frame -> Map Frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"[TF ERROR] Using fallback: {e}")
            robot_x = target_x - 1.0
            robot_y = target_y

        # Calculate Approach Vector
        dist_x = target_x - robot_x
        dist_y = target_y - robot_y
        dist = math.hypot(dist_x, dist_y)
        
        # Target Too Close
        if dist < 0.01:
            self.get_logger().warn("Target too close to robot")
            return None

        # Unit Vector Toward Target
        ux = dist_x / dist
        uy = dist_y / dist

        # Goal: Standoff Distance from Target
        goal_x = target_x - ux * standoff
        goal_y = target_y - uy * standoff

        # Orient Towards Target
        yaw = math.atan2(target_y - goal_y, target_x - goal_x)
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        # Create Nav2 Goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.target_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        
        return goal

def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()