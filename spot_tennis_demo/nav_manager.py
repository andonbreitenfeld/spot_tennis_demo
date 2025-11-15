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
import tf2_geometry_msgs

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ball_pose = None

        self.sub = self.create_subscription(
            PoseStamped,
            '/ball_stable_pose',
            self.ball_cb,
            10,
        )

        self.srv = self.create_service(
            Trigger,
            '/nav_to_ball',
            self.handle_nav_to_ball,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        self.target_frame = 'spot_nav/map'
        self.base_frame = 'base_link'
        self.standoff = 0.30  # 1 ft

    def ball_cb(self, msg):
        self.ball_pose = msg

    def handle_nav_to_ball(self, request, response):
        # Check for no ball pose
        if self.ball_pose is None:
            response.success = False
            response.message = "Nav2 - No ball pose received"
            self.get_logger().warn("Nav2 - No ball pose received")
            return response

        # Check nav2 server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = "Nav2 unavailable"
            self.get_logger().warn("Nav2 unavailable")
            return response

        ball_x = self.ball_pose.pose.position.x
        ball_y = self.ball_pose.pose.position.y

        # Robot Transform: Base Frame -> Map Frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )

            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y

        except Exception as e:
            self.get_logger().warn(f"[TF ERROR] Using fallback: {e}")
            robot_x = ball_x - 1.0
            robot_y = ball_y
        
        dist_x = ball_x - robot_x
        dist_y = ball_y - robot_y
        dist = math.hypot(dist_x, dist_y)

        ux = dist_x / dist
        uy = dist_y / dist

        # 1 ft away from ball, shortest path
        goal_x = ball_x - ux * self.standoff
        goal_y = ball_y - uy * self.standoff

        # Orient towards the ball
        yaw = math.atan2(ball_y - goal_y, ball_x - goal_x)
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.target_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"[NAV2] Goal=({goal_x:.2f}, {goal_y:.2f}) Ball=({ball_x:.2f}, {ball_y:.2f})"
        )

        self.nav_client.send_goal_async(goal)

        response.success = True
        response.message = "Sent Nav2 Goal"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()