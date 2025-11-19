#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class BallNavTarget(Node):
    def __init__(self):
        super().__init__('ball_nav_target')

        self.map_frame = 'spot_nav/map'
        self.front_frame = 'front_rail'
        self.standoff = 1.0  # 0.5 meters

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, '/ball_stable_pose', self.ball_cb, 10)
        self.pub_nav = self.create_publisher(PoseStamped, '/ball_nav_pose', 10)

    def ball_cb(self, ball_pose: PoseStamped):
        # Ball in map frame
        try:
            if ball_pose.header.frame_id != self.map_frame:
                ball = self.tf_buffer.transform(
                    ball_pose, self.map_frame, timeout=Duration(seconds=0.2)
                )
            else:
                ball = ball_pose

        except Exception:
            return

        ball_x = ball.pose.position.x
        ball_y = ball.pose.position.y

        # front_rail in map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.front_frame, Time(), timeout=Duration(seconds=0.2)
            )
        except Exception:
            return

        rail_x = tf.transform.translation.x
        rail_y = tf.transform.translation.y

        # Vector between ball and front rail
        dx = ball_x - rail_x
        dy = ball_y - rail_y
        dist = math.hypot(dx, dy)
        
        if dist < 1e-3:
            return

        ux = dx / dist
        uy = dy / dist

        # Compute goal standoff from ball in that direction
        goal_x = ball_x - ux * self.standoff
        goal_y = ball_y - uy * self.standoff

        yaw = math.atan2(uy, ux)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.pub_nav.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = BallNavTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
