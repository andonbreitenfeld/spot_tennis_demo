#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class SurveyGoalServer(Node):
    def __init__(self):
        super().__init__('survey_goal_server')

        # Publishes survey goals that BT's GoalUpdater will read
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/survey_goal',
            10
        )

        # Behavior Tree calls this to request the NEXT survey pose
        self.srv = self.create_service(
            Trigger,
            '/next_survey_goal',
            self.handle_next_goal
        )

        # Your two survey poses
        self.poses = [
            self.make_pose(2.0, 4.0),
            self.make_pose(1.5, 2.0)
        ]

        # Start with pose A
        self.index = 0

        self.get_logger().info(
            "SurveyGoalServer started. Will alternate between Pose A and B "
            "ONLY when BT requests /next_survey_goal."
        )

    def make_pose(self, x, y):
        """
        Helper to create a PoseStamped in the map frame.
        Z and orientation are hard-coded for your use case.
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # Orientation: facing forward by default (z=0, w=1)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose

    def handle_next_goal(self, request, response):
        """
        Behavior Tree calls this service BEFORE each survey NavigateToPose.
        This function publishes exactly ONE pose and then flips index.
        """
        pose = self.poses[self.index]
        pose.header.stamp = self.get_clock().now().to_msg()

        # Publish the selected pose
        self.pose_pub.publish(pose)
        self.get_logger().info(
            f"Published Survey Pose {'A' if self.index == 0 else 'B'}"
        )

        # Flip pose ONLY NOW (after BT requested it)
        self.index = 1 - self.index

        # Reply to BT
        response.success = True
        response.message = "Published next survey pose"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SurveyGoalServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
