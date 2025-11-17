#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class BTExecutor(Node):
    def __init__(self):
        super().__init__('bt_executor')

        # BT xml path (not used yet, just logged)
        self.declare_parameter('bt_xml_file', '')
        bt_xml_file = self.get_parameter('bt_xml_file').get_parameter_value().string_value
        if bt_xml_file:
            self.get_logger().info(f"BT xml file: {bt_xml_file}")
        else:
            self.get_logger().warn("bt_xml_file param empty (placeholder BT)")

        # Track ball state
        self.ball_detected = False
        self.sent_goal_for_this_ball = False

        # Subscribe to ball_detected
        self.sub_detected = self.create_subscription(
            Bool,
            '/ball_detected',
            self.ball_detected_cb,
            10,
        )

        # Service client to nav_manager
        self.nav_client = self.create_client(
            Trigger,
            '/nav_to_ball',
        )

        # timer just to retry service connection / debug
        self.timer = self.create_timer(1.0, self.timer_cb)

    def ball_detected_cb(self, msg: Bool):
        # True when ball stable, False otherwise
        if msg.data and not self.ball_detected:
            self.get_logger().info("[BT] ball_detected -> True")
        if not msg.data and self.ball_detected:
            self.get_logger().info("[BT] ball_detected -> False (reset)")
            # allow sending a new goal next time it becomes True
            self.sent_goal_for_this_ball = False

        self.ball_detected = msg.data

        # If ball just detected and we haven't sent a goal yet
        if self.ball_detected and not self.sent_goal_for_this_ball:
            self.try_send_nav_to_ball()

    def timer_cb(self):
        # just logs if service not available yet
        if not self.nav_client.service_is_ready():
            self.get_logger().debug("[BT] waiting for /nav_to_ball service...")
        # no other periodic work for now

    def try_send_nav_to_ball(self):
        if not self.nav_client.service_is_ready():
            self.get_logger().warn("[BT] /nav_to_ball not ready yet, skipping")
            return

        self.get_logger().info("[BT] calling /nav_to_ball")
        req = Trigger.Request()
        future = self.nav_client.call_async(req)

        # attach callback to future
        future.add_done_callback(self.nav_response_cb)

        # avoid spamming multiple calls for the same detection
        self.sent_goal_for_this_ball = True

    def nav_response_cb(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"[BT] /nav_to_ball call failed: {e}")
            return

        if resp.success:
            self.get_logger().info(f"[BT] nav_to_ball success: {resp.message}")
        else:
            self.get_logger().warn(f"[BT] nav_to_ball failed: {resp.message}")


def main(args=None):
    rclpy.init(args=args)
    node = BTExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
