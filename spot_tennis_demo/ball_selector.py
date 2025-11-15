#!/usr/bin/env python3
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from yolo_msgs.msg import DetectionArray # type: ignore
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class BallSelector(Node):
    def __init__(self):
        super().__init__('ball_selector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.detections_cb,
            10,
        )

        self.pub_stable_pose = self.create_publisher(
            PoseStamped,
            '/ball_stable_pose',
            10,
        )

        self.pub_detected = self.create_publisher(
            Bool,
            '/ball_detected',
            10,
        )

        self.target_frame = 'spot_nav/map'

        self.window_size = 10   # Last 10 Positions for Stability Check
        self.stability_threshold = 0.5   # (50 cm)
        self.ball_window = deque(maxlen=self.window_size)

    def detections_cb(self, msg: DetectionArray):
        # No Detections
        if not msg.detections:
            self.ball_window.clear()    # clear queue
            self.pub_detected.publish(Bool(data=False))
            return

        best_det = None
        best_dist = None

        # Select Closest Ball
        for det in msg.detections:
            p = det.bbox3d.center.position
            dist = math.sqrt(p.x**2 + p.y**2)
            if best_det is None or dist < best_dist:
                best_det = det
                best_dist = dist

        pose_robot = PoseStamped()
        pose_robot.header = msg.header
        pose_robot.header.frame_id = best_det.bbox3d.frame_id
        pose_robot.pose = best_det.bbox3d.center

        # Base Frame -> Map Frame
        try:
            pose_map = self.tf_buffer.transform(
                pose_robot,
                self.target_frame,
                timeout=Duration(seconds=0.1)
            )

        except Exception as e:
            self.ball_window.clear()
            self.pub_detected.publish(Bool(data=False))
            self.get_logger().warn(
            f"[TF ERROR] Could not transform {pose_robot.header.frame_id} -> {self.target_frame}: {e}"
            )
            return

        x = pose_map.pose.position.x
        y = pose_map.pose.position.y
        z = pose_map.pose.position.z

        # Store Position in Queue
        self.ball_window.append((x, y, z))

        # Not Enough Stored Frames
        if len(self.ball_window) < self.window_size:
            self.pub_detected.publish(Bool(data=False))
            return

        # Average Position
        avg_x = sum(px for px, _, _ in self.ball_window) / len(self.ball_window)
        avg_y = sum(py for _, py, _ in self.ball_window) / len(self.ball_window)
        avg_z = sum(pz for _, _, pz in self.ball_window) / len(self.ball_window)

        # Deviation from Average
        max_dev = max(
            math.sqrt((px - avg_x)**2 + (py - avg_y)**2 + (pz - avg_z)**2)
            for px, py, pz in self.ball_window
        )

        # Unstable Ball
        if max_dev > self.stability_threshold:
            self.pub_detected.publish(Bool(data=False))
            self.get_logger().info(
            f"[UNSTABLE] Ball moving too much (max_dev={max_dev:.2f} > {self.stability_threshold})"
            )
            return

        # Stable Ball
        stable = PoseStamped()
        stable.header = pose_map.header
        stable.pose = pose_map.pose
        stable.pose.position.x = avg_x
        stable.pose.position.y = avg_y
        stable.pose.position.z = avg_z

        self.pub_stable_pose.publish(stable)
        self.pub_detected.publish(Bool(data=True))

        self.get_logger().info(
            f"Ball Detected: x={avg_x:.2f}, y={avg_y:.2f}, z={avg_z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BallSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
