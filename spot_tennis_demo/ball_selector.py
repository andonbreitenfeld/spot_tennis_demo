import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from yolo_msgs.msg import DetectionArray

class BallSelector(Node):
    def __init__(self):
        super().__init__('ball_selector')

        # Input YOLO 3D detections
        self.sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.detections_cb,
            10,
        )

        # Output the single best tennis ball
        self.pub = self.create_publisher(
            PoseStamped,
            '/tennis_ball_pose',
            10,
        )

    def detections_cb(self, msg: DetectionArray):
        best_det = None
        best_dist = None

        for det in msg.detections:
            p = det.bbox3d.center.position
            dist = math.sqrt(p.x ** 2 + p.y ** 2)

            if best_det is None or dist < best_dist:
                best_det = det
                best_dist = dist

        if best_det is None:
            return

        ball_pose = PoseStamped()
        ball_pose.header = msg.header
        ball_pose.pose = best_det.bbox3d.center

        self.pub.publish(ball_pose)

        self.get_logger().info(
            f"[SELECT] dist={best_dist:.2f}m | "
            f"x={ball_pose.pose.position.x:.2f}, "
            f"y={ball_pose.pose.position.y:.2f}, "
            f"z={ball_pose.pose.position.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BallSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
