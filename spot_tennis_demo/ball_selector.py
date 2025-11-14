import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from yolo_msgs.msg import DetectionArray
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
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

        self.pub = self.create_publisher(
            PoseStamped,
            '/tennis_ball_pose',
            10,
        )
        
        self.target_frame = 'odom'

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

        pose_cam = PoseStamped()
        pose_cam.header = msg.header
        pose_cam.header.frame_id = best_det.bbox3d.frame_id
        pose_cam.pose = best_det.bbox3d.center

        # Base Frame -> Map Frame
        try:
            pose_map = self.tf_buffer.transform(
                pose_cam,
                self.target_frame,
                timeout=Duration(seconds=0.1),
            )
        except Exception as e:  # CHANGED — catch generic Exception (TransformException missing in Python)
            self.get_logger().warn(
                f"TF {pose_cam.header.frame_id} -> {self.target_frame} failed: {e}"
            )
            return

        self.pub.publish(pose_map)
        self.get_logger().info(
            f"[BALL] closest={best_dist:.2f}m (frame={pose_cam.header.frame_id}) | "
            f"map: x={pose_map.pose.position.x:.2f}, "
            f"y={pose_map.pose.position.y:.2f}, "
            f"z={pose_map.pose.position.z:.2f}"
        )
def main(args=None):
    rclpy.init(args=args)
    node = BallSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
