import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Purpose: take in detected objects (with transform from obj -> camera), convert to TransformStamped, add to TF tree
class ObjectPose3D(Node):
    def __init__(self):
        super().__init__('object_pose_3d')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.subscription = self.create_subscription(
            DetectionArray,
            'yolo/detections_3d',
            self.listener_callback,
            qos_profile,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        for i, detection in enumerate(msg.detections):
            frame_name = f"tennis_ball_{i}"
            self.broadcast_frame(detection, frame_name)

    def broadcast_frame(self, detection, frame_name):
        position = detection.bbox3d.center.position
        orientation = detection.bbox3d.center.orientation

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = detection.bbox3d.frame_id
        t.child_frame_id = frame_name

        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z

        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPose3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
