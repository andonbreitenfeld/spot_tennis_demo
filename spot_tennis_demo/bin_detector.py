#!/usr/bin/env python3
import rclpy
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class BinDetector(Node):
    def __init__(self):
        super().__init__('bin_detector')
        
        # TF for transforming to map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.target_tag_id = 0  # Bin tag ID
        
        # Subscribe to apriltag detections
        self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detections_cb,
            10
        )
        
        # Publish bin pose in map frame
        self.pub = self.create_publisher(PoseStamped, '/bin_pose', 10)
        
        self.get_logger().info(f'Bin detector started (tag ID {self.target_tag_id})')
        
    def detections_cb(self, msg):
        # Find our bin tag
        bin_detection = None
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                bin_detection = detection
                break
        
        if bin_detection is None:
            return
        
        try:
            # Use TF to get bin pose in map frame
            transform = self.tf_buffer.lookup_transform(
                'spot_nav/map',
                'bin_tag_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            pose_map = PoseStamped()
            pose_map.header.stamp = self.get_clock().now().to_msg()
            pose_map.header.frame_id = 'spot_nav/map'
            pose_map.pose.position.x = transform.transform.translation.x
            pose_map.pose.position.y = transform.transform.translation.y
            pose_map.pose.position.z = transform.transform.translation.z
            pose_map.pose.orientation = transform.transform.rotation
            
            self.pub.publish(pose_map)
            self.get_logger().info(
                f'Bin at x={pose_map.pose.position.x:.2f}, '
                f'y={pose_map.pose.position.y:.2f}'
            )
            
        except Exception as e:
            self.get_logger().warn(f'{e}')

def main():
    rclpy.init()
    rclpy.spin(BinDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()