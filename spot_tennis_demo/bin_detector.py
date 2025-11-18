#!/usr/bin/env python3
import rclpy
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class BinDetector(Node):
    def __init__(self):
        super().__init__('bin_detector')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.target_tag_id = 0
        self.bin_pose_saved = None
        
        # Subscribe to AprilTag Detections
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detections_cb,
            10
        )
        
        # Publish Bin Pose
        self.pub = self.create_publisher(PoseStamped, '/bin_pose', 10)
        
        # Service: Get Bin Pose
        self.srv = self.create_service(
            Trigger,
            '/get_bin_pose',
            self.get_bin_pose_callback
        )
        
        self.get_logger().info('Bin detector started')
        
    def detections_cb(self, msg):
        # Already Saved
        if self.bin_pose_saved:
            return
        
        # Find Bin Tag
        bin_detection = None
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                bin_detection = detection
                break
        
        # No Bin Tag Detected
        if bin_detection is None:
            return
        
        try:
            # Bin Transform: Camera Frame -> Map Frame
            transform = self.tf_buffer.lookup_transform(
                'spot_nav/map',
                'bin_tag_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # Create Pose in Map Frame
            pose_map = PoseStamped()
            pose_map.header.stamp = self.get_clock().now().to_msg()
            pose_map.header.frame_id = 'spot_nav/map'
            pose_map.pose.position.x = transform.transform.translation.x
            pose_map.pose.position.y = transform.transform.translation.y
            pose_map.pose.position.z = transform.transform.translation.z
            pose_map.pose.orientation = transform.transform.rotation
            
            # Save and Publish Bin Pose
            self.bin_pose_saved = pose_map
            self.pub.publish(self.bin_pose_saved)
            
            self.get_logger().info(
                f'Bin saved at x={pose_map.pose.position.x:.2f}, '
                f'y={pose_map.pose.position.y:.2f}, '
                f'z={pose_map.pose.position.z:.2f}'
            )
            
            # Stop Listening for New Detections
            self.destroy_subscription(self.sub)
            
        except Exception as e:
            self.get_logger().warn(f'[TF ERROR] {e}')
    
    def get_bin_pose_callback(self, request, response):
        # Bin Not Detected Yet
        if self.bin_pose_saved is None:
            response.success = False
            response.message = "Bin not detected yet"
            self.get_logger().warn('Bin pose requested but not saved yet')
        else:
            # Return Saved Bin Pose
            response.success = True
            response.message = (
                f"x={self.bin_pose_saved.pose.position.x:.3f}, "
                f"y={self.bin_pose_saved.pose.position.y:.3f}, "
                f"z={self.bin_pose_saved.pose.position.z:.3f}"
            )
            self.get_logger().info('Bin pose given')
        
        return response

def main():
    rclpy.init()
    node = BinDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()