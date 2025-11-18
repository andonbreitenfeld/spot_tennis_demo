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
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.target_tag_id = 0
        self.nav_pose_saved = None
        
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detections_cb,
            10
        )
        
        self.pub_nav = self.create_publisher(PoseStamped, '/bin_nav_pose', 10)
        self.pub_live = self.create_publisher(PoseStamped, '/bin_tag_live', 10)
        
        self.get_logger().info('Bin detector started')
        
    def detections_cb(self, msg):
        bin_detection = None

        # Find Bin Tag
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                bin_detection = detection
                break
        
        if bin_detection is None:
            return
        
        try:
            # Exact tag pose in map frame
            tag_pose_tag_frame = PoseStamped()
            tag_pose_tag_frame.header.stamp = self.get_clock().now().to_msg()
            tag_pose_tag_frame.header.frame_id = 'bin_tag_link'
            tag_pose_tag_frame.pose.orientation.w = 1.0
            
            tag_pose_map = self.tf_buffer.transform(
                tag_pose_tag_frame,
                'spot_nav/map',
                timeout=Duration(seconds=0.5)
            )
            
            # Always publish live tag pose
            self.pub_live.publish(tag_pose_map)
            
            # Save navigation pose once
            if self.nav_pose_saved is None:
                nav_pose_tag_frame = PoseStamped()
                nav_pose_tag_frame.header.stamp = self.get_clock().now().to_msg()
                nav_pose_tag_frame.header.frame_id = 'bin_tag_link'
                nav_pose_tag_frame.pose.position.z = 1.0  # 1m forward
                nav_pose_tag_frame.pose.orientation.y = 1.0  # 180 deg around Y
                nav_pose_tag_frame.pose.orientation.w = 0.0
                
                # Swap axes to match Spot
                
                # Transform to map frame
                nav_pose_map = self.tf_buffer.transform(
                    nav_pose_tag_frame,
                    'spot_nav/map',
                    timeout=Duration(seconds=0.5)
                )
                
                nav_pose_map.pose.position.z = 0.0  # Set z to ground

                # Save and publish
                self.nav_pose_saved = nav_pose_map
                self.pub_nav.publish(nav_pose_map)
                
                self.get_logger().info(
                    f'Bin nav pose saved at ({nav_pose_map.pose.position.x:.2f}, '
                    f'{nav_pose_map.pose.position.y:.2f})'
                )
            
        except Exception as e:
            self.get_logger().warn(
                f'Bin Detector Error: {type(e).__name__}: {e}'
            )

def main():
    rclpy.init()
    node = BinDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()