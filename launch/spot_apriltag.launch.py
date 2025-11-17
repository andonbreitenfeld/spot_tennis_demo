from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('spot_tennis_demo'),
        'config',
        'spot_apriltag.yaml'
    ])
    
    apriltag_frontleft = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_frontleft',
        output='screen',
        remappings=[
            ('image_rect', '/spot_image_server/rgb/frontleft/image'),
            ('camera_info', '/spot_image_server/rgb/frontleft/camera_info'),
            ('detections', '/apriltag/detections'),
        ],
        parameters=[config_file]
    )
    
    return LaunchDescription([apriltag_frontleft])