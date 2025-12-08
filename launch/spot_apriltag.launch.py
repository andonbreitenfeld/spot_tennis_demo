from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the shared apriltag config
    config_file = PathJoinSubstitution([
        FindPackageShare('spot_tennis_demo'),
        'config',
        'spot_apriltag.yaml'
    ])

    # All available cameras
    camera_names = ['frontleft', 'frontright', 'hang_rgb']
    topic_prefix = '/spot_image_server/rgb/'

    nodes = []

    for camera_name in camera_names:
        node = Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name=f'apriltag_node_{camera_name}',
            output='screen',
            remappings=[
                # Inputs
                ('image_rect',  f'{topic_prefix}{camera_name}/image'),
                ('camera_info', f'{topic_prefix}{camera_name}/camera_info'),
                ('detections',  '/apriltag/detections'),
            ],
            parameters=[config_file]
        )
        nodes.append(node)

    return LaunchDescription(nodes)