
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='synexens_ros2',
            executable='synexens_ros2_node',
            parameters=[{
                # Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
                'tf_prefix': '',
                # The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
                'fps': 10,
                # Generate a point cloud from depth data. Requires depth_enabled
                'point_cloud_enabled': False,
                # Mapping 1 Depth to RGB 0 RGB to Depth default 1
                'mapping_mode': 1
            }]
        )
    ])

