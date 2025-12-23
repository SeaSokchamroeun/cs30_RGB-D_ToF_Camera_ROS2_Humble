
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path
 
def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('synexens_ros2'), 'rviz', 'view.rviz')
    print(rviz_config_dir)
    return LaunchDescription([
        Node(
            package='synexens_ros2',
            executable='synexens_ros2_node',
            parameters=[{
                # Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
                'tf_prefix': '',
                # The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
                'fps': 30,
                # Generate a point cloud from depth data. Requires depth_enabled
                'point_cloud_enabled': True,
                # Mapping 1 Depth to RGB 0 RGB to Depth default 1
                'mapping_mode': 1
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir]
        )
    ])