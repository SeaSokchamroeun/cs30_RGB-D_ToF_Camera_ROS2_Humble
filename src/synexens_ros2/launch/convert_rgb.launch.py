from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc',
            remappings=[
                ('image_raw', '/camera1/rgb_raw'),
                ('image', '/camera1/rgb_converted')
            ],
            parameters=[{'use_sim_time': False}]
        ),
    ])
