from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Convert RGB
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc_rgb',
            remappings=[
                ('image_raw', '/camera1_HV0130315L0317/rgb_raw'),
                ('image', '/camera1_HV0130315L0317/rgb_converted')
            ],
            parameters=[{'use_sim_time': False}]
        ),
        # Convert IR
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc_ir',
            remappings=[
                ('image_raw', '/camera1_HV0130315L0317/ir_raw'),
                ('image', '/camera1_HV0130315L0317/ir_converted')
            ],
            parameters=[{'use_sim_time': False}]
        ),
    ])
