from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share',
        'rtf_sensors',
        'config',
        '_roll_pitch_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='rtf_sensors',
            executable='plane_orientation_node',
            name='plane_orientation_node',
            output='screen',
            parameters=[config_path]
        )
    ])
