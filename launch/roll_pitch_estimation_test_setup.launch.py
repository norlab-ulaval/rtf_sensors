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
            executable='dps310_node',
            name='dps310_1_node',
            output='screen',
            parameters=[{'frame_id': 'dps310_1'},
                        {'i2c_address': 0x76},
                        {'i2c_bus': 1}]
        ),
        Node(
            package='rtf_sensors',
            executable='dps310_node',
            name='dps310_2_node',
            output='screen',
            parameters=[{'frame_id': 'dps310_2'},
                        {'i2c_address': 0x77},
                        {'i2c_bus': 0},]
        ),
        Node(
            package='rtf_sensors',
            executable='dps310_node',
            name='dps310_3_node',
            output='screen',
            parameters=[{'frame_id': 'dps310_3'},
                        {'i2c_address': 0x76},
                        {'i2c_bus': 0},]
        ),
    ])
