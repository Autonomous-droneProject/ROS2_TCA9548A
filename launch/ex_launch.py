import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('tca9548a')
    config_file_path = os.path.join(pkg_dir, 'config', 'ex_all_params.yaml')

    return LaunchDescription([
        # The central TCA manager node
        Node(
            package='tca9548a',
            executable='tca9548a_manager',
            name='tca9548a_manager',
            parameters=[config_file_path],
            output='screen',
            emulate_tty=True
        ),
        
        # The generic I2C manager node that acts as a client
        Node(
            package='tca9548a',
            executable='generic_i2c_manager',
            name='generic_i2c_manager',
            parameters=[config_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])