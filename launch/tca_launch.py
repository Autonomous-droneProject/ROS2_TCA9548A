import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the parameter file
    pkg_dir = get_package_share_directory('tca9548a')
    config_file_path = os.path.join(pkg_dir, 'config', 'tca_params.yaml')

    # Create the launch description with the tca9548a_manager
    return LaunchDescription([
        Node(
            package='tca9548a',
            executable='tca9548a_manager',
            name='tca9548a_manager',
            parameters=[config_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])