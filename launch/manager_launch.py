import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of your package and the path to the single parameter file
    pkg_dir = get_package_share_directory('tca9548a')
    config_file_path = os.path.join(pkg_dir, 'config', 'all_params.yaml')

    # Create the launch description with all three nodes
    return LaunchDescription([
        # Node 1: The first TCA9548A device node
        Node(
            package='tca9548a',
            executable='tca9548a_node',
            name='tca_node_one',
            parameters=[config_file_path],
            output='screen',
            emulate_tty=True
        ),

        # Node 2: The second TCA9548A device node
        Node(
            package='tca9548a',
            executable='tca9548a_node',
            name='tca_node_two',
            parameters=[config_file_path],
            output='screen',
            emulate_tty=True
        ),

        # Node 3: The manager node that controls the other two
        Node(
            package='tca9548a',
            executable='tca9548a_manager',
            name='tca9548a_manager',
            parameters=[config_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])