import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of your package and the config directory
    pkg_dir = get_package_share_directory('tca9548a')
    config_dir = os.path.join(pkg_dir, 'config')

    # Path to the parameter files
    tca_one_params_file = os.path.join(config_dir, 'tca_one_params.yaml')
    tca_two_params_file = os.path.join(config_dir, 'tca_two_params.yaml')
    manager_params_file = os.path.join(config_dir, 'manager_params.yaml')

    # Create the launch description with all three nodes
    return LaunchDescription([
        # Node 1: The first TCA9548A device node
        Node(
            package='tca9548a',
            executable='tca9548a_node',
            name='tca_node_one',
            parameters=[tca_one_params_file],
            output='screen',
            emulate_tty=True
        ),

        # Node 2: The second TCA9548A device node
        Node(
            package='tca9548a',
            executable='tca9548a_node',
            name='tca_node_two',
            parameters=[tca_two_params_file],
            output='screen',
            emulate_tty=True
        ),

        # Node 3: The manager node that controls the other two
        Node(
            package='tca9548a',
            executable='tca9548a_manager',
            name='tca9548a_manager',
            parameters=[manager_params_file],
            output='screen',
            emulate_tty=True
        ),
    ])