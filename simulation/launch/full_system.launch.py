import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_package_share_directory = get_package_share_directory('openrover_simulation')

    # Reuses everything rover_sim.launch.py already does — starting
    # Gazebo, loading our world (with the rover in it), and bridging
    # /cmd_vel, /odom, /tf, /joint_states, /clock to real ROS 2
    # topics — so none of that setup is duplicated here.
    simulation_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_package_share_directory, 'launch', 'rover_sim.launch.py')
        )
    )

    # Our command interface node (from the control/ package), which
    # turns discrete commands like FORWARD/STOP (published on
    # /rover_command) into velocity commands on /cmd_vel.
    command_interface_node = Node(
        package='openrover_control',
        executable='command_interface_node',
        name='command_interface_node',
        output='screen',
    )

    return LaunchDescription([
        simulation_launch_description,
        command_interface_node,
    ])
