import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Find where this package was installed, so we can build absolute
    # paths to our world, model, and config files. This works no
    # matter where the user's workspace lives on disk.
    simulation_package_share_directory = get_package_share_directory('openrover_simulation')
    world_file_path = os.path.join(simulation_package_share_directory, 'worlds', 'rover_world.sdf')
    models_directory_path = os.path.join(simulation_package_share_directory, 'models')
    bridge_config_file_path = os.path.join(simulation_package_share_directory, 'config', 'ros_gz_bridge.yaml')

    # Gazebo finds models referenced as "model://rover" (see the
    # <include> in rover_world.sdf) by searching every folder listed
    # in the GZ_SIM_RESOURCE_PATH environment variable. We add our
    # models/ folder to it here, keeping any existing value so we
    # don't break other resource paths that might already be set.
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_resource_path:
        combined_resource_path = models_directory_path + os.pathsep + existing_resource_path
    else:
        combined_resource_path = models_directory_path

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_resource_path,
    )

    # ros_gz_sim already ships a launch file that knows how to start
    # Gazebo correctly. Rather than reimplementing that, we include it
    # and just tell it which world file to load via "gz_args".
    gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': world_file_path}.items(),
    )

    # ros_gz_bridge mirrors the Gazebo topics listed in
    # ros_gz_bridge.yaml onto real ROS 2 topics (like /cmd_vel and
    # /odom), so ordinary ROS 2 nodes can talk to the simulated rover.
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config_file_path}],
        output='screen',
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo_launch_description,
        ros_gz_bridge_node,
    ])
