import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Find where this package was installed, so we can build an
    # absolute path to our world file. This works no matter where the
    # user's workspace lives on disk.
    simulation_package_share_directory = get_package_share_directory('openrover_simulation')
    world_file_path = os.path.join(simulation_package_share_directory, 'worlds', 'empty_world.sdf')

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

    return LaunchDescription([
        gazebo_launch_description,
    ])
