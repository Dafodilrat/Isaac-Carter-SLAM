from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the slam_toolbox package directory
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_path = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    params_file = os.path.join(
        get_package_share_directory('isaac_slam_config'),
            'config',
            'mapper_params_online_async.yaml'
        )

    # Include SLAM Toolbox’s original async launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'slam_params_file': params_file
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/isaac_slam/isaac_slam_config/config/mapping.rviz'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # --- Combine everything into a LaunchDescription ---
    return LaunchDescription([
        slam_toolbox_launch,
        rviz_node
    ])
