from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # --- Paths ---
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(bringup_dir, 'launch', 'bringup_launch.py')

    isaac_slam_dir = get_package_share_directory('isaac_slam_config')
    default_map = os.path.join(isaac_slam_dir, 'maps', 'warehouse.yaml')
    default_params = os.path.join(isaac_slam_dir, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(isaac_slam_dir, 'config', 'mapping.rviz')

    # --- Arguments ---
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map yaml file to load'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameter file to use'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Isaac) clock if true'
    )

    # --- Nav2 bringup include ---
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # --- Optional Map Server node (redundant if bringup already starts it) ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # --- Optional RViz2 node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', default_rviz]
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        nav2_bringup,
        map_server,
        rviz_node
    ])
