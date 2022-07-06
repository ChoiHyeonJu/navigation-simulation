import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_file = "stella_bot.urdf"
    world_file_name = "newworld.world"

    package_name = "choi_stella_cartographer"
    pkg_path = os.path.join(get_package_share_directory(package_name))

    world_path = os.path.join(pkg_path, "worlds", world_file_name)
    urdf_file = os.path.join(pkg_path, "urdf", robot_file)

#
    stella_cartographer_prefix = get_package_share_directory('choi_stella_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  stella_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                default='stella2.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('choi_stella_cartographer'),
                                   'rviz', 'stellabot3.rviz')
#

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'skidbot'],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true'),
#
            DeclareLaunchArgument(
                'cartographer_config_dir',
                default_value=cartographer_config_dir,
                description='Full path to config file to load'),

            DeclareLaunchArgument(
                'configuration_basename',
                default_value=configuration_basename,
                description='Name of lua file for cartographer'),

            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-configuration_directory', cartographer_config_dir,
                           '-configuration_basename', configuration_basename]),

            DeclareLaunchArgument(
                'resolution',
                default_value=resolution,
                description='Resolution of a grid cell in the published occupancy grid'),

            DeclareLaunchArgument(
                'publish_period_sec',
                default_value=publish_period_sec,
                description='OccupancyGrid publishing period'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid2.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                                  'publish_period_sec': publish_period_sec}.items(),
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'),
#

            ExecuteProcess(
                cmd=["gazebo", "--verbose", world_path, "-s", "libgazebo_ros_factory.so"],
                    output="screen",
            ),
            robot_state_publisher_node,
            spawn_entity,
        ]
    )