import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    bringup_dir = get_package_share_directory('choi_stella_cartographer')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    #
    robot_file = "diffST_bot.urdf"
    world_file_name = "newworld.world"


    world_path = os.path.join(bringup_dir, "worlds", world_file_name)
    urdf_file = os.path.join(bringup_dir, "urdf", robot_file)
    rviz_config_dir = os.path.join(get_package_share_directory('choi_stella_cartographer'),
                                   'rviz', 'nav2_default_view.rviz')

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
    
    #
    
    params_file = LaunchConfiguration('params_file')
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('choi_stella_cartographer'),
            'maps',
            'newworld.yaml'))


    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
            

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
            
       ExecuteProcess(
            cmd=["gazebo", "--verbose", world_path, "-s", "libgazebo_ros_factory.so"],
                output="screen",
        ),
        robot_state_publisher_node,
        spawn_entity,
       
    ])
