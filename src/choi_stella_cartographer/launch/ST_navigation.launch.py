import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch.actions import ExecuteProcess

from geometry_msgs.msg import TransformStamped


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('choi_stella_cartographer')

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
        parameters=[{'use_sim_time':False}], #true
        arguments=[urdf_file],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'skidbot'],
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    #map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        #'map_subscribe_transient_local': map_subscribe_transient_local
       }

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        #DeclareLaunchArgument(
        #    'map_subscribe_transient_local', default_value='false',
        #    description='Whether to set the map subscriber QoS to transient local'),

        DeclareLaunchArgument(
            'node_tf2_fp2map', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'node_tf2_fp2link', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'node_tf2_fp2odom', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'node_tf2_fp2rightwheel', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'node_tf2_fp2leftwheel', default_value='',
            description='Top-level namespace'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

        Node(
            package='rviz2',
            executable='rviz2',
            #name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time }],
            output='screen',
            #remappings=remappings
            ),

        ExecuteProcess(
            cmd=["gazebo", "--verbose", world_path, "-s", "libgazebo_ros_factory.so"],
                output="screen",
        ),
        robot_state_publisher_node,
        spawn_entity,

        Node(
            name='tf2_ros_fp_link',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
        ),

        Node(
            name='tf2_ros_fp_map',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'map', 'odom'],
        ),

        Node(
            name='tf2_ros_fp_odom',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.1', '0.0', '0.0', '0.0', 'odom', 'base_footprint'],

        ),

        Node(
            name='tf2_ros_fp_rightwheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'front_right_wheel'],
        ),

        Node(
            name='tf2_ros_fp_leftwheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'front_left_wheel'],
        ),


    ])