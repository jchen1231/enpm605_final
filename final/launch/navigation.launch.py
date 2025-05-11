from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    goals_file_arg = DeclareLaunchArgument(
        'goals_file',
        default_value='goal_poses.yaml',
        description='Used to provide goal poses for cube 2 and final destination'
    )
    
    enable_rosbot_gazebo_arg = DeclareLaunchArgument(
        "enable_rosbot_gazebo",
        default_value="true",
        description="Enable the ROSbot Gazebo simulation",
    )
    
    # Set an absolute path for the map file to debug potential issues
    map_yaml_path = PathJoinSubstitution(
        [FindPackageShare("final"), "maps", "final_husarion_world.yaml"]
    )
    
    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=map_yaml_path,
        description="Full path to map yaml file to load"
    )

    # Log the map path for debugging
    log_map_path = LogInfo(
        msg=["Map file path: ", LaunchConfiguration("map")]
    )

    # Parameter files
    nav2_params_path = PathJoinSubstitution(
        [FindPackageShare("final"), "config", "nav2_params.yaml"]
    )

    start_x_arg = DeclareLaunchArgument('x', default_value='0.0')
    start_y_arg = DeclareLaunchArgument('y', default_value='2.0')

    # 1. ROSbot Gazebo simulation launch
    rosbot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbot_gazebo'),
                'launch',
                'simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'start_rviz': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_rosbot_gazebo")),
    )

    # # 2. Standalone Map Server (for better debugging)
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         {'yaml_filename': LaunchConfiguration('map')},
    #         {'topic_name': 'map'},
    #         {'frame_id': 'map'}
    #     ]
    # )
    
    # Static transform publisher to ensure map to odom transformation exists
    # This is a temporary fix to establish the TF tree
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # 3. Nav2 Localization - use the standard Nav2 localization launch instead of individual nodes
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items(),
    )
    
    # # 4. Lifecycle manager for map_server and amcl
    # lifecycle_manager_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         {'autostart': True},
    #         {'node_names': ['map_server', 'amcl']}
    #     ]
    # )
    
    # 5. Nav2 Navigation Stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': nav2_params_path,
        }.items(),
    )
    
    # 6. RViz with navigation configuration
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    
    navigation_node = Node(
        package='final',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[
            {'goals_file': LaunchConfiguration('goals_file')},
            {'camera_pose': [-8.0, 7.0, 0.25, 0.0, 0.0, 1.57]},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'x': LaunchConfiguration('x')},
            {'y': LaunchConfiguration('y')}
        ]
    )

    # Add the launch arguments first
    ld.add_action(use_sim_time_arg)
    ld.add_action(goals_file_arg)
    ld.add_action(enable_rosbot_gazebo_arg)
    ld.add_action(map_file_arg)
    ld.add_action(log_map_path)
    ld.add_action(start_x_arg)
    ld.add_action(start_y_arg)
    
    # Add actions in the correct order
    ld.add_action(rosbot_gazebo_launch)
    
    # Debug map server components individually instead of using localization_launch
    # ld.add_action(map_server_node)
    ld.add_action(static_tf_node)
    ld.add_action(localization_launch)
    # ld.add_action(lifecycle_manager_node)
    
    ld.add_action(navigation_launch)
    ld.add_action(rviz_node)
    
    ld.add_action(navigation_node)
    
    return ld