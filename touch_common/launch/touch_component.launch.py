#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='touch_base',
        description='Reference frame for the touch device'
    )
    
    units_arg = DeclareLaunchArgument(
        'units',
        default_value='mm',
        description='Units for position measurements (mm, cm, dm, m)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1000',
        description='Publishing rate in Hz'
    )
    
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='Default Device',
        description='Name of the haptic device'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='touch_container',
        description='Name of the component container'
    )

    # Robot description
    robot_description_content = Command([
        FindExecutable(name='cat'), ' ',
        PathJoinSubstitution([
            FindPackageShare('touch_description'),
            'urdf',
            'touch.urdf'
        ])
    ])

    # Create shared robot description parameter
    shared_robot_description = ParameterValue(robot_description_content, value_type=str)

    # Component Manager
    container = Node(
        package='rclcpp_components',
        executable='component_container',
        name=LaunchConfiguration('container_name'),
        output='screen',
    )

    # Load touch state component
    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='touch_common',
                plugin='TouchROS',
                name='touch_haptic_node',
                parameters=[{
                    'device_name': LaunchConfiguration('device_name'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                    'reference_frame': LaunchConfiguration('reference_frame'),
                    'units': LaunchConfiguration('units'),
                    'robot_description': shared_robot_description
                }],
            ),
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': shared_robot_description,
            'publish_frequency': 30.0
        }]
    )

    # Robot description topic publisher with QoS settings
    robot_description_topic_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_description_topic_publisher',
        parameters=[{
            'robot_description': shared_robot_description,
            'use_tf_static': False,
            'qos_overrides./robot_description.publisher.durability': 'transient_local',
            'qos_overrides./robot_description.publisher.depth': 1,
            'qos_overrides./robot_description.publisher.history': 'keep_last'
        }],
        remappings=[
            ('joint_states', '/dummy_joint_states'),  # Avoid conflict with main joint_states
        ],
        arguments=['--ros-args', '--log-level', 'WARN']  # Reduce log verbosity
    )

    # RViz node (conditional)
    rviz_node = TimerAction(
        period=3.0,  # Delay RViz startup to ensure robot_description is available
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('touch_common'),
                'launch',
                'touch.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('launch_rviz'))
        )]
    )

    return LaunchDescription([
        reference_frame_arg,
        units_arg,
        publish_rate_arg,
        device_name_arg,
        launch_rviz_arg,
        container_name_arg,
        container,
        load_composable_nodes,
        robot_state_publisher_node,
        robot_description_topic_pub,
        rviz_node
    ])