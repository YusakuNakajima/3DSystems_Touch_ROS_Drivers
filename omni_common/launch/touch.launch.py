#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
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
    
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='touch',
        description='Namespace prefix for topics'
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

    # Robot description
    robot_description_content = Command([
        FindExecutable(name='cat'), ' ',
        PathJoinSubstitution([
            FindPackageShare('omni_description'),
            'urdf',
            'omni.urdf'
        ])
    ])

    # Create shared robot description parameter
    shared_robot_description = ParameterValue(robot_description_content, value_type=str)

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': shared_robot_description,
            'publish_frequency': 30.0
        }],
        remappings=[
            ('joint_states', [LaunchConfiguration('prefix'), '/joint_states'])
        ]
    )

    # Omni state node
    omni_state_node = Node(
        package='omni_common',
        executable='omni_state',
        name='omni_state',
        output='screen',
        parameters=[{
            'device_name': LaunchConfiguration('device_name'),
            'prefix': LaunchConfiguration('prefix'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'units': LaunchConfiguration('units'),
            'robot_description_name': 'robot_description',
            'robot_description': shared_robot_description
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
                FindPackageShare('omni_common'),
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
        prefix_arg,
        device_name_arg,
        launch_rviz_arg,
        robot_state_publisher_node,
        robot_description_topic_pub,
        omni_state_node,
        rviz_node
    ])