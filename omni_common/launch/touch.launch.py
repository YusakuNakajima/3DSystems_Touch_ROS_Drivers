from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Launch引数取得
    prefix = LaunchConfiguration('prefix').perform(context)
    device_name = LaunchConfiguration('device_name').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
    reference_frame = LaunchConfiguration('reference_frame').perform(context)
    units = LaunchConfiguration('units').perform(context)

   
    urdf_path = os.path.join(
        get_package_share_directory('omni_description'),
        'urdf/omni.urdf'
    )
    robot_description = ParameterValue(
        Command([
            TextSubstitution(text='cat'),
            TextSubstitution(text=' '),  # 空白を明示的に入れる
            TextSubstitution(text=urdf_path)
        ]),
        value_type=str
    )
    
    return [
        # omni_state
        Node(
            package='omni_common',
            executable='omni_state',
            name='omni_state',
            output='screen',
            parameters=[{
                'device_name': device_name,
                'prefix': prefix,
                'publish_rate': int(publish_rate),
                'reference_frame': reference_frame,
                'units': units,
                'robot_description': robot_description,
                # 'robot_description_name': robot_description_name
            }]
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{prefix}_robot_state_publisher',
            parameters=[{
                'robot_description': robot_description 
            }],
            # remappings=[
            #     ('/joint_states', f'{prefix}/joint_states'),
            #     ('/robot_description', robot_description_name)
            # ]
        ),

        # rviz (別途起動)
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(
                    get_package_share_directory('omni_common'),
                    'launch/touch.rviz')],
                output='screen',
            )
        ], condition=IfCondition(LaunchConfiguration('launch_rviz')))
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reference_frame', default_value='touch_base'),
        DeclareLaunchArgument('units', default_value='m'),
        DeclareLaunchArgument('publish_rate', default_value='1000'),
        DeclareLaunchArgument('prefix', default_value='touch'),
        DeclareLaunchArgument('device_name', default_value='Default Device'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)
    ])
