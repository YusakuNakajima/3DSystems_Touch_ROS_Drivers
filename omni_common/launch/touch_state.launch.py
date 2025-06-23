from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Launch引数取得
    device_name = LaunchConfiguration('device_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    reference_frame = LaunchConfiguration('reference_frame').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
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
    
    robot_description_name = f'{prefix}_robot_description'

    return [
        # omni_state ノードの起動
        Node(
            package='omni_common',
            executable='omni_state',
            name='omni_state',
            output='screen',
            parameters=[{
                'device_name': device_name,
                'omni_name': prefix,
                'publish_rate': int(publish_rate),
                'reference_frame': reference_frame,
                'units': units,
                'robot_description_name': robot_description_name,
                robot_description_name: robot_description,
            }]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        # 引数宣言
        DeclareLaunchArgument('reference_frame', default_value='base'),
        DeclareLaunchArgument('units', default_value='m'),
        DeclareLaunchArgument('publish_rate', default_value='1000'),
        DeclareLaunchArgument('prefix', default_value='touch'),
        DeclareLaunchArgument('device_name', default_value='Default Device'),
        OpaqueFunction(function=launch_setup)
    ])
