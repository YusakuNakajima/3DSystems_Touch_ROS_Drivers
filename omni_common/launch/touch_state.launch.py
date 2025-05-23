from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LaunchConfiguration を明示的に変数に束縛
    device_name = LaunchConfiguration('device_name')
    prefix = LaunchConfiguration('prefix')
    reference_frame = LaunchConfiguration('reference_frame')
    publish_rate = LaunchConfiguration('publish_rate')
    units = LaunchConfiguration('units')
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

    return LaunchDescription([
        # 引数宣言
        DeclareLaunchArgument('reference_frame', default_value='base'),
        DeclareLaunchArgument('units', default_value='mm'),
        DeclareLaunchArgument('publish_rate', default_value='1000'),
        DeclareLaunchArgument('prefix', default_value='touch'),
        DeclareLaunchArgument('device_name', default_value='Default Device'),

        # omni_state ノードの起動
        Node(
            package='omni_common',
            executable='omni_state',
            name='omni_state',
            output='screen',
            parameters=[{
                'device_name': device_name,
                'omni_name': prefix,
                'publish_rate': publish_rate,
                'reference_frame': reference_frame,
                'units': units,
                'robot_description': robot_description,
            }]
        ),
    ])
