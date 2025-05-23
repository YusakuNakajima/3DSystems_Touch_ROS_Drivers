from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # LaunchConfiguration を明示的に変数に束縛
    device_name = LaunchConfiguration('device_name')
    prefix = LaunchConfiguration('prefix')
    reference_frame = LaunchConfiguration('reference_frame')
    publish_rate = LaunchConfiguration('publish_rate')
    units = LaunchConfiguration('units')

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
            }]
        ),
    ])
