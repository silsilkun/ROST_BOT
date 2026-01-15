from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mode = LaunchConfiguration('mode')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    model = LaunchConfiguration('model')

    # bringup launch 실행
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dsr_bringup2'),
                'launch',
                'dsr_bringup2_rviz.launch.py'
            )
        )
    )

    # perception 노드
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception',
        output='screen',
    )

    # estimation 노드
    estimation_node = Node(
        package='estimation',
        executable='estimation_node',
        name='estimation',
        output='screen'
    )

    # 60초 후 perception + estimation 실행
    delayed_nodes = TimerAction(
        period=60.0,
        actions=[
            perception_node,
            estimation_node,
            control_client_node
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument('mode', default_value='real'),
        DeclareLaunchArgument('host', default_value='110.120.1.18'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('model', default_value='e0509'),

        bringup_launch,
        delayed_nodes
    ])
