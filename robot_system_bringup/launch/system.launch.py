from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    perception_config = os.path.join(
    get_package_share_directory('perception'),
    'config',
    'camera.yaml'
    )


    # 1. 로봇 bringup (즉시 실행)
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_system',
        output='screen'
    )

    # 2. perception 노드
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception',
        output='screen',
        parameters=[perception_config]
    )

    # 3. estimation 노드
    estimation_node = Node(
        package='estimation',
        executable='estimation_node',
        name='estimation',
        output='screen'
    )

    # 4. 60초 후 perception + estimation 실행
    delayed_nodes = TimerAction(
        period=60.0,  # 초 단위
        actions=[
            perception_node,
            estimation_node
        ]
    )

    return LaunchDescription([
        control_node,
        delayed_nodes
    ])
