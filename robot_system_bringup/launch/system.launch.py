from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='real')
    host_arg = DeclareLaunchArgument('host', default_value='110.120.1.18')
    port_arg = DeclareLaunchArgument('port', default_value='12345')
    model_arg = DeclareLaunchArgument('model', default_value='e0509')
    startup_delay_arg = DeclareLaunchArgument('startup_delay_sec', default_value='10.0')

    dsr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dsr_bringup2'),
                'launch',
                'dsr_bringup2_rviz.launch.py',
            ])
        ),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'model': LaunchConfiguration('model'),
        }.items(),
    )

    control_node = Node(
        package='control',
        executable='control_node',
        output='screen',
    )

    estimation_node = Node(
        package='estimation',
        executable='estimation_node',
        output='screen',
    )

    delayed_nodes = TimerAction(
        period=LaunchConfiguration('startup_delay_sec'),
        actions=[control_node, estimation_node],
    )

    return LaunchDescription([
        mode_arg,
        host_arg,
        port_arg,
        model_arg,
        startup_delay_arg,
        dsr_bringup,
        delayed_nodes,
    ])
