from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Robot namespace'
    )

    arg_launch_gateway = DeclareLaunchArgument(
        'launch_gateway',
        choices=['true', 'false'],
        default_value='true',
        description='Launch ROS2 socketcan gateway',
    )

    arg_canbus_dev = DeclareLaunchArgument(
        'canbus_dev', default_value='vcan1', description='CAN bus device'
    )

    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    launch_gateway = LaunchConfiguration('launch_gateway')
    canbus_dev = LaunchConfiguration('canbus_dev')

    # Clearpath ROS2 Socketcan Interface
    pkg_clearpath_ros2_socketcan_interface = FindPackageShare('clearpath_ros2_socketcan_interface')
    launch_file_receiver = PathJoinSubstitution(
        [pkg_clearpath_ros2_socketcan_interface, 'launch', 'receiver.launch.py']
    )
    launch_file_sender = PathJoinSubstitution(
        [pkg_clearpath_ros2_socketcan_interface, 'launch', 'sender.launch.py']
    )

    launch_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_receiver]),
        launch_arguments=[
            ('namespace', namespace),
            ('interface', canbus_dev),
            ('from_can_bus_topic', PathJoinSubstitution([canbus_dev, 'rx'])),
        ],
        condition=IfCondition(launch_gateway),
    )
    launch_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_sender]),
        launch_arguments=[
            ('namespace', namespace),
            ('interface', canbus_dev),
            ('to_can_bus_topic', PathJoinSubstitution([canbus_dev, 'tx'])),
        ],
    )

    # Wiferion
    wiferion_node = Node(
        name='wiferion_node',
        executable='wiferion_node',
        package='wiferion_charger',
        parameters=[{'canbus_dev': canbus_dev}],
        namespace=namespace,
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(arg_canbus_dev)
    ld.add_action(arg_launch_gateway)
    ld.add_action(arg_namespace)
    ld.add_action(launch_receiver)
    ld.add_action(launch_sender)
    ld.add_action(wiferion_node)
    return ld
