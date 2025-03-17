from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_clearpath_ros2_socketcan_interface = FindPackageShare('clearpath_ros2_socketcan_interface')
    launch_file_receiver = PathJoinSubstitution([
        pkg_clearpath_ros2_socketcan_interface, 'launch', 'receiver.launch.py'])
    launch_file_sender = PathJoinSubstitution([
        pkg_clearpath_ros2_socketcan_interface, 'launch', 'sender.launch.py'])

    launch_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_receiver]),
        launch_arguments=[
            ('interface', 'vcan1'),
            ('from_can_bus_topic', 'vcan1/rx')]
    )

    launch_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_sender]),
        launch_arguments=[
            ('interface', 'vcan1'),
            ('to_can_bus_topic', 'vcan1/tx')]
    )

    wiferion_node = Node(
        name='wiferion_node',
        executable='wiferion_node',
        package='wiferion_charger',
        parameters=[
            {'canbus_dev': 'vcan1'}
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(launch_receiver)
    ld.add_action(launch_sender)
    ld.add_action(wiferion_node)
    return ld
