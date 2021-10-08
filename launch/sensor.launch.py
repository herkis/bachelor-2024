import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        #launch.actions.ExecuteProcess(
        #    cmd=['ros2', 'bag', 'record', '-a'],
        #    output='screen'
        #),
        Node(
            package='sensor_thermometer',
            namespace='thermometer',
            executable='thermometer_publisher',
            name='sensors'
        ),
        Node(
            package='sensor_barometer',
            namespace='barometer',
            executable='barometer_publisher',
            name='sensors'
        ),
    ])