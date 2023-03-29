import launch
from launch import LaunchDescription
from launch_ros.actions import Node

#Once the battery code is written re-add the code below

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_oxygen',
            namespace='oxygen',
            executable='oxygen_publisher',
            name='sensors'
        ),
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
        Node(
            package='sensor_salinity',
            namespace='salinity',
            executable='salinity_publisher',
            name='sensors'
        ),
        #Node(
        #    package='sensor_battery',
        #    namespace='battery',
        #    executable='battery_publisher',
        #    name='sensors'
        #),
        Node(
            package='modem_communication',
            namespace='modem',
            executable='modem_sender',
            name='modem'
        ),
    ])