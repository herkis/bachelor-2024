import launch
from launch import LaunchDescription
from launch_ros.actions import Node

# Declare Sample time as float
sample_time = 10.0
n_sensors = 5
#rigg_ID = 1


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_oxygen',
            namespace='oxygen',
            executable='oxygen_publisher',
            name='sensors',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_thermometer',
            namespace='thermometer',
            executable='thermometer_publisher',
            name='sensors',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_barometer',
            namespace='barometer',
            executable='barometer_publisher',
            name='sensors',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_salinity',
            namespace='salinity',
            executable='salinity_publisher',
            name='sensors',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_battery',
            namespace='battery',
            executable='battery_publisher',
            name='sensors',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='modem_communication',
            namespace='subnero',
            executable='modem_subscriber',
            name='modem',
            # parameters=[{'rigg_ID': rigg_ID}, # handeled by modem
            parameters=[{'rigg_ID': rigg_ID},
                        {'sensor_count': n_sensors}
            ]
        ),
    ])