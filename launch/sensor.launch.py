import launch
from launch import LaunchDescription
from launch_ros.actions import Node

# Declare Sample time as float
sample_time = 10.0
n_sensors = 5
transfer_delay = 6.0


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_oxygen',
            namespace='sensors',
            executable='oxygen_publisher',
            name='oxygen',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_thermometer',
            namespace='sensors',
            executable='thermometer_publisher',
            name='thermometer',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_barometer',
            namespace='sensors',
            executable='barometer_publisher',
            name='barometer',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_salinity',
            namespace='sensors',
            executable='salinity_publisher',
            name='salinity',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='sensor_battery',
            namespace='sensors',
            executable='battery_publisher',
            name='battery',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='logger',
            namespace='logger',
            executable='internal_logger',
            name='internal',
            parameters=[]
        ),
        Node(
            package='logger',
            namespace='logger',
            executable='external_logger',
            name='external',
            parameters=[]
        ),
        Node(
            package='modem_communication',
            namespace='modem',
            executable='modem_subscriber',
            name='subnero',
            parameters=[
                        {'sample_time': sample_time},
                        {'sensor_count': n_sensors},
                        {'transfer_delay': transfer_delay}
            ]
        ),
    ])

