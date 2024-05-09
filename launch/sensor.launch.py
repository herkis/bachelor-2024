from launch import LaunchDescription
from launch_ros.actions import Node
import random

sample_time = 10.0
n_sensors = 5
transfer_delay = 6.0
modem_IP = '192.168.42.86'
modem_port = 1100
precision = 2
log_header = 'Time,Pressure,Voltage,Current,Oxygen,Salinity,Temperature'

ms = 1000
step = 200
lower_bounds = [2 * ms, 4 * ms]
upper_bounds = [4 * ms, 6 * ms]
random_bounds = [
    random.randrange(lower_bounds[0], upper_bounds[0], step),
    random.randrange(lower_bounds[1], upper_bounds[1], step)
]

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
        # Include all other nodes similarly...
        Node(
            package='cv_algorithm',
            namespace='camera',
            executable='computer_vision',
            name='computer_vision',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='auto_shutdown',
            namespace='sensors',
            executable='battery_monitor',
            name='shutdown',
            parameters=[{'sample_time': sample_time}]
        ),
        Node(
            package='image_client',
            namespace='modem',
            executable='bilde_klient',
            name='imagetransfer',
            parameters=[
                {'sample_time': sample_time},
                {'transfer_delay': transfer_delay},
                {'modem_IP': modem_IP},
                {'modem_port': modem_port},
                {'lower_bound': random_bounds[0]},
                {'upper_bound': random_bounds[1]}
            ]
        ),
    ])
