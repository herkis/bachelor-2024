# import launch # Not needed?
from launch import LaunchDescription
from launch_ros.actions import Node
import random

sample_time = 10.0              # Sample time in seconds (float)
n_sensors = 5                   # How many sensors that are in use
transfer_delay = 6.0            # How long transferring data takes in seconds(float)
modem_IP = '192.168.42.195'     # IP for the modem
modem_port = 1100               # Port for the modem
precision = 2

# Random generator to avoid deadlocks
lower_bounds = [2, 4]   # Seconds
upper_bounds = [4, 6]   # Seconds
random_bounds = [
    random.randrange(lower_bounds[0]*1000, lower_bounds[1]*1000, 200),
    random.randrange(upper_bounds[0]*1000, upper_bounds[1]*1000, 200)
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
            executable='modem_data_handler',
            name='data_handler',
            parameters=[
                        {'sensor_count': n_sensors},
                        {'transfer_precision': precision}
            ]
        ),
        Node(
            package='modem_communication',
            namespace='modem',
            executable='modem_data_communicator',
            name='subnero',
            parameters=[
                        {'sample_time': sample_time},
                        {'transfer_delay': transfer_delay},
                        {'modem_IP': modem_IP},
                        {'modem_port': modem_port},
                        {'lower_bound': random_bounds[0]},
                        {'upper_bound': random_bounds[1]}
            ]
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
        Node(
            package='auto_shutdown',
            namespace='sensors',
            executable='battery_monitor',
            name='shutdown',
            parameters=[{'sample_time': sample_time}]
        ),
    ])