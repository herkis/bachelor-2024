from rclpy.node import Node

# tsys01 needed in order to utilize the BlueRobotics TSYS01 Python Library which must be installed
from sensor_salinity import catlas01
from sensor_interfaces.msg import Salinity


class SalinityDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('SalinityDataPublisher')
        self.publisher_ = self.create_publisher(Salinity, 'salinity_data', 10)
        read_period = 0.01  # seconds
        self.timer = self.create_timer(read_period, self.salinity_read_and_publish)

        self.sensor = catlas01.CATLAS01()
        # if not self.sensor.init():
        #     print("Sensor could not be initialized")
        #     exit(1)

    def salinity_read_and_publish(self):
        # Custom conductivity message to publish. Can be found in the brov2_interfaces.
        msg = Salinity()

        # Reading thermometer and loading data into custom message
        if self.sensor.read():
                msg.salinity_value     = self.sensor._salinity
        else:
                print("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /thermometer_data
        self.publisher_.publish(msg)
        self.get_logger().info('O: %0.2f µs/cm' % (msg.salinity_value))
