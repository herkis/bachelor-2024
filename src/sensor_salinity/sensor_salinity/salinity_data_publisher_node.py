from rclpy.node import Node

# tsys01 needed in order to utilize the BlueRobotics TSYS01 Python Library which must be installed
from sensor_salinity import catlas01
from sensor_interfaces.msg import Salinity
import time
import re, uuid

class SalinityDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('SalinityDataPublisher')
        self.publisher_ = self.create_publisher(Salinity, 'salinity_data', 10)  # Creates a publisher over the topic salinity_data
        read_period = 2  # Does a reading every 2 seconds
        self.timer = self.create_timer(read_period, self.salinity_read_and_publish)

        self.sensor = catlas01.CATLAS01()
        # if not self.sensor.init():
        #     print("Sensor could not be initialized")
        #     exit(1)

    def salinity_read_and_publish(self):
        # Custom conductivity message to publish. Can be found in the brov2_interfaces.
        msg = Salinity()

        # Adding a way to read the time 
        tim = time.localtime()
        msg.local_time =  time.strftime("%H:%M",tim)

        # Getting the mac address of the system
        msg.mac = ':'.join(re.findall('..','%012x' % uuid.getnode()))

        # Reading salinity and loading data into custom message
        if self.sensor.read():
                msg.salinity_value     = self.sensor._salinity
        else:
                print("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /salinity_data
        self.publisher_.publish(msg)
        self.get_logger().info('Mac: %s  O: %0.2f µs/cm  %s' % (msg.mac,
                                                                msg.salinity_value,
                                                                msg.local_time))
