from rclpy.node import Node

# tsys01 needed in order to utilize the BlueRobotics TSYS01 Python Library which must be installed
from sensor_oxygen import doatlas01
from sensor_interfaces.msg import Oxygen
import time

class OxygenDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('OxygenDataPublisher')
        self.publisher_ = self.create_publisher(Oxygen, 'oxygen_data', 10)  # Creates a publisher over the topic oxygen_data
        self.sample_time  = self.declare_parameter('sample_time', 2.0).value  # Gets sample time as a parameter, default = 2
        self.timer = self.create_timer(self.sample_time, self.oxygen_read_and_publish)

        self.sensor = doatlas01.DOATLAS01()
        # if not self.sensor.init():                                                                             ## CHANGE
        #     self.get_logger().error("Sensor could not be initialized")
        #     exit(1)

    def oxygen_read_and_publish(self):
        # Custom dissolved oxygen message to publish. Can be found in the brov2_interfaces.
        msg = Oxygen()
        
        # Getting the local time 
        current_time = time.localtime()
        msg.local_time =  time.strftime("%H:%M:%S",current_time)

        # Reading dissolved oxygen and loading data into custom message
        if self.sensor.read():
                msg.oxygen_concentration     = self.sensor._oxygen
        else:
                self.get_logger().error("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /oxygen_data
        self.publisher_.publish(msg)
        self.get_logger().info('\t\ttime: %s  O: %0.2f mg/L' % (msg.local_time,
                                                            msg.oxygen_concentration))
