from rclpy.node import Node

# tsys01 needed in order to utilize the BlueRobotics TSYS01 Python Library which must be installed
from sensor_oxygen import doatlas01
from sensor_interfaces.msg import Oxygen
import time
import  re, uuid

class OxygenDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('OxygenDataPublisher')
        self.publisher_ = self.create_publisher(Oxygen, 'oxygen_data', 10)  # Creates a publisher over the topic oxygen_data
        read_period = 2  # Does a reading every 2 seconds
        self.timer = self.create_timer(read_period, self.oxygen_read_and_publish)

        self.sensor = doatlas01.DOATLAS01()
        # if not self.sensor.init():                                                                             ## CHANGE
        #     print("Sensor could not be initialized")
        #     exit(1)

    def oxygen_read_and_publish(self):
        # Custom dissolved oxygen message to publish. Can be found in the brov2_interfaces.
        msg = Oxygen()
        
        # Getting the local time
        tim = time.localtime()
        msg.local_time =  time.strftime("%H:%M",tim)

        # Getting the mac address of the system
        msg.mac = ':'.join(re.findall('..','%012x' % uuid.getnode()))

        # Reading dissolved oxygen and loading data into custom message
        if self.sensor.read():
                msg.oxygen_concentration     = self.sensor._oxygen
        else:
                print("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /oxygen_data
        self.publisher_.publish(msg)
        #self.get_logger().info('Mac: %s  O: %0.2f mg/L  %s' % (msg.mac,
        #                                                       msg.oxygen_concentration,
        #                                                       msg.local_time))
        
        self.get_logger().info('time: %s  O: %0.2f mg/L' % (msg.local_time,
                                                            msg.oxygen_concentration))
