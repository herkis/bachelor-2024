from rclpy.node import Node

# tsys01 needed in order to utilize the BlueRobotics TSYS01 Python Library which must be installed
from sensor_oxygen import doatlas01
from sensor_interfaces.msg import Oxygen


class OxygenDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('OxygenDataPublisher')
        self.publisher_ = self.create_publisher(Oxygen, 'oxygen_data', 10)
        read_period = 0.1  # seconds
        self.timer = self.create_timer(read_period, self.oxygen_read_and_publish)

        self.sensor = doatlas01.DOATLAS01()
        # if not self.sensor.init():                                                                             ## CHANGE
        #     print("Sensor could not be initialized")
        #     exit(1)

    def oxygen_read_and_publish(self):
        # Custom thermometer message to publish. Can be found in the brov2_interfaces.
        msg = Oxygen()

        # Reading thermometer and loading data into custom message
        if self.sensor.read():
                msg.oxygen_concentration     = self.sensor.oxygen()
        else:
                print("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /thermometer_data
        self.publisher_.publish(msg)
        self.get_logger().info('O: %0.2f mg/L' % (msg.oxygen_concentration))
