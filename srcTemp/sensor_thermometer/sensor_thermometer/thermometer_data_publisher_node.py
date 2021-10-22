from rclpy.node import Node

# tsys01 needed in order to utilize the BlueRobotics TSYS01 Python Library which must be installed
from sensor_thermometer import tsys01
from sensor_interfaces.msg import Thermometer



class ThermometerDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('ThermometerDataPublisher')
        self.publisher_ = self.create_publisher(Thermometer, 'thermometer_data', 10)
        read_period = 0.1  # seconds
        self.timer = self.create_timer(read_period, self.thermometer_read_and_publish)

        self.sensor = tsys01.TSYS01()
        if not self.sensor.init():
            print("Sensor could not be initialized")
            exit(1)

    def thermometer_read_and_publish(self):
        # Custom thermometer message to publish. Can be found in the brov2_interfaces.
        msg = Thermometer()

        # Reading thermometer and loading data into custom message
        if self.sensor.read():
                msg.temperature_celsius     = self.sensor.temperature()                         # Default is degrees C (no arguments)
                msg.temperature_farenheit   = self.sensor.temperature(tsys01.UNITS_Farenheit)   # Request Farenheit
        else:
                print("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /thermometer_data
        self.publisher_.publish(msg)
        self.get_logger().info('T: %0.2f C  %0.2f F' % (msg.temperature_celsius, 
                                                        msg.temperature_farenheit))
