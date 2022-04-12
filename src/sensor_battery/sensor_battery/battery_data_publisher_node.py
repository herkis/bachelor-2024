from rclpy.node import Node

from sensor_battery import ads1x15
from sensor_interfaces.msg import Battery

class BatteryDataPublisher(Node):
    # Initialize
    def __init__(self):
        super().__init__('BatteryDataPublisher')
        self.publisher_ = self.create_publisher(Barometer, 'battery_data', 10)    # Creates a publisher over the topic battery_data
        read_period = 2  # Does a reading every 2 seconds
        self.timer = self.create_timer(read_period, self.battery_read_and_publish)

        self.sensor = ads1x15.ADS1x15()

    def battery_read_and_publish(self):
        # Custom battery message to publish. Can be found in the sensor_interfaces.
        msg = Battery()

        
        # TODO: legg inn kode for avlesning av batteri her samt faktisk publishe 

        adc = self.sensor.read_adc(0,gain=GAIN) # Reads the value from channel A0

        V = adc*(5.0/1024)*11.0 # To convert the numbers read from adc to more logical values

        self.publisher_.publish(msg)
        self.get_logger().info(##Write in here)


