from rclpy.node import Node

from sensor_battery import ads1x15
from sensor_interfaces.msg import Battery

class BatteryDataPublisher(Node):
    # Initialize
    def __init__(self):
        super().__init__('BatteryDataPublisher')
        self.publisher_ = self.create_publisher(Battery, 'battery_data', 10)    # Creates a publisher over the topic battery_data
        read_period = 2  # Does a reading every 2 seconds
        self.timer = self.create_timer(read_period, self.battery_read_and_publish)

        self.sensor = ads1x15.ADS1x15()

    def battery_read_and_publish(self):
        # Custom battery message to publish. Can be found in the sensor_interfaces.
        msg = Battery()

        # Choose a gain of 1 for reading voltages from 0 to 4.09V.
        # Or pick a different gain to change the range of voltages that are read:
        #  - 2/3 = +/-6.144V
        #  -   1 = +/-4.096V
        #  -   2 = +/-2.048V
        #  -   4 = +/-1.024V
        #  -   8 = +/-0.512V
        #  -  16 = +/-0.256V
        # See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
        GAIN = 1
        
        # TODO:
        # Get readings from ADS1115 and convert to proper values.(https://discuss.bluerobotics.com/t/need-help-connecting-the-power-sense-module-r2-to-a-arduino/4679)
        # Compare these values to a graph of battery lifecycle to determine battery percentage. 

        adc_value0 = self.read_adc(0, gain=GAIN) #Reads the ADC-value on channel A0

        self.publisher_.publish(msg)
        self.get_logger().info()##Write in here)


