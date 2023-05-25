from rclpy.node import Node

from sensor_battery import ads1x15
from sensor_interfaces.msg import Battery
import time


class BatteryDataPublisher(Node):
    # HARDWARE CONSTANTS
    A0 = 0                      # Channel 0 on ADC connected to voltage read pin
    A1 = 1                      # Channel 1 on ADC connected to current read pin
    GAIN = 1                    # 4.096V reference point
    REFERENCE = 4.096           # Volt
    MAX_VALUE = 2**15           # Bits (the 16th-bit is sign reserved)
    VOLTAGE_OFFSET = 0.33       # Volt
    CURRENT_SENSE = 37.8788     # Ampere / Volt
    VOLTAGE_SENSE = 11          # Volt / Volt
    MIN_BATTERY_VOLATAGE = 19.2 # Volt
    MAX_BATTERY_VOLTAGE = 25.2  # Volt

    # Initialize
    def __init__(self):
        super().__init__('BatteryDataPublisher')
        self.publisher_ = self.create_publisher(Battery, 'battery_data', 10)    # Creates a publisher over the topic battery_data
        self.sample_time  = self.declare_parameter('sample_time', 2.0).value  # Gets sample time as a parameter, default = 2
        self.timer = self.create_timer(self.sample_time, self.battery_read_and_publish)

        self.sensor = ads1x15.ADS1115()
        
        # Calculate the voltage and current constants
        self.voltage_constant = (self.REFERENCE/self.MAX_VALUE) * self.VOLTAGE_SENSE
        self.current_constant = (self.REFERENCE/self.MAX_VALUE - self.VOLTAGE_OFFSET) * self.CURRENT_SENSE


    def battery_read_and_publish(self):
        # Custom battery message to publish. Can be found in the sensor_interfaces.
        msg = Battery()
       
        # Getting the local time 
        current_time = time.localtime()
        msg.local_time =  time.strftime("%H:%M:%S",current_time)

        # Reads voltage and current from ADC and prints it every second
        voltage_value = self.sensor.read_adc(self.A0, gain=self.GAIN)
        current_value = self.sensor.read_adc(self.A1, gain=self.GAIN)

        # Calculates all values
        V = voltage_value * self.voltage_constant + 0.6 # Adding 0.6 because it works
        I = current_value * self.current_constant
        percent = 100 / (self.MAX_BATTERY_VOLTAGE - self.MIN_BATTERY_VOLATAGE) * V - 320    # Does not work
        
        # Populates the message
        msg.battery_voltage = V
        msg.battery_current = I
        msg.battery_percent = percent

        self.publisher_.publish(msg)
        self.get_logger().info('\t\ttime: %s  V: %0.2f  %%: %d' % (msg.local_time,
                                                                   msg.battery_voltage,
                                                                   msg.battery_percent))