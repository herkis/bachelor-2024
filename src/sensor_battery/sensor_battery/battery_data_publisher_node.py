from rclpy.node import Node

from sensor_battery import ads1x15
from sensor_interfaces.msg import Battery
import time
import  re, uuid

class BatteryDataPublisher(Node):
    # Initialize
    def __init__(self):
        super().__init__('BatteryDataPublisher')
        self.publisher_ = self.create_publisher(Battery, 'battery_data', 10)    # Creates a publisher over the topic battery_data
        self.sample_time  = self.declare_parameter('sample_time', 2.0).value  # Gets sample time as a parameter, default = 2
        self.timer = self.create_timer(self.sample_time, self.battery_read_and_publish)

        self.sensor = ads1x15.ADS1x15()

    def battery_read_and_publish(self):
        # Custom battery message to publish. Can be found in the sensor_interfaces.
        msg = Battery()
       
        # Getting the local time
        tim = time.localtime()
        msg.local_time =  time.strftime("%H:%M",tim)

        # Getting the mac address of the system
        msg.mac = ':'.join(re.findall('..','%012x' % uuid.getnode()))

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
        #if self.sensor.read():
        adc_value0 = self.read_adc(0, gain=GAIN) #Reads the ADC-value on channel A0
        adc_value1 = self.read_adc(1, gain=GAIN) #Reads the ADC-value on channel A1

        I = (adc_value0*(5/1024)-0.33)*38.8788 #Lurer på om det skal være 1023 ikke 1024 siden det starter på 0 ikke 1
        V = adc_value1*(5/1024)*11.0

        msg.battery_voltage = V
        msg.battery_current = I
        msg.battery_percent = 404.0
        #else:
        #    print("Sensor read failed!")
        #    exit(1)

        self.publisher_.publish(msg)
        self.get_logger().info('Mac: %s  Percent: %0.2f %  V: %0.2f  I: %0.2f  %s' % (msg.mac,
                                                                                      msg.battery_percent,
                                                                                      msg.battery_voltage,
                                                                                      msg.battery_current,
                                                                                      msg.local_time))
