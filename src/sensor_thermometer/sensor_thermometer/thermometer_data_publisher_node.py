# This code is responsible for getting the data from the sensor and publishing it.
from rclpy.node import Node

# tsys01 needed in order to utilize the BlueRobotics TSYS01 Python Library which must be installed
from sensor_thermometer import tsys01
from sensor_interfaces.msg import Thermometer
import time
import  re, uuid        # Used for getting the mac address 

class ThermometerDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('ThermometerDataPublisher')
        self.publisher_ = self.create_publisher(Thermometer, 'thermometer_data', 10)    # Creates a publisher over the topic thermometer_data
        read_period = 2  # Does a reading every 2 seconds
        self.timer = self.create_timer(read_period, self.thermometer_read_and_publish)

        # Assign the function TSYS01() in the tsys01.py to self.sensor
        self.sensor = tsys01.TSYS01()
        if not self.sensor.init():
            # If sensor can not be detected
            print("Sensor could not be initialized")
            exit(1)

    def thermometer_read_and_publish(self):
        # Custom thermometer message to publish. Can be found in the brov2_interfaces.
        msg = Thermometer()

        # Geting the local time
        tim = time.localtime()
        msg.local_time =  time.strftime("%H:%M",tim)

        # Getting the mac address of the system:
        msg.mac = ':'.join(re.findall('..','%012x' % uuid.getnode()))

        # Calls the function sensor.read in TSY01 to get the desired values from the sensors
        if self.sensor.read():
                msg.temperature_celsius     = self.sensor.temperature()                         # Default is degrees C (no arguments)
                msg.temperature_farenheit   = self.sensor.temperature(tsys01.UNITS_Farenheit)   # Request Farenheit
        else:
                print("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /thermometer_data
        self.publisher_.publish(msg)
        #self.get_logger().info('Mac: %s  T: %0.2f C  %0.2f F  %s' % (msg.mac,
        #                                                             msg.temperature_celsius, 
        #                                                             msg.temperature_farenheit,
        #                                                             msg.local_time))
        
        self.get_logger().info('\ttime: %s  T: %0.2f C' % (msg.local_time,
                                                        msg.temperature_celsius))

