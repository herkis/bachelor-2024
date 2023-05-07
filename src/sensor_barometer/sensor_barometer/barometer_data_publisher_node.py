from rclpy.node import Node

# ms5837 needed in order to utilize the BlueRobotics MS5837 Python Library which must be installed
from sensor_barometer import ms5837
from sensor_interfaces.msg import Barometer
import time

class BarometerDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('BarometerDataPublisher')
        self.publisher_ = self.create_publisher(Barometer, 'barometer_data', 10)    # Creates a publisher over the topic barometer_data
        self.sample_time  = self.declare_parameter('sample_time', 2.0).value  # Gets sample time as a parameter, default = 2
        self.timer = self.create_timer(self.sample_time, self.barometer_read_and_publish)

        self.sensor = ms5837.MS5837_30BA()
        # self.sensor.setFluidDensity() # Configuring fluid density for fresh or saltwater. Defaulting to fresh water
        if not self.sensor.init():
            # If sensor can not be detected
            self.get_logger().error("Sensor could not be initialized")
            exit(1)

    def barometer_read_and_publish(self):
        # Custom barometer message to publish. Can be found in the sensor_interfaces.
        msg = Barometer()

        # Getting the local time  
        current_time = time.localtime()
        msg.local_time =  time.strftime("%H:%M:%S",current_time)


        # Reading barometer and loading data into custom message
        if self.sensor.read():
                msg.depth                   = self.sensor.depth()                               # Depth in meters using the fluid density (kg/m^3) configured by setFluidDensity()
                msg.pressure_mbar           = self.sensor.pressure()                            # Default is mbar (no arguments)
                msg.pressure_psi            = self.sensor.pressure(ms5837.UNITS_psi)            # Request psi
        else:
                self.get_logger().error("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /barometer_data
        self.publisher_.publish(msg)
        self.get_logger().info('\ttime: %s  Depth: %0.2f m  P: %0.1f mbar' % (msg.local_time,
                                                                            msg.depth, 
                                                                            msg.pressure_mbar))
