from rclpy.node import Node

from sensor_battery import ads1x15
from sensor_interfaces.msg import Battery
import time
import  re, uuid


class BatteryDataPublisher(Node):
    ## DEFINNING CONSTANTS
    A0 = 0 #Channel 0 on ADC connected to voltage read pin
    A1 = 1 #Channel 1 on ADC connected to current read pin
    GAIN = 1 # 4.096V reference point
    REFERENCE = 4.096 #Volt
    MAX_VALUE = 2**15 #Bits (the 16th-bit is sign reserved)
    NOMINAL_BATTERY_VOLTAGE = 22.2 #Volt (6S lipo battery nominal voltage)
    VOLTAGE_OFFSET = 0.33 #Volt
    CURRENT_SENSE = 37.8788 #Ampere / Volt
    MIN_BATTERY_VOLATAGE = 19.2 #Volt
    MAX_BATTERY_VOLTAGE = 25.2 #Volt

    # Initialize
    def __init__(self):
        super().__init__('BatteryDataPublisher')
        self.publisher_ = self.create_publisher(Battery, 'battery_data', 10)    # Creates a publisher over the topic battery_data
        self.sample_time  = self.declare_parameter('sample_time', 2.0).value  # Gets sample time as a parameter, default = 2
        self.timer = self.create_timer(self.sample_time, self.battery_read_and_publish)

        self.sensor = ads1x15.ADS1115()
        ## CALIBRATION
        self.get_logger().info('ADC calibration in progress...')
        calibration_list = []

        # Gather 10 samples of pin voltage of the span of 5 seconds
        for i in range(10):
            calibration_list.append(self.sensor.read_adc(self.A0, gain=self.GAIN))
            time.sleep(0.5)
            #print(calibration_list)

        # Calculating average bit value from voltage sense pin measured with A0 on the ADC
        cal_value = sum(calibration_list) / len(calibration_list)
        # Calculate the voltage from the calibration value
        self.cal_voltage = cal_value*self.REFERENCE/self.MAX_VALUE
        # Calculate the voltage and current constants
        self.voltage_constant = (self.REFERENCE/self.MAX_VALUE) * (self.NOMINAL_BATTERY_VOLTAGE/self.cal_voltage)
        self.current_constant = (self.REFERENCE/self.MAX_VALUE - self.VOLTAGE_OFFSET) * self.CURRENT_SENSE
        self.get_logger().info('ADC calibration done!')
        

    def battery_read_and_publish(self):
        # Custom battery message to publish. Can be found in the sensor_interfaces.
        msg = Battery()
       
        # Getting the local time 
        current_time = time.localtime()
        msg.local_time =  time.strftime("%H:%M:%S",current_time)
        # Getting the mac address of the system
        msg.mac = ':'.join(re.findall('..','%012x' % uuid.getnode()))

        # Reads voltage and current from ADC and prints it every second
        value = [self.sensor.read_adc(self.A0, gain=self.GAIN), 
                 self.sensor.read_adc(self.A1, gain=self.GAIN)]

        V = value[0]*self.voltage_constant
        I = value[1]*self.current_constant
        percent = 100/(self.MAX_BATTERY_VOLTAGE - self.MIN_BATTERY_VOLATAGE)*V-320
        
        msg.battery_voltage = V
        msg.battery_current = I
        msg.battery_percent = percent
        #else:
        #    print("Sensor read failed!")
        #    exit(1)

        self.publisher_.publish(msg)
        #self.get_logger().info('Mac: %s  Percent: %0.2f  V: %0.2f  I: %0.2f  %s' % (msg.mac,
        #                                                                              msg.battery_percent,
        #                                                                              msg.battery_voltage,
        #                                                                              msg.battery_current,
        #                                                                              msg.local_time))
        self.get_logger().info('\t\ttime: %s  V: %0.2f  %: %0.0f' % (msg.local_time,
                                                                     msg.battery_voltage,
                                                                     msg.battery_percent))