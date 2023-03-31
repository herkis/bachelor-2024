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

        ## DEFINNING CONSTANTS
        A0 = 0 #Channel 0 on ADC connected to voltage read pin
        A1 = 1 #Channel 1 on ADC connected to current read pin
        GAIN = 1 # 4.096V reference point
        REFERENCE = 4.096 #Volt
        RESOLUTION = 2**15 #Bits (the 16th-bit is sign reserved)
        NOMINAL_BATTERY_VOLTAGE = 23.1 #Volt
        VOLTAGE_OFFSET = 0.33 #Volt
        CURRENT_SENSE = 37.8788 #Ampere / Volt

        ## CALIBRATION
        self.get_logger().info('ADC calibration in progress...')
        calibration_list = []

        # Gather 10 samples of pin voltage of the span of 5 seconds
        for i in range(10):
            calibration_list.append(self.read_adc(A0, gain=GAIN))
            time.sleep(0.5)
            #print(calibration_list)

        # Calculating average bit value from coltage sense pin measured with A0 on the ADC
        cal_value = sum(calibration_list) / len(calibration_list)
        # Calculate the voltage from the calibration value
        cal_voltage = cal_value*REFERENCE/RESOLUTION

        # Main loop reads voltage and current from ADC and prints it every second
        while True:
            value = [self.sensor.read_adc(A0, gain=GAIN), self.sensor.read_adc(A1, gain=GAIN)]

            V = (value[0]*REFERENCE/RESOLUTION) * (NOMINAL_BATTERY_VOLTAGE/cal_voltage)
            I = (value[1]*REFERENCE/RESOLUTION - VOLTAGE_OFFSET) * CURRENT_SENSE

            # print('V = %0.2f  I = %0.2f' % (V,I))

            time.sleep(1)
        
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
