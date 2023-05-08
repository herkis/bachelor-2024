from rclpy.node import Node
from sensor_interfaces.msg import Barometer, Battery, Modem, Oxygen, Salinity, Thermometer
import time


class ModemDataHandler(Node):
    # Class Variables
    times_checked = 0

    barometer_data = {
        'time': '00:00',
        'depth': 0.0,
        'pressure': 0.0
    }
    battery_data = {
        'time': '00:00',
        'voltage': 0.0,
        'current': 0.0,
        'percent': 0.0
    }
    oxygen_data = {
        'time': '00:00',
        'oxygen': 0.0
    }
    salinity_data = {
        'time': '00:00',
        'salinity': 0.0
    }
    temperature_data = {
        'time': '00:00',
        'temperature': 0.0
    }
    
    def __init__(self):
        super().__init__('ModemDataHandler')

        self.internal_modem_publisher_ = self.create_publisher(Modem, 'internal_modem_data', 10)  # Creates a publisher over the topic internal_modem_data
        self.n_sensors  = self.declare_parameter('sensor_count', 5).value  # Gets how many sensors it is expecting values from


        self.barometer_subscription = self.create_subscription(
            Barometer, 
            '/sensors/barometer_data', 
            self.barometer_callback, 
            10)

        self.battery_subscription = self.create_subscription(
            Battery, 
            '/sensors/battery_data', 
            self.battery_callback, 
            10)

        self.oxygen_subscription = self.create_subscription(
            Oxygen, 
            '/sensors/oxygen_data', 
            self.oxygen_callback, 
            10)

        self.salinity_subscription = self.create_subscription(
            Salinity, 
            '/sensors/salinity_data', 
            self.salinity_callback, 
            10)

        self.temperature_subscription = self.create_subscription(
            Thermometer, 
            '/sensors/thermometer_data', 
            self.temperature_callback, 
            10)

        #Prevent unused variable warning
        self.barometer_subscription   # Prevent unused variable warning
        self.battery_subscription   # Prevent unused variable warning
        self.oxygen_subscription   # Prevent unused variable warning
        self.salinity_subscription   # Prevent unused variable warning
        self.temperature_subscription   # Prevent unused variable warning


    def publish_data(self):
        if self.times_checked >= self.n_sensors:
            # Getting the local time
            current_time = time.localtime()
            local_time =  time.strftime("%H:%M:%S",current_time)

            data = '%s,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f' % (local_time,
                                                         self.barometer_data['depth'],
                                                         self.battery_data['voltage'],
                                                         self.oxygen_data['oxygen'],
                                                         self.salinity_data['salinity'],
                                                         self.temperature_data['temperature'])
            
            modem_msg = Modem()
            modem_msg.internal_data = data

            self.get_logger().info(data)
            self.internal_modem_publisher_.publish(modem_msg)
            self.times_checked = 0


    def barometer_callback(self, msg:Barometer):
        self.barometer_data['pressure'] = msg.pressure_mbar
        self.barometer_data['depth'] = msg.depth
        self.barometer_data['time'] = msg.local_time

        self.times_checked += 1
        self.publish_data()

    def battery_callback (self, msg:Battery):
        self.battery_data['voltage'] = msg.battery_voltage
        self.battery_data['current'] = msg.battery_current
        self.battery_data['percent'] = msg.battery_percent
        self.battery_data['time'] = msg.local_time

        self.times_checked += 1
        self.publish_data()
 
    def oxygen_callback(self, msg:Oxygen):
        self.oxygen_data['oxygen'] = msg._oxygen_concentration
        self.oxygen_data['time'] = msg.local_time

        self.times_checked += 1
        self.publish_data()
 
    def salinity_callback(self, msg:Salinity):
        self.salinity_data['salinity'] = msg.salinity_value
        self.salinity_data['time'] = msg.local_time

        self.times_checked += 1
        self.publish_data()
 
    def temperature_callback(self, msg:Thermometer):
        self.temperature_data['temperature'] = msg.temperature_celsius
        self.temperature_data['time'] = msg.local_time

        self.times_checked += 1
        self.publish_data()
