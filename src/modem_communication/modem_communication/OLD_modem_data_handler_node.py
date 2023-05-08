from rclpy.node import Node
from sensor_interfaces.msg import Barometer, Battery, Modem, Oxygen, Salinity, Thermometer
from unetpy import UnetSocket
import time


class ModemSubscriberNode(Node):
    # Class variable
    MODEM_PORT = 1100
    times_checked = 0
    start_time = 0.0

    # Memory Variables
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
        super().__init__('ModemSubscriberNode')

        self.internal_modem_publisher_ = self.create_publisher(Modem, 'internal_modem_data', 10)  # Creates a publisher over the topic internal_modem_data
        self.external_modem_publisher_ = self.create_publisher(Modem, 'external_modem_data', 10)  # Creates a publisher over the topic external_modem_data
        
        self.sample_time  = self.declare_parameter('sample_time', 2.0).value  # Gets sample time as a parameter, default = 2
        self.n_sensors  = self.declare_parameter('sensor_count', 5).value  # Gets how many sensors it is expecting values from
        self.transfer_delay  = self.declare_parameter('transfer_delay', 6.0).value  # How many seconds a transmition usually takes
        self.MODEM_IP  = self.declare_parameter('modem_IP', '0.0.0.0').value  # IP for the modem
        self.start_time = time.time()

        # Open a scoket to the modem 
        self.sock  = UnetSocket(self.MODEM_IP, self.MODEM_PORT)

        # Check if the modem is connected
        # This feature is UNTESTED on an actual modem
        try:
            self.sock.isConnected()
        except:
            self.get_logger().error('Could not establish connection with modem')
            exit(1)


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


    def send_data_modem(self):
        if self.times_checked >= self.n_sensors:
            # Getting the local time
            current_time = time.localtime()
            local_time =  time.strftime("%H:%M:%S",current_time)
            # data = '%i,%s,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f' % (self.rigg_ID, # Handeled by modem
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
            
            try:
                self.sock.send(data, 0) # Sending to everyone that wants to listen
                self.get_logger().info('DATA SENT TO MODEM')
            except:
                self.get_logger().error('COULD NOT SEND DATA TO MODEM')
            self.times_checked = 0

            while (time.time() - self.start_time) > self.transfer_delay:
                self.get_logger().info('Listening')
                self.sock.setTimeout(self.sample_time - self.transfer_delay)
                rx = self.sock.receive()
                if rx:
                    external_data = str(rx.from_) + ',' + bytearray(rx.data).decode()

                    external_msg = Modem()
                    external_msg.external_data = external_data
                    self.external_modem_publisher_.publish(external_msg)


    def barometer_callback(self, msg:Barometer):
        self.barometer_data['pressure'] = msg.pressure_mbar
        self.barometer_data['depth'] = msg.depth
        self.barometer_data['time'] = msg.local_time

        self.get_logger().info('DATA RECIEVED')
        self.times_checked += 1
        self.send_data_modem()

    def battery_callback (self, msg:Battery):
        self.battery_data['voltage'] = msg.battery_voltage
        self.battery_data['current'] = msg.battery_current
        self.battery_data['percent'] = msg.battery_percent
        self.battery_data['time'] = msg.local_time

        self.get_logger().info('DATA RECIEVED')
        self.times_checked += 1
        self.send_data_modem()
 
    def oxygen_callback(self, msg:Oxygen):
        self.oxygen_data['oxygen'] = msg._oxygen_concentration
        self.oxygen_data['time'] = msg.local_time

        self.get_logger().info('DATA RECIEVED')
        self.times_checked += 1
        self.send_data_modem()
 
    def salinity_callback(self, msg:Salinity):
        self.salinity_data['salinity'] = msg.salinity_value
        self.salinity_data['time'] = msg.local_time

        self.get_logger().info('DATA RECIEVED')
        self.times_checked += 1
        self.send_data_modem()
 
    def temperature_callback(self, msg:Thermometer):
        self.temperature_data['temperature'] = msg.temperature_celsius
        self.temperature_data['time'] = msg.local_time

        self.get_logger().info('DATA RECIEVED')
        self.times_checked += 1
        self.send_data_modem()
