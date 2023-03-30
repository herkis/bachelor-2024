import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import *

class ModemSubscriberNode(Node):
    
    def __init__(self):
        super().__init__('modem_subscriber')

        #defining variables
        ##  Convert to dict  ##
        self.barometer_data = [0.0,0.0]
        self.oxygen_data = [0.0,0.0]
        self.salinity_data = [0.0,0.0]
        self.temperature_data = [0.0,0.0]

        self.barometer_subscription = self.create_subscription(
            Barometer, 
            '/barometer/barometer_data', 
            self.barometer_callback, 
            10)

        self.oxygen_subscription = self.create_subscription(
            Oxygen, 
            '/oxygen/oxygen_data', 
            self.oxygen_callback, 
            10)

        self.salinity_subscription = self.create_subscription(
            Salinity, 
            '/salinity/salinity_data', 
            self.salinity_callback, 
            10)

        self.temperature_subscription = self.create_subscription(
            Thermometer, 
            '/thermometer/thermometer_data', 
            self.temperature_callback, 
            10)

        #Prevent unused variable warning
        self.barometer_subscription   # Prevent unused variable warning
        self.oxygen_subscription   # Prevent unused variable warning
        self.salinity_subscription   # Prevent unused variable warning
        self.temperature_subscription   # Prevent unused variable warning

    def barometer_callback(self, msg:Barometer):
        self.barometer_data[0] = msg.pressure_mbar
        self.barometer_data[1] = msg.local_time
        self.get_logger().info('Extracted %0.2f mbar at time: %s'% (self.barometer_data[0], self.barometer_data[1]))
        print('\nhello form logger\n')
 
    def oxygen_callback(self, msg:Oxygen):
        self.oxygen_data[0] = msg._oxygen_concentration
        self.oxygen_data[1] = msg.local_time
        self.get_logger().info('Extracted %0.2f O at time: %s'% (self.oxygen_data[0], self.oxygen_data[1]))
        print('\nhello form logger\n')
 
    def salinity_callback(self, msg:Salinity):
        self.salinity_data[0] = msg.salinity_value
        self.salinity_data[1] = msg.local_time
        self.get_logger().info('Extracted %0.2f O at time: %s'% (self.salinity_data[0], self.salinity_data[1]))
        print('\nhello form logger\n')
 
    def temperature_callback(self, msg:Thermometer):
        self.temperature_data[0] = msg.temperature_celsius
        self.temperature_data[1] = msg.local_time
        self.get_logger().info('Extracted %0.2f C at time: %s'% (self.temperature_data[0], self.temperature_data[1]))
        print('\nhello form logger\n')
        

def main(args=None):
    rclpy.init(args=args)

    modem_subscriber = ModemSubscriberNode()

    rclpy.spin(modem_subscriber)


    modem_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()