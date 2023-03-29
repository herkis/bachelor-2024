import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import Thermometer

# This is an example of how the subscriber node can be written. Very similar to the publisher code

class ModemSubscriberNode(Node):
    
    def __init__(self):
        #defining variables
        self.temp_data = [0,0]


        super().__init__('modem_subscriber')
        self.temp_subscriber = self.create_subscription(Thermometer, 'thermometer_data', self.temp_callback, 10)
        self.temp_subscriber   # Prevent unused variable warning

    def temp_callback(self, msg:Thermometer):
        self.temp_data[0] = Thermometer.temperature_celsius
        self.temp_data[1] = Thermometer.local_time
        self.get_logger().info('Extracted %0.2f C at time: %s'% (self.temp_data[0], self.temp_data[1]))
        

def main(args=None):
    rclpy.init(args=args)

    modem_subscriber = ModemSubscriberNode()

    rclpy.spin(modem_subscriber)


    modem_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()