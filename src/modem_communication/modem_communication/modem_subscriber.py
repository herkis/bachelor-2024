import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import Thermometer

class ModemSubscriberNode(Node):
    
    def __init__(self):
        super().__init__('modem_subscriber')

        #defining variables
        self.temp_data = [0.0,0.0]

        self.temp_subscriber = self.create_subscription(Thermometer, 'sensor_interfaces/msg/Thermometer', self.temp_callback, 10)
        self.temp_subscriber   # Prevent unused variable warning

    def temp_callback(self, msg:Thermometer):
        self.temp_data[0] = msg.temperature_celsius
        self.temp_data[1] = msg.local_time
        self.get_logger().info('Extracted %0.2f C at time: %s'% (self.temp_data[0], self.temp_data[1]))
        

def main(args=None):
    rclpy.init(args=args)

    modem_subscriber = ModemSubscriberNode()

    rclpy.spin(modem_subscriber)


    modem_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()