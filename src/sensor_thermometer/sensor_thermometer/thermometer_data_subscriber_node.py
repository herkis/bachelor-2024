import rclpy
from rclpy.node import Node
from sensor_thermometer import tsys01
from sensor_interfaces.msg import Thermometer

# This is an example of how the subscriber node can be written. Very similar to the publisher code

class ThermometerDataSubscriber(Node):
    
    def __init__(self):
        super().__init__('ThermometerDataSubscriber')
        self.subscription = self.create_subscription(Thermometer, 'thermometer_data', self.listener_callback, 10)
        self.subscription   # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Mac: %s  T: %0.2f C  %0.2f F  %s' % (msg.mac,
                                                                     msg.temperature_celsius, 
                                                                     msg.temperature_farenheit,
                                                                     msg.local_time))

def main(args=None):
    rclpy.init(args=args)

    thermometer_data_subscriber = ThermometerDataSubscriber()

    rclpy.spin(thermometer_data_subscriber)


    thermometer_data_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()