import rclpy
from rclpy.node import node
from sensor_thermometer import tsys01
from sensor_interfaces.msg import Thermometer

class ThermometerDataSubscriber(Node):
    
    def __init__(self):
        super().__init__('ThermometerDataSubscriber')
        self.subscription = self.create_subscription(Thermometer, 'thermometer_data', self.listener_callback, 10)
        self.subscription   #prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)

    thermometer_data_subscriber = ThermometerDataSubscriber()

    rclpy.spin(thermometer_data_subscriber)


    thermometer_data_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()