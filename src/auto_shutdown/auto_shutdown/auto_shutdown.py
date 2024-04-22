import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import Battery
import subprocess

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('BatteryMonitor')
        self.subscription = self.create_subscription(
            Battery, 'battery_data', self.battery_callback, 10)
        self.subscription  # prevent unused variable warning

    def battery_callback(self, msg):
        self.get_logger().info(f'Received battery data: {msg.battery_percent}%')
        if msg.battery_percent < 20.0:
            self.get_logger().info('Battery low, shutting down...')
            # Command to shut down the Raspberry Pi safely
            subprocess.run(['sudo', 'shutdown', '-h', 'now'])

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
