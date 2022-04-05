import rclpy
from sensor_battery import battery_data_publisher_node as node


def main(args=None):
    rclpy.init(args=args)
    
    # Construct the publisher
    battery_data_publisher = node.BatteryDataPublisher()

    # Reading and publishing data at defined rate (2 seconds)
    rclpy.spin(battery_data_publisher)

    # Clean up when script is stopped 
    battery_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()