import rclpy
from sensor_oxygen import oxygen_data_publisher_node as node


def main(args=None):
    rclpy.init(args=args)
    
    # Construct the publisher
    oxygen_data_publisher = node.OxygenDataPublisher()

    # Reading and publishing data at defined rate (0.1 seconds)
    rclpy.spin(oxygen_data_publisher)

    # Clean up when script is stopped
    oxygen_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()