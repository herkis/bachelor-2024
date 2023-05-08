import rclpy
from logger import external_logger_node as node
        

def main(args=None):
    rclpy.init(args=args)

    # Construct the publisher
    external_logger = node.ExternalLoggerNode()

    # Reading and publishing data at defined rate 
    rclpy.spin(external_logger)

    # Clean up when script is stopped
    external_logger.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()