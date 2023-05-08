import rclpy
from logger import internal_logger_node as node
        

def main(args=None):
    rclpy.init(args=args)

    # Construct the publisher
    internal_logger = node.InternalLoggerNode()

    # Reading and publishing data at defined rate 
    rclpy.spin(internal_logger)

    # Clean up when script is stopped
    internal_logger.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()