import rclpy
from modem_communication import OLD_modem_data_handler_node as node
        

def main(args=None):
    rclpy.init(args=args)

    # Construct the publisher
    modem_subscriber = node.ModemSubscriberNode()

    # Reading and publishing data at defined rate 
    rclpy.spin(modem_subscriber)

    # Clean up when script is stopped
    modem_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()