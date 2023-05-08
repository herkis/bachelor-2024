import rclpy
from modem_communication import modem_data_sender_node as node
        

def main(args=None):
    rclpy.init(args=args)

    # Construct the publisher
    modem_sender = node.ModemCommunicator()

    # Reading and publishing data at defined rate 
    rclpy.spin(modem_sender)

    # Clean up when script is stopped
    modem_sender.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()