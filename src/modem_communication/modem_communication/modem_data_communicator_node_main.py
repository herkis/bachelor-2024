import rclpy
from modem_communication import modem_data_communicator_node as node
        

def main(args=None):
    rclpy.init(args=args)

    # Construct the publisher
    modem_communicator = node.ModemCommunicator()

    # Reading and publishing data at defined rate 
    rclpy.spin(modem_communicator)

    # Clean up when script is stopped
    modem_communicator.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()