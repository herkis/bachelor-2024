from rclpy.node import Node
from sensor_interfaces.msg import Modem
from unetpy import UnetSocket
import time

# NOT TESTED WITH A MODEM

class ModemCommunicator(Node):
    # Class variable
    start_time = 0.0

    
    def __init__(self):
        super().__init__('ModemCommunicator')

        self.external_modem_publisher_ = self.create_publisher(Modem, 'external_modem_data', 10)  # Creates a publisher over the topic external_modem_data

        self.sample_time  = self.declare_parameter('sample_time', 2.0).value  # Gets sample time as a parameter, default = 2
        self.transfer_delay  = self.declare_parameter('transfer_delay', 6.0).value  # How many seconds a transmition usually takes
        self.MODEM_IP  = self.declare_parameter('modem_IP', '0.0.0.0').value  # IP for the modem
        self.MODEM_PORT  = self.declare_parameter('modem_port', 1100).value  # API port for the modem
        self.start_time = time.time()

        # Open a scoket to the modem 
        self.sock  = UnetSocket(self.MODEM_IP, self.MODEM_PORT)

        # Check if the modem is connected
        # This feature is UNTESTED on an actual modem
        try:
            self.sock.isConnected()
        except:
            self.get_logger().error('Could not establish connection with modem')
            exit(1)


        self.internal_data_subscription = self.create_subscription(
            Modem, 
            '/modem/internal_modem_data', 
            self.modem_callback, 
            10)

        self.internal_data_subscription   # Prevent unused variable warning 


    def modem_callback(self, msg:Modem):
        data = msg.internal_data
        
        try:
            self.sock.send(data, 0) # Sending to everyone that wants to listen
            self.get_logger().info('DATA SENT TO MODEM')
        except:
            self.get_logger().error('COULD NOT SEND DATA TO MODEM')

        # Only [if] works in simulation check if [while] works IRL
        if (time.time() - self.start_time) > self.transfer_delay:
            #self.get_logger().info('Listening')
            self.sock.setTimeout(self.sample_time - self.transfer_delay)
            rx = self.sock.receive()
            if rx:
                external_data = str(rx.from_) + ',' + bytearray(rx.data).decode()

                external_msg = Modem()
                external_msg.external_data = external_data
                self.external_modem_publisher_.publish(external_msg)
