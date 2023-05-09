from rclpy.node import Node
from sensor_interfaces.msg import Modem
import datetime
import csv


class ExternalLoggerNode(Node):

    def __init__(self):
        super().__init__('external_logger')

        current_datetime = datetime.datetime.now().strftime("%d_%m_%Y__%H_%M_%S")
        directory = 'log_data/external/'
        self.file = directory + 'external_log_' + current_datetime + '.csv'
        header = 'Modem_ID,Time,Depth,Voltage,Oxygen,Salinity,Temerature'

        with open(self.file, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file, delimiter=';')
            writer.writerow([header])


        self.external_modem_subscription = self.create_subscription(
            Modem, 
            '/modem/external_data', 
            self.external_modem_callback, 
            10)


        self.external_modem_subscription   # Prevent unused variable warning

    def external_modem_callback(self, msg:Modem):
        data = msg.external_data
        with open(self.file, 'a', newline='') as csv_file:
                writer = csv.writer(csv_file, delimiter=';')
                writer.writerow([data])
