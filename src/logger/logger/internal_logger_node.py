from rclpy.node import Node
from sensor_interfaces.msg import Modem
import datetime
import csv


class InternalLoggerNode(Node):
    
    def __init__(self):
        super().__init__('internal_logger')

        current_datetime = datetime.datetime.now().strftime("%d_%m_%Y__%H_%M_%S")
        directory = 'log_data/internal/'
        self.file = directory + 'internal_log_' + current_datetime + '.csv'
        header = 'Time,Depth,Voltage,Oxygen,Salinity,Temerature'

        with open(self.file, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file, delimiter=';')
            writer.writerow([header])


        self.internal_modem_subscription = self.create_subscription(
            Modem, 
            '/modem/internal_modem_data', 
            self.internal_modem_callback, 
            10)


        self.internal_modem_subscription   # Prevent unused variable warning

    def internal_modem_callback(self, msg:Modem):
        data = msg.internal_data
        with open(self.file, 'a', newline='') as csv_file:
                writer = csv.writer(csv_file, delimiter=';')
                writer.writerow([data])

 