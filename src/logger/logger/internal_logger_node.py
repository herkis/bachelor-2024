from rclpy.node import Node
from sensor_interfaces.msg import Modem
import datetime
import csv
import os

class InternalLoggerNode(Node):
    
    def __init__(self):
        super().__init__('internal_logger')

        # Fetches local date and time
        current_date = datetime.datetime.now().strftime("%d_%m_%Y")
        current_time = datetime.datetime.now().strftime("%H_%M_%S")

        # Builds file name and destination
        directory = 'log_data/' + current_date + '/internal/'
        self.file = directory + 'internal_log_' + current_time + '.csv'

        header = 'Time,Pressure,Voltage,Oxygen,Salinity,Temperature'

        # Creating a new directroy if none exists
        if not os.path.exists(directory):
            os.makedirs(directory)

        # Opens a new .csv file
        with open(self.file, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file, delimiter=';')
            writer.writerow([header])

        self.internal_modem_subscription = self.create_subscription(
            Modem, 
            '/modem/internal_data', 
            self.internal_modem_callback, 
            10)

    def internal_modem_callback(self, msg:Modem):
        data = msg.internal_data
        with open(self.file, 'a', newline='') as csv_file:
                writer = csv.writer(csv_file, delimiter=';')
                writer.writerow([data])

 