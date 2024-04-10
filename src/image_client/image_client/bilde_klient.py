

# Kjempe dårlig kode som må bli forkastet innen et smule bedre innsyn i klient/service system


from src.MyMessageFolder import MyMessage


from sensor_msgs.msg import Image


import rclpy
from rclpy.node import Node


import cv2 # pip install opencv-python
from cv_bridge import CvBridge




class ImageClient(Node):


   def __init__(self):
       super().__init__("image_client")
       self.cli = self.create_client(MyMessage, "my_message")
       while not self.cli.wait_for_service(timeout_sec=1.0):
           self.get_logger().info("service not available, waiting...")
       self.req = EvoImageTransfer.Request()


       self.image_path = "path/til/bilde.jpg"
       self.bridge = CvBridge()


       self.send_image(self.image_path)


   def send_image_request(self, image_path):


       # leses som en numpy array i BGR format (ikke RGB)
       cv_image = cv2.imread(image_path)


       # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
       ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
       self.req.image = ros_image


       self.future = self.cli.call_async(self.req)
       rclpy.spin_until_future_complete(self, self.future)
       return self.future.result()




def main():
   rclpy.init()


   image_client = ImageClient()


   while rclpy.ok():
       rclpy.spin_once(image_client)
       if image_client.future.done():
           try:
               response = image_client.future.result()
           except Exception as e:
               image_client.get_logger().info(f'Service call failed: {e}')
           else:
               image_client.get_logger().info(f'Service call succeeded: {response.success}')
           break


   image_client.destroy_node()
   rclpy.shutdown()




if __name__ == '__main__':
   main()


