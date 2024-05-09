import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.timer_period = 1  # seconds (adjust as needed)
        self.timer = self.create_timer(self.timer_period, self.publish_image)
        self.bridge = CvBridge()
        self.image_path = "/home/SUMS/bachelor-2024/images/output_0.jpg"  # Update this path

    def publish_image(self):
        cv_image = cv2.imread(self.image_path)
        if cv_image is not None:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing image')
        else:
            self.get_logger().info('No image found, please check the image path.')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
