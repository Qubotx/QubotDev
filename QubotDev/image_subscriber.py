import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class SubscriberNodeClass(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        self.bridgeObject = CvBridge()
        self.topicNameFrames='topic_camera_image'
        self.queueSize=20
        self.subscription=self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callback_function,
            self.queueSize)
        self.subscription #in order to prevent 'unused variable warning'
        
    def listener_callback_function(self, imageMessage):
        self.get_logger().info('Image frame received')
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        
        window_name = "Camera Video"
        cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        cv2.imshow(window_name, openCVImage)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    subscriberObject = SubscriberNodeClass()
    rclpy.spin(subscriberObject)
    subscriberObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
