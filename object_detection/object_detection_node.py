import rclpy

from rclpy.node import Node 

from sensor_msgs.msg import Image
from cvlib.object_detection import draw_bbox
import cvlib as cv
from cv_bridge import CvBridge

import time

class Detector(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.pub = self.create_publisher(Image, "/object_detection/output", 1)
        self.subscriber = self.create_subscription(Image, "/ankhbot/camera/image_raw", self.callback, 10)
        self.cv_bridge = CvBridge()

    def callback(self, msg):
        time_now = time.time()
        img_opencv = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "rgb8") 
        bbox, label, conf = cv.detect_common_objects(img_opencv)
        output_image = draw_bbox(img_opencv, bbox, label, conf)

        img_msg = self.cv_bridge.cv2_to_imgmsg(output_image, encoding = "rgb8")
        img_msg.header =  msg.header
        self.pub.publish(img_msg)
        self.get_logger().info("detection took {}s".format(time.time()-time_now))
    
def main(args = None):
    rclpy.init(args = args)
    detector = Detector()
    rclpy.spin(detector)
    detector.destoy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()