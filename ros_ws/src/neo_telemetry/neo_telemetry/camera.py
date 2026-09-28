#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np



class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_AVAILABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.compressed_feed_publisher = self.create_publisher(CompressedImage, 'neo_front_cam', qos_profile)
        self.front_warped_publisher = self.create_publisher(CompressedImage, 'neo_front_warped', qos_profile)
        timer_period = 0.033
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        self.capWidth = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.capHeight = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.pts_src = np.array([[140, 200], [500, 200], [40, 350], [600, 350]])

        self.pts_dst = np.array([[255, 317], [390, 315], [255, 425], [390, 425]])

        self.h, self.status = cv2.findHomography(self.pts_src, self.pts_dst)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            
            im_out = cv2.warpPerspective(frame, self.h, [self.capWidth, self.capHeight])
            
            low_res_image = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_LINEAR)
            self.compressed_feed_publisher.publish(self.br.cv2_to_compressed_imgmsg(low_res_image, 'jpg'))
            self.front_warped_publisher.publish(self.br.cv2_to_compressed_imgmsg(im_out, 'jpg'))
            

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.cap.release()
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()