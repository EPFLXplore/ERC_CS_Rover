import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int8MultiArray

import cv2
from cv_bridge import CvBridge
import threading

from time import sleep

class NewCameras(Node):
    def __init__(self):

        super().__init__('new_cameras_cs')

        # publishers for the cameras
        self.cam_pub = self.create_publisher(CompressedImage, 'camera_new', 1)

        self.camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L) 

        self.bridge = CvBridge()

        self.publish_feeds()



    def publish_feeds(self):
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
        while True:
            ret, frame = self.camera.read()
            if not ret:
                break
            self.cam_pub.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
            sleep(1/15)





def main(args=None):

    print("Start cameras_publisher node...")
    
    rclpy.init(args=args)

    cameras_publisher = NewCameras()
    
    print("Cameras ready")

    rclpy.spin(cameras_publisher)

    # cameras_publisher.stop_camera()
    cameras_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

