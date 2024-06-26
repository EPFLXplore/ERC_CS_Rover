import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int8MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import cv2
from cv_bridge import CvBridge
import threading

from time import sleep

class NewCameras(Node):
    def __init__(self):

        super().__init__('new_cameras_cs')

        self.camera_ids = ["/dev/video0"]#, "/dev/video2"]
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # publishers for the cameras
        self.cam_pubs = [self.create_publisher(CompressedImage, 'camera_' + str(i), qos_profile) for i in range(len(self.camera_ids))]
        self.bridge = CvBridge()

        # self.publish_feeds()
        for i in range(len(self.camera_ids)):
            threading.Thread(target=publish_feeds, args=(self.camera_ids[i], self.cam_pubs[i], self.bridge,)).start()



def publish_feeds(camera_id, publisher, bridge):
    print("Starting camera " + camera_id)
    camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
    camera.set(cv2.CAP_PROP_FPS, 15)
    
    while True:
        print("Open " + camera_id)
        while True:
            ret, frame = camera.read()
            print("Capturing " + camera_id)
            if not ret:
                break
            publisher.publish(bridge.cv2_to_compressed_imgmsg(frame))
            sleep(1/15)
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
        camera.set(cv2.CAP_PROP_FPS, 15)
        sleep(1)





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

