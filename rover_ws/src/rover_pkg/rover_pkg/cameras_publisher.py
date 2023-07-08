
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
from xplore_interfaces.msg import CameraError
from xplore_interfaces.srv import EnableCamera, DisableCamera

import os
import subprocess

NUMBER_CAMERAS = 6
STARTING_PORT = 8000


class CamerasPublisher(Node):

    def __init__(self):

        super().__init__('cameras_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_0', 10)

        self.camerasIPList = [-1] * NUMBER_CAMERAS  # -1 means camera is disabled, otherwise it is the ip adress of the camera
        self.cameraStreamList = [] * NUMBER_CAMERAS 


        self.CameraDisconnectedPublisher = self.create_publisher(CameraError, 'camera_disconnected', 10)  #change name of cameraError
        self.EnableCameraService = self.create_service(EnableCamera, 'enable_camera', self.enable_camera_callback)
        self.DisableCameraService = self.create_service(DisableCamera, 'disable_camera', self.disable_camera_callback)

        #TO DO: error handling with error publisher

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)      
        #self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def enable_camera_callback(self, request, response):

        if(request.index < 0 or request.index >= NUMBER_CAMERAS):
            #TO DO: Send error message to console
            response.success = False
            return response

        if(self.cameraStreamList[request.index].isOpened()):
            response.ip_adress = self.camerasList[request.index]
            response.success = True
            return response

        #TO DO: Enable camera
        self.start_camera(request.index)
        cap = cv2.VideoCapture(request.index)
    
        if(cap.isOpened()):
            self.cameraStreamList[request.index] = cap
            self.camerasList[request.index] = STARTING_PORT + request.index
            response.ip_adress = self.camerasList[request.index]
            response.success = True
            return response

        response.success = False
        response.ip_adress = -1
        return response

    def disable_camera_callback(self, request, response):

        if(request.index < 0 or request.index >= NUMBER_CAMERAS):
            #TO DO: Send error message to console
            response.success = False
            return response

        if(self.camerasList[request.index] == -1):
            response.success = True
            return response
        
        self.cameraStreamList[request.index].release() #TO DO: Check if error could occur
        self.cameraStreamList[request.index] = None
        self.camerasIPList[request.index] = -1

        response.success = True
        return response

    def timer_callback(self):
        return

        #ret, frame = self.cap.read()
            
        #if ret == True:
        #   self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

        #self.get_logger().info('Publishing video frame')

    def start_camera(self, index):
            self.get_logger().warning('Camera started')
            #gstreamer_str = 'sudo gst-launch-1.0 v4l2src ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=8080'
            #cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)
            #ret, frame = cap.read()
            
            cmd = self.gstreamer_pipeline()
            subprocess.run(cmd, shell=True)
            

    def gstreamer_pipeline(
    sensor_id=5,
    capture_width=1948,
    capture_height=1096,
    display_width=1948,
    display_height=1096,
    framerate=30,
    flip_method=2):
        """return (
            "nvarguscamerasrc sensor-id=%d sensor-mode=1 !"
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv ! xvimagesink -e"
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "x264enc pass=qual quantizer=20 tune=zerolatency ! "
            "rtph264pay ! "
            "udpsink host=127.0.0.1 port=8080"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                display_width,
                display_height,
            )
        )"""
        return 'sudo gst-launch-1.0 v4l2src ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=8080'
        """return (
            "gst-launch-1.0 v4l2src !"  
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
            "videoconvert ! "
            "x264enc pass=qual quantizer=20 tune=zerolatency ! "
            "rtph264pay ! "
            "udpsink host=127.0.0.1 port=8080"
            % (
                capture_width,
                capture_height,
                framerate,
            )
        )"""

  
def main(args=None):
  
  rclpy.init(args=args)
  camerasPublisher = CamerasPublisher()
  camerasPublisher.start_camera(1)
  rclpy.spin(camerasPublisher)

  #image_publisher.destroy_node()
  #rclpy.shutdown()
  
if __name__ == '__main__':
  print("Starting cameras publisher")
  main()