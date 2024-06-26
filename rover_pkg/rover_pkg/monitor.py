import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from jtop import jtop
import threading


class Monitor(Node):

    def __init__(self):
        super().__init__('performance_monitor')
        self.perf_pub = self.create_publisher(String, '/ROVER/performance', 10)
        self.initializeJtop(self.perf_pub)

    def timer_callback(self, jetson):
        msg = String()
        msg.data = jetson.json()
        self.perf_pub.publish(msg)

    def initializeJtop(self, publisher):
        jetson = jtop()
        jetson.attach(self.timer_callback)
        threading.Thread(target=jetson.loop_for_ever).start()

def main(args=None):

    print("Start performance monitor node...")

    rclpy.init(args=args)
    monitor = Monitor()
    print("Jtop ready")

    rclpy.spin(monitor)

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
