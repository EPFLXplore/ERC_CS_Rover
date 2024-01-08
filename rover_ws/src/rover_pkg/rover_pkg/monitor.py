import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from jtop import jtop


class Monitor(Node):

    def __init__(self):
        super().__init__('performance_monitor')
        self.create_timer(1.0, self.timer_callback)
        self.perf_pub = self.create_publisher(String, 'CS/performance', 10)

    def timer_callback(self):
        msg = String()
        msg.data = jtop().json()
        self.perf_pub.publish(msg)


def main(args=None):

    print("Start performance monitor node...")

    rclpy.init(args=args)
    monitor = Monitor()
    print("Cameras ready")

    rclpy.spin(monitor)

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()