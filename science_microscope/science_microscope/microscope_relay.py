#!/usr/bin/env python3
"""
microscope_relay_node.py

Runs on the Pi 5. Receives the RTP/H264 microscope feed from the Pi Zero
(192.168.55.2) on UDP port 5000, and:

  1. Relays the RTP stream UNMODIFIED to the CS (169.254.55.166) on the
     same port, exactly as received -- no decode, no re-encode, minimal
     added latency.
  2. Decodes a second tap of the same stream, downsizes to 480p, JPEG
     compresses at quality=5, and publishes it as a ROS2
     sensor_msgs/msg/CompressedImage topic that the CS can subscribe to
     as a low-bandwidth fallback.

Requires: python3-gi (PyGObject), gstreamer1.0-plugins-{base,good,bad},
rclpy, sensor_msgs.
"""

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# --- Config -----------------------------------------------------------
LISTEN_PORT = 5000
CS_HOST = "169.254.55.166"
CS_PORT = 5014
ROS_TOPIC = "/microscope/image/compressed"
JPEG_QUALITY = 5          # 0-100, deliberately very low ("hypercompressed")
SCALE_WIDTH = 854
SCALE_HEIGHT = 480
# -----------------------------------------------------------------------

PIPELINE_DESC = f"""
    udpsrc port={LISTEN_PORT}
        caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000"
    ! tee name=t

    t. ! queue leaky=downstream max-size-buffers=200
       ! udpsink host={CS_HOST} port={CS_PORT} sync=false async=false

    t. ! queue leaky=downstream max-size-buffers=5
       ! rtpjitterbuffer latency=0
       ! rtph264depay
       ! h264parse
       ! avdec_h264
       ! videoscale
       ! video/x-raw,width={SCALE_WIDTH},height={SCALE_HEIGHT}
       ! videoconvert
       ! jpegenc quality={JPEG_QUALITY}
       ! appsink name=ros_sink emit-signals=true sync=false max-buffers=1 drop=true
"""


class MicroscopeRelayNode(Node):
    def __init__(self):
        super().__init__("microscope_relay_node")
        self.publisher = self.create_publisher(CompressedImage, ROS_TOPIC, 10)
        self.get_logger().info(
            f"Relaying RTP to {CS_HOST}:{CS_PORT}, "
            f"publishing ROS2 topic {ROS_TOPIC} "
            f"({SCALE_WIDTH}x{SCALE_HEIGHT}, jpeg q={JPEG_QUALITY})"
        )

        Gst.init(None)
        self.pipeline = Gst.parse_launch(PIPELINE_DESC)

        ros_sink = self.pipeline.get_by_name("ros_sink")
        ros_sink.connect("new-sample", self._on_new_sample)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self.pipeline.set_state(Gst.State.PLAYING)

    def _on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "microscope"
            msg.format = "jpeg"
            msg.data = bytes(mapinfo.data)
            self.publisher.publish(msg)
        finally:
            buf.unmap(mapinfo)

        return Gst.FlowReturn.OK

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"GStreamer error: {err} ({debug})")
        elif t == Gst.MessageType.EOS:
            self.get_logger().warn("GStreamer EOS received")

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main():
    rclpy.init()
    node = MicroscopeRelayNode()

    # Pump both the GLib mainloop (for GStreamer bus messages) and rclpy
    # by spinning rclpy and letting GStreamer run on its own bus watch,
    # which uses the default GLib context rclpy does not own -- so we
    # run a GLib loop in this thread via rclpy timer callbacks instead.
    loop = GLib.MainLoop()

    def spin_ros(loop_ref=loop):
        rclpy.spin_once(node, timeout_sec=0)
        return True  # keep the GLib timeout alive

    GLib.timeout_add(1, spin_ros)  # ~1ms tick to keep rclpy responsive

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
