import argparse
import asyncio
import json
import logging
import os
import platform
import ssl

import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge


from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

ROOT = os.path.dirname(__file__)

class NewCamerasWebRTC(Node):
    def __init__(self):

        super().__init__('webrtc_cameras_cs')

        # publishers for the cameras
        self.cam_pub = self.create_publisher(CompressedImage, 'camera_webrtc', 1) #

        self.bridge = CvBridge() #
        self.relay = None
        self.ros_relay = None
        self.webcam = None
        self.pcs = set()

        if True:
            logging.basicConfig(level=logging.DEBUG)
        else:
            logging.basicConfig(level=logging.INFO)

        if False:
            ssl_context = ssl.SSLContext()
            ssl_context.load_cert_chain(args.cert_file, args.key_file)
        else:
            ssl_context = None

        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)
        app.router.add_post("/offer", self.offer)
        web.run_app(app, host="0.0.0.0", port=8080, ssl_context=ssl_context)


    def create_local_tracks(self, play_from, decode):
        global relay, webcam

        if play_from:
            player = MediaPlayer(play_from, decode=decode)
            return player.audio, player.video
        else:
            options = {"framerate": "30", "video_size": "640x480"}
            if relay is None:
                if platform.system() == "Darwin":
                    webcam = MediaPlayer(
                        "default:none", format="avfoundation", options=options
                    )
                elif platform.system() == "Windows":
                    webcam = MediaPlayer(
                        "video=Integrated Camera", format="dshow", options=options
                    )
                else:
                    webcam = MediaPlayer("/dev/video0", format="v4l2", options=options)
                relay = MediaRelay()
            return None, relay.subscribe(webcam.video)


    def force_codec(self, pc, sender, forced_codec):
        kind = forced_codec.split("/")[0]
        codecs = RTCRtpSender.getCapabilities(kind).codecs
        transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
        transceiver.setCodecPreferences(
            [codec for codec in codecs if codec.mimeType == forced_codec]
        )


    async def index(self, request):
        content = open(os.path.join(ROOT, "index.html"), "r").read()
        return web.Response(content_type="text/html", text=content)


    async def javascript(self, request):
        content = open(os.path.join(ROOT, "client.js"), "r").read()
        return web.Response(content_type="application/javascript", text=content)


    async def offer(self, request):
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self.pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print("Connection state is %s" % pc.connectionState)
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)

        # open media source
        audio, video = self.create_local_tracks(
            #args.play_from, decode=not args.play_without_decoding
            None, False
        )

        if audio:
            audio_sender = pc.addTrack(audio)
            # if args.audio_codec:
            #     self.force_codec(pc, audio_sender, args.audio_codec)
            # elif args.play_without_decoding:
            #     raise Exception("You must specify the audio codec using --audio-codec")

        if video:
            video_sender = pc.addTrack(video)
            # if args.video_codec:
            #     self.force_codec(pc, video_sender, args.video_codec)
            # elif args.play_without_decoding:
            #     raise Exception("You must specify the video codec using --video-codec")

        await pc.setRemoteDescription(offer)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )


    async def on_shutdown(self, app):
        # close peer connections
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()



    # parser = argparse.ArgumentParser(description="WebRTC webcam demo")
    # parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    # parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    # parser.add_argument("--play-from", help="Read the media from a file and sent it.")
    # parser.add_argument(
    #     "--play-without-decoding",
    #     help=(
    #         "Read the media without decoding it (experimental). "
    #         "For now it only works with an MPEGTS container with only H.264 video."
    #     ),
    #     action="store_true",
    # )
    # parser.add_argument(
    #     "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    # )
    # parser.add_argument(
    #     "--port", type=int, default=8080, help="Port for HTTP server (default: 8080)"
    # )
    # parser.add_argument("--verbose", "-v", action="count")
    # parser.add_argument(
    #     "--audio-codec", help="Force a specific audio codec (e.g. audio/opus)"
    # )
    # parser.add_argument(
    #     "--video-codec", help="Force a specific video codec (e.g. video/H264)"
    # )

    # args = parser.parse_args()
