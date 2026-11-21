"""The robot's camera as live video for the Android app (WebRTC).

    /camera/image_raw   sensor_msgs/msg/Image   -> video (VP8 or H.264, as the phone prefers)
    http://<this PC>:8080/offer                 <- the phone's offer; the answer goes back

The node subscribes to the camera only while a phone watches, so no viewer means no work: no
copying of pictures, no encoding. Each picture is sent once, when it arrives.

Two threads: the ROS executor thread receives pictures and turns them into video frames; the
asyncio event loop in the main thread runs the web server and WebRTC. Frames cross with
loop.call_soon_threadsafe; subscribing and unsubscribing run on the ROS thread through
executor.create_task.
"""

import asyncio
import signal
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from . import requirements

requirements.check_or_exit()

from . import frames  # noqa: E402
from .server import VideoServer  # noqa: E402


class CameraSide(Node):
    def __init__(self):
        super().__init__("vacuum_video")
        self.declare_parameter("camera_topic", "camera/image_raw")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.deliver = None  # set by main(): hands a frame to the event loop
        self._sub = None
        self._reported = set()

    def param(self, name):
        return self.get_parameter(name).value

    def watch(self, on: bool) -> None:
        """Subscribe while someone watches. Call on the ROS thread."""
        if on and self._sub is None:
            self._sub = self.create_subscription(Image, self.param("camera_topic"), self._on_image, qos_profile_sensor_data)
            self.get_logger().info(f"a phone is watching: subscribed to {self.param('camera_topic')}")
        elif not on and self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
            self.get_logger().info("nobody is watching: unsubscribed from the camera")

    def _on_image(self, msg):
        try:
            frame = frames.to_frame(msg)
        except frames.UnsupportedImage as exc:
            if str(exc) not in self._reported:
                self._reported.add(str(exc))
                self.get_logger().error(f"cannot send the camera's pictures: {exc}")
            return
        if self.deliver is not None:
            self.deliver(frame)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CameraSide()
    log = node.get_logger()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def watched(on):  # called on the event loop
        async def apply():
            node.watch(on)

        executor.create_task(apply)

    hub = frames.FrameHub(on_watched=watched)
    server = VideoServer(hub, log, host=node.param("host"), port=node.param("port"))

    def deliver(frame):  # called on the ROS thread
        try:
            loop.call_soon_threadsafe(hub.push, frame)
        except RuntimeError:
            pass  # the loop has closed: shutting down

    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    exit_code = 0
    try:
        port = loop.run_until_complete(server.start())
    except OSError as exc:
        log.error(f"cannot listen on {node.param('host')}:{node.param('port')}: {exc}")
        exit_code = 1
    else:
        node.deliver = deliver

        def spin():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        threading.Thread(target=spin, daemon=True).start()
        log.info(f"video for the app on http://{node.param('host')}:{port}/offer")
        loop.run_until_complete(stop.wait())
        log.info("stopping")
        loop.run_until_complete(server.stop())
    finally:
        node.deliver = None
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        loop.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
