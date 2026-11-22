"""The object finder node: looks for things named in plain words in what the camera sees now.

    /perception/find   vacuum_interfaces/srv/FindObjects   names in, what was found (and where on the map)

It listens to the camera only while it answers: the colour picture (camera/image_raw), the depth
picture (camera/depth/image_raw) and camera/camera_info, and takes the transform from the camera to
the map at the moment the picture was taken. The detector (finder.py) is loaded on the first request.
"""

import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from vacuum_interfaces.msg import Detection
from vacuum_interfaces.srv import FindObjects

from . import finder, geometry

FRAME_WAIT = 3.0  # s for a fresh colour and depth picture
PAIR_GAP = 0.15  # s: the depth picture must be this close in time to the colour one


def image_array(msg: Image) -> Optional[np.ndarray]:
    """sensor_msgs/Image as a numpy array: RGB (H, W, 3) uint8, or depth (H, W) float32 metres."""
    if msg.encoding in ("rgb8", "bgr8"):
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a[:, :, ::-1].copy() if msg.encoding == "bgr8" else a.copy()
    if msg.encoding == "rgba8":
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 4]
        return a.reshape(msg.height, msg.width, 4)[:, :, :3].copy()
    if msg.encoding == "32FC1":
        return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width].copy()
    if msg.encoding == "16UC1":
        return np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width] / 1000.0
    return None


def marked_jpeg(rgb: np.ndarray, found, quality: int = 80) -> bytes:
    """The picture with each find boxed and named."""
    bgr = np.ascontiguousarray(rgb[:, :, ::-1])
    for f in found:
        x0, y0, x1, y1 = (int(round(v)) for v in f.box)
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 200, 255), 2)
        cv2.putText(bgr, f"{f.label} {f.score:.2f}", (x0 + 3, max(y0 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
    ok, data = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return data.tobytes() if ok else b""


class FinderNode(Node):
    def __init__(self):
        super().__init__("object_finder")
        self.declare_parameter("model", finder.MODEL)
        self.declare_parameter("device", "auto")
        self.declare_parameter("load_at_start", False)
        self._finder = finder.Finder(self.get_parameter("model").value, self.get_parameter("device").value)
        self._lock = threading.Lock()  # one look at a time
        self._frames = {}
        self._group = ReentrantCallbackGroup()
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self.tf, self)
        self.create_service(FindObjects, "perception/find", self._on_find, callback_group=ReentrantCallbackGroup())
        if self.get_parameter("load_at_start").value:
            threading.Thread(target=self._load, daemon=True).start()

    def _load(self):
        started = time.monotonic()
        self._finder.load()
        self.get_logger().info(f"object finder {self._finder.model_name} loaded on {self._finder.device} in {time.monotonic() - started:.1f} s")

    def _grab(self) -> Tuple[Optional[Image], Optional[Image], Optional[CameraInfo]]:
        """A colour picture taken after now, the depth picture taken with it, and the camera's numbers."""
        got = {"rgb": [], "depth": [], "info": []}
        since = self.get_clock().now()
        subs = [
            self.create_subscription(Image, "camera/image_raw", got["rgb"].append, qos_profile_sensor_data, callback_group=self._group),
            self.create_subscription(Image, "camera/depth/image_raw", got["depth"].append, qos_profile_sensor_data, callback_group=self._group),
            self.create_subscription(CameraInfo, "camera/camera_info", got["info"].append, qos_profile_sensor_data, callback_group=self._group),
        ]
        try:
            end = time.monotonic() + FRAME_WAIT
            while time.monotonic() < end:
                time.sleep(0.03)
                fresh = [m for m in list(got["rgb"]) if Time.from_msg(m.header.stamp) >= since]
                if not fresh or not got["info"]:
                    continue
                rgb = fresh[-1]
                t = Time.from_msg(rgb.header.stamp).nanoseconds * 1e-9
                pairs = [d for d in list(got["depth"]) if abs(Time.from_msg(d.header.stamp).nanoseconds * 1e-9 - t) <= PAIR_GAP]
                if pairs:
                    return rgb, min(pairs, key=lambda d: abs(Time.from_msg(d.header.stamp).nanoseconds * 1e-9 - t)), got["info"][-1]
            return (got["rgb"][-1] if got["rgb"] else None), None, (got["info"][-1] if got["info"] else None)
        finally:
            for s in subs:
                self.destroy_subscription(s)

    def _transform(self, target: str, source: str, stamp) -> Optional[np.ndarray]:
        try:
            t = self.tf.lookup_transform(target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.5))
        except TransformException:
            try:
                t = self.tf.lookup_transform(target, source, Time())
            except TransformException:
                return None
        tr, q = t.transform.translation, t.transform.rotation
        return geometry.matrix_from((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))

    def _on_find(self, request, response):
        names = [n.strip() for n in request.names if n.strip()]
        if not names:
            response.success, response.message = False, "nothing to look for"
            return response
        with self._lock:
            try:
                return self._find(names, float(request.min_score), response)
            except Exception as exc:
                self.get_logger().error(f"looking for {names} failed: {exc!r}")
                response.success, response.message = False, f"the object finder failed: {exc}"
                return response

    def _find(self, names, min_score, response):
        if not self._finder.loaded:
            self._load()
        rgb_msg, depth_msg, info = self._grab()
        if rgb_msg is None:
            response.success, response.message = False, "no picture from the camera"
            return response
        rgb = image_array(rgb_msg)
        if rgb is None:
            response.success, response.message = False, f"the camera sends {rgb_msg.encoding} pictures, which the finder cannot read"
            return response
        started = time.monotonic()
        found = self._finder.find(rgb, names, min_score)
        took = time.monotonic() - started
        depth = image_array(depth_msg) if depth_msg is not None else None
        to_map = self._transform("map", rgb_msg.header.frame_id, rgb_msg.header.stamp)
        robot = self._transform("map", "base_link", rgb_msg.header.stamp)
        robot_pose = None
        if robot is not None:
            robot_pose = (float(robot[0, 3]), float(robot[1, 3]), float(np.arctan2(robot[1, 0], robot[0, 0])))
        for f in found:
            d = Detection(label=f.label, score=f.score, box=[float(v) for v in f.box])
            if depth is not None and info is not None and to_map is not None:
                k = info.k
                point = geometry.near_point(f.box, depth, k[0], k[4], k[2], k[5])
                if point is not None:
                    x, y, _ = geometry.transform(to_map, point)
                    d.located, d.x, d.y = True, x, y
                    if robot_pose is not None:
                        d.distance, d.bearing = geometry.distance_and_bearing(robot_pose, x, y)
            response.detections.append(d)
        response.picture_jpeg = list(marked_jpeg(rgb, found))
        response.success = True
        seen = ", ".join(f"{d.label} {d.score:.2f}" + (f" at ({d.x:.2f}, {d.y:.2f}) {d.distance:.1f} m" if d.located else "")
                         for d in response.detections)
        response.message = f"looked for {', '.join(names)}: " + (seen or "none seen")
        self.get_logger().info(f"{response.message} ({took:.2f} s on {self._finder.device})")
        return response


def main():
    rclpy.init()
    node = FinderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
