"""Camera pictures (sensor_msgs/Image) to video frames, and the frames to the phones' tracks.
No ROS imports: anything with the Image fields will do, so the tests use plain objects.

Each picture is sent once, when it arrives: the video runs at the camera's own rate (15 a second
in the simulator), and a camera that stops sends nothing rather than the same picture again.
"""

import asyncio
import fractions
import time
from typing import Callable, Optional, Set

import av
import numpy as np
from aiortc import MediaStreamTrack

VIDEO_CLOCK = 90000  # RTP video timestamps count at 90 kHz
TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK)

# sensor_msgs/Image encodings this node can send, and their pixel layout for av.
FORMATS = {"rgb8": ("rgb24", 3), "bgr8": ("bgr24", 3), "rgba8": ("rgba", 4), "bgra8": ("bgra", 4), "mono8": ("gray", 1)}


class UnsupportedImage(ValueError):
    pass


def to_frame(msg) -> av.VideoFrame:
    """A sensor_msgs/Image as a video frame. Raises UnsupportedImage for other encodings."""
    if msg.encoding not in FORMATS:
        raise UnsupportedImage(f"encoding '{msg.encoding}' is not supported (use one of {', '.join(FORMATS)})")
    fmt, channels = FORMATS[msg.encoding]
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0 or step < width * channels or len(msg.data) < step * height:
        raise UnsupportedImage(f"a {width}x{height} picture with {len(msg.data)} bytes and a row step of {step}")
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=step * height).reshape(height, step)
    pixels = rows[:, : width * channels]
    pixels = pixels.reshape(height, width) if channels == 1 else pixels.reshape(height, width, channels)
    return av.VideoFrame.from_ndarray(np.ascontiguousarray(pixels), format=fmt)


class CameraTrack(MediaStreamTrack):
    """One phone's video. recv() waits for the next picture; if the phone's connection falls
    behind, older pictures are dropped and only the newest is sent."""

    kind = "video"

    def __init__(self):
        super().__init__()
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=1)

    def offer(self, frame: av.VideoFrame) -> None:
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        self._queue.put_nowait(frame)

    async def recv(self) -> av.VideoFrame:
        return await self._queue.get()


class FrameHub:
    """Hands each new picture to every track. Tells the node when the first viewer arrives and
    when the last one leaves, so it subscribes to the camera only while someone watches.
    Runs on the event loop's thread."""

    def __init__(self, on_watched: Callable[[bool], None], clock: Callable[[], float] = time.monotonic):
        self._tracks: Set[CameraTrack] = set()
        self._on_watched = on_watched
        self._clock = clock
        self._start: Optional[float] = None
        self.frames_in = 0

    @property
    def viewers(self) -> int:
        return len(self._tracks)

    def add(self, track: CameraTrack) -> None:
        self._tracks.add(track)
        if len(self._tracks) == 1:
            self._on_watched(True)

    def remove(self, track: CameraTrack) -> None:
        if track in self._tracks:
            self._tracks.discard(track)
            if not self._tracks:
                self._on_watched(False)

    def push(self, frame: av.VideoFrame) -> None:
        """A new picture: stamp it with when it arrived, on the video clock, and pass it on."""
        now = self._clock()
        if self._start is None:
            self._start = now
        frame.pts = int((now - self._start) * VIDEO_CLOCK)
        frame.time_base = TIME_BASE
        self.frames_in += 1
        for track in list(self._tracks):
            track.offer(frame)
