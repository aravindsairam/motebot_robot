"""Pictures to frames, the hub, and the offer endpoint with a real WebRTC client on this computer.
No ROS."""

import asyncio
import json
import urllib.request
from types import SimpleNamespace

import numpy as np
import pytest
from aiortc import RTCPeerConnection, RTCSessionDescription

from vacuum_video import frames
from vacuum_video.server import VideoServer


class Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)

    warning = error = info


def image(width=4, height=2, encoding="rgb8", step=None, fill=None):
    channels = frames.FORMATS[encoding][1]
    step = step if step is not None else width * channels
    data = bytearray(step * height)
    if fill is not None:
        for row in range(height):
            for col in range(width):
                data[row * step + col * channels: row * step + col * channels + channels] = bytes(fill(row, col))
    return SimpleNamespace(width=width, height=height, step=step, encoding=encoding, data=bytes(data))


def test_an_rgb_picture_keeps_its_pixels():
    msg = image(fill=lambda r, c: (r * 100, c * 50, 7))
    pixels = frames.to_frame(msg).to_ndarray(format="rgb24")
    assert pixels.shape == (2, 4, 3)
    assert pixels[1, 3].tolist() == [100, 150, 7]


def test_padding_at_the_end_of_rows_is_skipped():
    msg = image(width=3, height=2, step=12, fill=lambda r, c: (r, c, 9))
    pixels = frames.to_frame(msg).to_ndarray(format="rgb24")
    assert pixels.shape == (2, 3, 3) and pixels[1, 2].tolist() == [1, 2, 9]


def test_other_layouts():
    assert frames.to_frame(image(encoding="bgr8", fill=lambda r, c: (1, 2, 3))).to_ndarray(format="rgb24")[0, 0].tolist() == [3, 2, 1]
    assert frames.to_frame(image(encoding="mono8")).width == 4


@pytest.mark.parametrize("msg", [
    SimpleNamespace(width=4, height=2, step=16, encoding="32FC1", data=bytes(32)),
    SimpleNamespace(width=4, height=2, step=12, encoding="rgb8", data=bytes(10)),
    SimpleNamespace(width=4, height=2, step=6, encoding="rgb8", data=bytes(24)),
    SimpleNamespace(width=0, height=2, step=0, encoding="rgb8", data=b""),
])
def test_pictures_that_cannot_be_sent_are_reported(msg):
    with pytest.raises(frames.UnsupportedImage):
        frames.to_frame(msg)


def test_the_hub_counts_viewers_and_stamps_frames_with_their_arrival():
    events, now = [], [10.0]
    hub = frames.FrameHub(on_watched=events.append, clock=lambda: now[0])

    async def body():
        a, b = frames.CameraTrack(), frames.CameraTrack()
        hub.add(a)
        hub.add(b)
        hub.remove(a)
        hub.remove(a)
        assert events == [True]
        hub.push(frames.to_frame(image()))
        now[0] = 10.5
        hub.push(frames.to_frame(image()))  # b has not taken the first: only the newest is kept
        got = await asyncio.wait_for(b.recv(), 1.0)
        assert got.pts == 45000 and got.time_base == frames.TIME_BASE
        hub.remove(b)
        assert events == [True, False]

    asyncio.run(body())


def post(url, body):
    request = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_a_phone_receives_the_pictures_and_nobody_watching_means_no_work():
    watched = []

    async def body():
        hub = frames.FrameHub(on_watched=watched.append)
        server = VideoServer(hub, Log(), host="127.0.0.1", port=0)
        port = await server.start()
        url = f"http://127.0.0.1:{port}/offer"
        loop = asyncio.get_running_loop()
        try:
            # Not offers: refused with a reason
            status, answer = await loop.run_in_executor(None, post, url, {"sdp": "x"})
            assert status == 400 and "sdp" in answer["error"]

            phone = RTCPeerConnection()
            phone.addTransceiver("video", direction="recvonly")
            received = []
            arrived = asyncio.Event()

            @phone.on("track")
            def on_track(track):
                async def read():
                    while True:
                        received.append(await track.recv())
                        if len(received) >= 5:
                            arrived.set()

                asyncio.ensure_future(read())

            await phone.setLocalDescription(await phone.createOffer())
            status, answer = await loop.run_in_executor(
                None, post, url, {"sdp": phone.localDescription.sdp, "type": phone.localDescription.type})
            assert status == 200 and answer["type"] == "answer"
            await phone.setRemoteDescription(RTCSessionDescription(**answer))

            for _ in range(100):  # until connected
                if hub.viewers:
                    break
                await asyncio.sleep(0.05)
            assert hub.viewers == 1 and watched == [True]
            picture = image(width=64, height=48, fill=lambda r, c: (200, r * 4, c * 3))
            for _ in range(60):
                hub.push(frames.to_frame(picture))
                await asyncio.sleep(1 / 15)
                if arrived.is_set():
                    break
            assert len(received) >= 5
            first = received[0].to_ndarray(format="rgb24")
            assert first.shape == (48, 64, 3) and abs(int(first[10, 10, 0]) - 200) < 20

            await phone.close()
            for _ in range(100):
                if not hub.viewers:
                    break
                await asyncio.sleep(0.05)
            assert hub.viewers == 0 and watched == [True, False]
        finally:
            await server.stop()

    asyncio.run(body())
