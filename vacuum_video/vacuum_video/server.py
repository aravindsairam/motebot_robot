"""The offer endpoint: a phone posts its WebRTC offer to http://<PC>:8080/offer and gets the
answer back, then receives the camera's video. No ROS.

Video only: no data channels, no audio. No outside servers are contacted (no STUN or TURN), so it
works on a home network without internet: the phone and the PC reach each other directly. The
answer carries all of the PC's candidates at once (no trickle), so the phone must also wait for
its own candidates before posting its offer.
"""

import asyncio
import json
from typing import Dict

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription

from .frames import CameraTrack, FrameHub

MAX_VIEWERS = 4  # each viewer is a separate encoding: a CPU core's worth at most
CONNECT_TIMEOUT = 20.0  # s for a phone to complete the connection after its offer


class VideoServer:
    def __init__(self, hub: FrameHub, logger, host: str = "0.0.0.0", port: int = 8080, max_viewers: int = MAX_VIEWERS):
        self._hub = hub
        self._log = logger
        self._host, self._port = host, port
        self._max_viewers = max_viewers
        self._peers: Dict[RTCPeerConnection, CameraTrack] = {}
        self._runner = None
        self._numbers = 0

    @property
    def viewer_count(self) -> int:
        return len(self._peers)

    async def start(self) -> int:
        """Start listening; returns the port (useful when 0 was asked for)."""
        app = web.Application(client_max_size=64 * 1024)
        app.router.add_post("/offer", self._offer)
        app.router.add_get("/", self._hello)
        self._runner = web.AppRunner(app, access_log=None)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self._host, self._port)
        await site.start()
        return site._server.sockets[0].getsockname()[1]

    async def stop(self) -> None:
        for pc in list(self._peers):
            await self._close(pc, "the video server is stopping")
        if self._runner is not None:
            await self._runner.cleanup()

    async def _hello(self, request):
        return web.Response(text="This is the vacuum robot's video. The app posts its WebRTC offer to /offer.\n")

    async def _offer(self, request):
        try:
            params = await request.json()
            offer = RTCSessionDescription(sdp=str(params["sdp"]), type=str(params["type"]))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return web.json_response({"error": "expected JSON with 'sdp' and 'type'"}, status=400)
        if offer.type != "offer":
            return web.json_response({"error": f"expected an offer, got '{offer.type}'"}, status=400)
        if len(self._peers) >= self._max_viewers:
            return web.json_response({"error": f"{self._max_viewers} phones are already watching"}, status=503)

        self._numbers += 1
        name = f"viewer {self._numbers} ({request.remote})"
        pc = RTCPeerConnection()  # no ICE servers: the phone and the PC talk directly
        track = CameraTrack()
        self._peers[pc] = track

        @pc.on("connectionstatechange")
        async def on_state():
            self._log.info(f"{name}: {pc.connectionState}")
            if pc.connectionState == "connected":
                self._hub.add(track)
            elif pc.connectionState in ("failed", "closed"):
                await self._close(pc, pc.connectionState)

        try:
            await pc.setRemoteDescription(offer)
            if not any(t.kind == "video" for t in pc.getTransceivers()):
                await self._close(pc, "the offer asks for no video")
                return web.json_response({"error": "the offer must ask to receive video"}, status=400)
            pc.addTrack(track)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)  # gathers every candidate before returning
        except Exception as exc:
            await self._close(pc, f"could not answer: {exc!r}")
            return web.json_response({"error": f"could not answer the offer: {exc}"}, status=400)

        asyncio.ensure_future(self._expire_if_not_connected(pc, name))
        self._log.info(f"{name}: answered; {len(self._peers)} watching or connecting")
        return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

    async def _expire_if_not_connected(self, pc, name):
        await asyncio.sleep(CONNECT_TIMEOUT)
        if pc in self._peers and pc.connectionState != "connected":
            await self._close(pc, f"{name} did not connect within {CONNECT_TIMEOUT:.0f} s")

    async def _close(self, pc, why: str) -> None:
        track = self._peers.pop(pc, None)
        if track is None:
            return
        self._hub.remove(track)
        track.stop()
        await pc.close()
        self._log.info(f"a viewer left ({why}); {len(self._peers)} watching")
