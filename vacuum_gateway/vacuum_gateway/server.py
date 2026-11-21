"""The WebSocket side of the gateway: phones connect, receive the robot's streams, and send
commands. No ROS: the node in gateway_node.py hands it each new message through publish(), and
gives it the commands it can carry out.

Everything here runs on one asyncio event loop, and publish() must be called on that loop's
thread (the ROS side uses loop.call_soon_threadsafe).

What a phone receives, in order: Hello; the latest copy of each stream, if recent; then every
copy the streams let through, a Heartbeat once a second, and a Reply to each of its commands.
Each phone has its own outbox in which a stream keeps only its newest copy, so a phone on a slow
link gets fewer copies instead of an ever-growing backlog, and cannot hold up the others.

Every command gets exactly one Reply, and only the phone that sent it receives it. Commands run
side by side, each with its own time limit, so a slow one (a mode change) cannot hold up Stop.
"""

import asyncio
import collections
import itertools
import logging
import time
from dataclasses import dataclass
from http import HTTPStatus
from typing import Any, Awaitable, Callable, Dict, List, Set, Tuple

from google.protobuf.message import DecodeError
from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request

from . import convert, framing
from . import vacuum_pb2 as pb
from .streams import HOLD, SEND, Stream

MAX_INCOMING_BYTES = 64 * 1024  # commands are small; the WebSocket layer refuses anything bigger
MAX_WAITING_IN_ORDER = 256  # messages that must all arrive, in order (Hello, replies)
MAX_COMMANDS_PER_PHONE = 8  # commands from one phone waiting for an answer at the same time

Result = pb.Reply.Result


@dataclass(frozen=True)
class Command:
    """A command the gateway can carry out.

    `run` takes the decoded message and returns (Reply result, message), or (result, message,
    {other Reply fields}) such as the goal id of an accepted goal. The gateway answers
    RESULT_TIMED_OUT if it takes longer than `timeout` seconds, which must be longer than the
    robot's own worst case for it. With `with_phone`, `run` also gets the number of the phone
    that sent it (driving needs to know who drives).
    """

    name: str
    message_type: Any  # the protobuf class of the command
    timeout: float
    run: Callable[..., Awaitable[tuple]]
    with_phone: bool = False


async def _pong(_message) -> Tuple[int, str]:
    return Result.RESULT_OK, ""


PING = Command("ping", pb.Ping, 5.0, _pong)


class Outbox:
    """What is waiting to be sent to one phone."""

    def __init__(self, max_in_order: int = MAX_WAITING_IN_ORDER):
        self._in_order: collections.deque = collections.deque()
        self._latest: Dict[int, bytes] = {}  # stream type -> newest copy not yet sent
        self._max_in_order = max_in_order
        self._ready = asyncio.Event()

    def put_in_order(self, frame: bytes) -> bool:
        """Queue a message that must arrive. False if the phone has stopped reading."""
        if len(self._in_order) >= self._max_in_order:
            return False
        self._in_order.append(frame)
        self._ready.set()
        return True

    def put_latest(self, key: int, frame: bytes) -> None:
        """Queue a stream's newest copy, replacing an older copy that has not been sent yet."""
        self._latest.pop(key, None)
        self._latest[key] = frame
        self._ready.set()

    async def take(self) -> List[bytes]:
        """Wait for something to send, then take all of it: the in-order messages first."""
        await self._ready.wait()
        self._ready.clear()
        frames = list(self._in_order)
        self._in_order.clear()
        frames.extend(self._latest.values())
        self._latest.clear()
        return frames


class _ForwardLog(logging.Handler):
    """Passes the WebSocket library's warnings and errors to our logger, one line each, without
    the traceback it would otherwise print on the terminal."""

    def __init__(self, logger):
        super().__init__(level=logging.WARNING)
        self._log = logger

    def emit(self, record):
        cause = f": {record.exc_info[1]!r}" if record.exc_info else ""
        self._log.warning(f"websocket: {record.getMessage()}{cause}")


def _not_websocket(connection: ServerConnection, request: Request):
    """Answer an ordinary web request (a browser, or the app's port check) with a short note
    instead of a failed handshake. WebSocket requests go on as usual."""
    if request.headers.get("Upgrade", "").lower() != "websocket":
        return connection.respond(HTTPStatus.UPGRADE_REQUIRED, "This is the vacuum robot's gateway. Connect with the app.\n")
    return None


class _Phone:
    def __init__(self, number: int, connection: ServerConnection):
        self.number = number
        self.connection = connection
        self.outbox = Outbox()
        self.heartbeats = 0
        self.commands_waiting = 0
        self.reported: Set = set()  # problems with this phone's messages that were logged already
        address = connection.remote_address
        self.name = f"phone {number} ({address[0]}:{address[1]})" if address else f"phone {number}"


class GatewayServer:
    def __init__(
        self,
        *,
        robot_name: str,
        gateway_version: str,
        streams: Dict[int, Stream],
        logger,
        commands: Dict[int, Command] = None,
        on_phone_left: Callable[[int], None] = None,
        host: str = "0.0.0.0",
        port: int = 9090,
        heartbeat_interval: float = 1.0,
        max_payload: int = 4 * 1024 * 1024,
        ping_interval: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._streams = streams
        self._log = logger
        self._commands = {pb.COMMAND_PING: PING, **(commands or {})}
        self._on_phone_left = on_phone_left  # called with the phone's number when it disconnects
        self._running: Set[asyncio.Future] = set()  # commands being carried out
        self._host, self._port = host, port
        self._heartbeat_interval = heartbeat_interval
        self._max_payload = max_payload
        self._ping_interval = ping_interval
        self._clock = clock
        hello = convert.hello(robot_name, gateway_version).SerializeToString()
        self._hello = framing.encode(pb.CHANNEL_DATA, pb.DATA_HELLO, hello, max_payload)
        self._phones: Set[_Phone] = set()
        self._numbers = itertools.count(1)
        self._server = None
        self._heartbeat_task = None
        self._refused_types: Set[int] = set()  # streams with a message too big to send, logged once
        self._held: Dict[int, asyncio.TimerHandle] = {}  # streams with a copy waiting for its time
        self._ws_log = logging.getLogger(f"vacuum_gateway.websockets.{id(self)}")
        self._ws_log.propagate = False
        self._ws_log.setLevel(logging.WARNING)
        self._ws_log.addHandler(_ForwardLog(logger))

    @property
    def phone_count(self) -> int:
        return len(self._phones)

    async def start(self) -> int:
        """Start listening. Returns the port (useful when 0 was asked for). Raises OSError if the
        port cannot be opened, for example because another gateway is using it."""
        self._server = await serve(
            self._serve_phone,
            self._host,
            self._port,
            compression=None,  # small messages; the map is compressed by its own message (step 3)
            max_size=MAX_INCOMING_BYTES,
            # WebSocket pings notice a phone that vanished without closing (Wi-Fi gone) within
            # about two intervals, and free its place. Driving does not depend on this: the
            # safety gate stops the robot as soon as commands stop.
            ping_interval=self._ping_interval,
            ping_timeout=self._ping_interval,
            process_request=_not_websocket,
            logger=self._ws_log,
        )
        self._heartbeat_task = asyncio.ensure_future(self._heartbeats())
        return self._server.sockets[0].getsockname()[1]

    async def stop(self) -> None:
        if self._heartbeat_task is not None:
            self._heartbeat_task.cancel()
        for task in list(self._running):
            task.cancel()
        for timer in self._held.values():
            timer.cancel()
        self._held.clear()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()

    def publish(self, msg_type: int, payload: bytes) -> None:
        """A new copy of a stream from the robot. Call on the event loop's thread."""
        stream = self._streams[msg_type]
        try:
            frame = framing.encode(pb.CHANNEL_DATA, msg_type, payload, self._max_payload)
        except framing.FrameError as exc:
            if msg_type not in self._refused_types:
                self._refused_types.add(msg_type)
                self._log.error(f"not sending {stream.name} to the phones: {exc}")
            return
        now = self._clock()
        decision = stream.offer(frame, now)
        if decision == SEND:
            self._fan_out(msg_type, frame)
        elif decision == HOLD and msg_type not in self._held:
            loop = asyncio.get_running_loop()
            self._held[msg_type] = loop.call_later(stream.hold_delay(now), self._release, msg_type)

    def _release(self, msg_type: int) -> None:
        """A held copy's time has come: send the newest copy of that stream."""
        self._held.pop(msg_type, None)
        frame = self._streams[msg_type].release(self._clock())
        if frame is not None:
            self._fan_out(msg_type, frame)

    def _fan_out(self, msg_type: int, frame: bytes) -> None:
        for phone in self._phones:
            phone.outbox.put_latest(msg_type, frame)

    # ------------------------------------------------------------------ one phone
    async def _serve_phone(self, connection: ServerConnection) -> None:
        phone = _Phone(next(self._numbers), connection)
        phone.outbox.put_in_order(self._hello)
        now = self._clock()
        for msg_type, stream in self._streams.items():
            frame = stream.replay(now)
            if frame is not None:
                phone.outbox.put_latest(msg_type, frame)
        self._phones.add(phone)
        self._log.info(f"{phone.name} connected; {len(self._phones)} connected")
        writer = asyncio.ensure_future(self._write(phone))
        try:
            async for message in connection:
                self._receive(phone, message)
        except ConnectionClosed:
            pass
        finally:
            self._phones.discard(phone)
            writer.cancel()
            self._log.info(f"{phone.name} disconnected; {len(self._phones)} connected")
            if self._on_phone_left is not None:
                try:
                    self._on_phone_left(phone.number)
                except Exception as exc:  # must not stop the server tidying up
                    self._log.error(f"after {phone.name} left: {exc!r}")

    async def _write(self, phone: _Phone) -> None:
        try:
            while True:
                for frame in await phone.outbox.take():
                    await phone.connection.send(frame)
        except ConnectionClosed:
            pass  # the reading side notices too, and tidies up
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # a fault here must not leave a phone connected but deaf
            self._log.error(f"sending to {phone.name} failed, closing its connection: {exc!r}")
            await phone.connection.close(1011, "gateway error")

    def _receive(self, phone: _Phone, message) -> None:
        if isinstance(message, str):
            self._report_once(phone, "text", "sent a text message; only binary messages are understood")
            return
        try:
            frame = framing.decode(message, MAX_INCOMING_BYTES)
        except framing.FrameError as exc:
            self._report_once(phone, "frame", f"sent a message that is not a valid frame ({exc})")
            return
        if frame.channel != pb.CHANNEL_COMMAND:
            self._report_once(phone, "channel", f"sent a message on channel {frame.channel}, not the command channel")
            return
        task = asyncio.ensure_future(self._answer(phone, frame.type, frame.payload))
        self._running.add(task)
        task.add_done_callback(self._running.discard)

    async def _answer(self, phone: _Phone, msg_type: int, payload: bytes) -> None:
        """Carry out one command and send its one Reply to the phone that sent it."""
        try:
            request_id = pb.RequestId.FromString(payload).request_id
        except DecodeError:
            request_id = 0  # unreadable: answered under 0, which the app never uses
        result, text, *more = await self._carry_out(phone, msg_type, payload)
        fields = more[0] if more else {}
        frame = framing.encode(
            pb.CHANNEL_DATA, pb.DATA_REPLY, convert.reply(request_id, result, text, **fields).SerializeToString(), self._max_payload
        )
        if not phone.outbox.put_in_order(frame):
            self._log.warning(f"{phone.name} is not reading its messages; closing its connection")
            await phone.connection.close(1008, "not reading")

    async def _carry_out(self, phone: _Phone, msg_type: int, payload: bytes) -> tuple:
        command = self._commands.get(msg_type)
        if command is None:
            self._report_once(phone, ("type", msg_type), f"sent command type 0x{msg_type:02x}, which this gateway does not know")
            return Result.RESULT_BAD_REQUEST, f"this gateway does not know command type 0x{msg_type:02x}"
        try:
            message = command.message_type.FromString(payload)
        except DecodeError:
            self._report_once(phone, ("unreadable", msg_type), f"sent a {command.name} command that could not be read")
            return Result.RESULT_BAD_REQUEST, f"the {command.name} command could not be read"
        if phone.commands_waiting >= MAX_COMMANDS_PER_PHONE:
            return Result.RESULT_BUSY, f"{MAX_COMMANDS_PER_PHONE} commands from this phone are already waiting for an answer"
        phone.commands_waiting += 1
        try:
            work = command.run(message, phone.number) if command.with_phone else command.run(message)
            return await asyncio.wait_for(work, command.timeout)
        except asyncio.TimeoutError:
            self._log.warning(f"{command.name} for {phone.name}: no answer from the robot within {command.timeout:.0f} s")
            return Result.RESULT_TIMED_OUT, f"no answer from the robot within {command.timeout:.0f} s; it may still act on it"
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # a fault in a handler must still produce an answer
            self._log.error(f"{command.name} for {phone.name} failed in the gateway: {exc!r}")
            return Result.RESULT_ERROR, f"the gateway failed while handling it ({exc})"
        finally:
            phone.commands_waiting -= 1

    def _report_once(self, phone: _Phone, key, text: str) -> None:
        """Log a problem with a phone's messages once per connection, not once per message."""
        if key not in phone.reported:
            phone.reported.add(key)
            self._log.warning(f"{phone.name} {text}; ignored")

    async def _heartbeats(self) -> None:
        while True:
            await asyncio.sleep(self._heartbeat_interval)
            for phone in list(self._phones):
                phone.heartbeats += 1
                payload = convert.heartbeat(phone.heartbeats).SerializeToString()
                phone.outbox.put_latest(pb.DATA_HEARTBEAT, framing.encode(pb.CHANNEL_DATA, pb.DATA_HEARTBEAT, payload, self._max_payload))
