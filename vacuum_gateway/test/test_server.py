"""The WebSocket server, with real connections on this computer's loopback address. No ROS."""

import asyncio
import time

import pytest
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed

from vacuum_gateway import framing
from vacuum_gateway import vacuum_pb2 as pb
from vacuum_gateway.server import MAX_COMMANDS_PER_PHONE, Command, GatewayServer, Outbox
from vacuum_gateway.streams import Stream

Result = pb.Reply.Result

LIMIT = 1 << 20
STATE_A = pb.RobotState(state="idle", reason="started").SerializeToString()
STATE_B = pb.RobotState(state="manual", reason="requested").SerializeToString()


class Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warning(self, text):
        self.lines.append(("warning", text))

    def error(self, text):
        self.lines.append(("error", text))

    def at(self, level):
        return [t for lv, t in self.lines if lv == level]


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def run(test):
    """Run an async test body with a started server: test(server, url, log, clock)."""

    async def main(**kwargs):
        log, clock = Log(), Clock()
        streams = {
            pb.DATA_ROBOT_STATE: Stream("the robot state", 0.4, 2.0),
            pb.DATA_SAFETY_STATUS: Stream("the safety status", 0.5, 2.0),
            pb.DATA_MAP: Stream("the map", repeat_interval=None, replay_max_age=None, min_interval=0.3),
        }
        options = {"heartbeat_interval": 10.0, "max_payload": LIMIT, "clock": clock, **kwargs}
        server = GatewayServer(robot_name="Test", gateway_version="9.9", streams=streams, logger=log,
                               host="127.0.0.1", port=0, **options)
        port = await server.start()
        try:
            await test(server, f"ws://127.0.0.1:{port}", log, clock)
        finally:
            await server.stop()

    return main


async def receive(ws, timeout=1.0, skip_heartbeats=True):
    while True:
        frame = framing.decode(await asyncio.wait_for(ws.recv(), timeout), LIMIT)
        if not (skip_heartbeats and frame.type == pb.DATA_HEARTBEAT):
            return frame


async def nothing_arrives(ws, seconds=0.3):
    try:
        frame = await receive(ws, timeout=seconds)
    except asyncio.TimeoutError:
        return True
    raise AssertionError(f"expected nothing, got type 0x{frame.type:02x}")


async def connected(url):
    """Connect and read the Hello."""
    ws = await connect(url, compression=None)
    hello = await receive(ws)
    assert hello.type == pb.DATA_HELLO
    return ws


def test_hello_comes_first():
    async def body(server, url, log, clock):
        async with connect(url, compression=None) as ws:
            frame = await receive(ws)
            assert (frame.channel, frame.type) == (pb.CHANNEL_DATA, pb.DATA_HELLO)
            hello = pb.Hello.FromString(frame.payload)
            assert hello.protocol_version == 1 and hello.robot_name == "Test" and hello.gateway_version == "9.9"

    asyncio.run(run(body)())


def test_stream_reaches_the_phone():
    async def body(server, url, log, clock):
        ws = await connected(url)
        server.publish(pb.DATA_ROBOT_STATE, STATE_A)
        frame = await receive(ws)
        assert (frame.type, frame.payload) == (pb.DATA_ROBOT_STATE, STATE_A)
        await ws.close()

    asyncio.run(run(body)())


def test_late_phone_gets_the_latest_copy_at_once():
    async def body(server, url, log, clock):
        server.publish(pb.DATA_ROBOT_STATE, STATE_A)
        server.publish(pb.DATA_ROBOT_STATE, STATE_B)
        clock.now += 1.0
        ws = await connected(url)
        frame = await receive(ws, timeout=0.5)
        assert (frame.type, frame.payload) == (pb.DATA_ROBOT_STATE, STATE_B)
        assert await nothing_arrives(ws)
        await ws.close()

    asyncio.run(run(body)())


def test_old_copy_is_not_given_to_a_late_phone():
    async def body(server, url, log, clock):
        server.publish(pb.DATA_ROBOT_STATE, STATE_A)
        clock.now += 2.5  # the robot stopped publishing 2.5 s ago
        ws = await connected(url)
        assert await nothing_arrives(ws)
        await ws.close()

    asyncio.run(run(body)())


def test_unchanged_copies_are_thinned_and_changes_pass():
    async def body(server, url, log, clock):
        ws = await connected(url)
        for _ in range(5):
            server.publish(pb.DATA_SAFETY_STATUS, STATE_A)
        assert (await receive(ws)).payload == STATE_A
        assert await nothing_arrives(ws)
        server.publish(pb.DATA_SAFETY_STATUS, STATE_B)  # changed: passes at once
        assert (await receive(ws)).payload == STATE_B
        clock.now += 0.5
        server.publish(pb.DATA_SAFETY_STATUS, STATE_B)  # unchanged, but the repeat is due
        assert (await receive(ws)).payload == STATE_B
        await ws.close()

    asyncio.run(run(body)())


def test_map_changes_are_paced_and_the_newest_arrives():
    async def body(server, url, log, clock):
        ws = await connected(url)
        server.publish(pb.DATA_MAP, b"m1")
        assert (await receive(ws)).payload == b"m1"
        server.publish(pb.DATA_MAP, b"m2")  # too soon: held
        server.publish(pb.DATA_MAP, b"m3")  # replaces it
        assert await nothing_arrives(ws, 0.15)
        frame = await receive(ws, timeout=1.0)
        assert (frame.type, frame.payload) == (pb.DATA_MAP, b"m3")
        assert await nothing_arrives(ws, 0.5)  # m2 never goes
        await ws.close()

    asyncio.run(run(body)(clock=time.monotonic))


def test_a_late_phone_gets_the_map_however_old():
    async def body(server, url, log, clock):
        server.publish(pb.DATA_MAP, b"the map")
        clock.now += 3600.0
        ws = await connected(url)
        assert (await receive(ws, timeout=0.5)).payload == b"the map"
        await ws.close()

    asyncio.run(run(body)())


def test_heartbeats_count_up_per_phone():
    async def body(server, url, log, clock):
        ws = await connected(url)
        sequence = []
        while len(sequence) < 3:
            frame = await receive(ws, skip_heartbeats=False)
            if frame.type == pb.DATA_HEARTBEAT:
                sequence.append(pb.Heartbeat.FromString(frame.payload).sequence)
        assert sequence == [1, 2, 3]
        await ws.close()

    asyncio.run(run(body, )(heartbeat_interval=0.05))


def test_every_phone_gets_every_stream():
    async def body(server, url, log, clock):
        phones = [await connected(url) for _ in range(3)]
        assert server.phone_count == 3
        server.publish(pb.DATA_ROBOT_STATE, STATE_A)
        for ws in phones:
            assert (await receive(ws)).payload == STATE_A
            await ws.close()

    asyncio.run(run(body)())


def test_too_big_message_is_refused_and_logged_once():
    async def body(server, url, log, clock):
        ws = await connected(url)
        server.publish(pb.DATA_ROBOT_STATE, b"x" * 101)
        server.publish(pb.DATA_ROBOT_STATE, b"y" * 101)
        assert await nothing_arrives(ws)
        assert len(log.at("error")) == 1 and "over the limit" in log.at("error")[0]
        server.publish(pb.DATA_ROBOT_STATE, b"small")
        assert (await receive(ws)).payload == b"small"
        await ws.close()

    asyncio.run(run(body)(max_payload=100))


def test_bad_messages_from_a_phone_are_logged_once_and_ignored():
    async def body(server, url, log, clock):
        ws = await connected(url)
        for _ in range(2):
            await ws.send(b"\x00")  # not a frame
            await ws.send("hello")  # text
            await ws.send(framing.encode(pb.CHANNEL_DATA, 1, b"", LIMIT))  # wrong channel
        await asyncio.sleep(0.2)
        warnings = log.at("warning")
        assert len(warnings) == 3, warnings
        server.publish(pb.DATA_ROBOT_STATE, STATE_A)  # still connected and served
        assert (await receive(ws)).payload == STATE_A
        await ws.close()

    asyncio.run(run(body)())


# ---------------------------------------------------------------- commands
async def send_command(ws, msg_type, message):
    await ws.send(framing.encode(pb.CHANNEL_COMMAND, msg_type, message.SerializeToString(), LIMIT))


async def receive_reply(ws, timeout=1.0):
    frame = await receive(ws, timeout)
    assert frame.type == pb.DATA_REPLY, f"expected a reply, got type 0x{frame.type:02x}"
    return pb.Reply.FromString(frame.payload)


def change_mode_command(behaviour, timeout=2.0):
    """A stand-in for the robot's change-mode command: `behaviour(message)` is awaited."""
    return {pb.COMMAND_CHANGE_MODE: Command("change mode", pb.ChangeMode, timeout, behaviour)}


def test_ping_is_answered_with_the_same_number():
    async def body(server, url, log, clock):
        ws = await connected(url)
        await send_command(ws, pb.COMMAND_PING, pb.Ping(request_id=4000000000))
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result) == (4000000000, Result.RESULT_OK)
        await ws.close()

    asyncio.run(run(body)())


def test_the_robots_answer_is_passed_on_word_for_word():
    async def refuse(message):
        return Result.RESULT_REFUSED, f"cannot go from idle to {message.mode}"

    async def body(server, url, log, clock):
        ws = await connected(url)
        await send_command(ws, pb.COMMAND_CHANGE_MODE, pb.ChangeMode(request_id=7, mode="flying"))
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result, reply.message) == (7, Result.RESULT_REFUSED, "cannot go from idle to flying")
        await ws.close()

    asyncio.run(run(body)(commands=change_mode_command(refuse)))


def test_a_command_can_answer_with_a_goal_id():
    async def accept(message):
        return Result.RESULT_OK, "on the way", {"goal_id": b"g" * 16}

    async def body(server, url, log, clock):
        ws = await connected(url)
        await send_command(ws, pb.COMMAND_GO_TO, pb.GoTo(request_id=5, x=1.0, y=2.0, heading=0.0))
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result, reply.goal_id) == (5, Result.RESULT_OK, b"g" * 16)
        await ws.close()

    asyncio.run(run(body)(commands={pb.COMMAND_GO_TO: Command("go to", pb.GoTo, 2.0, accept)}))


def test_a_command_can_ask_which_phone_sent_it_and_hear_when_it_leaves():
    seen, left = [], []

    async def note(message, phone):
        seen.append(phone)
        return Result.RESULT_OK, ""

    async def body(server, url, log, clock):
        a, b = await connected(url), await connected(url)
        for ws, request_id in ((a, 1), (b, 2), (a, 3)):
            await send_command(ws, pb.COMMAND_DRIVE, pb.Drive(request_id=request_id, forward=0.1))
            assert (await receive_reply(ws)).result == Result.RESULT_OK
        await a.close()
        for _ in range(50):
            if left:
                break
            await asyncio.sleep(0.01)
        assert seen[0] == seen[2] != seen[1]
        assert left == [seen[0]]
        await b.close()

    commands = {pb.COMMAND_DRIVE: Command("drive", pb.Drive, 1.0, note, with_phone=True)}
    asyncio.run(run(body)(commands=commands, on_phone_left=left.append))


def test_unknown_command_gets_an_answer_with_its_number():
    async def body(server, url, log, clock):
        ws = await connected(url)
        for _ in range(2):
            await send_command(ws, 0x7E, pb.Ping(request_id=9))
            reply = await receive_reply(ws)
            assert (reply.request_id, reply.result) == (9, Result.RESULT_BAD_REQUEST)
            assert "0x7e" in reply.message
        assert len([w for w in log.at("warning") if "0x7e" in w]) == 1  # logged once per connection
        await ws.close()

    asyncio.run(run(body)())


def test_unreadable_command_gets_an_answer():
    async def body(server, url, log, clock):
        ws = await connected(url)
        await ws.send(framing.encode(pb.CHANNEL_COMMAND, pb.COMMAND_CHANGE_MODE, b"\xff\xff\xff", LIMIT))
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result) == (0, Result.RESULT_BAD_REQUEST)
        await ws.close()

    asyncio.run(run(body)(commands=change_mode_command(lambda m: None)))


def test_the_reply_goes_only_to_the_phone_that_asked():
    async def body(server, url, log, clock):
        asker, other = await connected(url), await connected(url)
        await send_command(asker, pb.COMMAND_PING, pb.Ping(request_id=1))
        assert (await receive_reply(asker)).request_id == 1
        assert await nothing_arrives(other)
        await asker.close()
        await other.close()

    asyncio.run(run(body)())


def test_a_slow_command_does_not_hold_up_the_next():
    async def slow(message):
        await asyncio.sleep(1.0)
        return Result.RESULT_OK, "now in docking"

    async def body(server, url, log, clock):
        ws = await connected(url)
        await send_command(ws, pb.COMMAND_CHANGE_MODE, pb.ChangeMode(request_id=1, mode="docking"))
        await send_command(ws, pb.COMMAND_PING, pb.Ping(request_id=2))
        first = await receive_reply(ws, timeout=0.5)
        assert first.request_id == 2  # the ping, while the mode change is still running
        second = await receive_reply(ws, timeout=2.0)
        assert (second.request_id, second.message) == (1, "now in docking")
        await ws.close()

    asyncio.run(run(body)(commands=change_mode_command(slow)))


def test_no_answer_in_time_is_reported():
    async def never(message):
        await asyncio.sleep(10)

    async def body(server, url, log, clock):
        ws = await connected(url)
        await send_command(ws, pb.COMMAND_CHANGE_MODE, pb.ChangeMode(request_id=3, mode="idle"))
        reply = await receive_reply(ws, timeout=2.0)
        assert (reply.request_id, reply.result) == (3, Result.RESULT_TIMED_OUT)
        assert "may still act" in reply.message
        await ws.close()

    asyncio.run(run(body)(commands=change_mode_command(never, timeout=0.2)))


def test_a_fault_in_a_command_still_gets_an_answer():
    async def broken(message):
        raise RuntimeError("boom")

    async def body(server, url, log, clock):
        ws = await connected(url)
        await send_command(ws, pb.COMMAND_CHANGE_MODE, pb.ChangeMode(request_id=4, mode="idle"))
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result) == (4, Result.RESULT_ERROR)
        assert len(log.at("error")) == 1
        await ws.close()

    asyncio.run(run(body)(commands=change_mode_command(broken)))


def test_too_many_waiting_commands_from_one_phone_are_refused():
    release = None

    async def wait(message):
        await release.wait()
        return Result.RESULT_OK, ""

    async def body(server, url, log, clock):
        nonlocal release
        release = asyncio.Event()
        ws = await connected(url)
        for i in range(MAX_COMMANDS_PER_PHONE + 1):
            await send_command(ws, pb.COMMAND_CHANGE_MODE, pb.ChangeMode(request_id=i + 1, mode="idle"))
        busy = await receive_reply(ws)
        assert (busy.request_id, busy.result) == (MAX_COMMANDS_PER_PHONE + 1, Result.RESULT_BUSY)
        release.set()
        answered = {(await receive_reply(ws)).request_id for _ in range(MAX_COMMANDS_PER_PHONE)}
        assert answered == set(range(1, MAX_COMMANDS_PER_PHONE + 1))
        await ws.close()

    asyncio.run(run(body)(commands=change_mode_command(wait)))


def test_a_phone_leaving_before_its_answer_is_harmless():
    async def slow(message):
        await asyncio.sleep(0.3)
        return Result.RESULT_OK, ""

    async def body(server, url, log, clock):
        ws = await connected(url)
        await send_command(ws, pb.COMMAND_CHANGE_MODE, pb.ChangeMode(request_id=1, mode="idle"))
        await ws.close()
        await asyncio.sleep(0.6)
        assert log.at("error") == []
        other = await connected(url)  # still serving
        await send_command(other, pb.COMMAND_PING, pb.Ping(request_id=2))
        assert (await receive_reply(other)).request_id == 2
        await other.close()

    asyncio.run(run(body)(commands=change_mode_command(slow)))


def test_a_phone_that_leaves_is_forgotten():
    async def body(server, url, log, clock):
        ws = await connected(url)
        assert server.phone_count == 1
        await ws.close()
        for _ in range(50):
            if server.phone_count == 0:
                break
            await asyncio.sleep(0.02)
        assert server.phone_count == 0
        assert any("disconnected" in t for t in log.at("info"))
        server.publish(pb.DATA_ROBOT_STATE, STATE_A)  # nobody to send to: no error

    asyncio.run(run(body)())


def test_stopping_closes_the_phones():
    async def body(server, url, log, clock):
        ws = await connected(url)
        await server.stop()
        with pytest.raises(ConnectionClosed):
            await asyncio.wait_for(ws.recv(), 2.0)

    asyncio.run(run(body)())


def test_plain_web_request_gets_a_note_and_no_traceback():
    async def body(server, url, log, clock):
        port = int(url.rsplit(":", 1)[1])
        reader, writer = await asyncio.open_connection("127.0.0.1", port)
        writer.write(b"GET / HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n")
        await writer.drain()
        answer = (await asyncio.wait_for(reader.read(), 2.0)).decode()
        writer.close()
        assert answer.startswith("HTTP/1.1 426") and "Connect with the app" in answer
        assert log.at("warning") == [] and log.at("error") == []
        ws = await connected(url)  # the gateway carries on serving phones
        await ws.close()

    asyncio.run(run(body)())


def test_port_in_use_is_reported():
    async def body(server, url, log, clock):
        port = int(url.rsplit(":", 1)[1])
        second = GatewayServer(robot_name="Test", gateway_version="9.9", streams={}, logger=Log(),
                               host="127.0.0.1", port=port)
        with pytest.raises(OSError):
            await second.start()

    asyncio.run(run(body)())


def test_outbox_keeps_the_newest_copy_and_sends_in_order_messages_first():
    async def body():
        box = Outbox(max_in_order=2)
        box.put_latest(1, b"old")
        box.put_latest(2, b"other")
        box.put_latest(1, b"new")
        assert box.put_in_order(b"hello")
        assert box.put_in_order(b"reply")
        assert not box.put_in_order(b"one too many")
        assert await asyncio.wait_for(box.take(), 1.0) == [b"hello", b"reply", b"other", b"new"]
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(box.take(), 0.1)  # nothing left

    asyncio.run(body())
