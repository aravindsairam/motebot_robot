"""The SetNoGoZones command, through the real WebSocket server, with a stand-in for the mode
manager's /set_no_go_zones service. No ROS."""

import asyncio

from test_server import LIMIT, Log, connected, receive, receive_reply, send_command

from vacuum_gateway import convert, zones
from vacuum_gateway import vacuum_pb2 as pb
from vacuum_gateway.server import Command, GatewayServer
from vacuum_gateway.streams import Stream
from vacuum_mode_manager import map_files

Result = pb.Reply.Result

STAIRWELL = pb.NoGoZones.Zone(name="stairwell", points_cm=[455, 200, 565, 200, 565, 330, 455, 330])
RUG = pb.NoGoZones.Zone(name="rug é", points_cm=[-100, -50, 0, 0, -50, 100])


class Robot:
    """Plays the mode manager: refuses with `refusal`, else saves the zones to the map's file as it
    does. `present` False: its service is absent."""

    def __init__(self, maps_dir, refusal="", present=True):
        self.maps_dir, self.refusal, self.present = maps_dir, refusal, present
        self.asked = []

    async def save(self, map_name, zone_list):
        if not self.present:
            return None
        self.asked.append((map_name, zone_list))
        if self.refusal:
            return False, self.refusal
        polygons = [{"name": name, "polygon": [list(c) for c in corners]} for name, corners in zone_list]
        map_files.write_zones(map_files.zones_path(self.maps_dir, map_name), polygons)
        return True, f"saved the no-go zones of '{map_name}'"


def run(robot, test):
    """Start a server whose only command is SetNoGoZones, wired as gateway_node wires it: after a
    save, the zones of the map in use are read again and published at once."""

    async def main():
        watch = zones.MapZones(robot.maps_dir)
        watch.follow("house")
        server = None

        async def send_now():
            found = watch.changed(force=True)
            if found is not None:
                server.publish(pb.DATA_NO_GO_ZONES, convert.no_go_zones(found[1], found[0]).SerializeToString())

        command = Command("set no-go zones", pb.SetNoGoZones, 2.0, zones.set_zones_command(robot.save, send_now))
        server = GatewayServer(robot_name="Test", gateway_version="9.9", logger=Log(), host="127.0.0.1", port=0,
                               streams={pb.DATA_NO_GO_ZONES: Stream("the no-go zones", repeat_interval=None, replay_max_age=None)},
                               commands={pb.COMMAND_SET_NO_GO_ZONES: command}, heartbeat_interval=10.0, max_payload=LIMIT)
        port = await server.start()
        try:
            await test(f"ws://127.0.0.1:{port}")
        finally:
            await server.stop()

    asyncio.run(main())


async def ask(ws, request_id, map_name, *zone_list):
    await send_command(ws, pb.COMMAND_SET_NO_GO_ZONES, pb.SetNoGoZones(request_id=request_id, map_name=map_name, zones=zone_list))


async def reply_and_zones(ws):
    """The Reply and the NoGoZones that a save brings, in whichever order they come. The zones are
    published before the command answers, but the reply may still overtake them in a phone's outbox."""
    got = {}
    while len(got) < 2:
        frame = await receive(ws, timeout=1.0)
        got[frame.type] = frame.payload
    return pb.Reply.FromString(got[pb.DATA_REPLY]), pb.NoGoZones.FromString(got[pb.DATA_NO_GO_ZONES])


def test_saved_zones_go_to_every_phone_at_once(tmp_path):
    robot = Robot(str(tmp_path))

    async def body(url):
        asker, other = await connected(url), await connected(url)
        await ask(asker, 7, "house", STAIRWELL, RUG)
        reply, sent_to_asker = await reply_and_zones(asker)
        assert (reply.request_id, reply.result, reply.message) == (7, Result.RESULT_OK, "saved the no-go zones of 'house'")
        # The robot was asked in metres, for the map the phone named.
        assert robot.asked == [("house", [("stairwell", [(4.55, 2.0), (5.65, 2.0), (5.65, 3.3), (4.55, 3.3)]),
                                          ("rug é", [(-1.0, -0.5), (0.0, 0.0), (-0.5, 1.0)])])]
        # Both phones get the zones as saved, with the map's name, without waiting for a look at the file.
        frame = await receive(other, timeout=0.5)
        assert frame.type == pb.DATA_NO_GO_ZONES
        for sent in (pb.NoGoZones.FromString(frame.payload), sent_to_asker):
            assert sent.map_name == "house"
            assert [(z.name, list(z.points_cm)) for z in sent.zones] == [(STAIRWELL.name, list(STAIRWELL.points_cm)),
                                                                        (RUG.name, list(RUG.points_cm))]
        await asker.close()
        await other.close()

    run(robot, body)


def test_deleting_every_zone(tmp_path):
    robot = Robot(str(tmp_path))
    map_files.write_zones(map_files.zones_path(str(tmp_path), "house"), [{"name": "a", "polygon": [[0, 0], [1, 0], [1, 1]]}])

    async def body(url):
        ws = await connected(url)
        await ask(ws, 8, "house")
        reply, sent = await reply_and_zones(ws)
        assert reply.result == Result.RESULT_OK
        assert sent.map_name == "house" and list(sent.zones) == []
        await ws.close()

    run(robot, body)


def test_the_robots_refusal_is_passed_on_and_nothing_is_sent(tmp_path):
    robot = Robot(str(tmp_path), refusal="stop the clean first: no-go zones cannot change during a clean")

    async def body(url):
        ws = await connected(url)
        await ask(ws, 9, "house", STAIRWELL)
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result, reply.message) == (
            9, Result.RESULT_REFUSED, "stop the clean first: no-go zones cannot change during a clean")
        try:
            frame = await receive(ws, timeout=0.3)
        except asyncio.TimeoutError:
            frame = None
        assert frame is None
        await ws.close()

    run(robot, body)


def test_an_odd_number_of_coordinates_never_reaches_the_robot(tmp_path):
    robot = Robot(str(tmp_path))

    async def body(url):
        ws = await connected(url)
        await ask(ws, 10, "house", STAIRWELL, pb.NoGoZones.Zone(name="bad", points_cm=[0, 0, 100]))
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result) == (10, Result.RESULT_BAD_REQUEST)
        assert "zone 2 (bad) has an odd number of coordinates" == reply.message
        assert robot.asked == []
        await ws.close()

    run(robot, body)


def test_no_mode_manager_means_not_available(tmp_path):
    robot = Robot(str(tmp_path), present=False)

    async def body(url):
        ws = await connected(url)
        await ask(ws, 11, "house", STAIRWELL)
        reply = await receive_reply(ws)
        assert (reply.request_id, reply.result) == (11, Result.RESULT_NOT_AVAILABLE)
        assert "/set_no_go_zones" in reply.message
        await ws.close()

    run(robot, body)


def test_the_save_is_reported_even_if_sending_the_zones_at_once_fails():
    async def saved(map_name, zone_list):
        return True, "saved"

    async def broken():
        raise RuntimeError("the ROS thread has stopped")

    run_command = zones.set_zones_command(saved, broken)
    result = asyncio.run(run_command(pb.SetNoGoZones(request_id=1, map_name="house", zones=[STAIRWELL])))
    assert result == (Result.RESULT_OK, "saved")
