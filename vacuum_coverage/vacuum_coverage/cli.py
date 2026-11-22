"""Terminal commands for cleaning.

    ros2 run vacuum_coverage clean                          clean everything, then dock
    ros2 run vacuum_coverage clean --area 1.2 0.2 3.2 1.4   clean a rectangle (map metres: x0 y0 x1 y1)
    ros2 run vacuum_coverage clean --spot 2.0 1.0           clean a square around a point
    ros2 run vacuum_coverage clean --plan "Living room"     run a saved plan
        options: --stay (do not dock afterwards)  --passes 2  --edges / --no-edges  --direction auto|x|y|DEGREES
        It starts the job and follows it until it ends; Ctrl-C stops the clean (asks for idle).
    ros2 run vacuum_coverage preview [same arguments]       the estimate, and the rows on /cleaning/plan
    ros2 run vacuum_coverage pause | resume
    ros2 run vacuum_coverage plans                          list the saved plans
    ros2 run vacuum_coverage plans --save NAME [--replace] [job arguments]
    ros2 run vacuum_coverage plans --delete NAME
    ros2 run vacuum_coverage history                        past cleans, newest first, and the totals
    ros2 run vacuum_coverage history --record ID [--out picture.png]
    ros2 run vacuum_coverage plan_picture [--map maps/house.yaml] [--x 0 --y 0] [job arguments] [--out plan.png]
        plans without ROS, from the map files, and writes a picture of the plan (step 1's check)
"""

import argparse
import dataclasses
import datetime
import os
import sys
import time



def _job_arguments(parser: argparse.ArgumentParser) -> None:
    where = parser.add_mutually_exclusive_group()
    where.add_argument("--area", nargs=4, type=float, metavar=("X0", "Y0", "X1", "Y1"), help="a rectangle, map metres")
    where.add_argument("--spot", nargs=2, type=float, metavar=("X", "Y"), help="a square around this point")
    where.add_argument("--plan", metavar="NAME", help="a saved plan")
    parser.add_argument("--direction", default="", metavar="auto|x|y|DEGREES",
                        help="which way the rows run (default: the robot's setting)")
    parser.add_argument("--passes", type=int, choices=(1, 2), default=1)
    edges = parser.add_mutually_exclusive_group()
    edges.add_argument("--edges", dest="edges", action="store_true", default=None, help="a pass along walls and furniture")
    edges.add_argument("--no-edges", dest="edges", action="store_false")
    parser.add_argument("--stay", action="store_true", help="do not dock when finished")


def _spec(args):
    from .jobspec import AREA, EVERYTHING, PLAN, SPOT, JobSpec

    if args.area:
        x0, y0, x1, y1 = args.area
        return JobSpec(what=AREA, area=[(x0, y0), (x1, y0), (x1, y1), (x0, y1)], direction=args.direction, passes=args.passes,
                       edges=args.edges, return_to_dock=not args.stay)
    if args.spot:
        return JobSpec(what=SPOT, spot=tuple(args.spot), direction=args.direction, passes=args.passes, edges=args.edges,
                       return_to_dock=not args.stay)
    if args.plan:
        return JobSpec(what=PLAN, plan_name=args.plan, return_to_dock=not args.stay)
    return JobSpec(what=EVERYTHING, direction=args.direction, passes=args.passes, edges=args.edges, return_to_dock=not args.stay)


def _job_msg(args):
    from .cleaning_node import CleaningNode

    return CleaningNode._spec_to_msg(_spec(args))


def _minutes(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    return f"{int(seconds // 60)}:{int(seconds % 60):02d}"


def _estimate_lines(e):
    yield f"rows:           {e.rows}" + (f", edges: {e.edges}" if e.edges else "")
    yield f"turns:          {e.turns}, transits: {e.transits}"
    yield f"distance:       {e.distance:.1f} m"
    yield f"planned floor:  {e.planned_area:.2f} m²  (left out {e.left_out_area:.2f} m², cannot reach {e.unreachable_area:.2f} m²)"
    yield f"expected time:  {e.time / 60:.1f} min  (planned in {e.planning_time:.2f} s)"


def _status_line(s) -> str:
    return (f"[{s.state}] {s.message}  |  {s.coverage:.0f}% of {s.planned_area:.1f} m²  cleaned {s.cleaned_area:.1f} m²  "
            f"{s.distance:.1f} m  {_minutes(s.cleaning_time)}  left ~{_minutes(s.time_left)}  "
            f"rows {s.rows_done}/{s.rows_total} skipped {s.rows_skipped}  bumps {s.bumps}")


# ---------------------------------------------------------------------- with ROS
class _Ros:
    def __init__(self, name):
        import rclpy
        from rclpy.node import Node

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node(name)

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"The cleaning node is not running (no {name} service).")
            return None
        for _ in range(2):  # a brand-new client can lose its first reply on Humble
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            if future.done():
                return future.result()
        print(f"No answer from {name} within {timeout:.0f} s.")
        return None

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def clean_main(argv=None):
    parser = argparse.ArgumentParser(prog="clean", description="Clean, and follow the clean until it ends.")
    _job_arguments(parser)
    args = parser.parse_args(argv)
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from vacuum_interfaces.msg import CleaningStatus
    from vacuum_interfaces.srv import ChangeState, StartCleaning

    ros = _Ros("clean_cli")
    code = 1
    try:
        r = ros.call(StartCleaning, "cleaning/start", StartCleaning.Request(job=_job_msg(args)))
        if r is None:
            return
        if not r.accepted:
            print(f"Refused: {r.message}")
            return
        print(f"Job {r.job_id} accepted: {r.message}. Ctrl-C stops the clean.", flush=True)
        latest = {}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ros.node.create_subscription(CleaningStatus, "cleaning/status", lambda m: latest.update(status=m), latched)
        final = (CleaningStatus.FINISHED, CleaningStatus.STOPPED, CleaningStatus.FAILED)
        last = {"line": "", "at": 0.0}
        try:
            while True:
                ros.rclpy.spin_once(ros.node, timeout_sec=0.2)
                s = latest.get("status")
                if s is None or s.job_id != r.job_id:
                    continue
                now = time.monotonic()
                if s.message != last["line"] or now - last["at"] > 5.0:
                    last["line"], last["at"] = s.message, now
                    print(_status_line(s), flush=True)
                if s.state in final:
                    code = 0 if s.state == CleaningStatus.FINISHED else 1
                    break
        except KeyboardInterrupt:
            print("\nStopping the clean (asking for idle)...")
            answer = ros.call(ChangeState, "change_state", ChangeState.Request(new_state="idle"), timeout=250.0)
            print(answer.message if answer is not None else "no answer from the mode manager")
    finally:
        ros.close()
        sys.exit(code)


def preview_main(argv=None):
    parser = argparse.ArgumentParser(prog="preview", description="Plan a clean without moving; the rows go to /cleaning/plan.")
    _job_arguments(parser)
    args = parser.parse_args(argv)
    from vacuum_interfaces.srv import PreviewCleaning

    ros = _Ros("preview_cli")
    try:
        r = ros.call(PreviewCleaning, "cleaning/preview", PreviewCleaning.Request(job=_job_msg(args)))
        if r is None:
            sys.exit(1)
        print(r.message)
        if r.success:
            for line in _estimate_lines(r.estimate):
                print(line)
        sys.exit(0 if r.success else 1)
    finally:
        ros.close()


def _trigger(name):
    from std_srvs.srv import Trigger

    ros = _Ros(f"{name}_cli")
    try:
        r = ros.call(Trigger, f"cleaning/{name}", Trigger.Request())
        if r is None:
            sys.exit(1)
        print(r.message)
        sys.exit(0 if r.success else 1)
    finally:
        ros.close()


def pause_main():
    _trigger("pause")


def resume_main():
    _trigger("resume")


def plans_main(argv=None):
    parser = argparse.ArgumentParser(prog="plans", description="List, save or delete saved cleaning plans.")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--save", metavar="NAME")
    action.add_argument("--delete", metavar="NAME")
    parser.add_argument("--replace", action="store_true", help="with --save: overwrite a plan of the same name")
    _job_arguments(parser)
    args = parser.parse_args(argv)
    from vacuum_interfaces.srv import DeleteCleaningPlan, ListCleaningPlans, SaveCleaningPlan

    ros = _Ros("plans_cli")
    try:
        if args.save:
            r = ros.call(SaveCleaningPlan, "cleaning/save_plan", SaveCleaningPlan.Request(name=args.save, job=_job_msg(args), replace=args.replace))
            if r is not None:
                print(r.message)
            sys.exit(0 if r is not None and r.success else 1)
        if args.delete:
            r = ros.call(DeleteCleaningPlan, "cleaning/delete_plan", DeleteCleaningPlan.Request(name=args.delete))
            if r is not None:
                print(r.message)
            sys.exit(0 if r is not None and r.success else 1)
        r = ros.call(ListCleaningPlans, "cleaning/list_plans", ListCleaningPlans.Request())
        if r is None:
            sys.exit(1)
        if not r.plans:
            print("No saved plans for this map.")
        for p in r.plans:
            saved = datetime.datetime.fromtimestamp(p.saved).strftime("%Y-%m-%d %H:%M")
            changed = "  (map changed: rows will be planned again)" if p.map_changed else ""
            print(f"{p.name:24s} {p.estimate.rows:3d} rows  {p.estimate.planned_area:5.1f} m²  ~{p.estimate.time / 60:.0f} min  saved {saved}{changed}")
    finally:
        ros.close()


def history_main(argv=None):
    parser = argparse.ArgumentParser(prog="history", description="Past cleans and lifetime totals.")
    parser.add_argument("--record", metavar="ID", help="show one record")
    parser.add_argument("--out", metavar="FILE", help="with --record: write its picture here")
    parser.add_argument("--max", type=int, default=20, help="how many records to list (0: all)")
    args = parser.parse_args(argv)
    from vacuum_interfaces.srv import GetCleaningRecord, ListCleaningHistory

    ros = _Ros("history_cli")
    try:
        if args.record:
            r = ros.call(GetCleaningRecord, "cleaning/get_record", GetCleaningRecord.Request(id=args.record))
            if r is None or not r.success:
                print(r.message if r is not None else "")
                sys.exit(1)
            print(_record_line(r.record))
            print(_status_line(r.record.status))
            if args.out:
                with open(args.out, "wb") as f:
                    f.write(bytes(r.picture))
                print(f"picture written to {args.out}")
            return
        r = ros.call(ListCleaningHistory, "cleaning/history", ListCleaningHistory.Request(max_records=args.max))
        if r is None:
            sys.exit(1)
        t = r.totals
        print(f"{t.cleans} cleans · {t.area:.1f} m² · {t.time / 3600:.1f} h · {t.distance:.0f} m")
        for rec in r.records:
            print(_record_line(rec))
    finally:
        ros.close()


def _record_line(rec) -> str:
    s = rec.status
    when = datetime.datetime.fromtimestamp(rec.started).strftime("%Y-%m-%d %H:%M")
    return (f"{rec.id}  {when}  {s.target:16s} {s.cleaned_area:5.1f} m²  {s.coverage:3.0f}%  {_minutes(s.cleaning_time)}  "
            f"{s.message if s.message.startswith(s.state) else s.state + ': ' + s.message}")


# ---------------------------------------------------------------------- without ROS
def plan_picture_main(argv=None):
    parser = argparse.ArgumentParser(prog="plan_picture", description="Plan from the map files (no ROS) and draw the plan.")
    parser.add_argument("--map", default=None, help="the map's .yaml (default: the current map in ros2_ws/maps)")
    parser.add_argument("--zones", default=None, help="the no-go zones file (default: the map's own, <map>_zones.yaml)")
    parser.add_argument("--x", type=float, default=0.0, help="where the robot is (map metres); default the start pose")
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--out", default="cleaning_plan.png")
    _job_arguments(parser)
    args = parser.parse_args(argv)

    from vacuum_mode_manager import map_files

    from . import mapdata, picture, planner
    from .planner import PlanError, Settings

    map_yaml = args.map or _default_map()
    if not map_yaml or not os.path.isfile(map_yaml):
        print("No map found: give one with --map path/to/map.yaml")
        sys.exit(1)
    m = mapdata.load(map_yaml, map_files.read_zones(args.zones or map_files.zones_path_for(map_yaml)))
    settings = Settings()
    spec = _spec(args)
    if not spec.direction:  # no robot here to ask for its setting
        spec = dataclasses.replace(spec, direction="auto")
    why = spec.check()
    if why or spec.what == "plan":
        print(why or "plan_picture cannot read saved plans; use --area, --spot or nothing")
        sys.exit(1)
    try:
        plan = planner.make_plan(m.grid, (args.x, args.y), settings, spec.options(False), spec.area_polygon(settings.spot_size))
    except PlanError as exc:
        print(f"Nothing to plan: {exc}")
        sys.exit(1)
    picture.plan_picture(m, plan, (args.x, args.y)).save(args.out)
    open_floor = float(m.grid.floor.sum()) * m.grid.resolution ** 2
    print(f"map: {map_yaml} ({open_floor:.2f} m² of open floor)")
    for line in picture.summary(plan):
        print(line)
    print(f"picture: {os.path.abspath(args.out)}")


def _default_map():
    workspace = os.environ.get("VACUUM_WS", os.path.join(os.getcwd(), "ros2_ws"))
    for maps in (os.path.join(workspace, "maps"), os.path.join(os.getcwd(), "maps")):
        marker = os.path.join(maps, "current_map.txt")
        if os.path.isfile(marker):
            with open(marker) as f:
                return os.path.join(maps, f.read().strip() + ".yaml")
    return ""
