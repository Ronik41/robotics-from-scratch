"""Milestone 2 launcher and Gazebo-ground-truth acceptance checks (inside Docker)."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from google.protobuf import text_format
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_stats_pb2 import WorldStatistics

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "simulation/config/scenario.json"
WORLD = ROOT / "simulation/worlds/delivery_room.sdf"
XACRO = ROOT / "simulation/rover/rover.urdf.xacro"
SCENARIO = json.loads(CONFIG.read_text())
PREFIX = f"/world/{SCENARIO['world']}"


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def pose_values(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def request(node, suffix, message, reply_type, timeout=1500):
    # Scene readiness does not imply every other service has been discovered.
    # Wait BEFORE mutations; never blindly retry a possibly applied multi_step.
    end = time.monotonic() + 15
    while PREFIX + suffix not in node.service_list():
        if time.monotonic() >= end:
            raise RuntimeError(f"Service discovery deadline: {PREFIX + suffix}")
        time.sleep(0.05)
    ok, response = node.request(PREFIX + suffix, message, type(message), reply_type, timeout)
    if not ok:
        raise RuntimeError(f"Service failed or timed out: {PREFIX + suffix}")
    return response


def wait_for_scene(node, processes, deadline=40):
    end = time.monotonic() + deadline
    while time.monotonic() < end:
        check_processes(processes)
        try:
            return request(node, "/scene/info", Empty(), Scene)
        except RuntimeError:
            time.sleep(0.2)
    raise RuntimeError("Gazebo scene did not become ready within 40 seconds")


def check_processes(processes):
    for process in processes:
        if process.poll() is not None:
            raise RuntimeError(f"Child exited ({process.returncode}): {process.args}")


def generate(evidence, drive=False, sensors=False, firmware=False):
    urdf = evidence / "rover.urdf"
    subprocess.run(["xacro", str(XACRO), f"drive:={str(drive).lower()}", f"sensors:={str(sensors).lower()}", f"firmware:={str(firmware).lower()}", "-o", str(urdf)], check=True, timeout=15)
    with (evidence / "rover.sdf").open("w") as out:
        subprocess.run(["gz", "sdf", "-p", str(urdf)], stdout=out, check=True, timeout=15)
    with (evidence / "validation.log").open("w") as out:
        for path in [WORLD, evidence / "rover.sdf"]:
            checked = subprocess.run(["gz", "sdf", "-k", str(path)],
                                     capture_output=True, text=True, timeout=15)
            out.write(checked.stdout + checked.stderr)
            if checked.returncode or "Valid." not in checked.stdout:
                raise RuntimeError(f"Invalid SDF: {path}: {checked.stdout} {checked.stderr}")
    model = ET.parse(evidence / "rover.sdf").getroot().find("model")
    joints = {j.attrib["name"]: j for j in model.findall("joint")}
    for name in ("left_wheel_joint", "right_wheel_joint"):
        if name not in joints or joints[name].attrib["type"] != "revolute":
            raise RuntimeError(f"Missing revolving wheel joint: {name}")
    for link in model.findall("link"):
        if float(link.findtext("inertial/mass", "0")) <= 0:
            raise RuntimeError(f"Link lost inertia during conversion: {link.attrib['name']}")
    save_json(evidence / "inputs.json", {
        "scenario": SCENARIO,
        "drive": drive, "sensors": sensors, "firmware": firmware,
        "sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in [XACRO, WORLD, CONFIG, ROOT / "simulation/config/gui.config",
                             ROOT / "scripts/rover_sim.py"]}})
    if drive:
        paths = [ROOT / "Dockerfile", ROOT / "compose.yaml"]
        for directory in ["scripts", "src", "simulation", "tests"]:
            paths.extend(p for p in (ROOT / directory).rglob("*")
                         if p.is_file() and "__pycache__" not in p.parts
                         and p.suffix in {".cc", ".msg", ".xml", ".txt", ".py", ".sh", ".xacro", ".sdf", ".yaml", ".json", ".rviz", ".config"})
        save_json(evidence / "source-sha256.json", {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)})
    shutil.copyfile("/opt/installed-packages.tsv", evidence / "installed-packages.tsv")
    return urdf


def spawn(node, urdf, evidence):
    # Refuse duplicates rather than silently renaming or moving an existing robot.
    scene = request(node, "/scene/info", Empty(), Scene)
    if any(m.name == SCENARIO["model"] for m in scene.model):
        raise RuntimeError("delivery_rover already exists; restart the scenario to reset it")
    p = SCENARIO["pose"]
    factory = EntityFactory(sdf_filename=str(urdf), name=SCENARIO["model"], allow_renaming=False)
    factory.pose.position.x, factory.pose.position.y, factory.pose.position.z = p["x"], p["y"], p["z"]
    # This first scenario deliberately has zero roll, pitch and yaw.
    if any(p[key] != 0 for key in ("roll", "pitch", "yaw")):
        raise ValueError("This launcher currently requires zero initial rotation")
    factory.pose.orientation.w = 1
    (evidence / "spawn-request.txt").write_text(text_format.MessageToString(factory))
    response = request(node, "/create", factory, Boolean, 5000)
    (evidence / "spawn-response.txt").write_text(text_format.MessageToString(response))
    if not response.data:
        raise RuntimeError("Gazebo rejected rover creation")
    end = time.monotonic() + 15
    while time.monotonic() < end:
        scene = request(node, "/scene/info", Empty(), Scene)
        rovers = [m for m in scene.model if m.name == SCENARIO["model"]]
        if rovers:
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("Spawn acknowledged but rover absent from actual scene")
    names = sorted(m.name for m in scene.model)
    if names != sorted(SCENARIO["expected_models"]):
        raise RuntimeError(f"Unexpected scene models: {names}")
    actual = pose_values(rovers[0].pose)
    expected = [p["x"], p["y"], p["z"], 0, 0, 0, 1]
    if max(abs(a-b) for a, b in zip(actual, expected)) > 1e-9:
        raise RuntimeError(f"Initial pose mismatch: {actual} != {expected}")
    (evidence / "initial-scene.pbtxt").write_text(text_format.MessageToString(scene))
    return {"models": names, "initial_pose_xyz_xyzw": actual}


def settle(node, processes, evidence):
    # Subscribe BEFORE stepping; use timestamps, not a guessed wall-time sleep.
    state, lock = {}, threading.Lock()

    def on_pose(message):
        with lock:
            for pose in message.pose:
                if pose.name == SCENARIO["model"]:
                    state["pose"] = pose_values(pose)
                    state["pose_time_ns"] = message.header.stamp.sec*10**9 + message.header.stamp.nsec

    def on_stats(message):
        with lock:
            state["iterations"] = message.iterations
            state["paused"] = message.paused
            state["sim_time_ns"] = message.sim_time.sec*10**9 + message.sim_time.nsec

    node.subscribe(Pose_V, PREFIX + "/pose/info", on_pose)
    node.subscribe(WorldStatistics, PREFIX + "/stats", on_stats)
    target = SCENARIO["settle_steps"]
    response = request(node, "/control", WorldControl(pause=True, multi_step=target), Boolean, 10000)
    if not response.data:
        raise RuntimeError("Physics stepping rejected")
    end = time.monotonic() + 45
    while time.monotonic() < end:
        check_processes(processes)
        with lock:
            snapshot = dict(state)
        if (snapshot.get("iterations") == target and snapshot.get("paused")
                and snapshot.get("pose_time_ns", 0) >= target*1_000_000):
            break
        time.sleep(0.05)
    else:
        raise RuntimeError(f"No completed paused physics sample: {state}")
    pose = snapshot["pose"]
    p = SCENARIO["pose"]
    if math.hypot(pose[0]-p["x"], pose[1]-p["y"]) > 0.005:
        raise RuntimeError(f"Rover drifted more than 5 mm: {pose}")
    if not 0.235 <= pose[2] <= 0.245 or max(abs(v) for v in pose[3:6]) > 0.005:
        raise RuntimeError(f"Rover did not settle upright on its supports: {pose}")
    save_json(evidence / "settled.json", snapshot)
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--drive", action="store_true", help="Start M3 ROS nodes and unpause")
    parser.add_argument("--firmware", action="store_true", help="M5 torque controller; requires --sensors --drive")
    parser.add_argument("--sensors", action="store_true", help="M4 simulated sensor interfaces; requires --drive")
    parser.add_argument("--rviz", action="store_true", help="Show RViz and keyboard teleop on the desktop")
    parser.add_argument("--check", action="store_true", help="Step 2 s, assert stable, then exit")
    parser.add_argument("--spawn-only", action="store_true", help="Spawn into an already paused delivery world")
    args = parser.parse_args()
    if (args.firmware and not args.sensors) or (args.sensors and not args.drive) or (args.drive and (args.check or args.spawn_only)) or (args.rviz and not args.drive):
        parser.error("--drive is a live mode; --rviz requires --drive")
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    processes, logs = [], []
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, stop)
    try:
        # A second launch in one container must not connect to the first server.
        launch_lock = None
        if not args.spawn_only:
            launch_lock = open("/tmp/delivery-rover-launch.lock", "w")
            try:
                fcntl.flock(launch_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("A rover launcher is already running in this container") from exc
        urdf = generate(evidence, args.drive, args.sensors, args.firmware)
        world_path = WORLD
        if args.sensors:
            world = ET.parse(WORLD)
            w = world.getroot().find('world')
            sensor_plugin = ET.SubElement(w, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
            ET.SubElement(sensor_plugin, 'render_engine').text = 'ogre2'
            ET.SubElement(w, 'plugin', filename='gz-sim-imu-system', name='gz::sim::systems::Imu')
            world_path = evidence / 'sensor-world.sdf'
            world.write(world_path, encoding='unicode')
        node = Node()
        if not args.spawn_only:
            command = ["gz", "sim", "-s", "-v", "4", "--seed", str(SCENARIO["seed"]), *(["--headless-rendering"] if args.sensors and not args.gui else []), str(world_path)]
            save_json(evidence / "server-command.json", command)
            log = (evidence / "gazebo.log").open("w"); logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        wait_for_scene(node, processes)
        result = spawn(node, urdf, evidence)
        if args.check:
            result["settled"] = settle(node, processes, evidence)
        if args.gui:
            command = ["gz", "sim", "-g", "-v", "3", "--gui-config", str(ROOT / "simulation/config/gui.config")]
            save_json(evidence / "gui-command.json", command)
            log = (evidence / "gui.log").open("w"); logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        if args.drive:
            command = ["ros2", "launch", str(ROOT / "simulation/launch/teleop.launch.py"),
                       f"urdf:={urdf}", f"rviz:={str(args.rviz).lower()}", f"sensors:={str(args.sensors).lower()}", f"firmware:={str(args.firmware).lower()}"]
            save_json(evidence / "ros-command.json", command)
            log = (evidence / "ros.log").open("w"); logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
            if not request(node, "/control", WorldControl(pause=False), Boolean, 10000).data:
                raise RuntimeError("Unpause rejected")
        result["status"] = "STARTED_M5" if args.firmware else "STARTED_M4" if args.sensors else "STARTED_M3" if args.drive else ("PASS" if args.check else "LOADED_PAUSED")
        save_json(evidence / "result.json", result)
        print(json.dumps(result), flush=True)
        while not args.check and not args.spawn_only and not stopping:
            check_processes(processes)
            time.sleep(0.3)
    except Exception as exc:
        save_json(evidence / "result.json", {"status": "FAIL", "error": str(exc)})
        raise
    finally:
        for process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
