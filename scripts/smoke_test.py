"""Exercise the packaged Gazebo example and a real ROS clock subscription."""

import glob
import hashlib
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    evidence = Path(sys.argv[1])
    evidence.mkdir(parents=True, exist_ok=True)
    shutil.copyfile("/opt/installed-packages.tsv", evidence / "installed-packages.tsv")
    subprocess.run(["gz", "sim", "--versions"], check=True)
    examples = glob.glob("/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim*/worlds/shapes.sdf")
    if len(examples) != 1:
        raise RuntimeError(f"Expected one packaged shapes world, found {examples}")
    world = Path(examples[0])
    world_xml = ET.parse(world).getroot().find("world")
    world_name = world_xml.attrib["name"]
    models = [model.attrib["name"] for model in world_xml.findall("model")]
    metadata = {
        "example": str(world),
        "sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
        "world": world_name,
        "models": models,
    }
    (evidence / "world.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata), flush=True)
    # Bridge only Gazebo -> ROS. A wall-time timeout must work even if /clock stops.
    topic = f"/world/{world_name}/clock"
    processes = []
    logs = []
    node = None
    rclpy.init()
    try:
        for name, command in [
            ("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                        f"{topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        "--ros-args", "-r", f"{topic}:=/clock"]),
            ("gazebo", ["gz", "sim", "-s", "-r", "-v", "3", str(world)]),
        ]:
            log = (evidence / f"{name}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        node = rclpy.create_node("milestone_one_clock_check")
        samples = []

        def receive_clock(message):
            samples.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec)

        subscription = node.create_subscription(Clock, "/clock", receive_clock,
                                                qos_profile_sensor_data)
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f"Child exited unexpectedly: {process.args} ({process.returncode})")
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(samples) >= 10 and samples[-1] - samples[0] >= 100_000_000:
                break
        else:
            raise RuntimeError(f"No advancing ROS /clock within 45 seconds; {len(samples)} samples")
        if any(b < a for a, b in zip(samples, samples[1:])):
            raise RuntimeError("Simulation clock moved backwards")
        result = {"status": "PASS", "clock_messages": len(samples),
                  "first_sim_time_ns": samples[0], "last_sim_time_ns": samples[-1],
                  "advance_ns": samples[-1] - samples[0]}
        (evidence / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        # Stop process groups, including the children created by `ros2 run` and `gz`.
        import os
        for process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
