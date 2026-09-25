"""Exercise separate spawn and reject a duplicate in a real paused Gazebo world."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rover_sim import Node, Empty, Scene, WORLD, request, wait_for_scene

evidence = Path(sys.argv[1]).resolve()
evidence.mkdir(parents=True, exist_ok=True)
with (evidence / "gazebo.log").open("w") as log:
    server = subprocess.Popen(["gz", "sim", "-s", "-v", "3", "--seed", "42", str(WORLD)],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        node = Node()
        wait_for_scene(node, [server])
        for name, expected_code in [("first", 0), ("duplicate", 1)]:
            with (evidence / f"{name}.log").open("w") as output:
                completed = subprocess.run([
                    "python3", "scripts/rover_sim.py", "--spawn-only",
                    "--evidence", str(evidence / name)], stdout=output,
                    stderr=subprocess.STDOUT, timeout=60)
            assert completed.returncode == expected_code, f"{name}: unexpected exit code"
        duplicate = json.loads((evidence / "duplicate/result.json").read_text())
        assert "already exists" in duplicate["error"], duplicate
        scene = request(node, "/scene/info", Empty(), Scene)
        assert len([m for m in scene.model if m.name.startswith("delivery_rover")]) == 1
        summary = {"status": "PASS", "first_spawn": "LOADED_PAUSED",
                   "duplicate_rejected": True, "rover_count": 1}
        (evidence / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary))
    finally:
        try:
            os.killpg(server.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL)
            server.wait()
