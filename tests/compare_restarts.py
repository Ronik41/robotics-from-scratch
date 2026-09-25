"""Compare real simulator observations across three disposable containers."""
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
results = [json.loads((root / f"run-{i}/result.json").read_text()) for i in range(1, 4)]
assert all(r["status"] == "PASS" for r in results), "A restart failed acceptance"
assert all(r["models"] == results[0]["models"] for r in results), "World contents changed"
inputs = [json.loads((root / f"run-{i}/inputs.json").read_text()) for i in range(1, 4)]
assert all(i == inputs[0] for i in inputs), "Inputs changed across restarts"
initial_delta = max(abs(a-b) for r in results for a, b in
                    zip(r["initial_pose_xyz_xyzw"], results[0]["initial_pose_xyz_xyzw"]))
settled_delta = max(abs(a-b) for r in results for a, b in
                    zip(r["settled"]["pose"], results[0]["settled"]["pose"]))
assert initial_delta <= 1e-9, f"Initial pose changed: {initial_delta}"
assert settled_delta <= 1e-6, f"Settled pose changed: {settled_delta}"
summary = {"status": "PASS", "fresh_containers": 3,
           "initial_tolerance": 1e-9, "settled_tolerance": 1e-6,
           "max_initial_component_delta": initial_delta,
           "max_settled_component_delta": settled_delta,
           "initial_pose_xyz_xyzw": results[0]["initial_pose_xyz_xyzw"],
           "settled_pose_xyz_xyzw": results[0]["settled"]["pose"],
           "scope": "Repeatable load and two-second passive settling on this image/platform; not cross-platform trajectory determinism."}
(root / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary))
