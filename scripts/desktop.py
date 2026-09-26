"""Supervise a container-only X desktop and localhost-published noVNC endpoint."""
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime, timezone

children = []
stopping = False


def stop(*_):
    global stopping
    stopping = True


for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, stop)

try:
    children.append(subprocess.Popen([
        "Xvfb", ":99", "-screen", "0", "1440x900x24", "-ac", "-nolisten", "tcp",
        "+extension", "GLX", "+render", "-noreset"]))
    for _ in range(100):
        if subprocess.run(["xdpyinfo"], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0:
            break
        if children[0].poll() is not None:
            raise RuntimeError("Xvfb exited")
        time.sleep(0.1)
    else:
        raise RuntimeError("Xvfb did not become ready")
    for command in [
        ["openbox", "--sm-disable"],
        ["x11vnc", "-display", ":99", "-localhost", "-rfbport", "5900",
         "-forever", "-shared", "-nopw", "-quiet"],
        ["websockify", "--web=/usr/share/novnc", "6080", "127.0.0.1:5900"],
    ]:
        children.append(subprocess.Popen(command))
    print("Desktop ready: http://localhost:6080/vnc.html?autoconnect=true&resize=scale", flush=True)
    evidence = os.environ.get("ROVER_EVIDENCE") or (
        "/workspace/evidence/milestone-2/" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-desktop")
    children.append(subprocess.Popen([
        "python3", "scripts/rover_sim.py", "--gui", "--evidence", evidence,
        *(["--drive", "--rviz"] if os.environ.get("ROVER_MILESTONE") in {"3", "4", "5", "6"} else []),
        *(["--sensors"] if os.environ.get("ROVER_MILESTONE") in {"4", "5", "6"} else []),
        *(["--firmware"] if os.environ.get("ROVER_MILESTONE") in {"5", "6"} else []),
        *(["--m6", os.environ.get('ROVER_M6_MODE', 'localization'), '--map',
           os.environ.get('ROVER_MAP', '/workspace/maps/delivery_room_v1/map.yaml')]
          if os.environ.get('ROVER_MILESTONE') == '6' else [])]))
    while not stopping:
        for child in children:
            if child.poll() is not None:
                raise RuntimeError(f"Desktop component exited: {child.args}")
        time.sleep(0.5)
finally:
    for child in reversed(children):
        child.terminate()
    for child in children:
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
