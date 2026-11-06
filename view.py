"""Watch the slider-crank mechanism in the MuJoCo viewer (all three variants start turning at once).

Usage:  uv run python view.py [--rpm 60]
The default speed is low so the motion can be followed in real time
(3000 rpm would be 50 revolutions per second).
"""

import argparse
import math
import time

import mujoco
import mujoco.viewer

from equivalent_system import ConRod
from slider_crank import build_model, load_model

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("--rpm", type=float, default=60.0, help="crank speed in rpm (default 60)")
args = ap.parse_args()

omega = args.rpm * 2 * math.pi / 60
m = load_model(build_model(ConRod.from_onshape(), omega, gravity=False))
d = mujoco.MjData(m)
d.ctrl[:] = omega  # all three motors

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        start = time.time()
        for _ in range(round(0.01 / m.opt.timestep)):  # 10 ms of simulation time
            mujoco.mj_step(m, d)
        viewer.sync()
        time.sleep(max(0.0, 0.01 - (time.time() - start)))
