import numpy as np
import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3 
from spatialgeometry import Mesh
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
import sys
# sys.path.append(os.path.dirname(__file__))
from Lilys_robot.XArm6 import XArm6
from Liams_robot.Cobot320 import Cobot320
# from Micahs_robot.A2.Veggie_Robot_B.abb_irb_120 import VeggieRobotAbb_irb120
from ir_support import UR3
from math import pi
from Micahs_robot.abb_irb_120 import abb_irb_120
# from ir_support.robots import XArm6


env = swift.Swift()
env.launch(realtime=True)

UR3 = UR3()
XArm = XArm6()
irb = abb_irb_120()
# Cobot = Cobot320()
# IRB = VeggieRobotAbb_irb120() 

# UR3.base = SE3(0.75,0.5,1)
# UR3.add_to_env(env)

XArm.base = SE3(0.95,0.35,0.8)
XArm.add_to_env(env)

irb.base = SE3(0.55,0.5,0.8)
irb.add_to_env(env)
# Cobot.base = SE3(-0.75,0.5,1)
# Cobot.add_to_env(env)

# IRB.base = SE3(-0.75,0.5,1)
# IRB.add_to_env(env)

current_path = os.path.abspath(os.path.dirname(__file__))

ENV = {
    "glass": {
        "path": os.path.join(current_path, "env", "glass.stl"),
        "scale": (0.8, 0.8, 0.8),
        "color": [0.6, 0.6, 0.6, 0.4]
    },
    "bench": {
        "path": os.path.join(current_path, "env", "bench.stl"),
        "scale": (0.8, 0.8, 0.8),
        "color": [0.6, 0.6, 0.6, 1]
    }
}

spawned_meshes = {}

def spawn_obj(env, name, pose):
    info = ENV[name]
    mesh = Mesh(
        filename=info["path"],
        pose=pose,
        scale=info["scale"],
        color=info["color"]
    )
    env.add(mesh)
    spawned_meshes[f"{name}_{len(spawned_meshes)}"] = mesh
    return mesh

# --- Example spawns ---
bench = spawn_obj(env, "bench", SE3(0, 0, 0))
glass =spawn_obj(env, "glass", SE3(0, 0, 0))


steps = 50

q0 = np.zeros(6)
q_pick = XArm.ikine_LM(SE3(1.15,-0.34,0.88), q0=q0).q
for q in rtb.jtraj(XArm.q, q_pick, steps).q:
    XArm.q = q
    env.step(0.05)



env.hold()