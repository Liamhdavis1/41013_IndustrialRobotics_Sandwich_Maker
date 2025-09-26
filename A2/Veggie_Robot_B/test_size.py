# test_size.py — minimal: show just the base link at origin

import os
from roboticstoolbox.backends.swift import Swift
from spatialgeometry import Mesh
from spatialmath import SE3
from math import pi
import trimesh


HERE = os.path.dirname(os.path.abspath(__file__))
MESH_FILE = os.path.join(HERE, "base_vs08_mm.dae")



def compute_ground_lift(path, rot_se3, scale):
    m = trimesh.load(path, force='mesh')
    m.apply_transform(rot_se3.A)        # apply your rotation
    minz = m.bounds[0, 2]               # lowest vertex z (meters)
    return (-minz) * scale              # convert to your visual scale


def main():
    if not os.path.exists(MESH_FILE):
        print(f"[ERR] Mesh file not found: {MESH_FILE}")
        return

    env = Swift()
    env.launch(realtime=True)

    # “Second smallest” from your ladder test ≈ 500x
    SCALE = 500.0
    ROT   = SE3.Rx(pi/2) 
    LIFT  = compute_ground_lift(MESH_FILE, ROT, SCALE)                                                  
    pose = ROT * SE3(0, 0, 0)
    base = Mesh(MESH_FILE, pose=pose, scale=[SCALE]*3, color=[0.7,0.7,0.7,1])
    env.add(base)
    env.add(base)

    env.hold()

if __name__ == "__main__":
    main()
