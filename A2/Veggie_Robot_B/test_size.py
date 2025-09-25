# test_size.py — quick visual size check for one mesh in Swift

import os, time
from spatialmath import SE3
import swift
from roboticstoolbox.backends.swift import Mesh, Shape  # <-- FIXED import

HERE = os.path.dirname(os.path.abspath(__file__))

# Point to your exported base link
MESH_FILE = os.path.join(HERE, "l1_base_vs08_mm.dae")

# If your mesh is in millimetres, use 0.001. If in metres, use 1.0.
VISUAL_SCALE = 0.001

def main():
    env = swift.Swift()
    env.launch(realtime=True)

    # Reference objects for scale
    env.add(Shape.Box(0.10, 0.10, 0.10, pose=SE3(0.30, 0, 0.05)))   # 10 cm cube
    env.add(Shape.Box(1.00, 1.00, 0.01, pose=SE3(0, 0, -0.005)))    # 1 m floor tile

    # Load your base mesh
    base = Mesh(filename=MESH_FILE)
    base.scale = [VISUAL_SCALE, VISUAL_SCALE, VISUAL_SCALE]
    base.T = SE3(0, 0, 0).A      # place at origin
    env.add(base)

    print(f"Loaded: {os.path.basename(MESH_FILE)}  (visual scale={VISUAL_SCALE})")
    print("Floor tile = 1m, cube = 0.1m for reference")

    env.hold()
    time.sleep(0.5)

if __name__ == "__main__":
    main()
