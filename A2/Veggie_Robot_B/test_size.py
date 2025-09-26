import os
from roboticstoolbox.backends.swift import Swift
from spatialgeometry import Mesh, Cuboid
from spatialmath import SE3

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_FILE = os.path.join(HERE, "l1_base_vs08_mm.dae")   # Use your mesh filename here

# -- Try extremely large scaling factors since your mesh is tiny ↓↓↓
VISUAL_SCALE = 1500.0   # Increase or decrease this as needed

def main():
    if not os.path.exists(MESH_FILE):
        print(f"[ERR] Mesh file not found: {MESH_FILE}")
        return
    print(f"[INFO] File exists, size = {os.path.getsize(MESH_FILE)/1e6:.2f} MB")

    env = Swift()
    env.launch(realtime=True)

    # # Reference geometry for scale
    # env.add(Cuboid([1, 1, 0.02], pose=SE3(0, 0, -0.01), color=[0.8,0.8,0.8,1]))  # 1m x 1m floor
    # env.add(Cuboid([0.1, 0.1, 0.1], pose=SE3(0.4, 0, 0.05), color=[0.1,0.6,1,1]))  # 0.1m cube

    # Add your mesh; adjust pose Z if mesh is sunken in floor
    base = Mesh(MESH_FILE, pose=SE3(0, 0, 0.1), scale=[VISUAL_SCALE]*3, color=[0.3,0.3,0.3,1])
    env.add(base)

    print(f"Mesh loaded at scale={VISUAL_SCALE}. Compare it to floor (1m) and blue cube (0.1m).")
    env.hold()

if __name__ == "__main__":
    main()
