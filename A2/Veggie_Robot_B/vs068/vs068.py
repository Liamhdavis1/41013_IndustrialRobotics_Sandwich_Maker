import os
from math import pi
import numpy as np
import trimesh
from spatialmath import SE3
from roboticstoolbox.backends.swift import Swift
from spatialgeometry import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))

BASE_FILE      = os.path.join(HERE, "base_vs08_mm.dae")
SHOULDER_FILE  = os.path.join(HERE, "l1_shoulder_vs068_mm.dae")
UPPERARM_FILE  = os.path.join(HERE, "l2_upperarm_vs068_mm.dae")

SCALE = 500.0
ROT   = SE3.Rx(pi/2)    # your “upright”
CLEAR = 0.0005          # 0.5 mm (meters) to avoid z-fighting

# Joint “angles” for visuals (spin about the intended axes in your chosen frame)
AZIM1 = pi              # J1 spin about Y (you already used this)
ELBOW = 0.0             # J2 angle about X (adjust to pose upper arm)

def z_bounds_after(path, rot: SE3, low_pct=2.0, high_pct=98.0):
    """Robust (zmin, zmax) in meters after applying 'rot' to the raw mesh."""
    m = trimesh.load(path, force='mesh')
    m.apply_transform(rot.A)
    z = m.vertices[:, 2]
    zmin = float(np.percentile(z, low_pct))
    zmax = float(np.percentile(z, high_pct))
    return zmin, zmax

def main():
    env = Swift(); env.launch(realtime=True)

    # --- L0: base at origin ---
    T_base = ROT * SE3(0, 0, 0)
    base = Mesh(BASE_FILE, pose=T_base, scale=[SCALE]*3, color=[0.72,0.72,0.72,1])
    env.add(base)

    # --- L1: shoulder above base ---
    R_sh = ROT * SE3.Ry(AZIM1)
    _, zmax_base = z_bounds_after(BASE_FILE, ROT)
    zmin_sh, _   = z_bounds_after(SHOULDER_FILE, R_sh)
    dz1_world    = (zmax_base - zmin_sh) * SCALE + CLEAR

    T_shoulder = R_sh * SE3(0, dz1_world, 0)   # stack along +Y in your scheme
    sh = Mesh(SHOULDER_FILE, pose=T_shoulder, scale=[SCALE]*3, color=[0.64,0.64,0.64,1])
    env.add(sh)

    # --- L2: upper arm above shoulder ---
    R_up = ROT * SE3.Ry(AZIM1) * SE3.Rx(ELBOW)
    _, zmax_sh  = z_bounds_after(SHOULDER_FILE, R_sh)
    zmin_up, _  = z_bounds_after(UPPERARM_FILE,  R_up)
    dz2_world   = (zmax_sh - zmin_up) * SCALE + CLEAR

    # cumulative Y-offset: shoulder lift + upper-arm lift
    T_upper = R_up * SE3(0, dz1_world + dz2_world, 0)
    ua = Mesh(UPPERARM_FILE, pose=T_upper, scale=[SCALE]*3, color=[0.58,0.58,0.58,1])
    env.add(ua)

    print(f"[OK] dz1={dz1_world:.4f} m, dz2={dz2_world:.4f} m  (visual scale={SCALE})")
    env.hold()

if __name__ == "__main__":
    main()
