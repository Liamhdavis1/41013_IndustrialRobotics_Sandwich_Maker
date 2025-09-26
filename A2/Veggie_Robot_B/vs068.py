# test_two_links_exact.py — base + shoulder, exact Z placement (robust)

import os
from math import pi
import numpy as np
import trimesh
from spatialmath import SE3
from roboticstoolbox.backends.swift import Swift
from spatialgeometry import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
BASE_FILE     = os.path.join(HERE, "base_vs08_mm.dae")
SHOULDER_FILE = os.path.join(HERE, "l1_shoulder_vs068_mm.dae")

SCALE = 500.0          # your visual scale
ROT   = SE3.Rx(pi/2)   # upright
CLEAR = 0.0005         # 0.5 mm in meters to avoid z-fighting

AZIM  = pi

def z_bounds_after(path, rot: SE3, low_pct=2.0, high_pct=98.0):
    """
    Return robust (zmin, zmax) in METERS after applying 'rot'.
    Uses percentiles to ignore tiny protrusions.
    """
    m = trimesh.load(path, force='mesh')
    m.apply_transform(rot.A)
    z = m.vertices[:, 2]
    zmin = float(np.percentile(z, low_pct))
    zmax = float(np.percentile(z, high_pct))
    return zmin, zmax

def main():
    env = Swift(); env.launch(realtime=True)

    # Base at origin (upright)
    pose_base = ROT * SE3(0, 0, 0)
    base = Mesh(BASE_FILE, pose=pose_base, scale=[SCALE]*3, color=[0.7, 0.7, 0.7, 1])
    env.add(base)

    # Robust Z bounds after same rotation
    _, zmax_base = z_bounds_after(BASE_FILE, ROT, low_pct=2,  high_pct=98)
    zmin_sh,  _  = z_bounds_after(SHOULDER_FILE, ROT, low_pct=2, high_pct=98)

    # delta_z in meters (raw mesh units) -> convert to world (since visuals are scaled)
    dz_world = (zmax_base - zmin_sh) * SCALE + CLEAR

    # IMPORTANT: translate along Z (blue), not Y
    pose_shoulder = ROT * SE3.Ry(AZIM) * SE3(0, dz_world, 0)
    shoulder = Mesh(SHOULDER_FILE, pose=pose_shoulder, scale=[SCALE]*3, color=[0.6, 0.6, 0.6, 1])
    env.add(shoulder)

    print(f"[OK] dz_world={dz_world:.4f} m (robust).")
    env.hold()

if __name__ == "__main__":
    main()
