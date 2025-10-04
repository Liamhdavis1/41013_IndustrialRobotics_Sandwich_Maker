# test_gamma_mesh.py — spawn ONE Cyton Gamma mesh at origin (Swift)

import os, numpy as np, swift
from spatialmath import SE3
import roboticstoolbox as rtb
from ir_support.robots.DHRobot3D import DHRobot3D

TEST_MESH = "Elbow_Yaw"   # e.g. gamma_base, Shoulder_Pitch, Elbow_Roll, ...

def find_mesh_dir(here, fname):
    for cand in (here, os.path.join(here, "gamma")):
        fpath = os.path.join(cand, fname)
        if os.path.exists(fpath):
            return cand, fpath
    raise FileNotFoundError(f"{fname} not found in {here} or ./gamma")

class SingleMeshTester(DHRobot3D):
    def __init__(self, mesh_name: str):
        # minimal 1-joint robot (dummy)
        links = [rtb.RevoluteDH(d=0.0, a=0.0, alpha=0.0, qlim=[0.0, 0.0])]

        # DHRobot3D expects link0 and link1 for 1 DOF
        link3D_names = {"link0": mesh_name, "link1": mesh_name}

        qtest = [0.0]
        qtest_transforms = [np.eye(4), np.eye(4)]

        here = os.path.abspath(os.path.dirname(__file__))
        mesh_dir, fpath = find_mesh_dir(here, mesh_name + ".stl")
        print("[Using mesh]", fpath)

        super().__init__(
            links,
            link3D_names,
            name=f"Test_{mesh_name}",
            link3d_dir=mesh_dir,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        self.base = SE3()
        self.q = [0.0]

    def show(self):
        env = swift.Swift(); env.launch(realtime=True)
        self.add_to_env(env)
        # hide duplicate link1 so only one mesh is visible
        for dict_name in ("_link3d_models", "_link3d_shapes", "_link3D"):
            d = getattr(self, dict_name, None)
            if isinstance(d, dict) and "link1" in d:
                try:
                    d["link1"].visible = False
                except Exception:
                    pass
        env.hold()

if __name__ == "__main__":
    SingleMeshTester(TEST_MESH).show()
