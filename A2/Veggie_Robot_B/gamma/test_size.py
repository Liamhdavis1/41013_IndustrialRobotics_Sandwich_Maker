# test_gamma_base_only.py
import os, numpy as np, swift
from spatialmath import SE3
import roboticstoolbox as rtb
from ir_support.robots.DHRobot3D import DHRobot3D

class GammaBaseOnly(DHRobot3D):
    def __init__(self):
        # minimal 1-joint robot (locked at 0) so DHRobot3D is happy
        links = [rtb.RevoluteDH(d=0.0, a=0.0, alpha=0.0, qlim=[0.0, 0.0])]

        # supply link0..link1 (we’ll hide link1)
        link3D_names = {"link0": "gamma_base", "link1": "gamma_base"}

        # no transforms (identity)
        qtest = [0.0]
        qtest_transforms = [np.eye(4), np.eye(4)]

        here = os.path.abspath(os.path.dirname(__file__))
        # look in current folder, or ./gamma subfolder
        for cand in (here, os.path.join(here, "gamma")):
            if os.path.exists(os.path.join(cand, "gamma_base.stl")):
                mesh_dir = cand
                break
        else:
            raise FileNotFoundError("gamma_base.stl not found next to this script or in ./gamma")

        super().__init__(
            links,
            link3D_names,
            name="GammaBaseOnly",
            link3d_dir=mesh_dir,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        self.base = SE3()   # world origin
        self.q = [0.0]      # locked joint

    def show(self):
        env = swift.Swift(); env.launch(realtime=True)
        self.add_to_env(env)
        # hide dummy link1 so only the base is visible
        try:
            self._link3d_models["link1"].visible = False
        except KeyError:
            pass
        env.hold()

if __name__ == "__main__":
    GammaBaseOnly().show()
