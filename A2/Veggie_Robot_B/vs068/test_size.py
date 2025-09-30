# test_shoulder_only.py — show just the shoulder STL at origin (Swift)

import os
import numpy as np
import swift
from spatialmath import SE3
import roboticstoolbox as rtb
from ir_support.robots.DHRobot3D import DHRobot3D

VISUAL_SCALE = 500.0  # if your STL is in mm

class ShoulderOnly(DHRobot3D):
    def __init__(self):
        # 1 dummy revolute joint (no movement)
        links = [rtb.RevoluteDH(d=0.0, a=0.0, alpha=0.0, qlim=[0.0, 0.0])]

        # DHRobot3D expects link0 and link1
        # - link0: dummy (we'll hide it)
        # - link1: your shoulder mesh to test
        link3D_names = {
            "link0": "l1_shoulder_vs068_mm",   # dummy, will be hidden
            "link1": "l1_shoulder_vs068_mm",   # the one we want to see
        }

        # Identity transforms → no offsets
        qtest = [0.0]
        qtest_transforms = [np.eye(4), np.eye(4)]

        here = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links,
            link3D_names,
            name="ShoulderOnly",
            link3d_dir=here,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        self.base = SE3()  # world origin
        self.q = [0.0]     # locked

    def show(self):
        env = swift.Swift(); env.launch(realtime=True)
        self.add_to_env(env)

        # visual-only scaling
        for m in getattr(self, "_link3d_models", {}).values():
            try:
                m.scale = [VISUAL_SCALE, VISUAL_SCALE, VISUAL_SCALE]
            except Exception:
                pass

        # Hide the dummy link0 so only the shoulder is visible
        try:
            self._link3d_models["link0"].visible = False
        except Exception:
            pass

        env.hold()

if __name__ == "__main__":
    ShoulderOnly().show()
