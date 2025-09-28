# vs068_two_links.py — base + shoulder using DHRobot3D (STL meshes)

import os
from math import pi
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import spatialmath.base as spb
import swift
from ir_support.robots.DHRobot3D import DHRobot3D
import numpy as np


# If your STL was exported in mm, you may still need to scale visuals.
# Tweak this until it looks right in Swift (1.0, 0.001, 100, 500, etc.).
VISUAL_SCALE = 500.0

class VeggieVS068_2L(DHRobot3D):
    def __init__(self):
        links = self._create_DH()

        # Only the two links we’re testing
        link3D_names = dict(
            link0='base_vs08_mm',          # base STL filename (no extension)
            link1='l1_shoulder_vs068_mm',  # shoulder STL filename (no extension)
        )

        # Joint test pose (one joint)
        qtest = [0.0]

        # Since you baked the axes in FreeCAD, meshes should already be upright.
        I = np.eye(4)
        qtest_transforms = [I, I]  # base, shoulder

        here = os.path.abspath(os.path.dirname(__file__))
        print("[Meshes]")
        for k, v in link3D_names.items():
            print(f"  {k}: {os.path.join(here, v + '.stl')}")

        super().__init__(
            links,
            link3D_names,
            name='VS068_TwoLinks',
            link3d_dir=here,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        # Put robot base at world origin
        self.base = SE3()

        # Comfortable start angle
        self.q = np.deg2rad([0.0])

    def _create_DH(self):
        """
        One revolute joint for the shoulder.
        Use the same numbers you used before (approx VS-068).
        """
        d1     = 0.120     # m
        a1     = 0.060     # m
        alpha1 = pi/2
        qlim1  = np.deg2rad([-170, 170])

        return [rtb.RevoluteDH(d=d1, a=a1, alpha=alpha1, qlim=qlim1)]

    # Visual-only scaling (does not change kinematics)
    def _rescale_visuals(self, s=VISUAL_SCALE):
        for attr in ("_link3d_models", "_link3d_shapes", "_link3D"):
            obj = getattr(self, attr, None)
            if isinstance(obj, dict):
                for m in obj.values():
                    try:
                        m.scale = [s, s, s]
                    except Exception:
                        pass

    def test(self):
        env = swift.Swift(); env.launch(realtime=True)
        self.add_to_env(env)
        self._rescale_visuals(VISUAL_SCALE)   # make the STL visible at the right size

        # Small shoulder motion so you can see it move
        q0 = self.q.copy()
        q1 = np.deg2rad([45])  # 45° about Z of the shoulder (as baked in FreeCAD)
        for q in rtb.jtraj(q0, q1, 80).q:
            self.q = q
            env.step(0.016)

        env.hold()


if __name__ == "__main__":
    r = VeggieVS068_2L()
    input("Press Enter to test base + shoulder...")
    r.test()
