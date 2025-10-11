# vs068_two_links.py — base + shoulder using DHRobot3D (STL meshes)

import os
from math import pi
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import spatialmath.base as spb
import swift
from ir_support.robots.DHRobot3D import DHRobot3D

VISUAL_SCALE = 500.0  # visual-only scale

class VeggieVS068_2L(DHRobot3D):
    def __init__(self):
        links = self._create_DH()

        # two links we’re testing
        link3D_names = dict(
            link0='base_vs08_mm',          # base STL (no extension)
            link1='l1_shoulder_vs068_mm',  # shoulder STL (no extension)
        )

        # joint test pose (one joint)
        qtest = [0.0]

        # ---------- place meshes with qtest_transforms ----------
        # Use your DH numbers so the base mesh is shifted by:
        #   Tx(-2*a1), Tz(+d1), and no extra rotation.
        a1 = 1.0      # <-- your a1 above (units: m)
        d1 = 0.120    # <-- your d1 above (units: m)

        T_base   = spb.transl(0, 0, 0)          # move base mesh only
        T_shoulder = np.eye(4)                        # shoulder mesh unchanged

        qtest_transforms = [T_base, T_shoulder]
        # --------------------------------------------------------

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

        self.base = SE3()                  # robot at world origin
        self.q = np.deg2rad([0.0])         # start angle

    def _create_DH(self):
        """ One revolute joint for the shoulder (approx VS-068). """
        d1     = 0.120     # m
        a1     = 1.0       # m  (big on purpose, you were experimenting)
        alpha1 = pi        # rad
        qlim1  = np.deg2rad([-170, 170])
        return [rtb.RevoluteDH(d=d1, a=a1, alpha=alpha1, qlim=qlim1)]

    # visual-only scaling
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
        self._rescale_visuals(VISUAL_SCALE)

        # small shoulder motion
        q0 = self.q.copy()
        q1 = np.deg2rad([45])
        for q in rtb.jtraj(q0, q1, 80).q:
            self.q = q
            env.step(0.016)
        env.hold()

if __name__ == "__main__":
    r = VeggieVS068_2L()
    input("Press Enter to test base + shoulder...")
    r.test()
