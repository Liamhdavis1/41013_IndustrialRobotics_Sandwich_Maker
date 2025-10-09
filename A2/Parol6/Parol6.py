import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
import time
import os
from math import pi
from ir_support.robots.DHRobot3D import DHRobot3D

class PAROL6(DHRobot3D):
    def __init__(self, baseTr=SE3()):
        links = self._create_DH()

        # STL file names must match those in your directory (case sensitive, no '.STL' extension)
        link3D_names = dict(
            link0='meshes/base_link',
            link1='meshes/L1',
            link2='meshes/L2',
            link3='meshes/L3',
            link4='meshes/L4',
            link5='meshes/L5',
            link6='meshes/L6'
        )

        qtest = [0, 0, 0, 0, 0, 0]
        qtest_transforms = [
            spb.transl(0, 0, 0),                              # base_link
            spb.transl(0, 0, 0),                              # L1
            spb.transl(0.0234207210610375, 0, 0.1105) @ spb.trotx(-1.5707963267949),  # L2 (z=+1)
            spb.transl(0, -0.18, 0) @ spb.trotz(3.1416) @ spb.trotx(-1.5708),         # L3 (z=-1)
            spb.transl(0.0435, 0, 0) @ spb.trotx(1.5707963267949) @ spb.trotz(3.14159265358979),  # L4 (z=-1)
            spb.transl(0, 0, -0.17635) @ spb.trotx(-1.5708),                         # L5 (z=-1)
            spb.transl(0, 0, 0) @ spb.trotx(1.5708)                                  # L6 (z=-1)
        ]



        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(links, link3D_names, name='PAROL6', link3d_dir=current_path,
                         qtest=qtest, qtest_transforms=qtest_transforms)

        self.base = baseTr
        self.workspace = [-0.8, 0.8, -0.8, 0.8, -0.01, 0.5]
        self.toolFilename = 'L6'  # Usually matches last STL for the wrist
        self.toolTr = SE3(0, 0, 0)  # Use SE3(0, 0, 0) if tool origin at the end of L6
        self.q = qtest

    def _create_DH(self):
        links = []
        # All angles in radians; all distances in meters. Offsets as per PAROL6 kinematics.
        links.append(rtb.RevoluteDH(d=0.1105,    a=0.02342,   alpha=-pi/2,           qlim=[-123.05*pi/180, 123.05*pi/180]))
        links.append(rtb.RevoluteDH(d=0,         a=0.18,      alpha=pi,     offset=-pi/2, qlim=[-145.01*pi/180, -3.38*pi/180]))
        links.append(rtb.RevoluteDH(d=0,         a=-0.0435,   alpha=pi/2,  offset=pi,   qlim=[107.87*pi/180, 287.87*pi/180]))
        links.append(rtb.RevoluteDH(d=-0.17635,  a=0,         alpha=-pi/2,           qlim=[-105.47*pi/180, 105.47*pi/180]))
        links.append(rtb.RevoluteDH(d=0,         a=0,         alpha=pi/2,            qlim=[-90*pi/180, 90*pi/180]))
        links.append(rtb.RevoluteDH(d=-0.0628,   a=-0.04525,  alpha=pi,     offset=pi,   qlim=[0, 2*pi]))
        return links

    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)
        self.add_to_env(env)
        env.hold()

if __name__ == "__main__":
    r = PAROL6()
    # input("Press Enter to test movement of PAROL6...")
    r.test()
