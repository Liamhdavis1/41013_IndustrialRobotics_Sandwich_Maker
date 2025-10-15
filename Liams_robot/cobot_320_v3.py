import numpy as np
import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
from math import pi

class Cobot320(DHRobot3D):

    def __init__(self):
        links = self._create_DH()

        link3D_names = dict(
            link0='Cobot320v3/L1',
            link1='Cobot320v3/L2',
            link2='Cobot320v3/L3',
            link3='Cobot320v3/L4',
            link4='Cobot320v3/L5',
            link5='Cobot320v3/L6',
            link6='Cobot320v3/L7'
        )

        qtest = [0, 0, 0, 0, 0, 0]
        # qtest_transforms = [
        #     spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
        #     spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
        #     spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
        #     spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
        #     spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
        #     spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
        #     spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2))
        # ]

        qtest_transforms = [
            spb.transl(0, 0, 0),                     
            spb.transl(0, 0, 0),                
            spb.transl(0, 0, 0),                
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0)
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links, link3D_names, name='myCobot', link3d_dir= current_path, qtest=qtest, qtest_transforms=qtest_transforms
        )

        self.q = qtest

    def _create_DH(self):
        links = []

        # DH parameters
        a = [0, 0.135, 0.120, 0, 0, 0]
        d = [0.1739, 0.08878, -0.08878, 0.08878, 0.095, 0.0655]
        alpha = [pi/2, 0, 0, pi/2, pi/2, pi/2]
        offset = [0, pi/2, 0, pi/2, pi, 0]
        qlim = [
            [np.deg2rad(-165), np.deg2rad(165)],  # Joint 1
            [np.deg2rad(-165), np.deg2rad(165)],  # Joint 2
            [np.deg2rad(-165), np.deg2rad(165)],  # Joint 3
            [np.deg2rad(-165), np.deg2rad(165)],  # Joint 4
            [np.deg2rad(-165), np.deg2rad(165)],  # Joint 5
            [np.deg2rad(-175), np.deg2rad(175)]   # Joint 6
        ]

    # Build revolute joints
    # def _create_DH(self):
    #     links = []
        a = [0, 0.11, 0.1, 0, 0, 0]
        d = [0.175, 0, 0, 0.072, -0.078, 0.10655]
        alpha = [pi/2, 0, 0, pi/2, pi/2, -pi]
        offset = [0, pi/2, 0, -pi/2, pi, -pi/2]
        qlim = [[-pi / 2, pi / 2] for _ in range(6)]

        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
            links.append(link)
        return links

    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)

        self.q = self._qtest
        self.add_to_env(env)

        q_goal = [self.q[i] - pi / 3 for i in range(len(self.q))]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        fig = self.plot(self.q)
        input("delay")

        for q in qtraj:
            self.q = q
            env.step(0.02)
            fig.step(0.01)

        env.hold()
        time.sleep(3)


if __name__ == "__main__":
    robot = Cobot320()
    robot.test()
