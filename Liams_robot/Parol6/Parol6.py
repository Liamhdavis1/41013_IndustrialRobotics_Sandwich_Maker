import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
from math import pi
import matplotlib.pyplot as plt
import numpy as np

class Parol6(DHRobot3D):

    def __init__(self):
        links = self._create_DH()

        link3D_names = dict(
            link0='Parol6_L1',
            link1='Parol6_L2',
            link2='Parol6_L3',
            link3='Parol6_L4',
            link4='Parol6_L5',
            link5='Parol6_L6'
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
            links, link3D_names, name='Parol6', link3d_dir= current_path, qtest=qtest, qtest_transforms=qtest_transforms
        )

        self.q = qtest

    def _create_DH(self):
        
        links = []

        a = [23.42, 180, -43.50, 0, 0, -45.25]
        d = [110.50, 0, 0, -176.35, 0, -62.8]
        alpha = [-pi/2, pi, pi/2, -pi/2, pi/2, pi]
        offset = [0, -pi/2, pi, 0, 0, pi]     
        
        qlim = [
            [-123.046875 * pi / 180, 123.046875 * pi / 180],  # J1
            [-145.0088 * pi / 180, -3.375 * pi / 180],        # J2
            [107.866 * pi / 180, 287.8675 * pi / 180],        # J3
            [-105.46975 * pi / 180, 105.46975 * pi / 180],    # J4
            [-90 * pi / 180, 90 * pi / 180],                  # J5
            [0 * pi / 180, 360 * pi / 180],                   # J6
        ]

        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
            links.append(link)
        return links

    def test(self):

        fig = Parol6.plot(Parol6.q, limits = [-10, 10, -10, 10, 0, 10], block=False)

        plt.draw()
        # env = swift.Swift()
        # env.launch(realtime=True)

        # self.q = self._qtest
        # self.add_to_env(env)

        # q_goal = [self.q[i] - pi / 3 for i in range(len(self.q))]
        # qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        # input("delay")

        # for q in qtraj:
        #     self.q = q
        #     env.step(0.02)
        #     fig.step(0.01)

        # env.hold()
        time.sleep(3)


if __name__ == "__main__":
    robot = Parol6()
    robot.test()
