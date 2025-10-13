import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
from math import pi

class Cobot320(DHRobot3D):

    # scale = {
    #     'link0': 0.01,
    #     'link1': 0.01,
    #     'link2': 0.01,
    #     'link3': 0.01,
    #     'link4': 0.01,
    #     'link5': 0.01,
    #     'link6': 0.01
    # }
    def __init__(self):
        links = self._create_DH()

        link3D_names = dict(
            link0='Cobot320_STL/L1',
            link1='Cobot320_STL/L2',
            link2='Cobot320_STL/L3',
            link3='Cobot320_STL/L4',
            link4='Cobot320_STL/L5',
            link5='Cobot320_STL/L6',
            link6='Cobot320_STL/L7'
        )

        qtest = [0, 0, 0, 0, 0, 0]
        qtest_transforms = [
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2))
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links, link3D_names, name='myCobot', link3d_dir= current_path, qtest=qtest, qtest_transforms=qtest_transforms
        )

        self.q = qtest

    def _create_DH(self):
        links = []
        a = [0, 1.1, 1, 0, 0, 0]
        d = [1.75, 0, 0, 0.72, -0.78, 1.0655]
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

        for q in qtraj:
            self.q = q
            env.step(0.02)

        env.hold()
        time.sleep(3)


if __name__ == "__main__":
    robot = Cobot320()
    robot.test()
