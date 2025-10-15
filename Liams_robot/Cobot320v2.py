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
            link0='Cobot320_stlv2/L1',
            link1='Cobot320_stlv2/L2',
            link2='Cobot320_stlv2/L3',
            link3='Cobot320_stlv2/L4',
            link4='Cobot320_stlv2/L5',
            link5='Cobot320_stlv2/L6',
            link6='Cobot320_stlv2/L7'
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
        scale = 0.1  # slightly smaller than 0.2 to fix subtle drift

        # scaled link parameters
        a_unscaled = [0, 1.1, 1.0, 0, 0, 0]
        d_unscaled = [1.75, 0, 0, 0.72, -0.78, 1.0655]
        a = [x * scale for x in a_unscaled]
        d = [x * scale for x in d_unscaled]


        # sometimes the last alpha sign causes orientation drift
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
<<<<<<< HEAD
        # alpha = [-pi/2, pi/2, 0, pi/2, pi/2, -pi]

        # offset tweaks: shifting the base and wrist alignment
        offset = [0, pi/2, 0, -pi/2, pi, 0]
        # offset = [pi/2, pi/2, 0, -pi/2, pi, -pi/2]
        
        
        qlim = [[-pi / 2, pi / 2] for _ in range(6)]
=======

        # offset tweaks: shifting the base and wrist alignment
        offset = [0, pi/2, 0, -pi/2, pi, 0]
>>>>>>> faf16a07a1dcdf9ce5b033608482c512a4604a2a

        qlim = [[-pi, pi] for _ in range(6)]

    # def _create_DH(self):
    #     links = []
    #     a = [0, 0.22, 0.2, 0, 0, 0]
    #     d = [0.338, 0, 0, 0.144, -0.156, 0.2131]
    #     alpha = [pi/2, 0, 0, pi/2, pi/2, -pi]
    #     offset = [0, pi/2, 0, -pi/2, pi, -pi/2]
    #     qlim = [[-pi / 2, pi / 2] for _ in range(6)]

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
