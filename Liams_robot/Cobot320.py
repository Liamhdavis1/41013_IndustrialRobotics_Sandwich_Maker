import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
from math import pi
from math import pi, radians as deg2rad

class Cobot320(DHRobot3D):
    def __init__(self):
        links = self._create_DH()

        link3D_names = dict(
            link0='Cobot320STL/J1',
            link1='Cobot320STL/J2',
            link2='Cobot320STL/J3',
            link3='Cobot320STL/J4',
            link4='Cobot320STL/J5',
            link5='Cobot320STL/J6',
            link6='Cobot320STL/J7'
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
        a = [0, 0.24, 0.215, 0, 0, 0]
        d = [0.315, 0, 0, 0.156, -0.1695, 0.10655]
        alpha = [pi/2, 0, 0, pi/2, pi/2, -pi]
        offset = [0,pi/2, 0, -pi/2, pi, -pi/2]

        # qlim = [[-pi, pi] for _ in range(6)]
        # for i in range(6):
        #     link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
        #     links.append(link)
        # return links
    
        # qlim = [
        #     [-pi, pi],
        #     [deg2rad(-180), deg2rad(0)],
        #     [deg2rad(-165), deg2rad(165)],  # Joint 3: ±165°
        #     [deg2rad(-165), deg2rad(165)],  # Joint 4: ±165°
        #     [deg2rad(-165), deg2rad(165)],  # Joint 5: ±165°
        #     [-pi, pi]
        # ]
        qlim = [ 
            [-2*pi, 2*pi],    # 1st revolute
            [-pi, pi/2],    # 2nd revolute
            [-pi, pi],          # 3rd revolute (LIMITED: only positive, elbow up)
            [-2*pi, 2*pi],    # 4th revolute
            [-2*pi, 2*pi],    # 5th revolute
            [-2*pi, 2*pi]     # 6th revolute
        ]
        links = []
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

        for q in qtraj:
            self.q = q
            env.step(0.02)
            fig.step(0.01)
            
        env.hold()
        time.sleep(3)


if __name__ == "__main__":
    robot = Cobot320()
    robot.test()