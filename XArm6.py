import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import os
import time
from math import pi

class XArm6(DHRobot3D):
    def __init__(self):
        """
        uFactory xArm6 robot, DH model + 3D visual.
        """
        links = self._create_DH()

        # STL/visual names (placeholders)
        link3D_names = dict(
            link0='base0',
            link1='link1',
            link2='link2z2',
            link3='link3z2',
            link4='link4',
            link5='link5z',
            link6='link6'
        )
        # Test configuration
        qtest = [0, 0, 0, 0, 0, 0]

        # qtest transforms – placeholders for STL alignment
        # You will need to fine-tune these to match your mesh frames.
        qtest_transforms = [
            spb.transl(0, 0, 0),                          # base
            spb.transl(0, 0, 0.152),                      # link1
            spb.transl(0, -0.078, 0.269) @ spb.trotx(-pi/2),                  # link2
            spb.transl(-0.053, -0.07, 0.5528) @ spb.trotx(-pi/2),           # link3
            spb.transl(-0.13, 0, 0.383),    # link4
            spb.transl(-0.14, 0, 0.249) @ spb.trotx(-pi/2), # link5
            spb.transl(-0.21, 0, 0.176) # link6 (tool flange)
        ]


        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(
            links,
            link3D_names,
            name='XArm6',
            link3d_dir=current_path,
            qtest=qtest,
            qtest_transforms=qtest_transforms
        )

        self.q = qtest

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        DH parameters for uFactory xArm6 (standard DH, meters).
        """

        # Parameters (converted from mm to meters)
        a = [0, 0.28948866, 0.0775, 0, 0.076, 0]
        d = [0.267, 0, 0, 0.3425, 0, 0.097]
        alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0]
        offset = [0, -1.3849179, 1.3849179, 0, 0, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]

        # Create the links
        links = []
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i],
                                offset=offset[i], qlim=qlim[i])
            links.append(link)

        return links


    # -----------------------------------------------------------------------------------#
    def test(self):
    #     """
    #     Test the class by adding 3d objects into a new Swift window and do a simple movement
    #     """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest
        # self.base = SE3(0.5,0.5,0)
        self.add_to_env(env)

        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        input("delay")

        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        time.sleep(3)
        env.hold()

        # q_goal = [0,0,0,0,0,0]
        # q_goal[2] = pi/6   # only move joint 3
        # traj = rtb.jtraj(self.q, q_goal, 30).q
        # for q in traj:
        #     self.q = q
        #     env.step(0.02)
            


# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = XArm6()
    r.test()