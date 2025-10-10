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
            link2='link2',
            link3='link3',
            link4='link4',
            link5='link5',
            link6='link6'
        )
        # Test configuration
        # qtest = [pi, 0, -pi/2, 0, 0, 0]
        qtest = [0, 0, 0, 0, 0, 0]

        # qtest transforms – placeholders for STL alignment
        # You will need to fine-tune these to match your mesh frames.
        # qtest_transforms = [
        #     spb.transl(0, 0, 0),                          # base
        #     spb.transl(0, 0, 0.152),                      # link1
        #     spb.transl(0, -0.078, 0.269) @ spb.trotx(-pi/2),                  # link2
        #     spb.transl(-0.053, -0.07, 0.5528) @ spb.trotx(-pi/2),           # link3
        #     spb.transl(-0.13, 0, 0.383),    # link4
        #     spb.transl(-0.14, 0, 0.249) @ spb.trotx(-pi/2), # link5
        #     spb.transl(-0.21, 0, 0.176) # link6 (tool flange)
        # ]
        # qtest_transforms = [
        #     spb.transl(0, 0, 0),                          # base
        #     spb.transl(0, 0, 0.146968) @ spb.trotz(pi),                      # link1
        #     spb.transl(0, 0.062271, 0.264968) @ spb.trotz(pi),                # link2
        #     spb.transl(0.053091, 0.044067, 0.549595) @ spb.trotz(pi),
        #     spb.transl(0.130547, -0.00214, 0.387046) @ spb.trotz(pi),
        #     spb.transl(0.1317, -0.014134, 0.207184) @ spb.trotz(pi),
        #     spb.transl(0.207821, 0, 0.143596) @ spb.trotz(pi)
        # ]
        #  @ spb.trotz(pi)
        qtest_transforms = [
            spb.transl(0, 0, 0),                          # base
            spb.transl(0, 0, 0),                      # link1
            spb.transl(0, 0, 0),                # link2
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0)
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
        # alpha = [-pi/2, 0, 0, pi/2, -pi/2, 0]
        alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0]
        offset = [0, -1.3849179, 1.3849179, 0, 0, 0]
        # offset = [0, -1.3849179, 1.3849179+0.767945, 0, 0, 0]
        # offset = [0, -1.22173, 1.22173, 0, 0, 0]
        # qlim = [[-2*pi, 2*pi] for _ in range(6)]
        deg2rad = lambda deg: deg * pi/180.0
        qlim = [
            [-pi, pi],  # Joint 1: ±360° → that’s ±π? Actually ±360° = ±2π, but spec says ±360°, so -2π to 2π? Use ±2π if full turn allowed
            [deg2rad(-118), deg2rad(120)],  # Joint 2
            [deg2rad(-225), deg2rad(11)],   # Joint 3
            [-pi, pi],  # Joint 4: ±360° → maybe [-2π, 2π]
            [deg2rad(-97), deg2rad(180)],   # Joint 5
            [-pi, pi]   # Joint 6: ±360° → maybe [-2π, 2π]
        ]

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
        fig = self.plot(self.q)
        input("delay")

        for q in qtraj:
            self.q = q
            env.step(0.02)
            fig.step(0.01)
        time.sleep(3)
        env.hold()

        # q_goal = [0,0,0,0,0,0]
        # q_goal[2] = pi/6   # only move joint 3
        # traj = rtb.jtraj(self.q, q_goal, 30).q
        # for q in traj:
        #     self.q = q
        #     env.step(0.02)
            
    # def test_joint_limits(self):
    #     """
    #     Test each joint across its qlim range to ensure stability and correctness.
    #     """
    #     # import numpy as np

    #     env = swift.Swift()
    #     env.launch(realtime=True)
    #     self.add_to_env(env)
        
    #     # Move each joint one at a time through its limit range
    #     for i in range(self.n):
    #         q_min, q_max = self.links[i].qlim
    #         print(f"Testing joint {i+1}: from {q_min:.2f} to {q_max:.2f} radians")
            
    #         # start at zero pose
    #         q_base = np.zeros(self.n)
            
    #         # make a smooth trajectory for that joint only
    #         q_traj = np.linspace(q_min, q_max, 50)
            
    #         for q_val in q_traj:
    #             q_current = q_base.copy()
    #             q_current[i] = q_val
    #             self.q = q_current
    #             env.step(0.02)
            
    #         # small pause before next joint
    #         input(f"Finished joint {i+1}, press Enter to continue...")

    #     env.hold()


# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = XArm6()
    # r.test()
    r.test_joint_limits()