# @file
# @brief Rough version of Veggie Robot Gamma defined by standard DH parameters with 3D model
# @author Micah Patching
# @date October 04, 2025

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import spatialgeometry as geometry
import time
import os
import numpy as np
from math import pi

# ----- Main Robot Class -----
class VeggieRobotAbb_irb120(DHRobot3D):
    def __init__(self):

        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link3D_names = dict(
            link0='base_link',
            link1='link_1',
            link2='link_2',
            link3='link_3',
            link4='link_4',
            link5='link_5',
            link6='link_6'
        )

        # A joint config and the 3D object transforms to match that config
        qtest = [0.0]*6



        qtest_transforms = [
            spb.transl(0, 0, 0) @ spb.rpy2tr(0, 0, 0, order='xyz'),             # base_link
            spb.transl(0, 0, 0) @ spb.rpy2tr(0, 0, 0, order='xyz'),             # link_1
            spb.transl(0, 0, 0.29) @ spb.rpy2tr(0, 0, 0, order='xyz'),          # link_2
            spb.transl(0, 0, 0.55) @ spb.rpy2tr(0, 0, 0, order='xyz'),          # link_3
            spb.transl(0, 0, 0.62) @ spb.rpy2tr(0, 0, 0, order='xyz'),          # link_4
            spb.transl(0.31, 0, 0.63) @ spb.rpy2tr(0, 0, 0, order='xyz'),       # link_5
            spb.transl(0.38, 0, 0.63) @ spb.rpy2tr(0, 0, 0, order='xyz'),       # link_6
        ]

        # Do i need to use np.clip
        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links,
            link3D_names,
            name='VeggieRobotAbb_irb120',
            link3d_dir=current_path,
            qtest=qtest,
            qtest_transforms=qtest_transforms
        )
        self.q = [0.0] * 6

    def _create_DH(self):
        """
        ABB IRB 120 (3 kg / 580 mm) — Standard DH (not modified).
        Lengths in metres, angles in radians.
        Table used:
        i   a_i (mm)  alpha_i (deg)  d_i (mm)    theta_i note
        1     0          -90           290       θ1
        2   270            0             0       θ2 - 90°
        3    70          -90             0       θ3
        4     0           90           302       θ4
        5     0          -90             0       θ5
        6     0            0            72       θ6 + 180°
        """

        # Joint limits (typical)
        qlim = [
            [np.deg2rad(-165), np.deg2rad(+165)],   # J1
            [np.deg2rad(-110), np.deg2rad(+110)],   # J2
            [np.deg2rad(-110), np.deg2rad(+70)],    # J3
            [np.deg2rad(-160), np.deg2rad(+160)],   # J4
            [np.deg2rad(-120), np.deg2rad(+120)],   # J5
            [np.deg2rad(-400), np.deg2rad(+400)],   # J6
        ]

        # DH lengths (m)
        a     = np.array([0, 270, 70, 0, 0, 0]) / 1000.0
        d     = np.array([290, 0,   0, 302, 0, 72]) / 1000.0

        # Rolls (rad)
        alpha = np.deg2rad([-90, 0, -90, 90, -90, 0])

        # Theta zero-offsets from the table:
        #   θ2 - 90°  → offset[1] = -π/2
        #   θ6 + 180° → offset[5] = +π
        offset = [0, -pi/2, 0, 0, 0, pi]

        links = []
        for i in range(6):
            links.append(
                rtb.RevoluteDH(
                    a=a[i],
                    d=d[i],
                    alpha=alpha[i],
                    offset=offset[i],
                    qlim=qlim[i],
                )
            )

        return links
                        
    def traj_then_teach(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime=True)
        self.q = self._qtest
        self.add_to_env(env)

        q_goal  = [
            self.q[0] +pi,
            self.q[1] +pi,
            self.q[2] +pi,
            self.q[3] +pi,
            self.q[4] +pi,
            self.q[5] +pi,
        ]
        qtraj = rtb.jtraj(self.q, q_goal, 150).q

        fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        fig._add_teach_panel(self, self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)       
            fig.step(0.02)
        fig.hold()
        env.hold()
        time.sleep(3)

        
# ----- Main Entrypoint -----
if __name__ == "__main__":
    r = VeggieRobotAbb_irb120()
    # input("Press Enter to spawn Veggie Robot Gamma")
    r.traj_then_teach()
