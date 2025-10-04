## @file
#  @brief Rough version of Veggie Robot Gamma defined by standard DH parameters with 3D model
#  @author [Your Name]
#  @date October 04, 2025

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
import numpy as np
import spatialgeometry as geometry

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class VeggieRobotGamma(DHRobot3D):
    def __init__(self):
        """
        Veggie Robot Gamma.
        See the use of `LinearUR3` and base class `DHRobot3D`
        """
        # DH links (using dummy parameters; update with actual specs if available)
        links = self._create_DH()

        # Names of the robot link files in the directory (without extensions)
        link3D_names = dict(
            link0='gamma_base',
            link1='Shoulder_Pitch',
            link2='Shoulder_Roll',
            link3='Elbow_Pitch',
            link4='Elbow_Roll',
            link5='Elbow_Yaw',  # Will prefer .stl if available, then .dae
            link6='Wrist_Pitch',
            link7='Wrist_Roll'
        )

        # A joint config and the 3D object transforms to match that config
        # Note: These are placeholders; adjust based on your model's actual alignments
        qtest = [0] * 7  # Neutral pose for calibration
        qtest_transforms = [
            spb.transl(0, 0, 0),  # link0 (base)
            spb.transl(0, 0, 0.1),  # link1 (adjust offsets as needed)
            spb.transl(0, 0, 0.2),
            spb.transl(0, 0, 0.3),
            spb.transl(0, 0, 0.4),
            spb.transl(0, 0, 0.5),
            spb.transl(0, 0, 0.6),
            spb.transl(0, 0, 0.7)
        ]  # Placeholder transforms to space out links; replace with accurate ones

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name='VeggieRobotGamma', link3d_dir=current_path, qtest=qtest, qtest_transforms=qtest_transforms)

        # Optional base rotation if needed (adjust as per your setup)
        # self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)

        # Set initial joint configuration
        self.q = [0.0] * 7

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model (dummy placeholders; update with real parameters)
        """
        links = []
        for _ in range(7):
            # Dummy revolute joint: zero offset/length, full range
            links.append(rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-pi, pi]))
        return links

    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3D objects into a new Swift window
        """
        env = swift.Swift()
        env.launch(realtime=True)
        self.add_to_env(env)
        env.hold()

# -----------------------------------------------------------------------------------#
class SingleMeshTester:
    """
    Class to test a single mesh file in Swift
    """
    def __init__(self, file_name):
        """
        :param file_name: The file name with extension (e.g., 'gamma_base.stl')
        """
        current_path = os.path.abspath(os.path.dirname(__file__))
        self.mesh_path = os.path.join(current_path, file_name)
        if not os.path.exists(self.mesh_path):
            print(f"File not found: {self.mesh_path}")
            self.mesh = None
        else:
            self.mesh = geometry.Mesh(self.mesh_path)

    def test(self):
        if self.mesh is None:
            print("No mesh to display.")
            return
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(self.mesh)
        env.hold()

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = VeggieRobotGamma()
    input("Press Enter to test single file")
    tester = SingleMeshTester('Elbow_Pitch.dae')
    tester.test()
    # input("Press Enter to spawn Veggie Robot Gamma")
    # r.test()