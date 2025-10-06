# @file
# @brief Rough version of Veggie Robot Gamma defined by standard DH parameters with 3D model
# @author [Your Name]
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

# ----- Mesh Directory Search Utility -----
def find_mesh_dir(here, fname):
    # Search current and ./gamma directory for mesh file existence
    for cand in (here, os.path.join(here, "gamma")):
        fpath = os.path.join(cand, fname)
        if os.path.exists(fpath):
            return cand
    raise FileNotFoundError(f"{fname} not found in {here} or ./gamma")


# ----- Main Robot Class -----
class VeggieRobotGamma(DHRobot3D):
    def __init__(self):
        # Mesh names (no extensions)
        link3D_names = dict(
            link0='gamma_base',
            link1='Shoulder_Roll',
            link2='Shoulder_Pitch',
            link3='Elbow_Roll',
            link4='Elbow_Pitch',
            link5='Elbow_Yaw',
            link6='Wrist_Pitch',
            link7='Wrist_Roll'
        )
        here = os.path.abspath(os.path.dirname(__file__))
        mesh_dir = find_mesh_dir(here, link3D_names['link0'] + '.dae')

        # Placeholder identity transforms; space out links gradually in z
        qtest = [0.0]*7
        qtest_transforms = [
            spb.rpy2tr(0, 0, 0, order='xyz'),  # link0: gamma_base
            spb.transl(0, 0, 0.054) @ spb.rpy2tr(pi, 0, -pi/2, order='xyz'),                        # link1: Shoulder_Roll
            spb.transl(-0.028, 0, 0.116) @ spb.rpy2tr(0, pi/2, 0, order='xyz') @ spb.rpy2tr(pi, 0, 0),    # link2: Shoulder_Pitch
            spb.transl(0, 0, 0.2008) @ spb.rpy2tr(3.92699+pi/2, 0, 0, order='xyz'),                             # link3: Elbow_Roll
            spb.transl(0, 0.027, 0.255) @ spb.rpy2tr(0, pi/2, pi/2, order='xyz') @ spb.rpy2tr(0, pi/2, 0), # link4: Elbow_Pitch
            spb.transl(0, 0, 0.325) @ spb.rpy2tr(0, pi/2, pi/2, order='xyz'),                              # link5: Elbow_Yaw
            spb.transl(0, 0.027, 0.395) @ spb.rpy2tr(0, pi/2, pi/2, order='xyz') @ spb.rpy2tr(0, pi/2, 0), # link6: Wrist_Pitch
            spb.transl(0.036, 0, 0.44) @ spb.rpy2tr(0, pi/2, 0, order='xyz'),                        # link7: Wrist_Roll
        ]

        links = self._create_DH()

        super().__init__(
            links,
            link3D_names,
            name='VeggieRobotGamma',
            link3d_dir=mesh_dir,
            qtest=qtest,
            qtest_transforms=qtest_transforms
        )
        self.q = [0.0] * 7

    def _create_DH(self):
        qlim = [
            [-3.9,  1.5],   # shoulder_roll
            [-2.6,  0.2],   # shoulder_pitch
            [-3.8,  1.4],   # elbow_roll
            [-3.2,  0.8],   # elbow_pitch
            [-3.2,  0.8],   # elbow_yaw
            [-3.1,  0.8],   # wrist_pitch
            [-3.8,  1.4],   # wrist_roll
        ]

        # 2) Visible z-offsets (meters) from URDF joint <origin xyz="0 0 dz">
        d = [
            0.05315,   # shoulder_roll_joint_init_xyz
            0.063,     # shoulder_pitch_joint_init_xyz
            0.089,     # elbow_roll_joint_init_xyz
            0.052,     # elbow_pitch_joint_init_xyz
            0.072,     # elbow_yaw_joint_init_xyz
            0.0718,    # wrist_pitch_joint_init_xyz
            0.051425,  # wrist_roll_joint_init_xyz
        ]

        # 3) Link lengths along x: assume zero (axes largely intersect in this design)
        a = [0.0]*7

        # 4) Twists alpha_i: angle between successive joint axes (≈ ±90° when axes are orthogonal)
        #    Axis sequence from URDF: Z, X, Z, X, (−Y), X, Z  → use ±pi/2 pattern
        alpha = [
            0.0,        # z -> z (parallel at base: 0)
            +pi/2,      # z -> x
            -pi/2,      # x -> z
            +pi/2,      # z -> x
            -pi/2,      # x -> (-y) ~ -90 about +x chain -> treat as -pi/2
            +pi/2,      # (-y) -> x (orthogonal)
            -pi/2,      # x -> z
        ]

        # 5) Offsets so DH q=0 matches URDF 'home' rpy about the joint axis.
        #    From your URDF rpy’s at each joint:
        #      shoulder_roll: rpy z = +1.2
        #      shoulder_pitch: rpy x = +1.2  (shows up as theta offset in DH after frame placement)
        #      elbow_roll: rpy z = +1.2
        #      elbow_pitch: rpy x = +1.2
        #      elbow_yaw: rpy y = -1.2
        #      wrist_pitch: rpy x = +1.2
        #      wrist_roll: rpy z = +1.2
        #   Start with these; adjust signs after FK check if any link appears rotated 180°.
        offset = [ +1.2, +1.2, +1.2, +1.2, -1.2, +1.2, +1.2 ]

        links = []
        for i in range(7):
            links.append(
                rtb.RevoluteDH(
                    a=a[i], d=d[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i]
                )
            )
        return links
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)
        self.q = self._qtest
        self.add_to_env(env)
        env.hold()


# ----- Single Mesh Tester -----
def test_single_file(
    file_name,
    visual_xyz=(0.0, 0.0, 0.0),   # from URDF <visual><origin xyz="...">
    visual_rpy=(0.0, 0.0, 0.0),   # from URDF <visual><origin rpy="..."> (radians)
):
    """
    Load and display a single mesh file in Swift with the URDF visual transform.
    Must be in script dir or ./gamma
    """
    import os
    import swift
    import spatialgeometry as geometry
    import spatialmath.base as spb
    from spatialmath import SE3

    env = swift.Swift()
    env.launch(realtime=True)

    here = os.path.abspath(os.path.dirname(__file__))

    # Search in current folder or ./gamma
    mesh_path = None
    for cand in (here, os.path.join(here, "gamma")):
        candidate = os.path.join(cand, file_name)
        if os.path.exists(candidate):
            mesh_path = candidate
            break

    if mesh_path is None:
        print(f"File not found: {file_name} in {here} or ./gamma")
        return

    # Create the mesh exactly like before (no T kwarg here)
    mesh = geometry.Mesh(mesh_path)
    env.add(mesh)

    # Apply URDF visual transform *after* creation
    T_visual = SE3(spb.transl(*visual_xyz) @ spb.rpy2tr(*visual_rpy, order="xyz"))
    mesh.T = T_visual

    print(f"Loaded {file_name}\nPose:\n{T_visual}")

    # Draw once and hold
    env.step(0.02)
    env.hold()


# ----- Main Entrypoint -----
if __name__ == "__main__":
    # Test a single mesh visually:
    # input("Press Enter to test single file")

    # test_single_file("gamma_base.dae", (0, 0, 0), (0, 0, 0))
    # test_single_file("Shoulder_Roll.dae", (0, 0, 0), (pi, 0, -pi/2))
    # test_single_file("Shoulder_Pitch.dae", (0, 0, 0), (0, 0, 0))   # Change name to test other files

    # # Then spawn the full robot:
    r = VeggieRobotGamma()
    # input("Press Enter to spawn Veggie Robot Gamma")
    r.test()
