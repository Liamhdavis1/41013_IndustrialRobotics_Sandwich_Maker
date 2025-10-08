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
    # Search current and ./abb_irb_120  directory for mesh file existence
    for cand in (here, os.path.join(here, "abb_irb_120")):
        fpath = os.path.join(cand, fname)
        if os.path.exists(fpath):
            return cand
    raise FileNotFoundError(f"{fname} not found in {here} or ./abb_irb_120")


# ----- Main Robot Class -----
class VeggieRobotGamma(DHRobot3D):
    def __init__(self):
        # Mesh names (no extensions)
        link3D_names = dict(
            link0='base_link',
            link1='link_1',
            link2='link_2',
            link3='link_3',
            link4='link_4',
            link5='link_5',
            link6='link_6'
        )
        here = os.path.abspath(os.path.dirname(__file__))
        mesh_dir = find_mesh_dir(here, link3D_names['link0'] + '.stl')

        # Placeholder identity transforms; space out links gradually in z
        qtest = [0.0]*6
        qtest_transforms = [
            spb.transl(0, 0, 0) @ spb.rpy2tr(0, 0, 0, order='xyz'),        # base_link
            spb.transl(0, 0, 0) @ spb.rpy2tr(0, 0, 0, order='xyz'),        # link_1
            spb.transl(0, 0, 0.29) @ spb.rpy2tr(0, 0, 0, order='xyz'),    # link_2
            spb.transl(0, 0, 0.55) @ spb.rpy2tr(0, 0, 0, order='xyz'),  # link_3
            spb.transl(0, 0, 0.62) @ spb.rpy2tr(0, 0, 0, order='xyz'),  # link_4
            spb.transl(0.31, 0, 0.63) @ spb.rpy2tr(0, 0, 0, order='xyz'),    # link_5
            spb.transl(0.38, 0, 0.63) @ spb.rpy2tr(0, 0, 0, order='xyz'),    # link_6
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

        # Twists (rad)
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
                        


    
    def test_jtraj(self):
        """
        Animate a smooth trajectory from the home pose to a goal configuration.
        """
        env = swift.Swift()
        env.launch(realtime=True)

        # Add robot to Swift scene
        self.q = self._qtest
        self.add_to_env(env)

        # --- Define goal configuration ---
        # Slightly move each joint, exaggerate wrist motion for visibility
        q_goal = [
            self.q[0],   # J1
            self.q[1],   # J2
            self.q[2],   # J3
            self.q[3] + pi/2,   # J4
            self.q[4],   # J5
            self.q[5],   # J6
        ]

        qtraj = rtb.jtraj(self.q, q_goal, 80).q
        for q in qtraj:
            self.q = q
            env.step(0.02)
        env.hold()

        # --- Animate ---
        for q in qtraj:
            self.q = q
            env.step(0.2)  # 20 ms per frame

        env.hold()
        time.sleep(2)



    def test_teach(self):
        """
        Open Swift and a non-blocking teach panel. When you move sliders in the
        teach UI, the Swift model updates in real time.
        """
        import matplotlib.pyplot as plt
        env = swift.Swift()
        env.launch(realtime=True)

        # start from your qtest (or any)
        self.q = self._qtest
        self.add_to_env(env)

        # --- Option A: use built-in teach panel (newer RTB versions) ---
        try:
            fig = self.teach(block=False)   # opens sliders; returns a Matplotlib Figure
        except Exception:
            # --- Option B: fallback for older RTB ---
            # Make a pyplot plot and attach the teach panel to it
            fig = self.plot(self.q)                 # makes a Matplotlib plot of the kinematic skeleton
            fig._add_teach_panel(self, self.q)      # adds sliders
            plt.show(block=False)

        # Mirror slider changes into Swift by polling robot.q
        last_q = None
        while plt.fignum_exists(plt.gcf().number):
            # When you move a slider in the teach UI, RTB sets self.q underneath
            q_now = np.array(self.q).copy()
            if last_q is None or not np.allclose(q_now, last_q, atol=1e-4):
                last_q = q_now
                env.step(0.02)   # redraw Swift at the new configuration
            plt.pause(0.03)      # keep the teach UI responsive

        env.hold()


    def traj_then_teach(self):
        """
        1) Open a single Swift window
        2) Run a jtraj animation to a non-trivial goal
        3) After the motion finishes, open the teach panel and keep Swift updating
        """
        import matplotlib.pyplot as plt

        # 1) Swift env
        env = swift.Swift()
        env.launch(realtime=True)


        # Start from your test config and add to scene
        self.q = self._qtest
        self.add_to_env(env)

        # 2) Trajectory (make sure it's not identical to start!)
        q_start = np.array(self.q).copy()
        q_goal  = [
            q_start[0] ,
            q_start[1] ,
            q_start[2] ,
            q_start[3] ,
            q_start[4] ,
            q_start[5] +pi,
        ]
        qtraj = rtb.jtraj(q_start, q_goal, 150).q

        fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        fig._add_teach_panel(self, self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)            # redraw Swift at 50 Hz


            fig.step(0.01)
        fig.hold()
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
    for cand in (here, os.path.join(here, "abb_irb_120")):
        candidate = os.path.join(cand, file_name)
        if os.path.exists(candidate):
            mesh_path = candidate
            break

    if mesh_path is None:
        print(f"File not found: {file_name} in {here} or ./abb_irb_120")
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
    # r.test_teach()
    # r.test_teach()
    r.traj_then_teach()
