# vs068.py — DENSO VS-068 style 6R arm (Veggie Arm)

import os, time
import numpy as np
import roboticstoolbox as rtb
import swift
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
from math import pi

class VeggieVS068(DHRobot3D):
    def __init__(self):
        links = self._create_DH()

        # Mesh names (no extensions). Reuse wrist mesh for link5 – that's fine.
        link3D_names = dict(
            link0='l1_base_vs08',
            link1='l2_shoulder_vs068',
            link2='l3_upperarm_vs068',
            link3='l4_forearm_vs068',
            link4='l5_wrist_vs068',
            link5='l5_wrist_vs068',   # reuse for J5
            link6='l6_flange_vs068',
        )

        # Identity transforms: meshes exported in their link frames
        I = np.eye(4)
        qtest_transforms = [I, I, I, I, I, I, I]

        # A small, safe test pose
        qtest = [0, np.deg2rad(-30), np.deg2rad(45),
                 np.deg2rad(-15), np.deg2rad(15), 0]

        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links,
            link3D_names,
            name='VS068_Veggie',
            link3d_dir=current_path,
            qtest=qtest,
            qtest_transforms=qtest_transforms
        )

        # Put base at world origin (adjust Z a little if you want it to sit on a "floor")
        self.base = SE3(0, 0, 0)

        # Home pose
        q_home = [0, np.deg2rad(-20), np.deg2rad(35),
                  np.deg2rad(-10), np.deg2rad(10), 0]
        self.q = np.clip(q_home, self.qlim[0, :], self.qlim[1, :])

    def scale_visuals(self, s=0.001):
        """Scale robot visual meshes."""
        import swift
        
        # Look for Swift mesh objects in the environment
        if hasattr(self, '_swift_shapes') and self._swift_shapes:
            for shape in self._swift_shapes:
                if hasattr(shape, 'scale'):
                    shape.scale = [s, s, s]
                    print(f"Scaled shape: {shape}")
        
        # Alternative: try to access Swift's internal mesh storage
        try:
            # This might work depending on your Swift/DHRobot3D version
            for link in self.links:
                if hasattr(link, '_shape') and link._shape:
                    link._shape.scale = [s, s, s]
        except:
            pass




    def _create_DH(self):
        # VS-068 DH in METERS (proper scale for simulation)
        a     = [0.060, 0.320, 0.260, 0.000, 0.000, 0.000]  # Back to meters
        d     = [0.120, 0.000, 0.000, 0.220, 0.000, 0.080]  # Back to meters
        alpha = [ +pi/2,   0.0,  -pi/2,  +pi/2,  -pi/2,   0.0 ]
        qlim = [
            np.deg2rad([-170, 170]),
            np.deg2rad([-120, 120]),
            np.deg2rad([-140, 140]),
            np.deg2rad([-180, 180]),
            np.deg2rad([-180, 180]),
            np.deg2rad([-360, 360]),
        ]
        return [rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlim[i]) for i in range(6)]


    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)

        # add robot + meshes once
        self.add_to_env(env)


        # Debug: print robot info
        print(f"Robot reach: {self.reach} units")
        print(f"Robot base: {self.base}")
    
        # Try different scale factors
        for scale in [1.0, 0.1, 0.01, 0.001]:
            input(f"Press enter to try scale {scale}")
            self.scale_visuals(scale)
            env.step(0.1)

        # small demo motion
        q0 = self.q.copy()
        q1 = q0.copy()
        q1[0] += np.deg2rad(30)
        q1[1] -= np.deg2rad(20)
        q1[2] += np.deg2rad(25)
        q1[3] -= np.deg2rad(20)
        q1[4] += np.deg2rad(15)
        q1[5] += np.deg2rad(45)

        for q in rtb.jtraj(q0, q1, 60).q:
            self.q = np.clip(q, self.qlim[0, :], self.qlim[1, :])
            env.step(0.016)

        env.hold()
        time.sleep(0.5)

if __name__ == "__main__":
    robot = VeggieVS068()
    input("Press Enter to open Swift and test VeggieVS068...")
    
    robot.test()
