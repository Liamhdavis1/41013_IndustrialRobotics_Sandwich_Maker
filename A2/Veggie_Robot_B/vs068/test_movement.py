##  @file test_movement_fixed.py
#   @brief DENSO VS-068 Robot (Veggie Arm) with direct DAE mesh loading
#   @author Based on working mesh loading approach
#   @date September 26, 2025

import swift
import roboticstoolbox as rtb
from spatialmath import SE3
import time
import os
import numpy as np
from math import pi
from spatialgeometry import Mesh

class VeggieVS068(rtb.DHRobot):
    def __init__(self):
        """
        DENSO VS-068 Robot (Veggie Arm) with kinematics only.
        Meshes are loaded separately to avoid DHRobot3D issues.
        """
        links = self._create_DH()
        super().__init__(links, name='VeggieVS068')
        
        # Set joint limits - must be (2, n) shape: [lower_limits, upper_limits]
        self.qlim = np.array([
            # Lower limits for all 6 joints
            np.deg2rad([-170, -120, -140, -180, -180, -360]),
            # Upper limits for all 6 joints  
            np.deg2rad([170, 120, 140, 180, 180, 360])
        ])
        
        # Initial joint configuration
        self.q = np.array([0, np.deg2rad(-20), np.deg2rad(35), np.deg2rad(-10), np.deg2rad(10), 0])
        
        # Store mesh objects for updating during motion
        self.meshes = []

    def _create_DH(self):
        """
        Create DENSO VS-068 DH model (approximate parameters in meters)
        """
        # DH parameters for VS-068
        a = [0.060, 0.320, 0.260, 0.000, 0.000, 0.000]
        d = [0.120, 0.000, 0.000, 0.220, 0.000, 0.080] 
        alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0]
        
        links = []
        for i in range(6):
            links.append(rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i]))
        
        return links

    def verify_mesh_files(self):
        """
        Verify all DAE mesh files exist in the current directory
        """
        current_path = os.path.abspath(os.path.dirname(__file__))
        files = [
            'base_vs08_mm.dae',
            'l1_shoulder_vs068_mm.dae', 
            'l2_upperarm_vs068_mm.dae',
            'l3_forearm_vs068_mm.dae',
            'l4_wrist01_vs068_mm.dae',
            'l5_wrist02_vs068_mm.dae',
            'l6_flange_vs068_mm.dae',
        ]
        
        missing_files = []
        for filename in files:
            full_path = os.path.join(current_path, filename)
            if not os.path.exists(full_path):
                missing_files.append(filename)
        
        if missing_files:
            print(f"Warning: Missing mesh files: {missing_files}")
            return False
        else:
            print("All mesh files found successfully!")
            return True

    def load_meshes_directly(self, env, scale=500.0):
        """
        Load and add DAE meshes directly to Swift environment with proper scaling.
        Uses forward kinematics to position each mesh correctly.
        """
        current_path = os.path.abspath(os.path.dirname(__file__))
        files = [
            'base_vs08_mm.dae',
            'l1_shoulder_vs068_mm.dae',
            'l2_upperarm_vs068_mm.dae', 
            'l3_forearm_vs068_mm.dae',
            'l4_wrist01_vs068_mm.dae',
            'l5_wrist02_vs068_mm.dae',
            'l6_flange_vs068_mm.dae',
        ]
        
        self.meshes = []
        
        # Load base mesh at origin
        base_path = os.path.join(current_path, files[0])
        base_mesh = Mesh(
            base_path, 
            pose=SE3(), 
            scale=[scale, scale, scale],
            color=[0.8, 0.8, 0.8, 1.0]
        )
        env.add(base_mesh)
        self.meshes.append(base_mesh)
        print(f"Loaded base mesh: {files[0]}")
        
        # Load remaining link meshes using forward kinematics
        fk_poses = self.fkine_all(self.q)
        
        for i in range(1, len(files)):
            full_path = os.path.join(current_path, files[i])
            if os.path.exists(full_path):
                # Use forward kinematics pose for this link
                pose = fk_poses[i-1]  # fkine_all returns poses for each link transform
                
                mesh = Mesh(
                    full_path,
                    pose=pose,
                    scale=[scale, scale, scale],
                    color=[0.6, 0.6, 0.8, 1.0]
                )
                env.add(mesh)
                self.meshes.append(mesh)
                print(f"Loaded link {i} mesh: {files[i]}")
            else:
                print(f"Warning: Mesh file {full_path} not found!")

    def update_mesh_poses(self, q):
        """
        Update mesh poses based on current robot joint configuration q
        """
        if len(self.meshes) == 0:
            return
            
        # Base mesh stays at origin
        self.meshes[0].pose = SE3()
        
        # Update other link meshes using forward kinematics
        if len(self.meshes) > 1:
            fk_poses = self.fkine_all(q)
            for i in range(1, len(self.meshes)):
                if i-1 < len(fk_poses):
                    self.meshes[i].pose = fk_poses[i-1]

    def test_movement(self):
        """
        Test the robot by displaying it in Swift and performing smooth movements
        """
        print("Starting DENSO VS-068 Robot Test...")
        
        # Verify mesh files exist
        if not self.verify_mesh_files():
            print("Some mesh files missing. Continuing anyway...")
        
        # Launch Swift environment
        env = swift.Swift()
        env.launch(realtime=True)
        print("Swift environment launched")
        
        # Load meshes directly
        print("Loading DAE meshes...")
        self.load_meshes_directly(env, scale=500.0)
        
        # Give time for meshes to load and render
        time.sleep(2)
        env.step(0.1)
        
        print(f"Robot reach: {self.reach:.3f} meters")
        print("Starting movement sequence...")
        
        # Define movement sequence
        movements = [
            # Movement 1: Base rotation
            {
                'name': 'Base Rotation',
                'target_q': [np.deg2rad(60), np.deg2rad(-20), np.deg2rad(35), np.deg2rad(-10), np.deg2rad(10), 0],
                'duration': 100
            },
            # Movement 2: Shoulder and elbow
            {
                'name': 'Shoulder & Elbow',
                'target_q': [np.deg2rad(60), np.deg2rad(-50), np.deg2rad(60), np.deg2rad(-10), np.deg2rad(10), 0],
                'duration': 80
            },
            # Movement 3: Wrist movements
            {
                'name': 'Wrist Motion',
                'target_q': [np.deg2rad(60), np.deg2rad(-50), np.deg2rad(60), np.deg2rad(45), np.deg2rad(-30), np.deg2rad(90)],
                'duration': 80
            },
            # Movement 4: Return to home
            {
                'name': 'Return Home',
                'target_q': [0, np.deg2rad(-20), np.deg2rad(35), np.deg2rad(-10), np.deg2rad(10), 0],
                'duration': 120
            }
        ]
        
        # Execute movement sequence
        current_q = self.q.copy()
        
        for movement in movements:
            print(f"Executing: {movement['name']}")
            target_q = np.array(movement['target_q'])
            
            # Generate smooth trajectory
            qtraj = rtb.jtraj(current_q, target_q, movement['duration']).q
            
            # Execute trajectory
            for q in qtraj:
                # Apply joint limits
                q_safe = np.clip(q, self.qlim[0, :], self.qlim[1, :])
                self.q = q_safe
                
                # Update mesh positions
                self.update_mesh_poses(self.q)
                
                # Step simulation
                env.step(0.03)
            
            current_q = target_q.copy()
            time.sleep(0.5)  # Pause between movements
        
        print("Movement sequence completed!")
        print("Press any key to exit...")
        env.hold()

if __name__ == "__main__":
    try:
        robot = VeggieVS068()
        input("Press Enter to start DENSO VS-068 movement test...")
        robot.test_movement()
        
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
        input("Press Enter to exit...")
