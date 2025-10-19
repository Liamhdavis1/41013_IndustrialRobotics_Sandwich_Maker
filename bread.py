import numpy as np
from scipy import linalg
import time
from roboticstoolbox import jtraj

import swift
from spatialmath.base import transl, rpy2tr, tr2rpy
from spatialmath import SE3
from math import pi

# Import the ABB IRB120 robot model
from Micahs_robot.abb_irb_120 import abb_irb_120

class BreadCollection:
    def __init__(self, robot = None, env = None):
        self.robot = robot
        self.env = env
        # RMRC Parameters for precise movements
        self.rmrc_time = 1.0                      # Time for RMRC movements (pick/place)
        self.delta_t = 0.02                       # Control frequency
        self.rmrc_steps = int(self.rmrc_time/self.delta_t)
        self.epsilon = 0.1                        # Manipulability threshold
        self.W = np.diag([1, 1, 1, 0.1, 0.1, 0.1])  # Weighting matrix
        
        # Movement parameters
        self.hover_height = 0.2                   # Height above objects (0.2m)
        self.convergence_tolerance = 1e-6         # IK tolerance

        # Define grip offset for mesh attachment (adjust as needed)
        self.grip_offset = SE3.Trans(0, 0, 0) * SE3.Rx(np.pi)  # Tool pointing down
    
    def rmrc_side_movement(self, robot, env, start_pos, end_pos, tool_orientation,
                        mesh=None, mesh_list=None, grip_offset=None,
                        mesh_offset_along_z=0.0, mesh_z_offsets=None):
        """
        RMRC for precise side-to-side movement (e.g., sliding along X to pick/place from the side).
        start_pos: [x, y, z] starting position
        end_pos: [x, y, z] ending position
        tool_orientation: [roll, pitch, yaw] in radians
        """
        if grip_offset is None:
            grip_offset = self.grip_offset

        print(f"    RMRC (side): {start_pos} → {end_pos}")
        
        # Create trajectory arrays
        x = np.linspace(start_pos, end_pos, self.rmrc_steps).T
        theta = np.tile(tool_orientation, (self.rmrc_steps, 1)).T
        
        # Allocate arrays
        q_matrix = np.zeros((self.rmrc_steps, 6))
        q_matrix[0, :] = robot.q.copy()
        
        for i in range(self.rmrc_steps - 1):
            T_now = robot.fkine(q_matrix[i, :]).A
            delta_x = x[:, i+1] - T_now[:3, 3]

            # Orientation
            Rd = rpy2tr(*theta[:, i+1])[:3, :3]
            Ra = T_now[:3, :3]
            S = ((Rd - Ra) / self.delta_t) @ Ra.T

            lin_vel = delta_x / self.delta_t
            ang_vel = np.array([S[2, 1], S[0, 2], S[1, 0]])
            xdot = self.W @ np.vstack((lin_vel[:, None], ang_vel[:, None]))

            J = robot.jacob0(q_matrix[i, :])
            m = np.sqrt(linalg.det(J @ J.T))
            m_lambda = (1 - m/self.epsilon) * 0.05 if m < self.epsilon else 0

            inv_J = linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T
            qdot = (inv_J @ xdot).T

            q_next = q_matrix[i, :] + self.delta_t * qdot
            q_next = np.clip(q_next, robot.qlim[0], robot.qlim[1])
            q_matrix[i+1, :] = q_next
            robot.q = q_next

            # Mesh update
            if mesh is not None:
                T_ee = robot.fkine(q_next)
                mesh.T = T_ee * grip_offset

            if mesh_list is not None:
                T_ee = robot.fkine(q_next)
                for m, z_off in zip(mesh_list, mesh_z_offsets):
                    if m is not None:
                        m.T = T_ee * grip_offset * SE3.Trans(0, 0, z_off + mesh_offset_along_z)

            env.step(self.delta_t)

        print(f"    RMRC side move complete. Final EE position: {robot.fkine(robot.q).t.round(3)}")


    def process_food_order_side(self, food_locations, station_location, mesh_list=None):
        """
        Version of process_food_order() that approaches food items from the SIDE (along X-axis)
        instead of vertically.
        """
        print("=== Food Order Processing System (Side Pickup) ===")

        robot = self.robot
        env = self.env

        current_z = station_location[2]

        # Tool pointing sideways (X direction)
        tool_orientation = [pi/2, 0, 0]  # roll=90°, pitch=0, yaw=0

        print(f"Processing side order with {len(food_locations)} ingredients...")
        print(f"Station location: {station_location}")

        for i, food_pos in enumerate(food_locations):
            mesh = mesh_list[i] if mesh_list and i < len(mesh_list) else None

            # Hover "beside" the food, offset along X
            hover_beside_food = [food_pos[0] - 0.15, food_pos[1], food_pos[2]]  # 15 cm away in -X
            hover_pose = SE3(transl(hover_beside_food) @ rpy2tr(*tool_orientation))

            q_hover, success, error = self.solve_ik_robust(robot, hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach side hover position for ingredient {i+1}")
                continue

            print(f"  Moving to side hover (IK+jtraj)...")
            self.execute_trajectory(robot, env, robot.q.copy(), q_hover, mesh=None)

            # Move sideways to "grab" the food
            print(f"  Picking up ingredient {i+1} (RMRC side in)...")
            start_pos = np.array(hover_beside_food)
            end_pos = np.array(food_pos)
            self.rmrc_side_movement(robot, env, start_pos, end_pos, tool_orientation, mesh=None)

            print(f"  Lifting back (RMRC side out)...")
            self.rmrc_side_movement(robot, env, end_pos, start_pos, tool_orientation, mesh=mesh)

            print(f"  ✓ Ingredient {i+1} side pickup completed!")
