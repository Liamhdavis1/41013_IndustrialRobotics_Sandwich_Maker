# food_order_robot.py
# Simplified food order processing system for ABB IRB120
# Combines IK+jtraj for large movements and RMRC for precise pick/place operations

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

class FoodOrderRobotv1:
    """
    Food order processing robot using combined IK+jtraj and RMRC control
    """
    
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
    
    # ========== IK and Trajectory Functions ==========
    def enforce_joint_limits(self, q, qlim):
        return np.clip(q, qlim[0, :], qlim[1, :])

    def pose_error_mm(self, T_actual, T_target):
        return float(np.linalg.norm(T_actual.t - T_target.t) * 1000.0)

    def generate_initial_guesses(self, robot, q_current):
        guesses = []
        guesses.append(q_current.copy())
        guesses.append(np.zeros(robot.n))
        for _ in range(3):
            q_rand = np.random.uniform(robot.qlim[0, :], robot.qlim[1, :])
            guesses.append(q_rand)
        if hasattr(robot, 'qr'):
            guesses.append(robot.qr)
        return guesses

    def solve_ik_robust(self, robot, target_pose, q_current):
        """
        Robust IK solver with multiple strategies
        """
        qlim = robot.qlim
        tol = self.convergence_tolerance
        
        # Try initial guess
        ik_result = robot.ikine_LM(
            target_pose, q0=q_current, mask=[1, 1, 1, 1, 1, 1],
            ilimit=2000, slimit=10, tol=tol, joint_limits=True
        )
        if ik_result.success:
            q_solution = self.enforce_joint_limits(ik_result.q, qlim)
            error = self.pose_error_mm(robot.fkine(q_solution), target_pose)
            if error <= 5.0:
                return q_solution, True, error

        # Try multiple guesses
        guesses = self.generate_initial_guesses(robot, q_current)
        for i, q0 in enumerate(guesses):
            ik_result = robot.ikine_LM(
                target_pose, q0=q0, mask=[1, 1, 1, 1, 1, 1],
                ilimit=2000, slimit=3, tol=tol, joint_limits=True
            )
            if ik_result.success:
                q_solution = self.enforce_joint_limits(ik_result.q, qlim)
                error = self.pose_error_mm(robot.fkine(q_solution), target_pose)
                if error <= 5.0:
                    return q_solution, True, error

        return q_current, False, float('inf')

    def execute_trajectory(self, robot, env, q_start, q_end, steps=50, mesh=None, mesh_list=None, grip_offset=None):
        if grip_offset is None:
            grip_offset = self.grip_offset
        
        traj = jtraj(q_start, q_end, steps)
        for q in traj.q:
            robot.q = q
            if mesh:
                mesh.T = robot.fkine(q) * grip_offset
            if mesh_list:
                T_ee = robot.fkine(q)
                for m in mesh_list:
                    if m is not None:
                        m.T = T_ee * grip_offset
            env.step(0.01)


    # ========== RMRC Function for Precise Pick/Place ==========
    def rmrc_vertical_movement(self, robot, env, start_pos, end_pos, tool_orientation, mesh=None, mesh_list=None, grip_offset=None):

        """
        Use RMRC for precise vertical movement (pick or place)
        start_pos: [x, y, z] starting position
        end_pos: [x, y, z] ending position  
        tool_orientation: [roll, pitch, yaw] in radians
        """
        if grip_offset is None:
            grip_offset = self.grip_offset

        print(f"    RMRC: {start_pos} → {end_pos}")
        
        # Create trajectory arrays
        x = np.linspace(start_pos, end_pos, self.rmrc_steps).T
        theta = np.tile(tool_orientation, (self.rmrc_steps, 1)).T
        
        # Allocate arrays
        q_matrix = np.zeros((self.rmrc_steps, 6))
        q_matrix[0, :] = robot.q.copy()  # Start from current position
        
        # RMRC control loop
        for i in range(self.rmrc_steps - 1):
            T_now = robot.fkine(q_matrix[i, :]).A
            delta_x = x[:, i+1] - T_now[:3, 3]
            
            # Orientation calculations
            Rd = rpy2tr(*theta[:, i+1])[:3, :3]
            Ra = T_now[:3, :3]
            S = ((Rd - Ra) / self.delta_t) @ Ra.T
            
            # Velocity calculations
            lin_vel = delta_x / self.delta_t
            ang_vel = np.array([S[2, 1], S[0, 2], S[1, 0]])
            xdot = self.W @ np.vstack((lin_vel[:, None], ang_vel[:, None]))
            
            # Jacobian and manipulability
            J = robot.jacob0(q_matrix[i, :])
            m = np.sqrt(linalg.det(J @ J.T))
            m_lambda = (1 - m/self.epsilon) * 0.05 if m < self.epsilon else 0
            
            # Joint velocities
            inv_J = linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T
            qdot = (inv_J @ xdot).T
            
            # Update joints with limits
            q_next = q_matrix[i, :] + self.delta_t * qdot
            q_next = np.clip(q_next, robot.qlim[0], robot.qlim[1])
            q_matrix[i+1, :] = q_next
            
            robot.q = q_next
            if mesh is not None:
                T_ee = robot.fkine(q_next)
                mesh.T = T_ee * grip_offset

            env.step(self.delta_t)
            
        print(f"    RMRC completed. Final position: {robot.fkine(robot.q).t.round(3)}")

    # ========== Main Food Order Processing Function ==========
    def process_food_order(self, food_locations, station_location, mesh_list = None):
        """
        Main function to process food order by iterating through ingredient locations
        
        food_locations: List of [x, y, z] coordinates for each food item
        station_location: [x, y, z] coordinate for the assembly station
        """
        print("=== Food Order Processing System ===")
        
        # Initialize environment and robot
        robot = self.robot
        env = self.env

        # Initialize current stack height at assembly station base height
        current_z = station_location[2]

        # Tool orientation (pointing downward for picking)
        tool_orientation = [pi, 0, 0]  # Roll=π, Pitch=0, Yaw=0

        print(f"Processing order with {len(food_locations)} ingredients...")
        print(f"Station location: {station_location}")
   
        # Tool orientation (pointing downward for picking)
        tool_orientation = [pi, 0, 0]  # Roll=π, Pitch=0, Yaw=0
        
        print(f"Processing order with {len(food_locations)} ingredients...")
        print(f"Station location: {station_location}")
        

        placed_meshes = []

        # Process each food item in the order list
        for i, food_pos in enumerate(food_locations):
            mesh = None
            if mesh_list is not None and i < len(mesh_list):
                mesh = mesh_list[i]
                print(f"  Attaching mesh {i}: {type(mesh)}")

            # Calculate hover position
            hover_above_food = [food_pos[0], food_pos[1], food_pos[2] + self.hover_height]
            hover_pose = SE3(transl(hover_above_food) @ rpy2tr(*tool_orientation))

            q_hover, success, error = self.solve_ik_robust(robot, hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach hover position for ingredient {i+1}")
                continue

            print(f"  Moving to hover above food (IK+jtraj)...")
            self.execute_trajectory(robot, env, robot.q.copy(), q_hover, mesh=None)
            
            print(f"  Picking up ingredient {i+1} (RMRC down)...")
            self.rmrc_vertical_movement(robot, env, hover_above_food, food_pos, tool_orientation, mesh=None)

            print(f"  Attaching mesh {i} to end effector...")
            attached_mesh = mesh
            
            print(f"  Lifting ingredient {i+1} (RMRC up)...")
            self.rmrc_vertical_movement(robot, env, food_pos, hover_above_food, tool_orientation, mesh=attached_mesh)
            
            # Station hover
            # Calculate current station location with stacking
            current_station_pos = [
                station_location[0],
                station_location[1],
                current_z  # Use updated stack height
            ]

            # Hover above the current stack height
            hover_above_station = [current_station_pos[0], current_station_pos[1], current_station_pos[2] + self.hover_height]

            station_hover_pose = SE3(transl(hover_above_station) @ rpy2tr(*tool_orientation))
            q_station, success, error = self.solve_ik_robust(robot, station_hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach station hover position for ingredient {i + 1}")
                continue

            print(f"  Moving to hover above station (IK+jtraj)...")
            self.execute_trajectory(robot, env, robot.q.copy(), q_station, mesh=attached_mesh)

            print(f"  Placing ingredient {i+1} at station (RMRC down)...")
            place_pos = current_station_pos
            self.rmrc_vertical_movement(robot, env, hover_above_station, place_pos, tool_orientation, mesh=attached_mesh)

            print(f"  Retracting from station (RMRC up)...")
            self.rmrc_vertical_movement(robot, env, place_pos, hover_above_station, tool_orientation)

            # Append the attached mesh before detaching it
            if attached_mesh is not None:
                placed_meshes.append(attached_mesh)
            # Detach mesh by stopping update (mesh left at place location)
            attached_mesh = None

            # Increment current stack height by the ingredient height for next ingredient
            ingredient_height = 0.01  # fixed height for all ingredients
            current_z += ingredient_height

            print(f"  ✓ Ingredient {i+1} completed! Stacking next ingredient at height {current_z:.3f} m")
        
        # Filter out None values from placed_meshes (safety check)
        placed_meshes = [mesh for mesh in placed_meshes if mesh is not None]
        print(f"Found {len(placed_meshes)} valid placed meshes to drag")


        # After placing all ingredients in the loop is done

        # 1. Get current EE pose and compute target offset by +0.1m in X (IK + jtraj)
        T_current = robot.fkine(robot.q)
        T_shift_0_1 = T_current * SE3.Trans(0.1, 0, 0)
        q_shift_0_1, success, error = self.solve_ik_robust(robot, T_shift_0_1, robot.q)
        if success:
            print("Moving EE by +0.1m in X using IK+jtraj...")
            self.execute_trajectory(robot, env, robot.q.copy(), q_shift_0_1, mesh_list=placed_meshes)
        else:
            print("Failed to reach workspace offset (+0.1m X)")

        # 2. RMRC linear move +0.3m in X dragging all placed meshes
        start_pos = T_shift_0_1.t
        end_pos = start_pos + np.array([0.3, 0, 0])
        print("Moving EE by +0.3m in X using RMRC dragging placed meshes...")

        self.rmrc_vertical_movement(robot, env, start_pos, end_pos, tool_orientation, mesh_list=placed_meshes)

        print(f"=== Order Complete! Processed {len(food_locations)} ingredients ===")
        input("Press Enter to close environment...")
        env.close()

def main():
    """Main function to run food order demo"""
    food_robot = FoodOrderRobotv1()

if __name__ == "__main__":
    main()
