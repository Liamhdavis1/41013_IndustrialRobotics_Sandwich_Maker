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
    Food order processing robot using combined IK+jtraj and RMRC control.
    """

    def __init__(self, robot=None, env=None, robot_name=None):
        self.robot = robot
        self.env = env
        self.robot_name = robot_name

        # RMRC parameters for precise movements (pick/place)
        self.rmrc_time = 1.0               # seconds
        self.delta_t = 0.02                # control frequency timestep
        self.rmrc_steps = int(self.rmrc_time / self.delta_t)
        self.epsilon = 0.1                 # manipulability threshold
        self.W = np.diag([1, 1, 1, 1, 1, 1])  # weighting matrix for RMRC

        # Movement parameters
        self.hover_height = 0.2            # height above objects in meters
        self.convergence_tolerance = 1e-6 # IK solver tolerance

        # Define grip offsets for mesh attachments (adjust as needed)
        self.grip_offset = SE3.Trans(0, 0, 0) * SE3.Rx(np.pi)  # tool pointing down default
        self.estop = False

        self.default_grip_offset = SE3.Trans(0, 0, 0) * SE3.Rx(np.pi)
        self.cobot_grip_offset = SE3.Rz(pi)  # 180° flip for cobot arm

    # ========== Emergency Stop ==========
    def set_estop(self, status: bool):
        if status != self.estop:
            self.estop = status
            print("Emergency Stop enabled! Robot motions will pause." if status else
                  "Emergency Stop released! Resuming robot motions.")

    def wait_if_estop(self):
        # Blocks while estop is active
        while self.estop:
            time.sleep(0.1)

    # ========== IK and Trajectory Functions ==========
    def enforce_joint_limits(self, q, qlim):
        return np.clip(q, qlim[0, :], qlim[1, :])

    def pose_error_mm(self, T_actual, T_target):
        # Euclidean position error in millimeters
        return float(np.linalg.norm(T_actual.t - T_target.t) * 1000.0)

    def generate_initial_guesses(self, robot, q_current):
        guesses = [q_current.copy(), np.zeros(robot.n)]
        guesses.extend(np.random.uniform(robot.qlim[0, :], robot.qlim[1, :], (3, robot.n)))
        if hasattr(robot, 'qr'):
            guesses.append(robot.qr)
        return guesses

    def solve_ik_robust(self, robot, target_pose, q_current):
        """
        Robust IK solver attempting multiple initial guesses.
        Returns (q_solution, success_flag, error_mm).
        """
        self.wait_if_estop()
        qlim = robot.qlim
        tol = self.convergence_tolerance

        # Try initial guess
        ik_result = robot.ikine_LM(target_pose, q0=q_current, mask=[1]*6,
                                  ilimit=2000, slimit=10, tol=tol, joint_limits=True)
        if ik_result.success:
            q_solution = self.enforce_joint_limits(ik_result.q, qlim)
            error = self.pose_error_mm(robot.fkine(q_solution), target_pose)
            if error <= 5.0:
                return q_solution, True, error

        # Try multiple guesses
        guesses = self.generate_initial_guesses(robot, q_current)
        for q0 in guesses:
            self.wait_if_estop()
            ik_result = robot.ikine_LM(target_pose, q0=q0, mask=[1]*6,
                                      ilimit=2000, slimit=3, tol=tol, joint_limits=True)
            if ik_result.success:
                q_solution = self.enforce_joint_limits(ik_result.q, qlim)
                error = self.pose_error_mm(robot.fkine(q_solution), target_pose)
                if error <= 5.0:
                    return q_solution, True, error

        return q_current, False, float('inf')

    def execute_trajectory(self, robot, env, q_start, q_end, steps=50, mesh=None, mesh_list=None, grip_offset=None):
        self.wait_if_estop()

        if grip_offset is None:
            grip_offset = self.cobot_grip_offset if self.robot_name == 'cobot' else self.default_grip_offset

        traj = jtraj(q_start, q_end, steps)
        for q in traj.q:
            self.wait_if_estop()
            robot.q = q
            if mesh:
                mesh.T = robot.fkine(q) * grip_offset
            if mesh_list:
                T_ee = robot.fkine(q)
                for m in mesh_list:
                    if m is not None:
                        m.T = T_ee * grip_offset
            env.step(0.01)

    def move_to_start_position(self, robot, env, target_pos, tool_orientation=[pi, 0, 0], mesh_list=None):
        """
        Move robot to a starting position using IK + jtraj.
        Returns True if successful, False otherwise.
        """
        self.wait_if_estop()

        target_pose = SE3(transl(target_pos) @ rpy2tr(*tool_orientation))
        q_target, success, error = self.solve_ik_robust(robot, target_pose, robot.q)

        if not success:
            print(f"Failed to solve IK for start position {target_pos}")
            print(f"Error: {error:.2f} mm")
            return False

        print(f"  ✓ Moving to start position {target_pos} (IK+jtraj)...")
        print(f"     IK error: {error:.2f} mm")

        self.execute_trajectory(robot, env, robot.q.copy(), q_target, steps=50, mesh_list=mesh_list)

        final_pos = robot.fkine(robot.q).t
        print(f"     Final position: {final_pos.round(3)}")
        return True

    # ========== RMRC Function for Precise Pick/Place ==========
    def rmrc_vertical_movement(self, robot, env, start_pos, end_pos, tool_orientation,
                               mesh=None, mesh_list=None, grip_offset=None,
                               mesh_offset_along_x=-0.28, mesh_z_offsets=None):
        """
        Use RMRC for precise vertical movement (pick or place).
        start_pos, end_pos: [x, y, z] coordinates.
        tool_orientation: [roll, pitch, yaw] in radians.
        """
        self.wait_if_estop()
        if grip_offset is None:
            grip_offset = self.grip_offset

        if mesh_list is not None and mesh_z_offsets is None:
            mesh_z_offsets = [0] * len(mesh_list)

        print(f"    RMRC: {start_pos} → {end_pos}")

        # Create interpolation trajectory for positions
        x = np.linspace(start_pos, end_pos, self.rmrc_steps).T
        Rd = rpy2tr(*tool_orientation)[:3, :3]

        q_matrix = np.zeros((self.rmrc_steps, 6))
        q_matrix[0, :] = robot.q.copy()  # Start from current joint positions

        for i in range(self.rmrc_steps - 1):
            self.wait_if_estop()
            T_now = robot.fkine(q_matrix[i, :]).A
            delta_x = x[:, i + 1] - T_now[:3, 3]

            Ra = T_now[:3, :3]
            S = ((Rd - Ra) / self.delta_t) @ Ra.T

            lin_vel = delta_x / self.delta_t
            ang_vel = np.array([S[2, 1], S[0, 2], S[1, 0]])
            xdot = self.W @ np.vstack((lin_vel[:, None], ang_vel[:, None]))

            J = robot.jacob0(q_matrix[i, :])
            det_val = np.clip(linalg.det(J @ J.T), 0, None)
            m = np.sqrt(det_val)

            lambda_min = 0.001
            if m < self.epsilon:
                m_lambda = max(lambda_min, (1 - m / self.epsilon) * 0.2)
            else:
                m_lambda = lambda_min

            inv_J = linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T
            qdot = (inv_J @ xdot).T

            q_next = q_matrix[i, :] + self.delta_t * qdot
            q_next = np.clip(q_next, robot.qlim[0], robot.qlim[1])
            q_matrix[i + 1, :] = q_next

            robot.q = q_next

            if mesh is not None:
                T_ee = robot.fkine(q_next)
                mesh.T = T_ee * grip_offset

            if mesh_list is not None:
                T_ee = robot.fkine(q_next)
                for m, z_off in zip(mesh_list, mesh_z_offsets):
                    if m is not None:
                        m.T = T_ee * grip_offset * SE3.Trans(mesh_offset_along_x, 0, z_off)

            env.step(self.delta_t)

        print(f"    RMRC completed. Final position: {robot.fkine(robot.q).t.round(3)}")

    def lower_to_height(self, robot, env, start_pos, target_height, tool_orientation):
        lowered_pos = start_pos.copy()
        lowered_pos[2] = target_height
        if start_pos[2] > target_height:
            self.rmrc_vertical_movement(robot, env, start_pos, lowered_pos, tool_orientation)

    def other_ik_solver(self, pick_pose: SE3, place_pose: SE3, mesh=None,
                        gripper_down_orientation_pick=None, gripper_down_orientation_place=None):
        self.wait_if_estop()

        if gripper_down_orientation_pick is None:
            gripper_down_orientation_pick = SE3.Rz(np.pi)  # Rotation about Z by pi

        if gripper_down_orientation_place is None:
            gripper_down_orientation_place = SE3.Rz(np.pi)

        safe_pose_above_pick = pick_pose * gripper_down_orientation_pick
        safe_pose_above_place = place_pose * gripper_down_orientation_place

        pick_pose_above_brick = pick_pose * gripper_down_orientation_pick
        place_pose_above_brick = place_pose * gripper_down_orientation_place

        q0_pick = np.array([0, 0, pi / 2, 0, 0, 0])
        q0_place = np.array([0, 0, pi / 2, 0, 0, 0])

        sol_safe_pick = self.robot.ikine_LM(safe_pose_above_pick, q0=q0_pick)
        sol_safe_place = self.robot.ikine_LM(safe_pose_above_place, q0=q0_place)

        q_safe_pick = np.clip(sol_safe_pick.q, self.robot.qlim[0], self.robot.qlim[1])
        q_safe_place = np.clip(sol_safe_place.q, self.robot.qlim[0], self.robot.qlim[1])

        sol_pick = self.robot.ikine_LM(pick_pose_above_brick, q0=q_safe_pick)
        sol_place = self.robot.ikine_LM(place_pose_above_brick, q0=q_safe_place)

        q_pick = np.clip(sol_pick.q, self.robot.qlim[0], self.robot.qlim[1])
        q_place = np.clip(sol_place.q, self.robot.qlim[0], self.robot.qlim[1])

        trajectory_pairs = [
            (self.robot.q, q_safe_pick),       # current to safe pick hover
            (q_safe_pick, q_pick),             # safe pick hover to pick pose
            (q_pick, q_safe_pick),             # pick pose back to safe hover
            (q_safe_pick, q_safe_place),       # safe pick hover to safe place hover
            (q_safe_place, q_place),           # safe place hover to place pose
            (q_place, q_safe_place)            # place pose back to safe hover
        ]

        for idx, (q_start, q_end) in enumerate(trajectory_pairs):
            traj = jtraj(q_start, q_end, 50)
            for q in traj.q:
                self.wait_if_estop()
                q_clipped = np.clip(q, self.robot.qlim[0], self.robot.qlim[1])
                self.robot.q = q_clipped
                ee_pose = self.robot.fkine(self.robot.q)

                if idx in [2, 3, 4] and mesh is not None:
                    if isinstance(mesh, list):
                        for m in mesh:
                            if m is not None:
                                m.T = ee_pose
                    else:
                        mesh.T = ee_pose

                self.env.step(0.01)

    # ========== Main Food Order Processing Function ==========
    def process_food_order(self, food_locations, station_location, mesh_list=None,
                           tool_orientation=[pi, 0, 0], initial_z=None, grip_offset=None):
        if grip_offset is None:
            grip_offset = self.grip_offset  # default grip offset

        print("=== Food Order Processing System ===")

        robot = self.robot
        env = self.env

        # Initialize stacking height
        current_z = station_location[2] if initial_z is None else initial_z

        print(f"Processing order with {len(food_locations)} ingredients...")
        print(f"Station location: {station_location}")

        placed_meshes = []

        for i, food_pos in enumerate(food_locations):
            mesh = mesh_list[i] if mesh_list is not None and i < len(mesh_list) else None
            if mesh is not None:
                print(f"  Attaching mesh {i}: {type(mesh)}")

            hover_above_food = [food_pos[0], food_pos[1], food_pos[2] + self.hover_height]
            hover_pose = SE3(transl(hover_above_food) @ rpy2tr(*tool_orientation))

            q_hover, success, error = self.solve_ik_robust(robot, hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach hover position for ingredient {i + 1}")
                continue

            print(f"  Moving to hover above food (IK+jtraj)...")
            self.execute_trajectory(robot, env, robot.q.copy(), q_hover, mesh=None)

            print(f"  Picking up ingredient {i + 1} (RMRC down)...")
            self.rmrc_vertical_movement(robot, env, hover_above_food, food_pos, tool_orientation, mesh=None, grip_offset=grip_offset)

            print(f"  Attaching mesh {i} to end effector...")
            attached_mesh = mesh

            print(f"  Lifting ingredient {i + 1} (RMRC up)...")
            self.rmrc_vertical_movement(robot, env, food_pos, hover_above_food, tool_orientation, mesh=attached_mesh, grip_offset=grip_offset)

            current_station_pos = [station_location[0], station_location[1], current_z]
            hover_above_station = [*current_station_pos[:2], current_station_pos[2] + self.hover_height]
            station_hover_pose = SE3(transl(hover_above_station) @ rpy2tr(*tool_orientation))

            q_station, success, error = self.solve_ik_robust(robot, station_hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach station hover position for ingredient {i + 1}")
                continue

            print(f"  Moving to hover above station (IK+jtraj)...")
            self.execute_trajectory(robot, env, robot.q.copy(), q_station, mesh=attached_mesh)

            print(f"  Placing ingredient {i + 1} at station (RMRC down)...")
            self.rmrc_vertical_movement(robot, env, hover_above_station, current_station_pos, tool_orientation, mesh=attached_mesh, grip_offset=grip_offset)

            if attached_mesh is not None:
                placed_meshes.append(attached_mesh)
            attached_mesh = None

            # Increment stack height for next ingredient
            current_z += 0.01  # fixed ingredient height

            print(f"  ✓ Ingredient {i + 1} completed! Stacking next at height {current_z:.3f} m")

        placed_meshes = [mesh for mesh in placed_meshes if mesh is not None]  # safety check
        print(f"Found {len(placed_meshes)} valid placed meshes to drag")

        return current_z

    def bread_movement(self, bread_locations, station_location, mesh_list=None):
        robot = self.robot
        env = self.env
        tool_orientation = [0, -pi / 2, pi]

        print(f"Processing order with {len(bread_locations)} ingredients...")
        print(f"Station location: {bread_locations}")

        placed_meshes = []
        hover_height = 0.2
        ingredient_spacing = 0.2  # sideways stacking step in y-axis

        current_station_pos = list(station_location)  # mutable copy

        for i, bread_pos in enumerate(bread_locations):
            mesh = mesh_list[i] if mesh_list is not None and i < len(mesh_list) else None
            if mesh is not None:
                print(f"  Attaching mesh {i}: {type(mesh)}")

            hover_above_food = [bread_pos[0] - hover_height, bread_pos[1], bread_pos[2]]
            hover_pose = SE3(transl(hover_above_food) @ rpy2tr(*tool_orientation))
            q_hover, success, error = self.solve_ik_robust(robot, hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach hover position for ingredient {i + 1}")
                continue
            self.execute_trajectory(robot, env, robot.q.copy(), q_hover, mesh=None)
            self.rmrc_vertical_movement(robot, env, hover_above_food, bread_pos, tool_orientation, mesh=None)

            attached_mesh = mesh
            grip_offset_bread = SE3.Ry(pi / 2) @ SE3.Rz(pi / 2) @ SE3.Ty(0.2)
            self.rmrc_vertical_movement(robot, env, bread_pos, hover_above_food, tool_orientation, mesh=attached_mesh, grip_offset=grip_offset_bread)

            hover_above_station = [current_station_pos[0], current_station_pos[1], current_station_pos[2] + hover_height]
            tool_orientation_station = [0, -pi / 2, 0]
            station_hover_pose = SE3(transl(hover_above_station) @ rpy2tr(*tool_orientation_station))
            q_station, success, error = self.solve_ik_robust(robot, station_hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach station hover position for ingredient {i + 1}")
                continue
            self.execute_trajectory(robot, env, robot.q.copy(), q_station, mesh=attached_mesh, grip_offset=grip_offset_bread)

            place_pos = current_station_pos
            self.rmrc_vertical_movement(robot, env, hover_above_station, place_pos, tool_orientation_station, mesh=attached_mesh, grip_offset=grip_offset_bread)

            # Increment y-axis for next slice placement
            current_station_pos[1] += ingredient_spacing

            print(f"  Retracting from station (RMRC up)...")
            self.rmrc_vertical_movement(robot, env, place_pos, hover_above_station, tool_orientation_station, grip_offset=grip_offset_bread)

            tool_orientation = [0, -pi / 2, pi]


def main():
    """Main function to run food order demo"""
    food_robot = FoodOrderRobotv1()


if __name__ == "__main__":
    main()
