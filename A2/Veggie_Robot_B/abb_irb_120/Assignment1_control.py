# move_irb120_demo.py
# Place in: abb_irb_120/Robot_control/
# Runs a simple pick→place style move using robust IK + jtraj on the ABB IRB120 model.

import os
import sys
import time
import numpy as np

import swift
import roboticstoolbox as rtb
from roboticstoolbox import jtraj
from spatialmath import SE3

from Robot_info.abb_irb_120 import abb_irb_120

class Assignment2Control:

    def __init__(self):
        # Add this line: default IK solver tolerance (e.g. 1e-6 is typical)
        self.convergence_tolerance = 1e-6

    # ---------- Utilities ----------
    def enforce_joint_limits(self, q, qlim):
        """Clamp a 1D joint array to robot limits."""
        return np.clip(q, qlim[0, :], qlim[1, :])

    def pose_error_mm(self, T_actual, T_target):
        """Translation error (mm)."""
        return float(np.linalg.norm(T_actual.t - T_target.t) * 1000.0)

    def generate_multiple_initial_guesses(self, robot, target_pose, current_q):
        """Generate multiple initial guesses for IK solver"""
        guesses = []

        # 1. Current position
        # Don't wan't future changes of q to affect guesses unless directly called on
        guesses.append(current_q.copy())    

        # 2. Zero position
        # Adds another array of 0's inside guesses by amount of robot joints
        guesses.append(np.zeros(robot.n))

        # 3. Random positions within joint limits
        # In range 3 provides multiple starting points for IK solver, increasing chance of finding a valid solution
        for _ in range(3):
            # Random q value between q limits per current q is found and appended in guesses list
            q_rand = np.random.uniform(robot.qlim[0, :], robot.qlim[1, :])
            guesses.append(q_rand)

        # 4. Home position if available
        if hasattr(robot, 'qr'):
            guesses.append(robot.qr)

        return guesses

    def solve_ik_robust(self, robot, target_pose, q_current):
        """Robust IK solver with multiple strategies"""
        qlim = robot.qlim

        # Strategy 1: Standard ikine_LM with current position
        ik_result = robot.ikine_LM(
            target_pose, 
            q0=q_current, 
            mask=[1, 1, 1, 1, 1, 1], 
            ilimit=2000,  # Increased iteration limit (amount of steps taken till it still cannot converge)
            slimit=10,     # Multiple searches (re-tries)
            tol=self.convergence_tolerance,
            joint_limits=True
        )

        # If the solution has been found within specified tolerance
        if ik_result.success:
            # Enforce joint angles when finding the solution
            q_solution = self.enforce_joint_limits(ik_result.q, qlim)
            # Show error is within 50mm between fkine of solution found and target pose
            error = self.pose_error_mm(robot.fkine(q_solution), target_pose)
            if error <= 5.0:
                print("IK Solution q, strat 1:", q_solution)
                print(f"  IK q_solution (radians): {q_solution}")
                print(f"  IK q_solution (deg): {np.degrees(q_solution)}")
                # Returns calculated joint angles from ik solver, whether solution was succesful and numeric pose error (in mm)
                return q_solution, True, error
                
                

        # Strategy 2: Multiple initial guesses
        guesses = self.generate_multiple_initial_guesses(robot, target_pose, q_current)

        # Uses guesses to find a possible joint configuration for ikine, resulted if strat 1 doesnt work
        # Avoids getting stuck into a local minima or fails to converge if initial guess is poor
        for i, q0 in enumerate(guesses):
            ik_result = robot.ikine_LM(
                target_pose, 
                q0=q0, 
                mask=[1, 1, 1, 1, 1, 1], 
                ilimit=2000,
                slimit=3,
                tol=self.convergence_tolerance,
                joint_limits=True
            )
            # If the solution has been found within specified tolerance
            if ik_result.success:
                # Enforce joint angles when finding the solution
                q_solution = self.enforce_joint_limits(ik_result.q, qlim)
                # Show error is within 50mm between fkine of solution found and target pose
                error = self.pose_error_mm(robot.fkine(q_solution), target_pose)
                if error <= 5.0:
                    print(f"  Success with guess {i+1}, error: {error:.2f}mm")
                    print("IK Solution q, strat 2:", q_solution)
                    print(f"  IK q_solution (radians): {q_solution}")
                    print(f"  IK q_solution (deg): {np.degrees(q_solution)}")
                    # Returns calculated joint angles from ik solver, whether solution was succesful and numeric pose error (in mm)
                    return q_solution, True, error
                    
        # Returns q_current and false indicating unsuccesful IK solving if both strategies fail. Float('inf') represents an infinite pose error 
        return q_current, False, float('inf')
    
    def move_between_positions(self, robot, env, manager, initial_positions, goal_positions, steps=50):
        # Current joint values of robot copy
        q_current = robot.q.copy()
        
        # End Effector relative to global frame
        pick_pose = SE3.Trans(0, 0, 0.1)

        # From Ik solver function, finds q pick value, the success of solver and the pick error
        q_pick, pick_success, pick_error = self.solve_ik_robust(robot, pick_pose, q_current)    # Uses robot, pick_pose and q_current

        print(f"  Pick pose 1 FK error: {pick_error:.2f} mm {'(PASS)' if pick_error <= 5 else '(FAIL)'}")
        if not pick_success or pick_error > 5:  
            print(f"  Skipping 1 due to IK failure")

        # Execute pick movement
        traj_pick = jtraj(q_current, q_pick, steps) # Generates a smooth minimum-jerk trajectory from q_current to q_pick where steps = 50
        checkpoint_interval = max(1, steps // 5)  # ~20% checkpoints, never 0

        # Loops through step_idx and q 50 times (traj_pick.q) beginning at 1, where step_idx is the iteration no. and q = traj_pick.q for each step
        for step_idx, q in enumerate(traj_pick.q):
            # occasional progress print
            # Either prints EE pose every time remainder of step_idx is 0 (divisible by 5), if step_idx = 1 pr of at last step
            if step_idx % checkpoint_interval == 0 or step_idx == 1 or step_idx == len(traj_pick.q):
                ee_pose = robot.fkine(q)
                # Prints step/number of steps for q: EE @ the EE pose tranform
                print(f"    Step {step_idx}/{len(traj_pick.q)}: EE @ {ee_pose.t.round(3)} (m)")
            # Set joint values to q for iteration (shows robot moving in env)
            robot.q = q
            env.step(0.01)
        q_current = q_pick

        # Place phase
        # End Effector relative to global frame
        place_pose = SE3.Trans(0.5, 0, 0.6)

        q_place, place_success, place_error = self.solve_ik_robust(robot, place_pose, q_current)

        print(f"  Place pose 1 FK error: {place_error:.2f} mm {'(PASS)' if place_error <= 5 else '(FAIL)'}")
        # Will skip brick if either of these fail
        if not place_success or place_error > 5:    
            print(f"  Skipping place due to IK failure")

        # Execute place movement
        traj_place = jtraj(q_current, q_place, steps)   # Generates a smooth minimum-jerk trajectory from q_current to q_place where steps = 50
        checkpoint_interval = max(1, steps // 5)  # ~20% checkpoints, never 0

        # Loops through step_idx and q 50 times (traj_pick.q) beginning at 1, where step_idx is the iteration no. and q = traj_pick.q for each step
        for step_idx, q in enumerate(traj_place.q):
            # occasional progress print
            # Either prints EE pose every time remainder of step_idx is 0 (divisible by 5), if step_idx = 1 pr of at last step
            if step_idx % checkpoint_interval == 0 or step_idx == 1 or step_idx == len(traj_place.q):
                ee_pose = robot.fkine(q)
                # Prints step/number of steps for q: EE @ the EE pose tranform
                print(f"    Step {step_idx}/{len(traj_place.q)}: EE @ {ee_pose.t.round(3)} (m)") 
            robot.q = q
            env.step(0.01)
        q_current = q_place


# ---------- Demo pipeline ----------
def run_demo():
    """
    Spawns the IRB120 into Swift and moves the TCP from
    (0.5, 0.2, 0.0) to (0.2, 0.2, 0.0) with a down-facing tool.
    """
    env = swift.Swift()
    env.launch(realtime=True)

    robot = abb_irb_120()

    robot.add_to_env(env)

    # Set the camera pose in the simulation environment (eye position and target look direction)
    env.set_camera_pose([2, -2, 2], [0, 0, 0])
    # Take one simulation step to update the environment (renders updates)
    env.step()

    manager = None
    initial_positions = None
    goal_positions = None

    mover = Assignment2Control()
    mover.move_between_positions(robot, env, manager, initial_positions, goal_positions)

    input("Press Enter to exit and close Swift...")
    env.close()

if __name__ == "__main__":
    run_demo()
