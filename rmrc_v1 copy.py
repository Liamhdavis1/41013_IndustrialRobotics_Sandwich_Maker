# rmrc_irb120_simplified.py
# Simplified RMRC implementation for ABB IRB120 following lab methodology
# Uses IK for initial positioning, then RMRC for trajectory control

import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import time
from roboticstoolbox import jtraj

import swift
from spatialmath.base import transl, rpy2tr, tr2rpy
from spatialmath import SE3
from math import pi

# Import the ABB IRB120 robot model
from Micahs_robot.abb_irb_120 import abb_irb_120



class SimplifiedRMRC:
    """
    Simplified RMRC implementation following lab methodology
    """
    
    def __init__(self):
        # RMRC Parameters (matching lab style)
        self.t = 12.0                              # Total time (s)
        self.delta_t = 0.02                        # Control frequency
        self.steps = int(self.t/self.delta_t)      # No. of steps for simulation
        self.epsilon = 0.1                         # Threshold for manipulability/DLS
        self.W = np.diag([1, 1, 1, 0.1, 0.1, 0.1]) # Weighting matrix for velocity vector
        
        # Initialize Assignment2Control for IK solving
        
    def execute_trajectory(self, robot, env, q_start, q_end, steps=50):
        """
        Execute smooth joint-space trajectory between q_start and q_end using jtraj.
        Updates robot in Swift environment each step.
        """
        traj = jtraj(q_start, q_end, steps)
        checkpoint_interval = max(1, steps // 5)

        for step_idx, q in enumerate(traj.q):
            if step_idx % checkpoint_interval == 0 or step_idx == 1 or step_idx == len(traj.q) - 1:
                ee_pose = robot.fkine(q)
                print(f"Step {step_idx}/{len(traj.q) - 1}: EE @ {ee_pose.t.round(3)} (m)")
            robot.q = q
            env.step(0.01)
        

    def enforce_joint_limits(self, q, qlim):
        return np.clip(q, qlim[0, :], qlim[1, :])

    def pose_error_mm(self, T_actual, T_target):
        return float(np.linalg.norm(T_actual.t - T_target.t) * 1000.0)

    def generate_initial_guesses(self, robot, q_current):
        guesses = []
        guesses.append(q_current.copy())           # current
        guesses.append(np.zeros(robot.n))           # zero position
        for _ in range(3):                          # random within limits
            q_rand = np.random.uniform(robot.qlim[0, :], robot.qlim[1, :])
            guesses.append(q_rand)
        if hasattr(robot, 'qr'):
            guesses.append(robot.qr)                # home position
        return guesses

    def solve_ik_robust(self, robot, target_pose, q_current):
        qlim = robot.qlim
        tol = 1e-6
        # Try initial guess
        ik_result = robot.ikine_LM(
            target_pose,
            q0=q_current,
            mask=[1, 1, 1, 1, 1, 1],
            ilimit=2000,
            slimit=10,
            tol=tol,
            joint_limits=True
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
                target_pose,
                q0=q0,
                mask=[1, 1, 1, 1, 1, 1],
                ilimit=2000,
                slimit=3,
                tol=tol,
                joint_limits=True
            )
            if ik_result.success:
                q_solution = self.enforce_joint_limits(ik_result.q, qlim)
                error = self.pose_error_mm(robot.fkine(q_solution), target_pose)
                if error <= 5.0:
                    return q_solution, True, error

        return q_current, False, float('inf')
                            


    def run_rmrc_demo(self):
        print("=== Simplified RMRC Demo for ABB IRB120 ===")
        
        # Initialize environment and robot
        env = swift.Swift()
        env.launch(realtime=True)
        robot = abb_irb_120()
        robot.add_to_env(env)
        env.set_camera_pose([2, -2, 2], [0, 0, 0])
        env.step()
        
        # Trajectory setup
        start_pos = [0.5, 0, 0]
        end_pos = [0.5, 0.1, 0.4]
        x = np.linspace(start_pos, end_pos, self.steps).T
        theta = np.tile([pi, 0, 0], (self.steps, 1)).T
        
        # Allocate arrays
        m = np.zeros(self.steps)
        q_matrix = np.zeros((self.steps, 6))
        qdot = np.zeros((self.steps, 6))
        pos_err = np.zeros((3, self.steps))
        ang_err = np.zeros((3, self.steps))

        # Solve first pose using IK
        T_start = SE3(transl(x[:, 0]) @ rpy2tr(*theta[:, 0]))
        q_start, success, err_mm = self.solve_ik_robust(robot, T_start, robot.q)
        if not success:
            print("Initial IK failed!")
            env.close()
            return
        self.execute_trajectory(robot, env, robot.q.copy(), q_start, steps=50)
        q_matrix[0, :] = q_start
        robot.q = q_start
        env.step()

        # RMRC main loop
        for i in range(self.steps - 1):
            T_now = robot.fkine(q_matrix[i, :]).A
            delta_x = x[:, i+1] - T_now[:3, 3]
            Rd = rpy2tr(*theta[:, i+1])[:3, :3]
            Ra = T_now[:3, :3]
            S = ((Rd - Ra) / self.delta_t) @ Ra.T
            lin_vel = delta_x / self.delta_t
            ang_vel = np.array([S[2, 1], S[0, 2], S[1, 0]])
            xdot = self.W @ np.vstack((lin_vel[:, None], ang_vel[:, None]))
            J = robot.jacob0(q_matrix[i, :])
            m[i] = np.sqrt(linalg.det(J @ J.T))
            m_lambda = (1 - m[i]/self.epsilon) * 0.05 if m[i] < self.epsilon else 0
            inv_J = linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T
            qdot[i, :] = (inv_J @ xdot).T
            # Joint limit enforcement
            q_next = q_matrix[i, :] + self.delta_t * qdot[i, :]
            q_next = np.clip(q_next, robot.qlim[0], robot.qlim[1])
            q_matrix[i+1, :] = q_next
            pos_err[:, i] = delta_x
            ang_err[:, i] = tr2rpy(Rd @ Ra.T, order='xyz')
            robot.q = q_next
            env.step(self.delta_t)
            if i % (self.steps // 10) == 0:
                print(f"Step {i}/{self.steps}: Error={np.linalg.norm(delta_x)*1000:.1f}mm, Manip={m[i]:.3f}")

        final_pos = robot.fkine(robot.q).t
        print(f"\n=== Completed ===\nTarget: {end_pos}\nActual: {final_pos}\nError: {np.linalg.norm(final_pos-np.array(end_pos))*1000:.1f}mm")
        input("Press Enter to close Swift environment...")
        env.close()
        return q_matrix, qdot, m    

    
def main():
    """
    Main function to run the simplified RMRC demo
    """
    rmrc = SimplifiedRMRC()
    rmrc.run_rmrc_demo()


if __name__ == "__main__":
    main()
