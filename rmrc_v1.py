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
from Robot_info.abb_irb_120 import abb_irb_120



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
        """
        Main demo function following lab structure
        """
        print("=== Simplified RMRC Demo for ABB IRB120 ===")
        
        # 1) Initialize Swift environment and robot
        env = swift.Swift()
        env.launch(realtime=True)
        print("Swift environment launched")
        
        robot = abb_irb_120()
        robot.add_to_env(env)
        print("ABB IRB120 added to environment")
        
        env.set_camera_pose([2, -2, 2], [0, 0, 0])
        env.step()
        
        # 2) Allocate array data (following lab style)
        m = np.zeros([self.steps, 1])              # Array for Measure of Manipulability
        q_matrix = np.zeros([self.steps, 6])       # Array for joint angles
        qdot = np.zeros([self.steps, 6])           # Array for joint velocities
        theta = np.zeros([3, self.steps])          # Array for roll-pitch-yaw angles
        x = np.zeros([3, self.steps])              # Array for x-y-z trajectory
        position_error = np.zeros([3, self.steps]) # For plotting trajectory error
        angle_error = np.zeros([3, self.steps])    # For plotting trajectory error
        
        # 3) Set up trajectory (straight line from [0.5,0,0] to [0.5,0,0.6])
        start_pos = [0.5, 0.0, 0.0]
        end_pos = [0.5, 0.0, 0.6]
        
        for i in range(self.steps):
            s = i / (self.steps - 1)               # Linear interpolation parameter
            x[0, i] = (1-s) * start_pos[0] + s * end_pos[0]  # X trajectory
            x[1, i] = (1-s) * start_pos[1] + s * end_pos[1]  # Y trajectory  
            x[2, i] = (1-s) * start_pos[2] + s * end_pos[2]  # Z trajectory
            theta[0, i] = pi                       # Roll angle (tool pointing down)
            theta[1, i] = 0                        # Pitch angle
            theta[2, i] = 0                        # Yaw angle
            
        print(f"Trajectory: {start_pos} → {end_pos}")
        print(f"Time: {self.t}s, Steps: {self.steps}, dt: {self.delta_t}s")
        
        # 4) Solve initial configuration using IK
        T_start = SE3(transl(x[:, 0]) @ rpy2tr(theta[0, 0], theta[1, 0], theta[2, 0]))
        q_start, ik_success, ik_error = self.solve_ik_robust(robot, T_start, robot.q)

        if not ik_success or ik_error > 5.0:
            print(f"Initial IK failed! Error: {ik_error:.1f}mm")
            env.close()
            return

        print(f"Initial configuration solved. Error: {ik_error:.1f}mm")
        print(f"Initial joint angles (deg): {np.degrees(q_start)}")

        # Use jtraj for a smooth transition
        q_current = robot.q.copy()              # current joint values
        self.execute_trajectory(robot, env, q_current, q_start, steps=50)

        # Once motion is complete, set starting config & prepare arrays
        q_matrix[0, :] = q_start
        robot.q = q_start
        env.step()
        time.sleep(1)


        
        # 5) RMRC trajectory tracking (following lab methodology)
        for i in range(self.steps-1):
            # Get current transform
            T_current = robot.fkine(q_matrix[i, :]).A
            
            # Calculate position error for next waypoint
            delta_x = x[:, i+1] - T_current[:3, 3]
            
            # Calculate orientation matrices
            Rd = rpy2tr(theta[0, i+1], theta[1, i+1], theta[2, i+1])[:3, :3]  # This is 3x3 # Desired rotation matrix
            Ra = T_current[:3, :3]                                     # Current rotation matrix
            Rdot = (1/self.delta_t) * (Rd - Ra)                       # Rotation matrix derivative
            S = Rdot @ Ra.T                                           # Skew symmetric matrix
            
            # Calculate velocities
            linear_velocity = (1/self.delta_t) * delta_x
            angular_velocity = np.array([S[2, 1], S[0, 2], S[1, 0]])  # Extract from skew symmetric
            
            # Apply weighting matrix
            xdot = self.W @ np.vstack((linear_velocity.reshape(3, 1), 
                                     angular_velocity.reshape(3, 1)))
            
            # Get Jacobian and calculate manipulability
            J = robot.jacob0(q_matrix[i, :])
            m[i] = np.sqrt(linalg.det(J @ J.T))
            
            # Apply Damped Least Squares if needed
            if m[i] < self.epsilon:
                m_lambda = (1 - m[i]/self.epsilon) * 0.05
            else:
                m_lambda = 0
                
            # Calculate joint velocities using DLS inverse
            inv_j = linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T
            qdot[i, :] = (inv_j @ xdot).T
            
            # Apply joint limits (following lab style)
            qlim = np.transpose(robot.qlim)
            for j in range(6):
                if q_matrix[i, j] + self.delta_t * qdot[i, j] < qlim[j, 0]:
                    qdot[i, j] = 0  # Stop the motor
                elif q_matrix[i, j] + self.delta_t * qdot[i, j] > qlim[j, 1]:
                    qdot[i, j] = 0  # Stop the motor
            
            # Update joint state
            q_matrix[i+1, :] = q_matrix[i, :] + self.delta_t * qdot[i, :]
            
            # Store errors for plotting
            position_error[:, i] = delta_x
            delta_theta = tr2rpy(Rd @ Ra.T, order='xyz')
            angle_error[:, i] = delta_theta
            
            # Update robot and environment
            robot.q = q_matrix[i+1, :]
            env.step(self.delta_t)
            
            # Print progress (every 10%)
            if i % (self.steps // 10) == 0:
                actual_pos = robot.fkine(robot.q).t
                error_norm = np.linalg.norm(delta_x)
                print(
                        f"Step {i:3d}/{self.steps}: "
                        f"Target={np.array2string(x[:, i+1], precision=3)}, "
                        f"Actual={np.array2string(actual_pos, precision=3)}, "
                        f"Error={error_norm*1000:.1f}mm, "
                        f"Manip={m[i,0]:.3f}"

                    )

        
        # 6) Final results
        final_pos = robot.fkine(robot.q).t
        final_error = np.linalg.norm(final_pos - np.array(end_pos))
        print(f"\n=== Trajectory Completed ===")
        print(f"Target end position: {end_pos}")
        print(f"Actual end position: {final_pos}")
        print(f"Final position error: {final_error*1000:.1f}mm")
        
        # 7) Plot results (following lab style)
        self.plot_results(x, q_matrix, qdot, m, position_error, angle_error, robot)
        
        input("Press Enter to close Swift environment...")
        env.close()
        
        return q_matrix, qdot, m
    
    def plot_results(self, x_traj, q_matrix, qdot_matrix, m, position_error, angle_error, robot):
        """
        Plot trajectory results following lab methodology
        """
        time_array = np.arange(self.steps) * self.delta_t
        
        # Calculate actual positions
        actual_positions = np.zeros((3, self.steps))
        for i in range(self.steps):
            T = robot.fkine(q_matrix[i, :])
            actual_positions[:, i] = T.t
        
        # Create plots
        plt.figure(figsize=(15, 10))
        
        # Position tracking
        plt.subplot(2, 3, 1)
        plt.plot(time_array, x_traj[0, :], 'r-', label='Desired X', linewidth=2)
        plt.plot(time_array, x_traj[1, :], 'g-', label='Desired Y', linewidth=2)
        plt.plot(time_array, x_traj[2, :], 'b-', label='Desired Z', linewidth=2)
        plt.plot(time_array, actual_positions[0, :], 'r--', label='Actual X')
        plt.plot(time_array, actual_positions[1, :], 'g--', label='Actual Y')
        plt.plot(time_array, actual_positions[2, :], 'b--', label='Actual Z')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Position Tracking')
        plt.legend()
        plt.grid(True)
        
        # Joint angles
        plt.subplot(2, 3, 2)
        for i in range(6):
            plt.plot(time_array, np.degrees(q_matrix[:, i]), label=f'Joint {i+1}')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Angle (deg)')
        plt.title('Joint Angles')
        plt.legend()
        plt.grid(True)
        
        # Joint velocities  
        plt.subplot(2, 3, 3)
        for i in range(6):
            plt.plot(time_array[:-1], np.degrees(qdot_matrix[:-1, i]), label=f'Joint {i+1}')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Velocity (deg/s)')
        plt.title('Joint Velocities')
        plt.legend()
        plt.grid(True)
        
        # Manipulability
        plt.subplot(2, 3, 4)
        plt.plot(time_array[:-1], m[:-1], 'k-', linewidth=2)
        plt.axhline(self.epsilon, color='r', linestyle='--', label=f'Threshold ({self.epsilon})')
        plt.xlabel('Time (s)')
        plt.ylabel('Manipulability')
        plt.title('Manipulability Measure')
        plt.legend()
        plt.grid(True)
        
        # Position errors
        plt.subplot(2, 3, 5)
        plt.plot(time_array[:-1], position_error[0, :-1] * 1000, label='X Error')
        plt.plot(time_array[:-1], position_error[1, :-1] * 1000, label='Y Error') 
        plt.plot(time_array[:-1], position_error[2, :-1] * 1000, label='Z Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Position Error (mm)')
        plt.title('Position Tracking Errors')
        plt.legend()
        plt.grid(True)
        
        # 3D trajectory comparison
        ax = plt.subplot(2, 3, 6, projection='3d')
        ax.plot(x_traj[0, :], x_traj[1, :], x_traj[2, :], 'r-', linewidth=3, label='Desired')
        ax.plot(actual_positions[0, :], actual_positions[1, :], actual_positions[2, :], 
                'b--', linewidth=2, label='Actual')
        ax.scatter(x_traj[0, 0], x_traj[1, 0], x_traj[2, 0], c='green', s=100, label='Start')
        ax.scatter(x_traj[0, -1], x_traj[1, -1], x_traj[2, -1], c='red', s=100, label='End')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Trajectory')
        ax.legend()
        
        plt.tight_layout()
        plt.show()


def main():
    """
    Main function to run the simplified RMRC demo
    """
    rmrc = SimplifiedRMRC()
    rmrc.run_rmrc_demo()


if __name__ == "__main__":
    main()
