import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath.base import *
from spatialmath import SE3
from math import pi
from Robot_info.abb_irb_120 import abb_irb_120

def rmrc_for_abb_irb_120():
    # Create robot instance
    robot = abb_irb_120()
    
    # Simulation parameters
    t = 6                              # total time (s)
    delta_t = 0.02                    # time step
    steps = int(t / delta_t)          # number of simulation steps
    epsilon = 0.1                     # manipulability threshold for damping
    W = np.diag([1, 1, 1, 0.1, 0.1, 0.1])  # weighting matrix (position vs orientation)
    
    # Define trajectory in task space: from (0,0,0) to (0,0,0.2) with fixed orientation (all zeros)
    x = np.zeros((3, steps))
    theta = np.zeros((3, steps))  # roll, pitch, yaw all zero
    
    # Linear interpolation in z from 0 to 0.2 m
    for i in range(steps):
        s = i / (steps - 1)
        x[:, i] = [0, 0, 0 + 0.2 * s]
        # Fixed orientation (all zeros) already set
    
    # Initial end-effector pose
    T0 = SE3.Trans(x[:, 0]) * SE3.RPY(theta[:, 0], order="xyz", unit="rad")
    q0_guess = np.zeros(6)
    
    # Solve IK for initial pose
    ik_sol = robot.ikine_LM(T0, q0=q0_guess)
    if not ik_sol.success:
        raise RuntimeError("Initial IK solution not found")
    q_matrix = np.zeros((steps, 6))
    q_matrix[0, :] = ik_sol.q
    qdot = np.zeros((steps, 6))
    
    qlim = np.transpose(robot.qlim)
    
    # RMRC control loop
    for i in range(steps - 1):
        T_current = robot.fkine(q_matrix[i, :]).A
        
        # Position and orientation errors
        delta_pos = x[:, i + 1] - T_current[:3, 3]
        Rd = rpy2r(theta[:, i + 1], order="xyz")
        Ra = T_current[:3, :3]
        Rdot = (1 / delta_t) * (Rd - Ra)
        S = Rdot @ Ra.T
        
        linear_vel = delta_pos / delta_t
        angular_vel = np.array([S[2, 1], S[0, 2], S[1, 0]])
        xdot = W @ np.vstack((linear_vel.reshape(3, 1), angular_vel.reshape(3, 1)))
        
        # Jacobian and manipulability
        J = robot.jacob0(q_matrix[i, :])
        manipulability = np.sqrt(np.linalg.det(J @ J.T))
        
        # Damped Least Squares for near singularities
        if manipulability < epsilon:
            damping_lambda = (1 - manipulability / epsilon) * 0.05
        else:
            damping_lambda = 0
        
        # Compute damped pseudo-inverse
        J_inv = linalg.inv(J.T @ J + damping_lambda * np.eye(6)) @ J.T
        
        # Calculate joint velocities
        qdot[i, :] = (J_inv @ xdot).flatten()
        
        # Joint limit enforcement
        for j in range(6):
            future_q = q_matrix[i, j] + delta_t * qdot[i, j]
            if future_q < qlim[j, 0] or future_q > qlim[j, 1]:
                qdot[i, j] = 0  # Stop joint if limit exceeded
        
        # Integrate joint velocities
        q_matrix[i + 1, :] = q_matrix[i, :] + delta_t * qdot[i, :]
        robot.q = q_matrix[i + 1, :]  # Update robot model for visualization (if needed)
    
    # Visualization (optional if using Swift or another sim)
    fig = robot.plot(q_matrix[0, :], limits=[-1, 1, -1, 1, 0, 1])
    fig.ax.plot(x[0, :], x[1, :], x[2, :], "k.", linewidth=0.2)
    for q in q_matrix:
        robot.q = q
        ee_pos = robot.fkine(q).A[:3, 3]
        fig.ax.plot(ee_pos[0], ee_pos[1], ee_pos[2], "r.", markersize=0.5)
        fig.step(delta_t)

    plt.show()
    input("Press Enter to finish...")

if __name__ == "__main__":
    rmrc_for_abb_irb_120()
