import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialmath.base import transl, rpy2tr, tr2rpy
from math import pi
from Robot_info.abb_irb_120 import abb_irb_120


def rmrc_vertical_trajectory():
    # Initialize robot
    robot = abb_irb_120()

    # Simulation parameters
    t = 12.0                   # Total time in seconds
    delta_t = 0.02             # Control timestep
    steps = int(t / delta_t)   # Number of control steps

    # Trajectory: fixed x=0.5, y=0, move z from 0 to 0.6 along waypoints
    x = np.zeros((3, steps))
    z_traj = np.linspace(0, 0.6, steps)
    x[0, :] = 0.5      # x fixed
    x[1, :] = 0.0      # y fixed
    x[2, :] = z_traj   # z increases linearly

    # Fixed orientation: tool pointing downward (rotate 180 deg about x)
    theta = np.zeros((3, steps))  # roll, pitch, yaw
    theta[0, :] = pi  # Roll 180 degrees

    # Preallocate joint state and velocity matrices
    q_matrix = np.zeros((steps, 6))
    qdot_matrix = np.zeros((steps, 6))
    position_error = np.zeros((3, steps))
    angle_error = np.zeros((3, steps))
    m = np.zeros((steps, 1))  # Manipulability measure

    # Initial pose transformation and IK to find q0
    T0 = transl(x[:, 0]) @ rpy2tr(theta[0, 0], theta[1, 0], theta[2, 0])
    q0_guess = np.zeros(6)
    ik_result = robot.ikine_LM(T0, q0=q0_guess)
    if not ik_result.success:
        raise RuntimeError("Initial IK solution not found.")
    q_matrix[0, :] = ik_result.q

    qlim = np.transpose(robot.qlim)

    # Weighting matrix for velocity components (pos vs orientation)
    W = np.diag([1, 1, 1, 0.1, 0.1, 0.1])
    epsilon = 0.1  # Manipulability threshold

    # RMRC control loop
    for i in range(steps - 1):
        T_current = robot.fkine(q_matrix[i, :]).A
        T_desired = transl(x[:, i + 1]) @ rpy2tr(theta[0, i + 1], theta[1, i + 1], theta[2, i + 1])

        # Position error
        delta_pos = T_desired[:3, 3] - T_current[:3, 3]

        # Orientation error - compute rotation difference
        Rd = T_desired[:3, :3]
        Ra = T_current[:3, :3]
        Rdot = (Rd - Ra) / delta_t
        S = Rdot @ Ra.T

        linear_velocity = delta_pos / delta_t
        angular_velocity = np.array([S[2, 1], S[0, 2], S[1, 0]])

        # Compose task space velocity vector
        xdot = W @ np.vstack((linear_velocity.reshape(3, 1), angular_velocity.reshape(3, 1)))

        # Compute geometric Jacobian at current joint configuration
        J = robot.jacob0(q_matrix[i, :])
        
        # Measure manipulability
        m[i] = np.sqrt(np.linalg.det(J @ J.T))

        # Damped least squares if near singularity
        if m[i] < epsilon:
            damp_lambda = (1 - m[i] / epsilon) * 0.05
        else:
            damp_lambda = 0

        # Compute DLS inverse
        J_inv = linalg.inv(J.T @ J + damp_lambda * np.eye(6)) @ J.T

        # Compute joint velocities
        qdot = (J_inv @ xdot).flatten()

        # Clamp velocities to respect joint limits after integration
        q_next = q_matrix[i, :] + delta_t * qdot
        for j in range(6):
            if q_next[j] < qlim[j, 0] or q_next[j] > qlim[j, 1]:
                qdot[j] = 0      # Freeze the joint velocity
                q_next[j] = q_matrix[i, j]

        # Save joint states and velocities
        q_matrix[i + 1, :] = q_next
        qdot_matrix[i, :] = qdot

        # Log errors for plotting
        position_error[:, i] = delta_pos
        angle_error[:, i] = tr2rpy(Rd @ Ra.T, order='xyz')

        # Update robot configuration for visualization
        robot.q = q_next

    # Plot desired vs actual Z positions
    plt.figure()
    plt.plot(z_traj, 'r-', label='Desired Z')
    actual_z = [robot.fkine(q).t[2] for q in q_matrix]
    plt.plot(actual_z, 'b--', label='Actual EE Z')
    plt.xlabel('Step')
    plt.ylabel('Z Position (m)')
    plt.title('RMRC Vertical Z-axis Tracking')
    plt.legend()
    plt.grid(True)

    # Plot manipulability measure
    plt.figure()
    plt.plot(m, 'k-', label='Manipulability')
    plt.axhline(epsilon, color='r', linestyle='--', label='Threshold')
    plt.xlabel('Step')
    plt.ylabel('Manipulability')
    plt.legend()
    plt.grid(True)

    plt.show()

    input("Press ENTER to finish...")


if __name__ == "__main__":
    rmrc_vertical_trajectory()
