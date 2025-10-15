import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from math import pi
from spatialmath import SE3

# Replace with your actual robot import
from Lilys_robot.XArm6 import XArm6  
from ir_support import EllipsoidRobot

# ------- Collision Detection between Ellipsoids -------
def detect_ellipsoid_collisions(ellipsoid_robot):
    n = len(ellipsoid_robot.ellipsoid_matrices)
    collisions = []
    transforms_radii = [ellipsoid_robot.get_ellipsoid_transform_and_radii(i) for i in range(n)]
    for i in range(n):
        Ti, ri = transforms_radii[i]
        ci = Ti[:3, 3]
        Ri = Ti[:3, :3]
        Qi = Ri @ np.diag(1 / (ri ** 2)) @ Ri.T
        for j in range(i + 1, n):
            Tj, rj = transforms_radii[j]
            cj = Tj[:3, 3]
            Rj = Tj[:3, :3]
            Qj = Rj @ np.diag(1 / (rj ** 2)) @ Rj.T
            # Eigenvalue method for intersection:
            try:
                eigvals = np.linalg.eigvals(np.linalg.inv(Qi) @ Qj)
                if np.any(np.real(eigvals) < 1):
                    collisions.append((i, j))
            except np.linalg.LinAlgError:
                # Singular matrix: treat as non-collision for safety
                continue
    return collisions

# ------- Teach Mode with Ellipsoid Collision Detection -------
def teach_with_collisions(robot, ellipsoid_robot, fig):
    print("Ellipsoid teach mode started! Move robot via GUI if supported.")

    # Callback triggered when robot configuration changes
    def on_update(q):
        robot.q = q
        ellipsoid_robot.ellipsoid_for_robot_links(q)
        ellipsoid_robot.plot_ellipsoids()
        collisions = detect_ellipsoid_collisions(ellipsoid_robot)
        if collisions:
            print(f"Collision detected between links: {collisions}")
        else:
            print("No collisions.")
        fig.step(0.01)  # update visualization

    ellipsoid_robot.teach(on_update=on_update)

# ------- Main Script -------
if __name__ == "__main__":
    # Initialize robot and ellipsoids
    XArm = XArm6()
    fig = XArm.plot(XArm.q, limits=[-1, 1, -1, 1, 0, 1], block=False)
    plt.draw()
    ellipsoid_robot = EllipsoidRobot(XArm, fig=fig, default_height=0.05, default_width=0.05)
    # Initial visualization
    ellipsoid_robot.ellipsoid_for_robot_links(XArm.q)
    ellipsoid_robot.plot_ellipsoids()
    fig.step(0.01)
    # Launch teach mode with ellipsoid collision checking
    teach_with_collisions(XArm, ellipsoid_robot, fig)
    fig.hold()


# import numpy as np
# import swift
# import roboticstoolbox as rtb
# import spatialmath.base as spb
# from spatialmath import SE3
# from spatialgeometry import Mesh
# import os
# import time
# from math import pi
# import matplotlib.pyplot as plt
# from Lilys_robot.XArm6 import XArm6
# from ir_support import EllipsoidRobot, schunk_UTS_v2_0, line_plane_intersection, make_ellipsoid

# # --- Setup environment ---
# # env = swift.Swift()
# # env.launch(realtime=True)

# XArm = XArm6()
# # XArm.base = SE3(0,0,0)
# # XArm.add_to_env(env)

# fig = XArm.plot(XArm.q, limits = [-1, 1, -1, 1, 0, 1], block=False)

# plt.draw()

# ellipsoid_robot = EllipsoidRobot(XArm, fig = fig, default_height= 0.05, default_width= 0.05)

# # q = [pi/6, 0, -pi/2, 0, 0, 0]
# ellipsoid_robot.ellipsoid_for_robot_links(XArm.q)
# ellipsoid_robot.plot_ellipsoids()
# fig.step(0.01)

# for i in range(len(ellipsoid_robot.ellipsoid_matrices)):
#     # Get the ellipsoid parameters for this link
#     T, radii = ellipsoid_robot.get_ellipsoid_transform_and_radii(i)            

# fig.hold()
# # env.hold()