import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialmath.base import transl
from numpy import pi, linalg
from Lilys_robot.XArm6 import XArm6
from ir_support import EllipsoidRobot

# ---------- Helper Rotations ----------
def rotx(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def roty(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

def get_algebraic_dist(points, center_point, radii):
    return np.sum(((points - center_point)/radii)**2, axis=1)

# ---------- Collision detection against cube ----------
def detect_cube_collisions(ellipsoid_robot, cube_points):
    collisions = []
    for i in range(len(ellipsoid_robot.ellipsoid_matrices)):
        T, radii = ellipsoid_robot.get_ellipsoid_transform_and_radii(i)
        radii = np.maximum(radii, 1e-6)  # prevent divide by zero

        # Transform cube points into ellipsoid local frame
        cube_points_h = np.hstack((cube_points, np.ones((cube_points.shape[0],1)))).T
        updated_cube_points = (linalg.inv(T) @ cube_points_h).T[:, :3]

        algebra_dist = get_algebraic_dist(updated_cube_points, [0,0,0], radii)
        inside_indices = np.where(algebra_dist < 1)
        if inside_indices[0].size > 0:
            collisions.append((i, inside_indices[0].size))
    return collisions

# ---------- Teach function ----------
def teach_with_cube_collision(XArm, ellipsoid_robot, fig, cube_points):
    print("Ellipsoid teach mode started! Move robot via GUI if supported.")

    def on_update(*args):
        q_vec = XArm.q
        ellipsoid_robot.ellipsoid_for_robot_links(q_vec)
        ellipsoid_robot.plot_ellipsoids()

        collisions = detect_cube_collisions(ellipsoid_robot, cube_points)
        if collisions:
            for link_idx, n_points in collisions:
                print(f"⚠️  Collision detected: link {link_idx}, {n_points} cube points inside")
        else:
            print("✅ No collisions with cube detected")

        fig.step(0.01)

    ellipsoid_robot.teach(on_update=on_update)

# ---------- Main robot vs cube function ----------
def question_robot_table_collision():
    plt.close('all')
    print("=== Robot vs Cube Collision Detection ===")

    # 1) Setup robot and ellipsoids
    robot = XArm6()
    robot.base = SE3(0,0,0.2)
    fig = robot.plot(robot.q, limits=[-0.5,0.5,-0.5,0.5,-0.1,0.5])

    ellipsoid_robot = EllipsoidRobot(robot, fig=fig, default_height=0.05, default_width=0.05)
    ellipsoid_robot.ellipsoid_for_robot_links(robot.q)
    ellipsoid_robot.plot_ellipsoids()
    fig.step(0.01)

    # 2) Cube points
    cube_center = np.array([0,0,-0.05])
    cube_halfsize = 0.25
    Y,Z = np.meshgrid(np.arange(-cube_halfsize, cube_halfsize+0.02, 0.02),
                      np.arange(-cube_halfsize, cube_halfsize+0.02, 0.02))
    size_mat = Y.shape
    X = np.ones(size_mat)*cube_halfsize
    one_face = np.hstack((X.reshape(-1,1), Y.reshape(-1,1), Z.reshape(-1,1)))
    cube_points = [
        one_face,
        one_face @ rotz(pi/2),
        one_face @ rotz(pi),
        one_face @ rotz(3*pi/2),
        one_face @ roty(pi/2),
        one_face @ roty(-pi/2)
    ]
    cube_points = np.concatenate(cube_points)
    cube_points += np.tile(cube_center, (cube_points.shape[0],1))

    # Plot cube
    plt.gca().plot(cube_points[:,0], cube_points[:,1], cube_points[:,2],
                   'r.', markersize=2, label='Cube (Table)')
    plt.pause(0.01)

    # 3) Launch teach mode with cube collision checking
    teach_with_cube_collision(robot, ellipsoid_robot, fig, cube_points)

if __name__=="__main__":
    question_robot_table_collision()
