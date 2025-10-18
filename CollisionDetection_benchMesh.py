import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialmath.base import transl
from numpy import pi, linalg
from Lilys_robot.XArm6 import XArm6
from ir_support import EllipsoidRobot
import trimesh

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

# ---------- Collision detection ----------
def detect_collisions(ellipsoid_robot, points):
    collisions = []
    for i in range(len(ellipsoid_robot.ellipsoid_matrices)):
        T, radii = ellipsoid_robot.get_ellipsoid_transform_and_radii(i)
        radii = np.maximum(radii, 1e-6)  # prevent divide by zero

        # Transform points into ellipsoid local frame
        points_h = np.hstack((points, np.ones((points.shape[0],1)))).T
        local_points = (linalg.inv(T) @ points_h).T[:, :3]

        algebra_dist = get_algebraic_dist(local_points, [0,0,0], radii)
        inside_indices = np.where(algebra_dist < 1)
        if inside_indices[0].size > 0:
            collisions.append((i, inside_indices[0].size))
    return collisions

# ---------- Teach function ----------
def teach_with_collision(XArm, ellipsoid_robot, fig, points):
    print("Ellipsoid teach mode started! Move robot via GUI if supported.")

    def on_update(*args):
        q_vec = XArm.q
        ellipsoid_robot.ellipsoid_for_robot_links(q_vec)
        ellipsoid_robot.plot_ellipsoids()

        collisions = detect_collisions(ellipsoid_robot, points)
        if collisions:
            for link_idx, n_points in collisions:
                print(f"⚠️  Collision detected: link {link_idx}, {n_points} points inside")
        else:
            print("✅ No collisions detected")

        fig.step(0.01)

    ellipsoid_robot.teach(on_update=on_update)

# ---------- Load bench STL and sample point cloud ----------
def load_bench_point_cloud(stl_file, num_points=10000):
    # current_path = os.path.abspath(os.path.dirname(__file__))
    stl_file = r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\bench.stl"
    # Load the STL file
    mesh = trimesh.load(stl_file)

    # Optional: scale to preserve proportions

    points = mesh.sample(num_points)
    return points

# ---------- Main function ----------
def question_robot_table_collision():
    plt.close('all')
    print("=== Robot vs Bench Collision Detection ===")


    # 1) Setup robot and ellipsoids
    robot = XArm6()
    robot.base = SE3(0,0,1)
    fig = robot.plot(robot.q, limits=[-0.5,0.5,-0.5,0.5,-0.1,0.5])

    ellipsoid_robot = EllipsoidRobot(robot, fig=fig, default_height=0.05, default_width=0.05)
    ellipsoid_robot.ellipsoid_for_robot_links(robot.q)
    ellipsoid_robot.plot_ellipsoids()
    fig.step(0.01)

    # 2) Load bench STL and sample points
    bench_points = load_bench_point_cloud("bench.stl", num_points=10000)

    # Plot bench point cloud
    ax = plt.gca()
    ax.scatter(bench_points[:,0], bench_points[:,1], bench_points[:,2],
               c='r', s=2, label='Bench')
    plt.pause(0.01)

    # 3) Launch teach mode with collision checking
    teach_with_collision(robot, ellipsoid_robot, fig, bench_points)

if __name__=="__main__":
    question_robot_table_collision()
