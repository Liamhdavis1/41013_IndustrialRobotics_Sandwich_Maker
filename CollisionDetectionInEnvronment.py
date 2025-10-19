import numpy as np
import swift
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Mesh
from ir_support.robots.DHRobot3D import DHRobot3D
from Lilys_robot.XArm6 import XArm6
from Micahs_robot.abb_irb_120 import abb_irb_120
from ir_support import UR3
from ir_support import EllipsoidRobot
import trimesh
import os
from math import pi
from numpy import linalg

# ---------------- Collision detection helpers ----------------
def get_algebraic_dist(points, center_point, radii):
    return np.sum(((points - center_point)/radii)**2, axis=1)

def detect_collisions(ellipsoid_robot, points):
    collisions = []
    for i in range(len(ellipsoid_robot.ellipsoid_matrices)):
        T, radii = ellipsoid_robot.get_ellipsoid_transform_and_radii(i)
        radii = np.maximum(radii, 1e-6)

        # Transform points into ellipsoid local frame
        points_h = np.hstack((points, np.ones((points.shape[0], 1)))).T
        local_points = (linalg.inv(T) @ points_h).T[:, :3]

        algebra_dist = get_algebraic_dist(local_points, [0, 0, 0], radii)
        inside_indices = np.where(algebra_dist < 1)[0]
        if inside_indices.size > 0:
            collisions.append((i, inside_indices.size))
    return collisions

def load_mesh_points(stl_path, num_points=10000):
    mesh = trimesh.load(stl_path)
    return mesh.sample(num_points)

# ---------------- Environment setup ----------------
env = swift.Swift()
env.launch(realtime=True)

UR3_robot = UR3()
XArm = XArm6()
irb = abb_irb_120()

UR3_robot.base = SE3(1.75, 0.2, 1)
UR3_robot.add_to_env(env)

XArm.base = SE3(0.95, 0.25, 1)
XArm.add_to_env(env)

irb.base = SE3(0, 0.25, 1) @ SE3.Rz(-pi/2)
irb.add_to_env(env)

current_path = os.path.abspath(os.path.dirname(__file__))

ENV = {
    "bench": {
        "path": os.path.join(current_path, "env", "benchv2.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 1]
    },
    "glass": {
        "path": os.path.join(current_path, "env", "glass.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 0.4]
    },
    "bread_rack": {
        "path": os.path.join(current_path, "env", "bread_rack.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 1]
    }
}

spawned_meshes = {}

def spawn_obj(env, name, pose):
    info = ENV[name]
    mesh = Mesh(
        filename=info["path"],
        pose=pose,
        scale=info["scale"],
        color=info["color"]
    )
    env.add(mesh)
    spawned_meshes[f"{name}_{len(spawned_meshes)}"] = mesh
    return mesh

# Spawn environment
bench = spawn_obj(env, "bench", SE3(0, 0, 0))
glass = spawn_obj(env, "glass", SE3(0, 0, 0))
bread_rack = spawn_obj(env, "bread_rack", SE3(0, 0, 0))

# ---------------- Collision setup ----------------
bench_points = load_mesh_points(ENV["bench"]["path"], num_points=8000)
ellipsoid_XArm = EllipsoidRobot(XArm)
ellipsoid_UR3 = EllipsoidRobot(UR3_robot)
ellipsoid_irb = EllipsoidRobot(irb)

# ---------------- Robot motion + collision check ----------------
input('delay')
steps = 50
q0 = np.zeros(6)
q_pick = XArm.ikine_LM(SE3(1.15, -0.34, 0.88), q0=q0).q

for q in rtb.jtraj(XArm.q, q_pick, steps).q:
    XArm.q = q

    # Update ellipsoids for current config
    ellipsoid_XArm.ellipsoid_for_robot_links(XArm.q)
    ellipsoid_UR3.ellipsoid_for_robot_links(UR3_robot.q)
    ellipsoid_irb.ellipsoid_for_robot_links(irb.q)

    # Check collisions
    collisions_XArm = detect_collisions(ellipsoid_XArm, bench_points)
    collisions_UR3 = detect_collisions(ellipsoid_UR3, bench_points)
    collisions_irb = detect_collisions(ellipsoid_irb, bench_points)

    if collisions_XArm:
        print(f"⚠️ Collision detected for XArm6: {len(collisions_XArm)} links involved")
    if collisions_UR3:
        print(f"⚠️ Collision detected for UR3: {len(collisions_UR3)} links involved")
    if collisions_irb:
        print(f"⚠️ Collision detected for ABB IRB 120: {len(collisions_irb)} links involved")

    env.step(0.05)

env.hold()
