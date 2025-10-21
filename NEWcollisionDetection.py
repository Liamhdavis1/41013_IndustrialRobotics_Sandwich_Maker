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
class CollsionDetection:
    def get_algebraic_dist(points, center_point, radii):
        return np.sum(((points - center_point)/radii)**2, axis=1)

    def detect_collisions(ellipsoid_robot, points):
        for i in range(len(ellipsoid_robot.ellipsoid_matrices)):
            T, radii = ellipsoid_robot.get_ellipsoid_transform_and_radii(i)
            radii = np.maximum(radii, 1e-6)
            points_h = np.hstack((points, np.ones((points.shape[0], 1)))).T
            local_points = (linalg.inv(T) @ points_h).T[:, :3]
            algebra_dist = CollsionDetection.get_algebraic_dist(local_points, [0, 0, 0], radii)
            if np.any(algebra_dist < 1):
                return True
        return False

    def load_mesh_points(stl_path, num_points=10000, pose=None):
        mesh = trimesh.load(stl_path)
        points = mesh.sample(num_points)
        if pose is not None:
            R = pose.A[:3, :3]; t = pose.t
            points = (R @ points.T).T + t
        return points


