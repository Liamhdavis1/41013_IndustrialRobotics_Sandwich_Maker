import numpy as np
from scipy import linalg
import spatialmath.base as spb

def get_algebraic_dist(points, center_point, radii):
    return np.sum(((points - center_point) / radii) ** 2, axis=1)

def collision_detection_ellipsoids(robot, ellipsoid_info_list, points):
    collision_results = []
    points_inside_each = []

    for i, (T, radii) in enumerate(ellipsoid_info_list):
        # Transform points into ellipsoid's local frame using inverse transform
        homo_points = np.hstack((points, np.ones((points.shape[0], 1)))).T  # 4 x N
        points_local_homo = linalg.inv(T) @ homo_points
        points_local = points_local_homo[:3, :].T  # N x 3

        # Calculate algebraic distances inside ellipsoid local coords with center at origin
        algebraic_dist = get_algebraic_dist(points_local, center_point=np.array([0, 0, 0]), radii=radii)
        inside_indices = np.where(algebraic_dist < 1)[0]

        collided = inside_indices.size > 0
        collision_results.append(collided)
        points_inside_each.append(inside_indices)

    return collision_results, points_inside_each
