import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
from math import pi

class Cobot320(DHRobot3D):

    # scale = {
    #     'link0': 0.01,
    #     'link1': 0.01,
    #     'link2': 0.01,
    #     'link3': 0.01,
    #     'link4': 0.01,
    #     'link5': 0.01,
    #     'link6': 0.01
    # }
    def __init__(self):
        links = self._create_DH()

        link3D_names = dict(
            link0='Cobot320_STL/L1',
            link1='Cobot320_STL/L2',
            link2='Cobot320_STL/L3',
            link3='Cobot320_STL/L4',
            link4='Cobot320_STL/L5',
            link5='Cobot320_STL/L6',
            link6='Cobot320_STL/L7'
        )

        qtest = [0, 0, 0, 0, 0, 0]
        qtest_transforms = [
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
            spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2))
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links, link3D_names, name='myCobot', link3d_dir= current_path, qtest=qtest, qtest_transforms=qtest_transforms
        )

        self.q = qtest

    def _create_DH(self):
        links = []
        a = [0, 1.1, 1, 0, 0, 0]
        d = [1.75, 0, 0, 0.72, -0.78, 1.0655]
        alpha = [pi/2, 0, 0, pi/2, pi/2, -pi]
        offset = [0, pi/2, 0, -pi/2, pi, -pi/2]
        qlim = [[-pi / 2, pi / 2] for _ in range(6)]

        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
            links.append(link)
        return links

    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)

        self.q = self._qtest
        self.add_to_env(env)

        q_goal = [self.q[i] - pi / 3 for i in range(len(self.q))]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        fig = self.plot(self.q)
        input("delay")

        for q in qtraj:
            self.q = q
            env.step(0.02)
            fig.step(0.01)
            
        env.hold()
        time.sleep(3)


if __name__ == "__main__":
    robot = Cobot320()
    robot.test()


# import swift
# import roboticstoolbox as rtb
# import spatialmath.base as spb
# from ir_support.robots.DHRobot3D import DHRobot3D
# import numpy as np
# from scipy import linalg
# import os
# import time
# from math import pi


# def get_algebraic_dist(points, center_point, radii):
#     return np.sum(((points - center_point) / radii) ** 2, axis=1)


# def collision_detection_ellipsoids(ellipsoid_info_list, points):
#     collision_results = []
#     points_inside_each = []

#     for i, (T, radii) in enumerate(ellipsoid_info_list):
#         # Transform points to ellipsoid local frame (ellipsoid at origin)
#         homo_points = np.hstack((points, np.ones((points.shape[0], 1)))).T  # 4 x N
#         points_local_homo = linalg.inv(T) @ homo_points
#         points_local = points_local_homo[:3, :].T  # N x 3

#         algebraic_dist = get_algebraic_dist(points_local, np.array([0, 0, 0]), radii)
#         inside_indices = np.where(algebraic_dist < 1)[0]

#         collided = inside_indices.size > 0
#         collision_results.append(collided)
#         points_inside_each.append(inside_indices)

#     return collision_results, points_inside_each


# class Cobot320(DHRobot3D):
#     def __init__(self):
#         links = self._create_DH()

#         link3D_names = dict(
#             link0='Cobot320_STL/L1',
#             link1='Cobot320_STL/L2',
#             link2='Cobot320_STL/L3',
#             link3='Cobot320_STL/L4',
#             link4='Cobot320_STL/L5',
#             link5='Cobot320_STL/L6',
#             link6='Cobot320_STL/L7'
#         )

#         qtest = [0, 0, 0, 0, 0, 0]
#         qtest_transforms = [
#             spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
#             spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
#             spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
#             spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
#             spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
#             spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2)),
#             spb.transl(0, 0, 0) @ spb.r2t(spb.rotz(pi / 2))
#         ]

#         current_path = os.path.abspath(os.path.dirname(__file__))

#         super().__init__(
#             links, link3D_names, name='myCobot', link3d_dir=current_path, qtest=qtest, qtest_transforms=qtest_transforms
#         )
#         self.q = qtest

#     def _create_DH(self):
#         links = []
#         a = [0, 1.1, 1, 0, 0, 0]
#         d = [1.75, 0, 0, 0.72, -0.78, 1.0655]
#         alpha = [pi / 2, 0, 0, pi / 2, pi / 2, -pi]
#         offset = [0, pi / 2, 0, -pi / 2, pi, -pi / 2]
#         qlim = [[-pi / 2, pi / 2] for _ in range(6)]

#         for i in range(6):
#             link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
#             links.append(link)
#         return links

#     def test(self):
#         env = swift.Swift()
#         env.launch(realtime=True)

#         self.q = self._qtest
#         self.add_to_env(env)

#         # Define ellipsoid radii per link (example values)
#         ellipsoid_radii = [
#             np.array([0.15, 0.15, 0.4]),
#             np.array([0.15, 0.15, 0.6]),
#             np.array([0.12, 0.12, 0.5]),
#             np.array([0.10, 0.10, 0.4]),
#             np.array([0.1, 0.1, 0.35]),
#             np.array([0.1, 0.1, 0.3])
#         ]

#         # Define environment points to test collision with (example: random points or load your own)
#         # For demonstration, random points near robot base
#         points = np.random.uniform(low=[0, -0.5, 0], high=[2, 0.5, 1], size=(1000, 3))

#         q_goal = [self.q[i] - pi / 3 for i in range(len(self.q))]
#         qtraj = rtb.jtraj(self.q, q_goal, 50).q

#         for q in qtraj:
#             self.q = q
#             # Get transforms of all links
#             transforms = self.fkine_all(q)

#             ellipsoid_info_list = []
#             for i, radii in enumerate(ellipsoid_radii):
#                 T = transforms[i].A  # 4x4 numpy
#                 ellipsoid_info_list.append((T, radii))

#             # Check collisions against points
#             collisions, points_inside_each = collision_detection_ellipsoids(ellipsoid_info_list, points)
#             collision_status = any(collisions)
#             print(f"At config {q}, Collision detected: {collision_status}")

#             # Visual feedback on points inside ellipsoids
#             colors = np.zeros((points.shape[0], 3))
#             colors[:] = [0, 0, 1]  # default blue

#             for inside_indices in points_inside_each:
#                 colors[inside_indices] = [0, 1, 0]  # green if inside ellipsoid

#             env.step(0.02)

#         env.hold()
#         time.sleep(3)


# if __name__ == "__main__":
#     robot = Cobot320()
#     robot.test()


# import swift
# import roboticstoolbox as rtb
# import spatialmath.base as spb
# from ir_support.robots.DHRobot3D import DHRobot3D
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy import linalg
# from math import pi

# def get_algebraic_dist(points, center_point, radii):
#     return np.sum(((points - center_point) / radii) ** 2, axis=1)

# def plot_ellipsoid(ax, center, radii, rotation, color, alpha=0.3):
#     u = np.linspace(0, 2 * np.pi, 50)
#     v = np.linspace(0, np.pi, 25)
#     x = radii[0] * np.outer(np.cos(u), np.sin(v))
#     y = radii[1] * np.outer(np.sin(u), np.sin(v))
#     z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

#     for i in range(len(u)):
#         for j in range(len(v)):
#             [x[i,j], y[i,j], z[i,j]] = rotation @ np.array([x[i,j], y[i,j], z[i,j]])

#     x += center[0]
#     y += center[1]
#     z += center[2]

#     ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)

# class Cobot320(DHRobot3D):
#     def __init__(self):
#         links = self._create_DH()

#         link3D_names = dict(
#             link0='Cobot320_STL/L1',
#             link1='Cobot320_STL/L2',
#             link2='Cobot320_STL/L3',
#             link3='Cobot320_STL/L4',
#             link4='Cobot320_STL/L5',
#             link5='Cobot320_STL/L6',
#             link6='Cobot320_STL/L7'
#         )

#         qtest = [0, 0, 0, 0, 0, 0]
#         qtest_transforms = [spb.transl(0,0,0) @ spb.r2t(spb.rotz(pi/2)) for _ in range(7)]

#         import os
#         current_path = os.path.abspath(os.path.dirname(__file__))

#         super().__init__(
#             links, link3D_names, name='myCobot', link3d_dir=current_path,
#             qtest=qtest, qtest_transforms=qtest_transforms
#         )
#         self.q = qtest

#     def _create_DH(self):
#         self.link_lengths = [0, 1.1, 1, 0, 0, 0]
#         links = []
#         d = [1.75,0,0,0.72,-0.78,1.0655]
#         alpha = [pi/2, 0, 0, pi/2, pi/2, -pi]
#         offset = [0, pi/2, 0, -pi/2, pi, -pi/2]
#         qlim = [[-pi/2, pi/2] for _ in range(6)]
#         for i in range(6):
#             link = rtb.RevoluteDH(d=d[i], a=self.link_lengths[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
#             links.append(link)
#         return links

#     def test(self):
#         # Start swift environment and plot robot
#         env = swift.Swift()
#         env.launch(realtime=True)
#         self.q = self._qtest
#         self.add_to_env(env)

#         # Joint config inducing likely self-collision for test
#         q_collision = [pi/4, -pi/2, pi/2, 0, 0, 0]

#         # Ellipsoid radii approx link size
#         ellipsoid_radii = [
#             np.array([self.link_lengths[i], 0.15, 0.15]) for i in range(6)
#         ]

#         fk = self.fkine_all(q_collision)

#         # Compute ellipsoid centers halfway along link x axis
#         ellipsoid_centers = []
#         for i in range(6):
#             local_point = np.array([self.link_lengths[i]/2, 0, 0, 1])
#             center = fk[i].A @ local_point
#             ellipsoid_centers.append(center[:3])

#         # Perform simple self-collision detection
#         collisions = [False]*6
#         ellipsoid_points = []
#         for i in range(6):
#             # Sample points on ellipsoid surface for link i
#             u = np.random.uniform(0, 2*np.pi, 500)
#             v = np.random.uniform(0, np.pi, 500)
#             x = ellipsoid_radii[i][0] * np.cos(u) * np.sin(v)
#             y = ellipsoid_radii[i][1] * np.sin(u) * np.sin(v)
#             z = ellipsoid_radii[i][2] * np.cos(v)
#             points = np.vstack((x,y,z))
#             rotated = fk[i].R @ points
#             translated = rotated + ellipsoid_centers[i].reshape(3,1)
#             ellipsoid_points.append(translated.T)

#         for i in range(6):
#             for j in range(i+1, 6):
#                 # Test if points from j inside i
#                 homo_pts = np.hstack((ellipsoid_points[j], np.ones((ellipsoid_points[j].shape[0],1)))).T
#                 inv_T = linalg.inv(fk[i].A)
#                 pts_local = (inv_T @ homo_pts)[:3,:].T
#                 dists = get_algebraic_dist(pts_local, np.zeros(3), ellipsoid_radii[i])
#                 if np.any(dists < 1):
#                     collisions[i] = True
#                     collisions[j] = True

#         # Add a matplotlib figure to visualize ellipsoids
#         fig = plt.figure()
#         ax = fig.add_subplot(111, projection='3d')
#         for i in range(6):
#             color = 'red' if collisions[i] else 'green'
#             plot_ellipsoid(ax, ellipsoid_centers[i], ellipsoid_radii[i], fk[i].R, color, alpha=0.5)

#         ax.set_xlabel('X')
#         ax.set_ylabel('Y')
#         ax.set_zlabel('Z')
#         ax.set_title('Cobot320 Ellipsoids with Collision Coloring')
#         plt.show()

#         env.hold()
#         import time
#         time.sleep(3)


# if __name__ == "__main__":
#     robot = Cobot320()
#     robot.test()