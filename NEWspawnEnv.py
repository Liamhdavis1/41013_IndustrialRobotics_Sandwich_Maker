import numpy as np
import swift
from spatialmath import SE3
from spatialgeometry import Mesh
from math import pi
from Micahs_robot.abb_irb_120 import abb_irb_120
from Lilys_robot.XArm6 import XArm6
from ir_support.robots import UR3
from Liams_robot.Cobot320 import Cobot320
from FoodOrderRobotv1 import FoodOrderRobotv1
import os
import trimesh
from NEWcollisionDetection import CollsionDetection  # your collision utils

ENV_OFFSET = np.array([1.5, 0.5, 0.0])
current_path = os.path.abspath(os.path.dirname(__file__))

INGREDIENTS = {
    "glass": {"path": os.path.join(current_path, "env", "glass.stl"),
              "scale": (1, 1, 1), "color": [0.6, 0.6, 0.6, 0.4]},
    "bench": {"path": os.path.join(current_path, "env", "benchv2.stl"),
              "scale": (1, 1, 1), "color": [0.6, 0.6, 0.6, 1]},
    "bread_rack": {"path": os.path.join(current_path, "env", "bread_rack.stl"),
                   "scale": (1, 1, 1), "color": [0.6, 0.6, 0.6, 1]},
    "tray": {"path": os.path.join(current_path, "env", "tray.stl"),
             "scale": (0.065, 0.08, 0.08), "color": [0.8, 0.8, 0.8, 1]},
    "bread_bottom": {"path": os.path.join(current_path, "env", "sandwich", "bread-bottom.stl"),
                     "scale": (0.1, 0.1, 0.1), "color": [0.95, 0.77, 0.53, 1]},
    "bread_top": {"path": os.path.join(current_path, "env", "sandwich", "bread-top.stl"),
                  "scale": (0.1, 0.1, 0.1), "color": [0.95, 0.77, 0.53, 1]},
    "ham": {"path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
            "scale": (0.1, 0.1, 0.1), "color": [0.7, 0.5, 0.7, 1]},
    "lettuce": {"path": os.path.join(current_path, "env", "sandwich", "lettace.stl"),
                "scale": (0.01, 0.01, 0.01), "color": [0.25, 0.86, 0.37, 1]},
    "tomato": {"path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
               "scale": (0.05, 0.05, 0.05), "color": [0.8, 0.1, 0.1, 1]},
    "salami": {"path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
               "scale": (0.05, 0.05, 0.03), "color": [0.7, 0.3, 0.3, 1]},
    "beef": {"path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
             "scale": (0.1, 0.1, 0.1), "color": [0.56, 0.3, 0.04, 1]},
    "chicken": {"path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
                "scale": (0.05, 0.05, 0.03), "color": [0.9, 0.8, 0.6, 1]},
    "cucumber": {"path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
                 "scale": (0.04, 0.04, 0.04), "color": [0.2, 0.6, 0.3, 1]},
    "beetroot": {"path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
                 "scale": (0.04, 0.04, 0.04), "color": [0.5, 0.09, 0.4, 1]}
}


spawned_meshes = {}


def spawn_ingredient(env, name, pose):
    info = INGREDIENTS[name]
    mesh = Mesh(filename=info["path"], pose=pose,
                scale=info["scale"], color=info["color"])
    env.add(mesh)
    spawned_meshes[f"{name}_{len(spawned_meshes)}"] = mesh
    return mesh


def spawn_pile(env, ingredient, n, center, dx_range, dy_range, dz_step, dz_jitter):
    x0, y0, z0 = np.array(center) + ENV_OFFSET
    pile = []
    for i in range(n):
        dx = np.random.uniform(*dx_range)
        dy = np.random.uniform(*dy_range)
        dz = i * dz_step + np.random.uniform(*dz_jitter)
        pose = SE3(x0 + dx, y0 + dy, z0 + dz)
        pile.append(spawn_ingredient(env, ingredient, pose))
    return pile


class RobotEnvironment:
    def __init__(self):
        print("Launching Swift environment...")
        self.env = swift.Swift()
        self.env.launch(realtime=True)

        bench_stl_path = INGREDIENTS['bench']['path']
        self.bench_points = CollsionDetection.load_mesh_points(bench_stl_path, num_points=8000)

        self.spawn_items(self.env)
        self.UR3_robot, self.XArm_robot, self.irb_robot, self.Cobot = self.setup_robots(self.env)
        self.piles = self.spawn_all_piles(self.env)
        self.bread_piles = self.spawn_bread(self.env)
        self.env.set_camera_pose([3, -3, 2], [1, 0.5, 0])
        self.env.step()

        self.meat_robot_ctrl = FoodOrderRobotv1(self.XArm_robot, self.env)
        self.veggie_robot_ctrl = FoodOrderRobotv1(self.irb_robot, self.env)
        self.cobot_ctrl = FoodOrderRobotv1(self.Cobot, self.env)
        self.UR3_robot = FoodOrderRobotv1(self.UR3_robot, self.env)


    def setup_robots(self, env):
        UR3_robot = UR3()
        XArm_robot = XArm6()
        irb_robot = abb_irb_120()
        Cobot = Cobot320()
        irb_robot.base = SE3(0, 0.1, 1) * SE3.Trans(ENV_OFFSET)
        Cobot.base = SE3(-0.95, 0.2, 1) * SE3.Trans(ENV_OFFSET)
        XArm_robot.base = SE3(0.9, 0.15, 1) * SE3.Trans(ENV_OFFSET)
        UR3_robot.base = SE3(1.75, 0.2, 1) * SE3.Trans(ENV_OFFSET)
        for r in [XArm_robot, irb_robot, UR3_robot, Cobot]:
            r.add_to_env(env)
        return UR3_robot, XArm_robot, irb_robot, Cobot


    def spawn_items(self, env):
        spawn_ingredient(env, "bench", SE3(*ENV_OFFSET))
        spawn_ingredient(env, "glass", SE3(0.5, 0, 0) * SE3(*ENV_OFFSET))
        spawn_ingredient(env, "bread_rack", SE3(-0.15, 0, 0) * SE3(ENV_OFFSET))


    def spawn_bread(self, env):
        bread_piles, storage_z = {}, [1.4, 1.1]
        for ingr in ["bread_bottom", "bread_top"]:
            bread_piles[ingr] = []
            for z in storage_z:
                for i in range(5):
                    pose = SE3(2.2, -0.2 + i * 0.2, z) * SE3.Trans(ENV_OFFSET) @ SE3.Rz(pi / 2)
                    bread_piles[ingr].append(spawn_ingredient(env, ingr, pose))
        return bread_piles


    def spawn_all_piles(self, env):
        cfgs = [
            ("ham", 8, (1.2, -0.35, 0.92)), ("tomato", 12, (0.25, -0.35, 0.92)),
            ("lettuce", 5, (-0.25, -0.35, 0.92)), ("salami", 12, (1.2, -0.1, 0.92)),
            ("beef", 8, (0.75, -0.35, 0.92)), ("chicken", 8, (0.75, -0.1, 0.92)),
            ("cucumber", 15, (0.25, -0.1, 0.92)), ("beetroot", 15, (-0.25, -0.1, 0.92))
        ]
        piles = {
            n: spawn_pile(env, n, count, center,
                        (-0.1, 0.1), (-0.02, 0.02), 0.003, (-0.07, 0.07))
            for n, count, center in cfgs
        }
        return piles


    def collect_ingredient_locations(self, piles, selected):
        locs, meshes = [], []
        for name in selected:
            for mesh in piles.get(name, [])[:2]:
                pos = SE3(mesh.T).t
                locs.append(list(pos))
                meshes.append(mesh)
        return locs, meshes


    def collect_bread_locations_and_meshes(self, bread_piles):
        locs, meshes = [], []
        for n in ["bread_top", "bread_bottom"]:
            for mesh in bread_piles.get(n, [])[:1]:
                pos = SE3(mesh.T).t
                locs.append(list(pos))
                meshes.append(mesh)
        return locs, meshes