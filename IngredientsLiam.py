import numpy as np
import swift
from spatialmath import SE3
from spatialgeometry import Mesh
from math import pi
from Micahs_robot.abb_irb_120 import abb_irb_120
from Lilys_robot.XArm6 import XArm6
from ir_support.robots import UR3
from FoodOrderRobotv1 import FoodOrderRobotv1
import os


# --- Ingredient configuration ---
current_path = os.path.abspath(os.path.dirname(__file__))

INGREDIENTS = {
    "glass": {
        "path": os.path.join(current_path, "env", "glass.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 0.4]
    },
    "bench": {
        "path": os.path.join(current_path, "env", "benchv2.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 1]
    },
    "bread_rack": {
        "path": os.path.join(current_path, "env", "bread_rack.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 1]
    },
    "bread_bottom": {
        "path": os.path.join(current_path, "env", "sandwich", "bread-bottom.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.95, 0.77, 0.53, 1]
    },
    "bread_top": {
        "path": os.path.join(current_path, "env", "sandwich", "bread-top.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.95, 0.77, 0.53, 1]
    },
    "ham": {
        "path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.7, 0.5, 0.7, 1]
    },
    "lettuce": {
        "path": os.path.join(current_path, "env", "sandwich", "lettace.stl"),
        "scale": (0.01, 0.01, 0.01),
        "color": [0.25, 0.86, 0.37, 1]
    },
    "tomato": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.05),
        "color": [0.8, 0.1, 0.1, 1]
    },
    "salami": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.03),
        "color": [0.7, 0.3, 0.3, 1]
    },
    "beef": {
        "path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.56, 0.3, 0.04, 1]
    },
    "chicken": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.03),
        "color": [0.9, 0.8, 0.6, 1]
    },
    "cucumber": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.04, 0.04, 0.04),
        "color": [0.2, 0.6, 0.3, 1]
    },
    "beetroot": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.04, 0.04, 0.04),
        "color": [0.5, 0.09, 0.4, 1]
    }
}

spawned_meshes = {}


# --- Utility functions ---
def spawn_ingredient(env, name, pose):
    info = INGREDIENTS[name]
    mesh = Mesh(
        filename=info["path"],
        pose=pose,
        scale=info["scale"],
        color=info["color"]
    )
    env.add(mesh)
    spawned_meshes[f"{name}_{len(spawned_meshes)}"] = mesh
    return mesh


def spawn_pile(env, ingredient, n, center, dx_range, dy_range, dz_step, dz_jitter):
    x0, y0, z0 = center
    pile = []
    for i in range(n):
        dx = np.random.uniform(*dx_range)
        dy = np.random.uniform(*dy_range)
        dz = i * dz_step + np.random.uniform(*dz_jitter)
        pose = SE3(x0 + dx, y0 + dy, z0 + dz)
        piece = spawn_ingredient(env, ingredient, pose)
        pile.append(piece)
    return pile


# --- Robot Environment Class ---
class RobotEnvironment:
    def __init__(self):
        print("Launching Swift environment from GUI...")
        self.env = swift.Swift()
        self.env.launch(realtime=True)

        # Spawn scene and robots
        self.spawn_items(self.env)
        self.UR3_robot, self.XArm_robot, self.irb_robot = self.setup_robots(self.env)
        self.piles = self.spawn_all_piles(self.env)
        self.bread_piles = self.spawn_bread(self.env)

        # Camera setup
        self.env.set_camera_pose([2, -2, 2], [0, 0, 0])
        self.env.step()

    def setup_robots(self, env):
        UR3_robot = UR3()
        XArm_robot = XArm6()
        irb_robot = abb_irb_120()

        # Set up robot base transforms and add to environment
        XArm_robot.base = SE3(0.95, 0.15, 1)
        irb_robot.base = SE3(0, 0.1, 1) * SE3.Rz(-pi / 2)
        UR3_robot.base = SE3(1.75, 0.2, 1)

        XArm_robot.add_to_env(env)
        irb_robot.add_to_env(env)
        UR3_robot.add_to_env(env)

        return UR3_robot, XArm_robot, irb_robot

    def spawn_items(self, env):
        spawn_ingredient(env, "bench", SE3(0, 0, 0))
        spawn_ingredient(env, "glass", SE3(0, 0, 0))
        spawn_ingredient(env, "bread_rack", SE3(0, 0, 0))

    def spawn_bread(self, env):
        bread_piles = {}
        storage_z = [1.4, 1.1]
        for ingredient in ["bread_bottom", "bread_top"]:
            bread_piles[ingredient] = []
            for z in storage_z:
                for i in range(5):
                    pose = SE3(2.37, -0.2 + i * 0.2, z) @ SE3.Rz(pi / 2)
                    slice_mesh = spawn_ingredient(env, ingredient, pose)
                    bread_piles[ingredient].append(slice_mesh)
        return bread_piles

    def spawn_all_piles(self, env):
        piles = {}
        piles["ham"] = spawn_pile(env, "ham", 8, (1.2, -0.35, 0.92), (-0.1, 0.1), (-0.02, 0.02), 0.002, (-0.05, 0.05))
        piles["tomato"] = spawn_pile(env, "tomato", 12, (0.25, -0.35, 0.92), (-0.1, 0.1), (-0.01, 0.01), 0.003, (-0.07, 0.07))
        piles["lettuce"] = spawn_pile(env, "lettuce", 5, (-0.25, -0.35, 0.92), (-0.1, 0.1), (-0.015, 0.01), 0.003, (-0.07, 0.07))
        piles["salami"] = spawn_pile(env, "salami", 12, (1.2, -0.1, 0.92), (-0.1, 0.1), (-0.01, 0.01), 0.003, (-0.07, 0.07))
        piles["beef"] = spawn_pile(env, "beef", 8, (0.75, -0.35, 0.92), (-0.1, 0.1), (-0.02, 0.02), 0.002, (-0.05, 0.05))
        piles["chicken"] = spawn_pile(env, "chicken", 8, (0.75, -0.1, 0.92), (-0.1, 0.1), (-0.02, 0.02), 0.002, (-0.05, 0.05))
        piles["cucumber"] = spawn_pile(env, "cucumber", 15, (0.25, -0.1, 0.92), (-0.1, 0.1), (-0.02, 0.02), 0.002, (-0.05, 0.05))
        piles["beetroot"] = spawn_pile(env, "beetroot", 15, (-0.25, -0.1, 0.92), (-0.1, 0.1), (-0.02, 0.02), 0.002, (-0.05, 0.05))
        return piles

    def collect_ingredient_locations(self, piles, selected):
        food_locations, food_meshes = [], []
        for ingredient_name in selected:
            pile_list = piles.get(ingredient_name, [])
            for mesh in pile_list[:2]:
                pose_se3 = SE3(mesh.T)
                pos = pose_se3.t
                food_locations.append([pos[0], pos[1], pos[2]])
                food_meshes.append(mesh)
        return food_locations, food_meshes

    def process_order(self, meat_selection, veggie_selection):
        print("Processing order...")
        meat_locations, meat_meshes = self.collect_ingredient_locations(self.piles, meat_selection)
        veggie_locations, veggie_meshes = self.collect_ingredient_locations(self.piles, veggie_selection)

        meat_robot = FoodOrderRobotv1(robot=self.XArm_robot, env=self.env)
        veggie_robot = FoodOrderRobotv1(robot=self.irb_robot, env=self.env)

        meat_robot.process_food_order(meat_locations, [0.95, 0.4, 1], mesh_list=meat_meshes)
        veggie_robot.process_food_order(veggie_locations, [0.1, 0.4, 1], mesh_list=veggie_meshes)
        print("Order complete.")
