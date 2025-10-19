import numpy as np
import swift
import roboticstoolbox as rtb
from spatialmath.base import transl, rpy2tr
from spatialmath import SE3
from spatialgeometry import Mesh
import os
from math import pi
from Micahs_robot.abb_irb_120 import abb_irb_120
from Lilys_robot.XArm6 import XArm6
from ir_support.robots import UR3
from FoodOrderRobotv1 import FoodOrderRobotv1
from bread import BreadCollection


env = swift.Swift()
env.launch(realtime=True)


def setup_robots(env):
    UR3_robot = UR3()
    XArm_robot = XArm6()
    irb_robot = abb_irb_120()

    # Set bases and add to env as required
    XArm_robot.base = SE3(0.95, 0.15, 1)
    XArm_robot.add_to_env(env)

    irb_robot.base = SE3(0, 0.1, 1) * SE3.Rz(-pi / 2)
    irb_robot.add_to_env(env)

    UR3_robot.base = SE3(1.75,0.2,1)
    UR3_robot.add_to_env(env)

    return UR3_robot, XArm_robot, irb_robot

current_path = os.path.abspath(os.path.dirname(__file__))

# --- Ingredient definitions ---
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

def spawn_items(env):
    item = {}
    item["bench"] = spawn_ingredient(env, "bench", SE3(0, 0, 0))
    item["glass"] = spawn_ingredient(env, "glass", SE3(0.5, 0, 0))
    item["bread_rack"] = spawn_ingredient(env, "bread_rack", SE3(0, 0, 0))
    return item

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

def spawn_all_piles(env):
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

def spawn_bread_row(env, ingredient, n=5, start_pos=(2.5, -0.2, 1.4), dy=0.05):
    x0, y0, z0 = start_pos
    row = []
    for i in range(n):
        pose = SE3(x0, y0 + i * dy, z0) @ SE3.Rz(pi/2)
        slice_mesh = spawn_ingredient(env, ingredient, pose)
        row.append(slice_mesh)
    return row


def spawn_bread(env):
    bread_piles = {}
    storage_z_levels = [1.4, 1.1]  # multiple shelves
    ingredients = ["bread_bottom", "bread_top"]

    for ingredient in ingredients:
        bread_piles[ingredient] = []  # initialize each pile

        for z in storage_z_levels:
            row = spawn_bread_row(env,ingredient=ingredient,n=5,start_pos=(2.37, -0.2, z),dy=0.2)
            bread_piles[ingredient].extend(row)  # append meshes from each shelf

    return bread_piles

def collect_bread_locations_and_meshes(bread_piles):
    """Collect both positions and mesh objects from ingredient piles"""
    food_locations = []
    food_meshes = []
    
    for ingredient_name in ['bread_top', 'bread_bottom']:
        pile_list = bread_piles.get(ingredient_name, [])
        # Take first 2 items from each pile (or adjust as needed)
        for mesh in pile_list[:1]: # Picks up 2
            pose_se3 = SE3(mesh.T)
            pos = pose_se3.t
            food_locations.append([pos[0], pos[1], pos[2]])
            food_meshes.append(mesh)  # Add the actual mesh object!
    return food_locations, food_meshes

def collect_veggie_locations_and_meshes(piles, veggie_selection):
    """Collect both positions and mesh objects from ingredient piles"""
    food_locations = []
    food_meshes = []
    
    for ingredient_name in veggie_selection:
        pile_list = piles.get(ingredient_name, [])
        # Take first 2 items from each pile (or adjust as needed)
        for mesh in pile_list[:2]: # Picks up 2
            pose_se3 = SE3(mesh.T)
            pos = pose_se3.t
            food_locations.append([pos[0], pos[1], pos[2]])
            food_meshes.append(mesh)  # Add the actual mesh object!
    return food_locations, food_meshes

def collect_meat_locations_and_meshes(piles, meat_selection):
    """Collect both positions and mesh objects from ingredient piles"""
    food_locations = []
    food_meshes = []
    
    for ingredient_name in meat_selection:
        pile_list = piles.get(ingredient_name, [])
        # Take first 2 items from each pile (or adjust as needed)
        for mesh in pile_list[:2]: # Picks up 2
            pose_se3 = SE3(mesh.T)
            pos = pose_se3.t
            food_locations.append([pos[0], pos[1], pos[2]])
            food_meshes.append(mesh)  # Add the actual mesh object!
    return food_locations, food_meshes


# --- Main ---
def main():
    env = swift.Swift()
    env.launch(realtime=True)
    spawn_items(env)

    # Setup robots
    UR3_robot, XArm_robot, irb_robot = setup_robots(env)
    
    # Spawn piles of ingredients
# Spawn piles of ingredients
    piles = spawn_all_piles(env)
    bread_piles = spawn_bread(env)

    # Collect bread locations and meshes


    # Set camera
    env.set_camera_pose([2, -2, 2], [0, 0, 0])
    env.step()

    # === Bread robot ===
    bread_robot = BreadCollection(robot=UR3_robot, env=env)
    bread_locations, bread_meshes = collect_bread_locations_and_meshes(bread_piles)
    bread_meshes = collect_bread_locations_and_meshes(piles)
    bread_station_location = [0.95, 0.4, 1]
    bread_robot.process_food_order_side(bread_locations, bread_station_location, mesh_list=bread_meshes)  
    
    # === Meat robot ===
    meat_robot = FoodOrderRobotv1(robot=XArm_robot, env=env)
    meat_locations, meat_meshes = collect_meat_locations_and_meshes(piles)
    meat_station_location = [0.95, 0.4, 1]
    meat_robot.process_food_order(meat_locations, meat_station_location, mesh_list=meat_meshes)

    # === Veggie robot ===
    veggie_robot = FoodOrderRobotv1(robot=irb_robot, env=env)
    veggie_locations, veggie_meshes = collect_veggie_locations_and_meshes(piles)
    veggie_station_location = [0.1, 0.4, 1]
    veggie_robot.process_food_order(veggie_locations, veggie_station_location, mesh_list=veggie_meshes)
    
    # Hold the environment for inspection
    env.hold()


if __name__ == "__main__":
    main()

