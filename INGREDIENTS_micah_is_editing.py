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

def setup_environment():
    env = swift.Swift()
    env.launch(realtime=True)
    return env

def setup_robots(env):
    UR3_robot = UR3()
    XArm_robot = XArm6()
    irb_robot = abb_irb_120()

    # Set bases and add to env as required
    XArm_robot.base = SE3(0.95, 0.35, 0.8)
    XArm_robot.add_to_env(env)

    irb_robot.base = SE3(0, 0.1, 1) * SE3.Rz(-pi / 2)
    irb_robot.add_to_env(env)

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
        "path": os.path.join(current_path, "env", "bench.stl"),
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

def spawn_individual_items(env):
    """Spawn all the individual items that were in your working version"""
    items = {}
    items["bench"] = spawn_ingredient(env, "bench", SE3(0, 0, 0))
    items["glass"] = spawn_ingredient(env, "glass", SE3(0, 0, 0))
    items["bread_bottom"] = spawn_ingredient(env, "bread_bottom", SE3(-1, 0.25, 1))
    items["ham1"] = spawn_ingredient(env, "ham", SE3(-0.99, 0.34, 1.01))
    items["ham2"] = spawn_ingredient(env, "ham", SE3(-1.03, 0.14, 1.01))
    items["lettuce1"] = spawn_ingredient(env, "lettuce", SE3(-1.01, 0.25, 1.04) @ SE3.Rz(pi/2))
    items["lettuce2"] = spawn_ingredient(env, "lettuce", SE3(-1.03, 0.27, 1.04) @ SE3.Rz(-pi/2))
    items["tomato1"] = spawn_ingredient(env, "tomato", SE3(-0.98, 0.34, 1.04))
    items["tomato2"] = spawn_ingredient(env, "tomato", SE3(-1.03, 0.24, 1.04))
    items["tomato3"] = spawn_ingredient(env, "tomato", SE3(-0.98, 0.14, 1.04))
    items["bread_top"] = spawn_ingredient(env, "bread_top", SE3(-0.7, 0.25, 1.0))
    return items

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

def collect_veggie_locations_and_meshes(piles):
    """Collect both positions and mesh objects from ingredient piles"""
    food_locations = []
    food_meshes = []
    
    for ingredient_name in ['beetroot']:
        pile_list = piles.get(ingredient_name, [])
        # Take first 2 items from each pile (or adjust as needed)
        for mesh in pile_list[:2]:
            pose_se3 = SE3(mesh.T)
            pos = pose_se3.t
            food_locations.append([pos[0], pos[1], pos[2]])
            food_meshes.append(mesh)  # Add the actual mesh object!
    return food_locations, food_meshes

def main():
    env = setup_environment()
    UR3_robot, XArm_robot, irb_robot = setup_robots(env)
    
    # Spawn individual items (bench, glass, bread, etc.)
    individual_items = spawn_individual_items(env)
    
    # Spawn all ingredient piles
    piles = spawn_all_piles(env)
    
    # Update camera and take a step to render everything
    env.set_camera_pose([2, -2, 2], [0, 0, 0])
    env.step()
    
    # Create a custom FoodOrderRobot that uses external env and robot
    micahs_robot = FoodOrderRobotv1(robot=irb_robot, env=env)
    
    # Prepare a smaller list of ingredient pick locations (first 2 from each pile)
    micahs_food_locations, micahs_food_meshes = collect_veggie_locations_and_meshes(piles)

    # Define the assembly station location
    micahs_station_location = [-0.4, 0.2, 1]
    
    print(f"Found {len(micahs_food_locations)} ingredients to process")
    print(f"Found {len(micahs_food_meshes)} mesh objects")
    
    # Run the food order processing task
    micahs_robot.process_food_order(micahs_food_locations, micahs_station_location, mesh_list=micahs_food_meshes)

if __name__ == "__main__":
    main()
