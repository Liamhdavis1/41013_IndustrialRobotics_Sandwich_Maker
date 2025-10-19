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
from ir_support.robots import LinearUR3
from FoodOrderRobotv1 import FoodOrderRobotv1
# from bread import BreadCollection


env = swift.Swift()
env.launch(realtime=True)


def setup_robots(env):
    UR3_robot = UR3()
    LinearUR3_robot = LinearUR3()
    XArm_robot = XArm6()
    irb_robot = abb_irb_120()

    # Set bases and add to env as required
    XArm_robot.base = SE3(0.95, 0.15, 1)
    XArm_robot.add_to_env(env)

    irb_robot.base = SE3(0, 0.1, 1) * SE3.Rz(pi / 2)
    irb_robot.add_to_env(env)
    # LinearUR3_robot.base = SE3(1.7,-0.2,1) *  SE3.Rx(pi/2)
    # LinearUR3_robot.add_to_env(env)

    UR3_robot.base = SE3(1.8,-0.1,1) * SE3.Rz(-pi/2)
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
    item["bread_rack"] = spawn_ingredient(env, "bread_rack", SE3(-0.1, 0, 0))
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

# def bread_base_storage(env, ingredient, n=5, start_pos=(2.5, -0.2, 1.4), dx=0.05):
#     x0, y0, z0 = start_pos
#     row = []
#     for i in range(n):
#         pose = SE3(x0 - i * dx, y0 , z0) 
#         slice_mesh = spawn_ingredient(env, ingredient, pose)
#         row.append(slice_mesh)
#     return row



# def spawn_bread(env):
#     bread_piles = {}
#     storage_z_levels = [1]  # multiple shelves
#     ingredients = ["bread_bottom"]

#     for ingredient in ingredients:
#         bread_piles[ingredient] = []  # initialize each pile

#         for z in storage_z_levels:
#             row = bread_base_storage(env,ingredient=ingredient,n=5,start_pos=(2.37, -0.2, z),dy=0.2)
#             bread_piles[ingredient].extend(row)  # append meshes from each shelf

#     return bread_piles


# def spawn_bread_row(env, ingredient="bread_bottom", n=3, start_pos=(2.2, -0.2, 1), dx=0.18):
#     """Spawn a single row of bread slices"""
#     bread = []
#     x0, y0, z0 = start_pos
#     for i in range(n):
#         pose = SE3(x0 - i * dx, y0, z0)  # spread along x-axis
#         slice_mesh = spawn_ingredient(env, ingredient, pose)
#         bread.append(slice_mesh)
#     return bread

def spawn_bread(env):
    bread_piles = {}
    storage_z = [1.1, 1.4]
    for ingredient in ["bread_bottom"]:
        bread_piles[ingredient] = []
        for z in storage_z:
            for i in range(5):
                pose = SE3(2.2, -0.2 + i * 0.2, z) @ SE3.Rz(pi / 2)
                slice_mesh = spawn_ingredient(env, ingredient, pose)
                bread_piles[ingredient].append(slice_mesh)
    return bread_piles



def collect_bread_locations_and_meshes(bread_dict, ingredient_name='bread_bottom'):
    food_locations = []
    food_meshes = []

    pile_list = bread_dict.get(ingredient_name, [])
    for mesh in pile_list[:2]:  # pick first slice
        pose_se3 = SE3(mesh.T)
        pos = pose_se3.t
        food_locations.append([pos[0], pos[1], pos[2]])
        # food_locations.append(pose_se3)
        food_meshes.append(mesh)

    return food_locations, food_meshes




def collect_veggie_locations_and_meshes(piles):
    """Collect both positions and mesh objects from ingredient piles"""
    food_locations = []
    food_meshes = []
    
    for ingredient_name in ['tomato']:
        pile_list = piles.get(ingredient_name, [])
        # Take first 2 items from each pile (or adjust as needed)
        for mesh in pile_list[:2]: # Picks up 2
            pose_se3 = SE3(mesh.T)
            pos = pose_se3.t
            food_locations.append([pos[0], pos[1], pos[2]])
            food_meshes.append(mesh)  # Add the actual mesh object!
    return food_locations, food_meshes

def collect_meat_locations_and_meshes(piles):
    """Collect both positions and mesh objects from ingredient piles"""
    food_locations = []
    food_meshes = []
    
    for ingredient_name in ['ham']:
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
    bread = spawn_bread(env)

    # Collect bread locations and meshes


    # Set camera
    env.set_camera_pose([2, -2, 2], [0, 0, 0])
    env.step()

    # === Bread robot ===
    bread_robot = FoodOrderRobotv1(robot=UR3_robot, env=env)
    bread_locations, bread_meshes = collect_bread_locations_and_meshes(bread)
    print(bread_locations)
    bread_station_location = [1.6,0.4,1]
    bread_robot.other_ik_solver(pick_pose=SE3(2.2, -0.2, 1.4), 
                                place_pose=SE3(1.6,0.4,1), 
                                mesh=None, 
                                gripper_down_orientation_pick=SE3.Ry(np.pi/2), 
                                gripper_down_orientation_place=SE3.Ry(np.pi/2))
    # bread_robot.process_food_order(bread_locations, bread_station_location, mesh_list=bread_meshes)  
    input('delay')
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

def bread():
    env = swift.Swift()
    
    env.launch(realtime=True)
    spawn_items(env)
    bread = spawn_bread(env)

    UR3_robot, XArm_robot, irb_robot = setup_robots(env)

    bread_robot = FoodOrderRobotv1(robot=UR3_robot, env=env)
    robot = UR3_robot

    bread_locations, bread_meshes = collect_bread_locations_and_meshes(bread)
    station_location = [1.75,0.2,1] 
    # Initialize current stack height at assembly station base height
    current_z = station_location[2]

    # Tool orientation (pointing downward for picking)
    tool_orientation = [0, -pi/2, -pi]  

    print(f"Processing order with {len(bread_locations)} ingredients...")
    print(f"Station location: {bread_locations}")

    hover_height = 0.2

    placed_meshes = []

    # Process each food item in the order list
    for i, bread_pos in enumerate(bread_locations):
        mesh = None
        if bread_meshes is not None and i < len(bread_meshes):
            mesh = bread_meshes[i]
            print(f"  Attaching mesh {i}: {type(mesh)}")

        # Calculate hover position
        hover_above_food = [bread_pos[0] - hover_height, bread_pos[1], bread_pos[2]]
        hover_pose = SE3(transl(hover_above_food) @ rpy2tr(*tool_orientation))

        q_hover, success, error = bread_robot.solve_ik_robust(robot, hover_pose, robot.q)
        if not success:
            print(f"  Failed to reach hover position for ingredient {i+1}")
            continue

        print(f"  Moving to hover above food (IK+jtraj)...")
        bread_robot.execute_trajectory(robot, env, robot.q.copy(), q_hover, mesh=None)
        
        print(f"  Picking up ingredient {i+1} (RMRC down)...")
        bread_robot.rmrc_vertical_movement(robot, env, hover_above_food, bread_pos, tool_orientation, mesh=None)

        print(f"  Attaching mesh {i} to end effector...")
        attached_mesh = mesh
        
        print(f"  Lifting ingredient {i+1} (RMRC up)...")
        bread_robot.rmrc_vertical_movement(robot, env, bread_pos, hover_above_food, tool_orientation, mesh=attached_mesh, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2))

        
        # Station hover
        # Calculate current station location with stacking
        # above_station_pos = [
        #     hover_above_food[0] + 0.1,
        #     hover_above_food[1] + 0.4,
        #     hover_above_food[2]  # Use updated stack height
        # ]

        # above_station_pose = SE3(transl(above_station_pos) @ rpy2tr(*tool_orientation))

        # # Solve IK for that pose
        # q_above_station, success, error = bread_robot.solve_ik_robust(robot, above_station_pose, robot.q)
        # if not success:
        #     print("  Could not solve IK for above station position")
        #     continue
        # bread_robot.execute_trajectory(robot, env, robot.q.copy(), q_above_station, mesh=attached_mesh, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2))
        # input('delay')
        # Hover above the current stack height
        hover_above_station = [station_location[0], station_location[1], station_location[2] + hover_height]

        station_hover_pose = SE3(transl(hover_above_station) @ rpy2tr(*tool_orientation))
        q_station, success, error = bread_robot.solve_ik_robust(robot, station_hover_pose, robot.q)
        if not success:
            print(f"  Failed to reach station hover position for ingredient {i + 1}")
            continue

        print(f"  Moving to hover above station (IK+jtraj)...")
        bread_robot.execute_trajectory(robot, env, robot.q.copy(), q_station, mesh=attached_mesh, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2))

        print(f"  Placing ingredient {i+1} at station (RMRC down)...")
        place_pos = station_location
        bread_robot.rmrc_vertical_movement(robot, env, hover_above_station, place_pos, tool_orientation, mesh=attached_mesh, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2))

        print(f"  Retracting from station (RMRC up)...")
        bread_robot.rmrc_vertical_movement(robot, env, place_pos, hover_above_station, tool_orientation, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2))
        
# def bread():
#     env = swift.Swift()
#     env.launch(realtime=True)
#     spawn_items(env)
#     bread_piles = spawn_bread(env)

#     UR3_robot, XArm_robot, irb_robot = setup_robots(env)
#     bread_robot = FoodOrderRobotv1(robot=UR3_robot, env=env)
#     robot = UR3_robot

#     bread_locations, bread_meshes = collect_bread_locations_and_meshes(bread_piles)
#     station_location = [1.75, 0.2, 1]
#     hover_height = 0.

#     # Tool orientation (horizontal along -x)
#     tool_orientation = [0, -pi/2, -pi]

#     # Initialize last joint angles for IK guesses
#     q_last = robot.q.copy()

#     print(f"Processing order with {len(bread_locations)} ingredients...")

#     for i, bread_pos in enumerate(bread_locations):
#         mesh = None
#         if bread_meshes is not None and i < len(bread_meshes):
#             mesh = bread_meshes[i]
#             print(f"  Attaching mesh {i}: {type(mesh)}")

#         # Hover above bread
#         hover_above_food = [bread_pos[0] - hover_height, bread_pos[1], bread_pos[2]]
#         hover_pose = SE3(transl(hover_above_food) @ rpy2tr(*tool_orientation))

#         q_hover, success, error = bread_robot.solve_ik_robust(robot, hover_pose, q_last)
#         if not success:
#             print(f"  Failed to reach hover position for ingredient {i+1}")
#             continue

#         print(f"  Moving to hover above food (IK+jtraj)...")
#         bread_robot.execute_trajectory(robot, env, q_last, q_hover, mesh=None)
#         q_last = q_hover.copy()

#         # Pick bread (RMRC down)
#         print(f"  Picking up ingredient {i+1} (RMRC down)...")
#         bread_robot.rmrc_vertical_movement(robot, env, hover_above_food, bread_pos, tool_orientation, mesh=None)

#         # Attach mesh
#         attached_mesh = mesh

#         # Lift bread (RMRC up)
#         print(f"  Lifting ingredient {i+1} (RMRC up)...")
#         bread_robot.rmrc_vertical_movement(
#             robot, env, bread_pos, hover_above_food, tool_orientation,
#             mesh=attached_mesh, grip_offset=SE3.Ry(pi/2) @ SE3.Rz(pi/2)
#         )

#         # Hover above station
#         hover_above_station = [station_location[0], station_location[1], station_location[2] + hover_height]
#         station_hover_pose = SE3(transl(hover_above_station) @ rpy2tr(*tool_orientation))

#         q_station, success, error = bread_robot.solve_ik_robust(robot, station_hover_pose, q_last)
#         if not success:
#             print(f"  Failed to reach station hover position for ingredient {i+1}")
#             continue

#         print(f"  Moving to hover above station (IK+jtraj)...")
#         bread_robot.execute_trajectory(
#             robot, env, q_last, q_station, mesh=attached_mesh,
#             grip_offset=SE3.Ry(pi/2) @ SE3.Rz(pi/2)
#         )
#         q_last = q_station.copy()

#         # Place bread (RMRC down)
#         print(f"  Placing ingredient {i+1} at station (RMRC down)...")
#         place_pos = station_location
#         bread_robot.rmrc_vertical_movement(
#             robot, env, hover_above_station, place_pos, tool_orientation,
#             mesh=attached_mesh, grip_offset=SE3.Ry(pi/2) @ SE3.Rz(pi/2)
#         )

#         # Retract (RMRC up)
#         print(f"  Retracting from station (RMRC up)...")
#         bread_robot.rmrc_vertical_movement(
#             robot, env, place_pos, hover_above_station, tool_orientation,
#             grip_offset=SE3.Ry(pi/2) @ SE3.Rz(pi/2)
#         )



if __name__ == "__main__":
    main()
    # bread()


