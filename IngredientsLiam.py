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
import numpy as np
from spatialmath import SE3
from spatialmath.base import rpy2tr
from scipy import linalg
from math import pi

# --- Environment Offset ---
ENV_OFFSET = np.array([1.5, 0.5, 0.0])    # Shift everything in environment

# --- Ingredient Configuration ---
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
    x0, y0, z0 = np.array(center) + ENV_OFFSET
    pile = []
    for i in range(n):
        dx = np.random.uniform(*dx_range)
        dy = np.random.uniform(*dy_range)
        dz = i * dz_step + np.random.uniform(*dz_jitter)
        pose = SE3(x0 + dx, y0 + dy, z0 + dz)
        piece = spawn_ingredient(env, ingredient, pose)
        pile.append(piece)
    return pile

def move_sandwich_mesh(mesh_list, start, end, env, steps=50):
    # Moves all sandwich meshes together from start to end position
    for s in np.linspace(start, end, steps):
        for m in mesh_list:
            if m is not None:
                m.T = SE3(s)
        env.step(0.02)

class RobotEnvironment:
    def __init__(self):
        print("Launching Swift environment from GUI...")
        self.env = swift.Swift()
        self.env.launch(realtime=True)

        # Setup
        self.spawn_items(self.env)
        self.UR3_robot, self.XArm_robot, self.irb_robot, self.Cobot = self.setup_robots(self.env)
        self.piles = self.spawn_all_piles(self.env)
        self.bread_piles = self.spawn_bread(self.env)
        self.env.set_camera_pose([3, -3, 2], [1, 0.5, 0])
        self.env.step()

        self.meat_robot_ctrl = FoodOrderRobotv1(robot=self.XArm_robot, env=self.env)
        self.veggie_robot_ctrl = FoodOrderRobotv1(robot=self.Cobot, env=self.env)

    def setup_robots(self, env):
        UR3_robot = UR3()
        XArm_robot = XArm6()
        irb_robot = abb_irb_120()
        Cobot = Cobot320()
        # Apply offset to bases
        irb_robot.base = SE3(-0.95, 0.15, 1) * SE3.Trans(ENV_OFFSET) * SE3.Rz(-pi / 2)
        Cobot.base = SE3(0, 0.1, 1) * SE3.Trans(ENV_OFFSET)
        XArm_robot.base = SE3(0.95, 0.15, 1) * SE3.Trans(ENV_OFFSET)
        UR3_robot.base = SE3(1.75, 0.2, 1) * SE3.Trans(ENV_OFFSET)
        

        XArm_robot.add_to_env(env)
        irb_robot.add_to_env(env)
        UR3_robot.add_to_env(env)
        Cobot.add_to_env(env)
        return UR3_robot, XArm_robot, irb_robot, Cobot

    def spawn_items(self, env):
        spawn_ingredient(env, "bench", SE3(*ENV_OFFSET))
        spawn_ingredient(env, "glass", SE3(*ENV_OFFSET))
        spawn_ingredient(env, "bread_rack", SE3(*ENV_OFFSET))

    def spawn_bread(self, env):
        bread_piles = {}
        storage_z = [1.4, 1.1]
        for ingredient in ["bread_bottom", "bread_top"]:
            bread_piles[ingredient] = []
            for z in storage_z:
                for i in range(5):
                    pose = SE3(2.37, -0.2 + i * 0.2, z) * SE3.Trans(ENV_OFFSET) @ SE3.Rz(pi / 2)
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
    
    def process_sandwich_individually(self, meat_locs, meat_meshes, veggie_locs, veggie_meshes, bread_locs, bread_meshes):
        print("Processing meats with XArm6...")
        self.meat_robot_ctrl.process_food_order(meat_locs, station_location=[2.45, 1.0, 1], mesh_list=meat_meshes)
        self.meat_robot_ctrl.rmrc_vertical_movement(self.meat_robot_ctrl, self.env, SE3[2.45, 1.0, 1], SE3[2.0, 1.0, 1], tool_orientation=any, mesh_list=meat_meshes)
        
        print("Processing veggies with Cobot320...")
        self.meat_robot_ctrl.rmrc_vertical_movement(self.meat_robot_ctrl, self.env, SE3[2.0, 1.0, 1], SE3[1.5, 1.0, 1], tool_orientation=any, mesh_list=meat_meshes)
        self.veggie_robot_ctrl.process_food_order(veggie_locs, station_location=[1.5, 1.0, 1], mesh_list=veggie_meshes)
        
        # You may add a bread_robot_ctrl for breads if needed:
        # print("Processing bread with UR3...")
        # self.bread_robot_ctrl.process_food_order(bread_locs, station_location=[2.0, 1.0, 1.1], mesh_list=bread_meshes)
        
        # Optional: Implement synchronization or waiting before final assembly
    
    print("All ingredient placements done, assembling sandwich...")
    # Implement sandwich assembly logic here, potentially by one robot or manually

    
    # def rmrc_slide(self, robot, start_pos, end_pos, mesh_list):
    #     # Calls the rmrc_vertical_movement method in FoodOrderRobotv1 instance for precise sliding
    #     if robot == self.XArm_robot:
    #         ctrl = self.meat_robot_ctrl
    #     elif robot == self.Cobot:
    #         ctrl = self.veggie_robot_ctrl
    #     else:
    #         raise ValueError("Unsupported robot for RMRC slide")

    #     ctrl.rmrc_vertical_movement(robot, self.env, start_pos, end_pos, [pi,0,0], mesh_list)

    # def move_robot_between(self, robot, start_q, end_pose_SE3, mesh_list):
    #     # Calls solve_ik_robust and execute_trajectory from FoodOrderRobotv1 instance
    #     if robot == self.XArm_robot:
    #         ctrl = self.meat_robot_ctrl
    #     elif robot == self.Cobot:
    #         ctrl = self.veggie_robot_ctrl
    #     else:
    #         raise ValueError("Unsupported robot for move_robot_between")

    #     q_end, success, error = ctrl.solve_ik_robust(robot, end_pose_SE3, start_q)
    #     if not success:
    #         print(f"IK failed: cannot reach {end_pose_SE3} (error={error})")
    #         return False
    #     ctrl.execute_trajectory(robot, self.env, start_q, q_end, mesh_list=mesh_list)
    #     return True

    # def process_sandwich_along_line(self, meat_locs, meat_meshes, veggie_locs, veggie_meshes):
    #     path = [
    #         [2.45, 1.0, 1],    # meat robot station
    #         [2.3, 1.0, 1],     # halfway handover point
    #         [1.7, 1.0, 1],     # veggie robot halfway pickup
    #         [1.5, 1.0, 1],     # veggie robot stacking station
    #         [1.4, 1.0, 1],     # delivery point
    #     ]

    #     # Meat stacking
    #     self.meat_robot_ctrl.process_food_order(meat_locs, path[0], mesh_list=meat_meshes)
    #     print("[INFO] Meat stacking complete.")

    #     # Meat robot slides sandwich to halfway handover
    #     self.rmrc_slide(self.XArm_robot, path[0], path[1], meat_meshes)
    #     print("[INFO] Meat robot slid sandwich to halfway handover.")

    #     # Veggie robot moves itself to halfway pickup (no mesh attach)
    #     self.move_robot_between(self.Cobot, self.Cobot.q.copy(), SE3(path[2]), None)
    #     print("[INFO] Veggie robot moved to halfway pickup.")

    #     # Veggie robot slides sandwich from halfway pickup to stacking station
    #     self.rmrc_slide(self.Cobot, path[2], path[3], meat_meshes + veggie_meshes)
    #     print("[INFO] Veggie robot slid sandwich to stacking station.")

    #     # Veggie robot stacks veggies
    #     self.veggie_robot_ctrl.process_food_order(veggie_locs, path[3], mesh_list=veggie_meshes)
    #     print("[INFO] Veggie stacking complete.")

    #     # Veggie robot slides assembled sandwich to delivery point
    #     self.rmrc_slide(self.Cobot, path[3], path[4], meat_meshes + veggie_meshes)
    #     print("[INFO] Veggie robot slid sandwich to delivery point.")


