import numpy as np
import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from spatialgeometry import Mesh
import os
import time
from math import pi
from Lilys_robot.XArm6 import XArm6
from Micahs_robot.abb_irb_120 import abb_irb_120
from ir_support.robots import UR3

# --- Setup environment ---
env = swift.Swift()
env.launch(realtime=True)

# ----robots -------
# UR3 = UR3()
# XArm = XArm6()
# Cobot = Cobot320()
# UR3.base = SE3(0.75,0.5,1)
# UR3.add_to_env(env)

# XArm.base = SE3(1, 0.15, 1)
# XArm.add_to_env(env)

current_path = os.path.abspath(os.path.dirname(__file__))

UR3 = UR3()
XArm = XArm6()
irb = abb_irb_120()
# Cobot = Cobot320()


UR3.base = SE3(1.75,0.2,1)
UR3.add_to_env(env)

XArm.base = SE3(0.95,0.25,1)
XArm.add_to_env(env)

irb.base = SE3(0,0.25,1) @ SE3.Rz(-pi/2)
irb.add_to_env(env)
# XArm.add_to_env(env)

# --- Ingredient definitions  ---
INGREDIENTS = {
    "glass": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\tomato.stl",
        "path": os.path.join(current_path, "env", "glass.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 0.4]
    },
    "bench": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\bench.stl",
        "path": os.path.join(current_path, "env", "benchv2.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 1]
    },
        "bread_rack" : {
        "path": os.path.join(current_path, "env", "bread_rack.stl"),
        "scale": (1,1,1),
        "color": [0.6, 0.6, 0.6, 1]
    },
    "bread_bottom": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\bread-bottom.stl",
        "path": os.path.join(current_path, "env", "sandwich", "bread-bottom.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.95, 0.77, 0.53, 1]
    },
    "bread_top": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\bread-top.stl",
        "path": os.path.join(current_path, "env", "sandwich", "bread-top.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.95, 0.77, 0.53, 1]
    },
    "ham": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\ham.stl",
        "path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.7, 0.5, 0.7, 1]
    },
    "lettuce": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\lettace.stl",
        "path": os.path.join(current_path, "env", "sandwich", "lettace.stl"),
        "scale": (0.01,0.01,0.01),
        "color": [0.25, 0.86, 0.37, 1]
    },
    "tomato": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\tomato.stl",
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.05),
        "color": [0.8, 0.1, 0.1, 1]
    },
    "salami": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\tomato.stl",
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.03),
        "color": [0.7, 0.3, 0.3, 1]
    },
    "beef": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\ham.stl",
        "path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.56, 0.3, 0.04, 1]
    },
        "chicken": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\tomato.stl",
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.03),
        "color": [0.9, 0.8, 0.6, 1]
    },
        "cucumber": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\tomato.stl",
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.04, 0.04, 0.04),
        "color": [0.2, 0.6, 0.3, 1]
    },
        "beetroot": {
        # "path": r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\sandwich\tomato.stl",
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.04, 0.04, 0.04),
        "color": [0.5, 0.09, 0.4, 1]
    }
}

# --- Mesh registry so you can reference them later ---
spawned_meshes = {}

# --- Function to spawn any ingredient with a unique name and pose ---
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

# --- Example spawns ---
bench = spawn_ingredient(env, "bench", SE3(0, 0, 0))
glass =spawn_ingredient(env, "glass", SE3(0, 0, 0))
bread_rack =spawn_ingredient(env, "bread_rack", SE3(0, 0, 0))
# bread_bottom = spawn_ingredient(env, "bread_bottom", SE3(1.2,0.4,1) @ SE3.Rz(pi/2))
bread_bottom = spawn_ingredient(env, "bread_bottom", SE3(-1, 0.25, 1))
ham1 = spawn_ingredient(env, "ham", SE3(-0.99, 0.34, 1.01))
ham2 = spawn_ingredient(env, "ham", SE3(-1.03, 0.14, 1.01))
lettuce1 = spawn_ingredient(env, "lettuce", SE3(-1.01, 0.25, 1.04)@ SE3.Rz(pi/2))
lettuce2 = spawn_ingredient(env, "lettuce", SE3(-1.03, 0.27, 1.04)@ SE3.Rz(-pi/2))
tomato1 = spawn_ingredient(env, "tomato", SE3(-0.98, 0.34, 1.04))
tomato2 = spawn_ingredient(env, "tomato", SE3(-1.03, 0.24, 1.04))
tomato3 = spawn_ingredient(env, "tomato", SE3(-0.98, 0.14, 1.04))
bread_top = spawn_ingredient(env, "bread_top", SE3(-0.7, 0.25, 1.0))

def spawn_pile(env, ingredient, n=8, center=(0, 0, 0), 
               dx_range=(-0.05, 0.05), dy_range=(-0.05, 0.05), dz_step=0.003, dz_jitter=(-0.001, 0.001)):
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

ham_pile = spawn_pile(
    env,
    ingredient="ham",
    n=8,
    center=(1.2, -0.35, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.02, 0.02),
    dz_step=0.002,
    dz_jitter=(-0.05, 0.05)
)

tomato_pile = spawn_pile(
    env,
    ingredient="tomato",
    n=12,
    center=(0.25, -0.35, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.01, 0.01),
    dz_step=0.003,
    dz_jitter=(-0.07, 0.07)
)

lettuce_pile = spawn_pile(
    env,
    ingredient="lettuce",
    n=5,
    center=(-0.25, -0.35, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.015, 0.01),
    dz_step=0.003,
    dz_jitter=(-0.07, 0.07)
)

salami_pile = spawn_pile(
    env,
    ingredient="salami",
    n=12,
    center=(1.2, -0.1, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.01, 0.01),
    dz_step=0.003,
    dz_jitter=(-0.07, 0.07)
)

beef_pile = spawn_pile(
    env,
    ingredient="beef",
    n=8,
    center=(0.75, -0.35, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.02, 0.02),
    dz_step=0.002,
    dz_jitter=(-0.05, 0.05)
)


chicken_pile = spawn_pile(
    env,
    ingredient="chicken",
    n=8,
    center=(0.75, -0.1, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.02, 0.02),
    dz_step=0.002,
    dz_jitter=(-0.05, 0.05)
)

cucumber_pile = spawn_pile(
    env,
    ingredient="cucumber",
    n=15,
    center=(0.25, -0.1, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.02, 0.02),
    dz_step=0.002,
    dz_jitter=(-0.05, 0.05)
)

beetroot_pile = spawn_pile(
    env,
    ingredient="beetroot",
    n=15,
    center=(-0.25, -0.1, 0.92),
    dx_range=(-0.1, 0.1),
    dy_range=(-0.02, 0.02),
    dz_step=0.002,
    dz_jitter=(-0.05, 0.05)
)

def spawn_bread_row(env, ingredient, n=5, start_pos=(2.5, -0.2, 1.4), dy=0.05):
    x0, y0, z0 = start_pos
    row = []
    for i in range(n):
        pose = SE3(x0, y0 + i * dy, z0) @ SE3.Rz(pi/2)
        slice_mesh = spawn_ingredient(env, ingredient, pose)
        row.append(slice_mesh)
    return row

storage_pose = (2.37, -0.2, 1.4)
bread_bottom_row = spawn_bread_row(env, "bread_bottom", n=5, start_pos=storage_pose, dy=0.2)
bread_top_row = spawn_bread_row(env, "bread_top", n=5, start_pos=storage_pose, dy=0.2)
storage_pose1 = (2.37, -0.2, 1.1)
bread_bottom_row1 = spawn_bread_row(env, "bread_bottom", n=5, start_pos=storage_pose1, dy=0.2)
bread_top_row1 = spawn_bread_row(env, "bread_top", n=5, start_pos=storage_pose1, dy=0.2) 

#===================================================================
#                         MAKE SANWICH 
#==================================================================
# def move_slice(env, slice_mesh, start_pose=None, end_pose=None, steps=40, delay=0.03):
#     if start_pose is None:
#         start_pose = slice_mesh.T  # get current transform

#     for alpha in np.linspace(0, 1, steps):
#         # Interpolate between start and end SE3 transforms
#         slice_mesh.T = end_pose
#         env.step(0)
#         time.sleep(delay)

# # Select one slice (e.g., third slice in the ham pile)
# ham_move = ham_pile[2]
# move_slice(env, ham_move, ham_move.T, SE3(1.0, 0.15, 1.03))
steps = 50
start = ham_pile[0].T
target = bread_bottom.T
q0 = np.zeros(6)
q_pick = XArm.ikine_LM(ham_pile[0].T, q0=q0).q
for q in rtb.jtraj(XArm.q, q_pick, steps).q:
    XArm.q = q
    env.step(0.05)
# print(ham_pile[0].T)

# steps = 50
# start = bread_top_row[0].T
# target = bread_bottom.T
# q0 = np.zeros(6)
# q_pick = UR3.ikine_LM(bread_top_row[0].T, q0=q0).q
# for q in rtb.jtraj(UR3.q, q_pick, steps).q:
#     UR3.q = q
#     env.step(0.05)
env.hold() 