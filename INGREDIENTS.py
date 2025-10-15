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
from ir_support.robots import UR3

# --- Setup environment ---
env = swift.Swift()
env.launch(realtime=True)

# ----robots -------
UR3 = UR3()
XArm = XArm6()
# Cobot = Cobot320()
# UR3.base = SE3(0.75,0.5,1)
# UR3.add_to_env(env)

XArm.base = SE3(1, 0.15, 1)
XArm.add_to_env(env)

current_path = os.path.abspath(os.path.dirname(__file__))

XArm = XArm6()
XArm.base = SE3(-0.7,0.1,1)
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
        "path": os.path.join(current_path, "env", "bench.stl"),
        "scale": (1, 1, 1),
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
bread_bottom = spawn_ingredient(env, "bread_bottom", SE3(1.2,0.4,1) @ SE3.Rz(pi/2))
# bread_bottom = spawn_ingredient(env, "bread_bottom", SE3(-1, 0.25, 1))
# ham1 = spawn_ingredient(env, "ham", SE3(-0.99, 0.34, 1.01))
# ham2 = spawn_ingredient(env, "ham", SE3(-1.03, 0.14, 1.01))
# lettuce1 = spawn_ingredient(env, "lettuce", SE3(-1.01, 0.25, 1.04)@ SE3.Rz(pi/2))
# lettuce2 = spawn_ingredient(env, "lettuce", SE3(-1.03, 0.27, 1.04)@ SE3.Rz(-pi/2))
# tomato1 = spawn_ingredient(env, "tomato", SE3(-0.98, 0.34, 1.04))
# tomato2 = spawn_ingredient(env, "tomato", SE3(-1.03, 0.24, 1.04))
# tomato3 = spawn_ingredient(env, "tomato", SE3(-0.98, 0.14, 1.04))
# bread_top = spawn_ingredient(env, "bread_top", SE3(-0.7, 0.25, 1.0))

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

# lift_pose = SE3(start.t[0], start.t[1], start.t[2]+0.1)
# q_lift = XArm.ikine_LM(lift_pose, q0=q_pick).q
# for q in rtb.jtraj(XArm.q, q_lift, steps).q:
#     XArm.q = q
#     env.step(0.05)

# # Move to place pose
# q_place = XArm.ikine_LM(target, q0=q_lift).q
# for q in rtb.jtraj(XArm.q, q_place, steps).q:
#     XArm.q = q
#     ham_pile[0].T = XArm.fkine(q)  # move brick with robot
#     env.step(0.05)

# # Lower brick
# lower_pose = SE3(target.t[0], target.t[1], target.t[2])
# q_lower = XArm.ikine_LM(lower_pose, q0=q_place).q
# for q in rtb.jtraj(XArm.q, q_lower, steps).q:
#     XArm.q = q
#     ham_pile[0].T = XArm.fkine(q)
#     env.step(0.05)

# # Move robot away
# q_safe = np.zeros(6)
# for q in rtb.jtraj(XArm.q, q_safe, steps).q:
#     XArm.q = q
#     env.step(0.05)
# lettuce_slice = lettuce_pile[1]
# move_slice(env, lettuce_slice, lettuce_slice.T, SE3(-1.03, -0.27, 1.28))

# tomato_slice = tomato_pile[0]
# move_slice(env, tomato_slice, tomato_slice.T, SE3(-0.98, -0.24, 1.30))


# slice_to_move = ham_pile[2].t  # e.g., third slice
# start_z = slice_to_move[2]  # get starting height
# for z in np.linspace(start_z, 1.2, 20):
#     # Create a new SE3 pose with updated Z
#     slice_to_move.T = SE3(slice_to_move.t[0],
#                              slice_to_move.t[1],
#                              z)
#     env.step(0)  # redraw scene
#     time.sleep(0.03)


# def spawn_ham_pile(env, n=12, center=(1.2, -0.35, 0.92)):
#     x0, y0, z0 = center
#     pile = []
#     for i in range(n):
#         # Small random offsets to make it look natural
#         dx = np.random.uniform(-0.1, 0.1)   # small horizontal scatter
#         dy = np.random.uniform(-0.01, 0.01)
#         dz = i * 0.003 + np.random.uniform(-0.05, 0.05)  # stacked closely, thin layers
#         pose = SE3(x0 + dx, y0 + dy, z0 + dz)
#         ham_piece = spawn_ingredient(env, "ham", pose)
#         pile.append(ham_piece)
#     return pile

# ham_pile = spawn_ham_pile(env)

# def spawn_tomato_pile(env, n=12, center=(0.25, -0.35, 0.92)):
#     x0, y0, z0 = center
#     pile = []
#     for i in range(n):
#         # Small random offsets to make it look natural
#         dx = np.random.uniform(-0.1, 0.1)   # small horizontal scatter
#         dy = np.random.uniform(-0.01, 0.01)
#         dz = i * 0.003 + np.random.uniform(-0.07, 0.07)  # stacked closely, thin layers
#         pose = SE3(x0 + dx, y0 + dy, z0 + dz)
#         tomato_piece = spawn_ingredient(env, "tomato", pose)
#         pile.append(tomato_piece)
#     return pile

# tomato_pile = spawn_tomato_pile(env)

# def spawn_lettuce_pile(env, n=5, center=(-0.25, -0.35, 0.92)):
#     x0, y0, z0 = center
#     pile = []
#     for i in range(n):
#         # Small random offsets to make it look natural
#         dx = np.random.uniform(-0.1, 0.1)   # small horizontal scatter
#         dy = np.random.uniform(-0.015, 0.01)
#         dz = i * 0.003 + np.random.uniform(-0.07, 0.07)  # stacked closely, thin layers
#         pose = SE3(x0 + dx, y0 + dy, z0 + dz)
#         lettuce_piece = spawn_ingredient(env, "lettuce", pose)
#         pile.append(lettuce_piece)
#     return pile

# lettuce_pile = spawn_lettuce_pile(env)

# --- Example of moving a spawned mesh later ---
# tomato1.pose = SE3(-1.1, -0.3, 1.04)  # move it easily by updating pose
# import time

# # Pick up one slice (simulate)
# slice_to_pick = ham_pile[2]
# for z in np.linspace(0.95, 1.1, 20):
#     slice_to_pick.pose = SE3(1.2, -0.4, z)
#     env.step(0)
#     time.sleep(0.05)

# # Move it above sandwich
# for x in np.linspace(1.2, -1.0, 30):
#     slice_to_pick.pose = SE3(x, -0.25, 1.1)
#     env.step(0)
#     time.sleep(0.05)

# # Lower it onto sandwich
# for z in np.linspace(1.1, 1.02, 15):
#     slice_to_pick.pose = SE3(-1.0, -0.25, z)
#     env.step(0)
#     time.sleep(0.05)


# for i in range(1):
#     env.step(5)
#     pick_pose = SE3(-0.7, 0.4, 1.0)
#     place_pose = SE3(-1, 0.25, 1.04)

#     # Define gripper orientation to down
#     gripper_down_orientation = SE3.Rx(pi)

#     # Find the safe poses above pick and place locations
#     safe_pose_above_pick = pick_pose * SE3.Trans(0, 0, 0.13) * gripper_down_orientation
#     safe_pose_above_place = place_pose * SE3.Trans(0, 0, 0.13) * gripper_down_orientation

#     # Poses above the brick for picking and placing, factor in brick height
#     pick_pose_above_brick = pick_pose * SE3.Trans(0, 0, 0.1) * gripper_down_orientation
#     place_pose_above_brick = place_pose * SE3.Trans(0, 0, 0.1) * gripper_down_orientation

#     # This is here to help force the base of the robot to be in the centre as it has had issues with reaching the limit and breaking
#     #Initial guess for IK based on pick and place positions, elbow bent up, away from floor
#     q0_pick  = np.array([pi, 0, pi/2, 0, 0, 0])
#     q0_place = np.array([pi, 0, pi/2, 0, 0, 0])

#     # IK for safe poses
#     sol_safe_pick = XArm.ikine_LM(safe_pose_above_pick, q0=q0_pick)
#     sol_safe_place = XArm.ikine_LM(safe_pose_above_place, q0=q0_place)
#     # Clip joint values to enforce limits
#     q_safe_pick = np.clip(sol_safe_pick.q, XArm.qlim[0], XArm.qlim[1])
#     q_safe_place = np.clip(sol_safe_place.q, XArm.qlim[0], XArm.qlim[1])

#     # Repeat above for pick and place poses
#     sol_pick = XArm.ikine_LM(pick_pose_above_brick, q0=q_safe_pick)
#     sol_place = XArm.ikine_LM(place_pose_above_brick, q0=q_safe_place)
#     q_pick = np.clip(sol_pick.q, XArm.qlim[0], XArm.qlim[1])
#     q_place = np.clip(sol_place.q, XArm.qlim[0], XArm.qlim[1])

#     # Define the trajectories between the key poses
#     trajectory_pairs = [
#         (XArm.q, q_safe_pick),        #0
#         (q_safe_pick, q_pick),        #1
#         (q_pick, q_safe_pick),        #2
#         (q_safe_pick, q_safe_place),  #3
#         (q_safe_place, q_place),      #4
#         (q_place, q_safe_place),      #5
#         (q_safe_place, XArm.q)        #6
#     ]

#     # Execute each trajectory segment
#     for idx, (q_start, q_end) in enumerate(trajectory_pairs):
#         traj = rtb.jtraj(q_start, q_end, 75)
#         for q in traj.q:
#             q_clipped = np.clip(q, XArm.qlim[0], XArm.qlim[1])
#             XArm.q = q_clipped
#             ee_pose = XArm.fkine(XArm.q)
#             env.step(0.01)

#             # Update bread_top mesh pose during carrying motion segments
#             if idx in [2, 3, 4]: 
#                 bread_top.T = ee_pose @ SE3.Ry(pi) @ SE3.Trans(0, 0, -0.1)


env.hold()
