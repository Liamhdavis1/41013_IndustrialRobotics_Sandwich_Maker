import numpy as np
import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from spatialgeometry import Mesh
import os
import time
from math import pi

# --- Setup environment ---
env = swift.Swift()
env.launch(realtime=True)

current_path = os.path.abspath(os.path.dirname(__file__))

# --- Ingredient definitions  ---
INGREDIENTS = {
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
}

# --- Mesh registry so you can reference them later ---
spawned_meshes = {}

# --- Function to spawn any ingredient with a unique name and pose ---
def spawn_ingredient(env, name, pose):
    """Create a new mesh instance from the INGREDIENTS list."""
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
bread_bottom = spawn_ingredient(env, "bread_bottom", SE3(-1, -0.25, 1))
ham1 = spawn_ingredient(env, "ham", SE3(-0.99, -0.34, 1.01))
ham2 = spawn_ingredient(env, "ham", SE3(-1.03, -0.14, 1.01))
lettuce1 = spawn_ingredient(env, "lettuce", SE3(-1.01, -0.25, 1.04)@ SE3.Rz(pi/2))
lettuce2 = spawn_ingredient(env, "lettuce", SE3(-1.03, -0.27, 1.04)@ SE3.Rz(-pi/2))
tomato1 = spawn_ingredient(env, "tomato", SE3(-0.98, -0.34, 1.04))
tomato2 = spawn_ingredient(env, "tomato", SE3(-1.03, -0.24, 1.04))
tomato3 = spawn_ingredient(env, "tomato", SE3(-0.98, -0.14, 1.04))
bread_top = spawn_ingredient(env, "bread_top", SE3(-1, -0.25, 1.04))

def spawn_pile(env, ingredient, n=8, center=(0, 0, 0), 
               dx_range=(-0.05, 0.05), dy_range=(-0.05, 0.05), dz_step=0.003, dz_jitter=(-0.001, 0.001)):
    """
    Spawns a 'pile' of any ingredient with configurable scatter.

    Parameters
    ----------
    env : swift.Swift
        The Swift environment.
    ingredient : str
        Ingredient key from the INGREDIENTS dict.
    n : int
        Number of items in the pile.
    center : tuple (x, y, z)
        Center position for the pile.
    dx_range, dy_range : tuple
        Random scatter range in X and Y directions.
    dz_step : float
        Vertical step between layers (controls how close they are).
    dz_jitter : tuple
        Random jitter range added to the Z height for natural stacking.
    """
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



env.hold()
