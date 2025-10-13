import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3 
from spatialgeometry import Mesh
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
import sys
# sys.path.append(os.path.dirname(__file__))
from Lilys_robot.XArm6 import XArm6
from Liams_robot.Cobot320 import Cobot320
from Micahs_robot.A2 import BaxterQ 
from ir_support import UR3
from math import pi
# from ir_support.robots import XArm6



UR3 = UR3()
XArm = XArm6()
Cobot = Cobot320()
# Baxter = BaxterQ() 

env = swift.Swift()
env.launch(realtime=True)

UR3.base = SE3(0.75,0.5,1)
UR3.add_to_env(env)

XArm.base = SE3(0,0.5,1)
XArm.add_to_env(env)

Cobot.base = SE3(-0.75,0.5,1)
Cobot.add_to_env(env)

# Baxter.base = SE3()

current_path = os.path.abspath(os.path.dirname(__file__))

# Build the path to bench.stl relative to this file
BENCH = os.path.join(current_path, "env", "bench.stl")

# BENCH = r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\bench.stl""
# BENCH = 'env/bench'
bench = Mesh(filename=BENCH, 
             pose=SE3(0, 0, 0), 
             color=[0.6, 0.6, 0.6, 1])
env.add(bench)

GLASS = os.path.join(current_path, "env", "glass.stl")

# BENCH = r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\bench.stl""
# BENCH = 'env/bench'
glass = Mesh(filename=GLASS, 
             pose=SE3(0, 0, 0), 
             color=[0.6, 0.6, 0.6, 0.4])
env.add(glass)

# BREAD_BOTTOM = r"C:\Users\lilyb\OneDrive\Documents\!Uni second Year\IR\A2v2\sandwich\bread-bottom.stl"
# bread_bottom = Mesh(filename=BREAD_BOTTOM, 
#              pose=SE3(-1, -0.25, 1),
#              scale=[0.1,0.1,0.1], 
#              color=[0.95, 0.77, 0.53, 1]
#              )
# env.add(bread_bottom)

# BREAD_TOP = r"C:\Users\lilyb\OneDrive\Documents\!Uni second Year\IR\A2v2\sandwich\bread-top.stl"
# bread_top = Mesh(filename=BREAD_TOP, 
#              pose=SE3(-1, -0.25, 1.04), 
#              scale=[0.1,0.1,0.1],
#              color=[0.95, 0.77, 0.53, 1]
#              )
# env.add(bread_top)

# HAM = r"C:\Users\lilyb\OneDrive\Documents\!Uni second Year\IR\A2v2\sandwich\ham.stl"
# ham = Mesh(filename=HAM, 
#              pose=SE3(-0.99, -0.34, 1.01), 
#              scale=[0.1,0.1,0.1],
#              color=[0.7, 0.5, 0.7, 1]
#              )
# env.add(ham)
# ham1 = Mesh(filename=HAM, 
#              pose=SE3(-1.03, -0.14, 1.01), 
#              scale=[0.1,0.1,0.1],
#              color=[0.7, 0.5, 0.7, 1]
#              )
# env.add(ham1)

# SALAMI = r"C:\Users\lilyb\OneDrive\Documents\!Uni second Year\IR\A2v2\sandwich\salami.stl"
# # salami = Mesh(filename=SALAMI, 
# #              pose=SE3(0, 0, 1.07), 
# #              scale=[0.1,0.1,0.1],
# #              color=[0.7, 0.5, 0.7, 1]
# #              )
# # env.add(salami)

# LETTACE = r"C:\Users\lilyb\OneDrive\Documents\!Uni second Year\IR\A2v2\sandwich\lettace.stl"
# lettace = Mesh(filename=LETTACE, 
#              pose=SE3(-1.03, -0.26, 1.04), 
#              scale=[0.01,0.01,0.01],
#              color=[0.25, 0.86, 0.37, 1]
#              )
# env.add(lettace)

# TOMATO = r"C:\Users\lilyb\OneDrive\Documents\!Uni second Year\IR\A2v2\sandwich\tomato.stl"
# tomato = Mesh(filename=TOMATO, 
#              pose=SE3(-0.98, -0.34, 1.04), 
#              scale=[0.05,0.05,0.05],
#              color=[0.8, 0.1, 0.1, 1]
#              )
# env.add(tomato)
# tomato1 = Mesh(filename=TOMATO, 
#              pose=SE3(-1.03, -0.24, 1.04), 
#              scale=[0.05,0.05,0.05],
#              color=[0.8, 0.1, 0.1, 1]
#              )
# env.add(tomato1)
# tomato2 = Mesh(filename=TOMATO, 
#              pose=SE3(-0.98, -0.14, 1.04), 
#              scale=[0.05,0.05,0.05],
#              color=[0.8, 0.1, 0.1, 1]
#              )
# env.add(tomato2)

# def add_mesh(env, path, pose, scale=(0.1, 0.1, 0.1), color=(1,1,1,1)):
#     mesh = Mesh(filename=path, pose=pose, scale=scale, color=color)
#     env.add(mesh)
#     return mesh

# bench = add_mesh(env, BENCH, SE3(0,0,0), color=[0.6, 0.6, 0.6, 1])
# bread_bottom = add_mesh(env, BREAD_BOTTOM, SE3(-1, -0.25, 1), color=[0.95, 0.77, 0.53, 1])
# bread_top = add_mesh(env, BREAD_TOP, SE3(-1, -0.25, 1.04), color=[0.95, 0.77, 0.53, 1])
# ham = add_mesh(env, HAM, SE3(-0.99, -0.34, 1.01), color=[0.7, 0.5, 0.7, 1])
# ham1 = add_mesh(env, HAM, SE3(-1.03, -0.14, 1.01), color=[0.7, 0.5, 0.7, 1])
# lettuce = add_mesh(env, LETTACE, SE3(-1.03, -0.26, 1.04), scale=(0.1,0.1,0.1), color=[0.25, 0.86, 0.37, 1])
# tomatoes = [
#     add_mesh(env, TOMATO, SE3(-0.98, -0.34, 1.04), scale=(0.05,)*3, color=[0.8,0.1,0.1,1]),
#     add_mesh(env, TOMATO, SE3(-1.03, -0.24, 1.04), scale=(0.05,)*3, color=[0.8,0.1,0.1,1]),
#     add_mesh(env, TOMATO, SE3(-0.98, -0.14, 1.04), scale=(0.05,)*3, color=[0.8,0.1,0.1,1])
# ]



env.hold()

