# food_order_robot.py
# Simplified food order processing system for ABB IRB120
# Combines IK+jtraj for large movements and RMRC for precise pick/place operations

import numpy as np
from scipy import linalg
import time
from roboticstoolbox import jtraj

import swift
from spatialmath.base import transl, rpy2tr, tr2rpy
from spatialmath import SE3
from math import pi

# Import the ABB IRB120 robot model
from Micahs_robot.abb_irb_120 import abb_irb_120
from FoodOrderRobotv1 import FoodOrderRobotv1

def moveBread():
    breadRobot = FoodOrderRobotv1()
    q_hover, success, error = self.solve_ik_robust(robot, hover_pose, robot.q)
    if not success:
        print(f"  Failed to reach hover position for ingredient {i+1}")
        continue

    print(f"  Moving to hover above food (IK+jtraj)...")
    self.execute_trajectory(robot, env, robot.q.copy(), q_hover, mesh=None)
    
    print(f"  Picking up ingredient {i+1} (RMRC down)...")
    self.rmrc_vertical_movement(robot, env, hover_above_food, food_pos, tool_orientation, mesh=None)

    print(f"  Attaching mesh {i} to end effector...")
    attached_mesh = mesh
    
    print(f"  Lifting ingredient {i+1} (RMRC up)...")
    self.rmrc_vertical_movement(robot, env, food_pos, hover_above_food, tool_orientation, mesh=attached_mesh)
            