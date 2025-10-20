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

class BreadMovement:
    def bread(self, bread_locations, station_location, mesh_list = None):
        robot = self.robot
        env = self.env
        
        env.launch(realtime=True)        
        station_location = [1.4,0.2,1] 
        tool_orientation = [0, -pi/2, pi]  

        print(f"Processing order with {len(bread_locations)} ingredients...")
        print(f"Station location: {bread_locations}")
        placed_meshes = []

        hover_height = 0.2

        # Process each food item in the order list
        for i, bread_pos in enumerate(bread_locations):
            mesh = None
            if mesh_list is not None and i < len(mesh_list):
                mesh = mesh_list[i]
                print(f"  Attaching mesh {i}: {type(mesh)}")

            # Calculate hover position
            hover_above_food = [bread_pos[0] - hover_height, bread_pos[1], bread_pos[2]]
            hover_pose = SE3(transl(hover_above_food) @ rpy2tr(*tool_orientation))

            q_hover, success, error = self.robot.solve_ik_robust(robot, hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach hover position for ingredient {i+1}")
                continue

            print(f"  Moving to hover above food (IK+jtraj)...")
            self.execute_trajectory(robot, env, robot.q.copy(), q_hover, mesh=None)
            
            print(f"  Picking up ingredient {i+1} (RMRC down)...")
            self.rmrc_vertical_movement(robot, env, hover_above_food, bread_pos, tool_orientation, mesh=None)

            print(f"  Attaching mesh {i} to end effector...")
            attached_mesh = mesh
            
            print(f"  Lifting ingredient {i+1} (RMRC up)...")
            self.rmrc_vertical_movement(robot, env, bread_pos, hover_above_food, tool_orientation, mesh=attached_mesh, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2) @ SE3.Ty(0.2))

    
            hover_above_station = [station_location[0], station_location[1], station_location[2] + hover_height]
            tool_orientation = [0, -pi/2, 0]  

            station_hover_pose = SE3(transl(hover_above_station) @ rpy2tr(*tool_orientation))
            q_station, success, error = self.solve_ik_robust(robot, station_hover_pose, robot.q)
            if not success:
                print(f"  Failed to reach station hover position for ingredient {i + 1}")
                continue

            print(f"  Moving to hover above station (IK+jtraj)...")
            self.robot.execute_trajectory(robot, env, robot.q.copy(), q_station, mesh=attached_mesh, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2) @ SE3.Ty(0.2))

            print(f"  Placing ingredient {i+1} at station (RMRC down)...")
            place_pos = station_location
            self.rmrc_vertical_movement(robot, env, hover_above_station, place_pos, tool_orientation, mesh=attached_mesh, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2) @ SE3.Ty(0.2))

            print(f"  Retracting from station (RMRC up)...")
            self.rmrc_vertical_movement(robot, env, place_pos, hover_above_station, tool_orientation, grip_offset = SE3.Ry(pi/2) @ SE3.Rz(pi/2) @ SE3.Ty(0.2))
            tool_orientation = [0, -pi/2, pi]  