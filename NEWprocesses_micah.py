import numpy as np
from math import pi
from spatialmath import SE3
from NEWspawnEnv import RobotEnvironment
from NEWcollisionDetection import CollsionDetection
from ir_support import EllipsoidRobot
from roboticstoolbox.backends.PyPlot import PyPlot
import roboticstoolbox as rtb
import time
from IngredientsLiam import spawn_ingredient, ENV_OFFSET


class processes:
    def __init__(self, env_instance: RobotEnvironment):
        self.env_instance = env_instance
        self.env = env_instance.env
        self.meat_robot_ctrl = env_instance.meat_robot_ctrl
        self.veggie_robot_ctrl = env_instance.veggie_robot_ctrl
        self.cobot_ctrl = env_instance.cobot_ctrl
        self.UR3_robot = env_instance.UR3_robot
        self.bench_points = env_instance.bench_points



        self.ellipsoid_meat = EllipsoidRobot(self.meat_robot_ctrl.robot, fig=None, default_height=0.08, default_width=0.04)
        # self.ellipsoid_veggie = EllipsoidRobot(self.veggie_robot_ctrl.robot, fig=None, default_height=0.08, default_width=0.04)
        # self.ellipsoid_cobot = EllipsoidRobot(self.cobot_ctrl.robot, fig=None, default_height=0.08, default_width=0.04)
        # self.ellipsoid_UR3 = EllipsoidRobot(self.UR3_robot.robot, fig=None, default_height=0.08, default_width=0.04)


    def process_sandwich_individually(self, meat_locs, meat_meshes, veggie_locs, veggie_meshes,
                                bread_bottom_locs, bread_bottom_meshes,
                                bread_top_locs, bread_top_meshes,
                                tray_locs, tray_meshes):
        print("Processing meats with XArm6...")
        offset = 0.24

        base_z = 1.0  # Assuming base z is 1.0
        tray_final_z = self.UR3_robot.process_food_order(tray_locs, [2.7, 1, base_z], tray_meshes, initial_z=base_z)
        bread_bottom_final_z = self.UR3_robot.process_food_order(bread_bottom_locs, [2.7, 1, base_z], bread_bottom_meshes, initial_z=tray_final_z)



        # Move to position before sliding
        success = self.UR3_robot.move_to_start_position(
        self.UR3_robot.robot,
        self.env,
        target_pos=[2.7 + offset, 1, base_z],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )
        # Slide order
        mesh_list = bread_bottom_meshes + tray_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
        self.UR3_robot.rmrc_vertical_movement(self.UR3_robot.robot,
            self.env,
            SE3(2.7 + offset, 1, base_z).t, SE3(2.45, 1, base_z).t,
            tool_orientation=[pi,0,0],
            mesh_list=mesh_list,
            mesh_offset_along_x=-offset,
            mesh_z_offsets=mesh_z_offsets)  # Fixed typo: removed meat_meshes


        # Move to position after sliding
        success = self.UR3_robot.move_to_start_position(
        self.UR3_robot.robot,
        self.env,
        target_pos=[2.9, 1, base_z],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )


        # Process food order for meats with special gap since previous is bread
        meat_initial_z = bread_bottom_final_z + 0.02  # To make gap 0.3 instead of 0.1
        meat_final_z = self.meat_robot_ctrl.process_food_order(meat_locs, [2.45-offset/2, 1, base_z], meat_meshes, initial_z=meat_initial_z)
        # Move to position before sliding
        success = self.meat_robot_ctrl.move_to_start_position(
        self.meat_robot_ctrl.robot,
        self.env,
        target_pos=[2.45 + offset, 1, base_z],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )
        # Slide order
        mesh_list = meat_meshes + bread_bottom_meshes + tray_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
        self.meat_robot_ctrl.rmrc_vertical_movement(self.meat_robot_ctrl.robot,
            self.env,
            SE3(2.45 + offset, 1, base_z).t, SE3(1.7, 1, base_z).t,
            tool_orientation=[pi,0,0],
            mesh_list=mesh_list,
            mesh_offset_along_x=-offset,
            mesh_z_offsets=mesh_z_offsets)
        
        # Move to position after sliding
        success = self.meat_robot_ctrl.move_to_start_position(
        self.meat_robot_ctrl.robot,
        self.env,
        target_pos=[2.7, 1, base_z],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )
        
        
        
        print("Processing veggies with IRB...")

        # Move to position before sliding
        success = self.veggie_robot_ctrl.move_to_start_position(
        self.veggie_robot_ctrl.robot,
        self.env,
        target_pos=[1.7 - offset, 1, base_z],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )

        mesh_list = meat_meshes + bread_bottom_meshes + tray_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
        self.veggie_robot_ctrl.rmrc_vertical_movement(self.veggie_robot_ctrl.robot,
            self.env,
            SE3(1.7 - offset, 1, base_z).t, SE3(1.1, 1, base_z).t,
            tool_orientation=[pi,0,0],
            mesh_list=mesh_list,
            mesh_offset_along_x=offset,
            mesh_z_offsets=mesh_z_offsets)
        

        

        # Adjust veggie initial z with gap on meat final z
        if len(meat_locs) > 0:
            veggie_initial_z = meat_final_z + 0.02  # Add gap for stacking veggies on meat
        else:
            veggie_initial_z = bread_bottom_final_z + 0.02  # No meat, stack on bread

        
        veggie_final_z = self.veggie_robot_ctrl.process_food_order(veggie_locs, 
            [1.1 + 2*offset, 1, base_z],
            veggie_meshes, initial_z=veggie_initial_z)

        # Move to position before sliding
        success = self.veggie_robot_ctrl.move_to_start_position(
            self.veggie_robot_ctrl.robot,
            self.env,
            target_pos=[1.1 + 2*offset, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None
        )

        # sliding to liam
        mesh_list = meat_meshes + veggie_meshes + bread_bottom_meshes + tray_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
        self.veggie_robot_ctrl.rmrc_vertical_movement(
            self.veggie_robot_ctrl.robot,
            self.env,
            SE3(1.1, 1.0, base_z).t,
            SE3(1.05, 0.75, base_z).t,
            tool_orientation=[pi, 0, 0],
            mesh_list=mesh_list,
            mesh_offset_along_x=-offset,
            mesh_z_offsets=mesh_z_offsets
        )

        # moves out of way
        success = self.veggie_robot_ctrl.move_to_start_position(
            self.veggie_robot_ctrl.robot,
            self.env,
            target_pos=[1.2 + 2*offset, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None
        )

        # Slide order meshes bottom to top: tray, bread, meat, veggies
        mesh_list = (tray_meshes + bread_bottom_meshes + meat_meshes + veggie_meshes)

        # Calculate updated z offsets relative to base_z
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]


        self.cobot_ctrl.process_food_order(bread_top_locs, 
                                           [1.05-offset/2, 0.75, base_z + 0.02], 
                                           bread_top_meshes, 
                                           tool_orientation=[0,0,0], 
                                           initial_z=veggie_final_z, grip_offset=SE3.Rz(pi))
        


        success = self.cobot_ctrl.move_to_start_position(
            self.cobot_ctrl.robot,
            self.env,
            target_pos=[1.05 - 2*offset, 0.75, base_z],
            tool_orientation=[0, 0, 0],
            mesh_list=None
        )
        

        
        self.cobot_ctrl.rmrc_vertical_movement(self.cobot_ctrl.robot,
            self.env,
            SE3(1.05 - offset, 0.75, base_z).t, SE3(0.55 - offset, 0.4, base_z).t,
            tool_orientation=[0,0,0],
            grip_offset=SE3.Rz(pi),
            mesh_list=(meat_meshes + veggie_meshes + bread_bottom_meshes + tray_meshes + bread_top_meshes)
            )
        
        self.cobot_ctrl.rmrc_vertical_movement(self.cobot_ctrl.robot,
            self.env,
            SE3(0.55 - offset, 0.4, base_z).t, (SE3(0.55 - offset, 0.4, base_z) * SE3.Rz(pi/2)).t,
            tool_orientation=[0,0, 0], 
            grip_offset=SE3.Rz(pi),
            mesh_list=(meat_meshes + veggie_meshes + bread_bottom_meshes + tray_meshes + bread_top_meshes)
            )



    #     veggie_initial_z = meat_final_z if len(meat_locs) > 0 else bread_bottom_final_z + 0.02  # Normal gap or special if no meat
    #     veggie_final_z = self.veggie_robot_ctrl.process_food_order(veggie_locs, [1.5, 1, base_z], veggie_meshes, initial_z=veggie_initial_z)
        
    #     # Move to position before sliding
    #     success = self.veggie_robot_ctrl.move_to_start_position(
    #     self.veggie_robot_ctrl.robot,
    #     self.env,
    #     target_pos=[1.5 + offset, 1, base_z],
    #     tool_orientation=[pi, 0, 0],
    #     mesh_list=None
    # )
    #     # Slide order
    #     mesh_list = meat_meshes + veggie_meshes + bread_bottom_meshes + tray_meshes
    #     mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
    #     self.veggie_robot_ctrl.rmrc_vertical_movement(self.veggie_robot_ctrl.robot,
    #         self.env,
    #         SE3(1.5 + offset, 1.0, base_z).t, SE3(1.3 + offset, 1.0, base_z).t,
    #         tool_orientation=[pi,0,0],
    #         mesh_list=mesh_list,
    #         mesh_offset_along_x=-offset,
    #         mesh_z_offsets=mesh_z_offsets)

        # Assuming bread_top would be added similarly if needed
        # For example:
        # bread_top_initial_z = veggie_final_z  # Normal gap, since previous not bread
        # bread_top_final_z = self.UR3_robot.process_food_order(bread_top_locs, [new_x, 1, base_z], bread_top_meshes, initial_z=bread_top_initial_z)
        # Then slide with all meshes, etc.


    def force_collision(self):
        steps = 50
        q0 = np.zeros(6)
        q_pick = self.meat_robot_ctrl.robot.ikine_LM(SE3(2.45, 1, 0.7), q0=q0).q
        q_original = self.meat_robot_ctrl.robot.q.copy()


        # Create trajectory
        traj = rtb.jtraj(self.meat_robot_ctrl.robot.q, q_pick, steps).q


        print("Moving robot to force collision...")


        for q in traj:
            self.meat_robot_ctrl.robot.q = q
            self.ellipsoid_meat.ellipsoid_for_robot_links(self.meat_robot_ctrl.robot.q)


        # Check for collision at this step
            collisions = CollsionDetection.detect_collisions(self.ellipsoid_meat, self.bench_points)
            if collisions:
                print(f"⚠️ Collision detected! Movement stopped.")
                self.meat_robot_ctrl.set_estop(True)  # stop robot immediately
                return True


            # Update visualization
            if hasattr(self.env, "step"):
                self.env.step()  # Swift or other env
            else:
                self.meat_robot_ctrl.robot.plot(q, block=False)
                # print("❌ No collision detected along trajectory. Adjust joint angles to force collision.")


            time.sleep(0.05)


        print("❌ No collision detected along trajectory. Adjust joint angles to force collision.")
        return False


        # traj_back = rtb.jtraj(self.meat_robot_ctrl.robot.q, q_original, steps).q
        # print("Resetting robot to original position...")
        # for q in traj_back:
        #     self.meat_robot_ctrl.robot.q = q
        #     self.env.draw(self.meat_robot_ctrl.robot)
        #     time.sleep(0.05)


        # print("✅ Robot reset complete.")