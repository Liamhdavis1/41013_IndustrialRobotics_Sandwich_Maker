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

        self.ellipsoid_meat = EllipsoidRobot(
            self.meat_robot_ctrl.robot,
            fig=None,
            default_height=0.08,
            default_width=0.04,
        )

    def process_sandwich_individually(
        self,
        meat_locs,
        meat_meshes,
        veggie_locs,
        veggie_meshes,
        bread_bottom_locs,
        bread_bottom_meshes,
        bread_top_locs,
        bread_top_meshes,
        tray_locs,
        tray_meshes,
    ):
        print("Processing meats with XArm6...")
        offset = 0.24
        base_z = 1.0  # Assumed base height for z-axis

        # Process tray and bread bottom orders with UR3 robot
        tray_final_z = self.UR3_robot.process_food_order(
            tray_locs, [2.7, 1, base_z], tray_meshes, initial_z=base_z
        )
        bread_bottom_final_z = self.UR3_robot.process_food_order(
            bread_bottom_locs, [2.7, 1, base_z], bread_bottom_meshes, initial_z=tray_final_z
        )

        # Move UR3 to start position before sliding
        self.UR3_robot.move_to_start_position(
            self.UR3_robot.robot,
            self.env,
            target_pos=[2.7 + offset, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None,
        )

        # Slide tray and bread bottom meshes
        mesh_list = bread_bottom_meshes + tray_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
        self.UR3_robot.rmrc_vertical_movement(
            self.UR3_robot.robot,
            self.env,
            SE3(2.7 + offset, 1, base_z).t,
            SE3(2.45, 1, base_z).t,
            tool_orientation=[pi, 0, 0],
            mesh_list=mesh_list,
            mesh_offset_along_x=-offset,
            mesh_z_offsets=mesh_z_offsets,
        )

        # Move UR3 to position after sliding
        self.UR3_robot.move_to_start_position(
            self.UR3_robot.robot,
            self.env,
            target_pos=[2.9, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None,
        )

        # Process meats with meat robot controller
        meat_initial_z = bread_bottom_final_z + 0.02  # 2 cm gap above bread bottom
        meat_final_z = self.meat_robot_ctrl.process_food_order(
            meat_locs, [2.45 - offset / 2, 1, base_z], meat_meshes, initial_z=meat_initial_z
        )

        # Move meat robot to start position before sliding
        self.meat_robot_ctrl.move_to_start_position(
            self.meat_robot_ctrl.robot,
            self.env,
            target_pos=[2.35 + offset, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None,
        )

        # Slide meats with additional meshes
        mesh_list = meat_meshes + bread_bottom_meshes + tray_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
        self.meat_robot_ctrl.rmrc_vertical_movement(
            self.meat_robot_ctrl.robot,
            self.env,
            SE3(2.45 + offset - 0.5, 1, base_z).t,
            SE3(1.7, 1, base_z).t,
            tool_orientation=[pi, 0, 0],
            mesh_list=mesh_list,
            mesh_offset_along_x=-offset,
            mesh_z_offsets=mesh_z_offsets,
        )

        # Move meat robot after sliding
        self.meat_robot_ctrl.move_to_start_position(
            self.meat_robot_ctrl.robot,
            self.env,
            target_pos=[2.7, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None,
        )

        print("Processing veggies with IRB...")

        # Move veggie robot before sliding
        self.veggie_robot_ctrl.move_to_start_position(
            self.veggie_robot_ctrl.robot,
            self.env,
            target_pos=[1.5 - offset, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None,
        )

        # Slide meats, bread, and tray for veggie robot
        mesh_list = meat_meshes + bread_bottom_meshes + tray_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]
        self.veggie_robot_ctrl.rmrc_vertical_movement(
            self.veggie_robot_ctrl.robot,
            self.env,
            SE3(1.7 - offset, 1, base_z).t,
            SE3(1.1, 1, base_z).t,
            tool_orientation=[pi, 0, 0],
            mesh_list=mesh_list,
            mesh_offset_along_x=offset,
            mesh_z_offsets=mesh_z_offsets,
        )

        # Determine veggie initial stacking height
        if len(meat_meshes) > 0:
            top_meat_z = max(SE3(m.T).t[2] for m in meat_meshes)
            veggie_initial_z = top_meat_z + 0.01  # 1 cm above highest meat
        else:
            top_bread_z = max(SE3(m.T).t[2] for m in bread_bottom_meshes)
            veggie_initial_z = top_bread_z + 0.01  # Above bread if no meat

        veggie_final_z = self.veggie_robot_ctrl.process_food_order(
            veggie_locs, [1.1 + 1.2 * offset, 1, base_z], veggie_meshes, initial_z=veggie_initial_z
        )

        # Move veggie robot before sliding to Liam
        self.veggie_robot_ctrl.move_to_start_position(
            self.veggie_robot_ctrl.robot,
            self.env,
            target_pos=[1.1 + 2 * offset, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None,
        )

        # Slide all meshes for veggie robot
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
            mesh_z_offsets=mesh_z_offsets,
        )

        # Move veggie robot out of the way
        self.veggie_robot_ctrl.move_to_start_position(
            self.veggie_robot_ctrl.robot,
            self.env,
            target_pos=[1.2 + 2 * offset, 1, base_z],
            tool_orientation=[pi, 0, 0],
            mesh_list=None,
        )

        # Assemble stacking order for the cobot
        mesh_list = tray_meshes + bread_bottom_meshes + meat_meshes + veggie_meshes
        mesh_z_offsets = [SE3(m.T).t[2] - base_z for m in mesh_list]

        # Process bread top order with cobot
        self.cobot_ctrl.process_food_order(
            bread_top_locs,
            [1.05 - offset - 0.02, 0.75, base_z + 0.02],
            bread_top_meshes,
            tool_orientation=[0, 0, 0],
            initial_z=veggie_final_z - 0.05,
            grip_offset=SE3.Rz(pi),
        )

        # Move cobot to start position before sliding
        self.cobot_ctrl.move_to_start_position(
            self.cobot_ctrl.robot,
            self.env,
            target_pos=[1.05 - 2 * offset, 0.75, base_z],
            tool_orientation=[0, 0, 0],
            mesh_list=None,
        )

        # Combine all meshes for cobot final sliding
        all_meshes = (
            meat_meshes + veggie_meshes + bread_bottom_meshes + tray_meshes + bread_top_meshes
        )
        base_height = 1.0  # Base height for environment
        top_mesh_z = max(SE3(m.T).t[2] for m in all_meshes)
        mesh_z_offsets = [SE3(m.T).t[2] - base_height for m in all_meshes]

        # Perform final vertical movement to slide all meshes with cobot
        self.cobot_ctrl.rmrc_vertical_movement(
            self.cobot_ctrl.robot,
            self.env,
            SE3(0.55 - offset, 0.4, base_z).t,
            (SE3(0.55 - offset, -0.2, base_z) * SE3.Rz(pi / 2)).t,
            tool_orientation=[0, 0, 0],
            grip_offset=SE3.Rz(pi),
            mesh_list=all_meshes,
            mesh_z_offsets=mesh_z_offsets,
        )

    def force_collision(self):
        steps = 50
        q0 = np.zeros(6)
        q_pick = self.meat_robot_ctrl.robot.ikine_LM(SE3(2.45, 1, 0.7), q0=q0).q
        self.q_original = self.meat_robot_ctrl.robot.q.copy()

        # Create trajectory
        traj = rtb.jtraj(self.meat_robot_ctrl.robot.q, q_pick, steps).q

        print("Moving robot to force collision...")

        for q in traj:
            self.meat_robot_ctrl.robot.q = q
            self.ellipsoid_meat.ellipsoid_for_robot_links(self.meat_robot_ctrl.robot.q)

        # Check for collision at this step
            collisions = CollsionDetection.detect_collisions(self.ellipsoid_meat, self.bench_points)
            if collisions:
                print(f"Collision detected! Movement stopped.")
                self.meat_robot_ctrl.set_estop(True)  # stop robot immediately
            
                print("Resetting robot to original position...")
                # Use current q and original q for reverse trajectory
                q_current = self.meat_robot_ctrl.robot.q.copy()
                traj_back = rtb.jtraj(q_current, self.q_original, steps).q
            
                # Move robot back gradually before releasing estop
                for q in traj_back:
                    self.meat_robot_ctrl.robot.q = q
                    if hasattr(self.env, "step"):
                        self.env.step()
                    else:
                        self.meat_robot_ctrl.robot.plot(q, block=False)
                    time.sleep(0.05)
            
                # Now release estop after completion of reset motion
                self.meat_robot_ctrl.set_estop(False)
                print("Robot reset complete.")
                return True

            # Update visualization
            if hasattr(self.env, "step"):
                self.env.step()  # Swift or other env
            else:
                self.meat_robot_ctrl.robot.plot(q, block=False)
                # print("No collision detected along trajectory. Adjust joint angles to force collision.")

            time.sleep(0.05)

        print("No collision detected along trajectory. Adjust joint angles to force collision.")
        return False
    
