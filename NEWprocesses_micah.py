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

    def process_sandwich_individually(self, meat_locs, meat_meshes, veggie_locs, veggie_meshes, bread_locs, bread_meshes):
        print("Processing meats with XArm6...")
        offset = 0.28
        tray_mesh = spawn_ingredient(self.env, "tray", SE3(2.5, 1, 1))

        bread = self.env_instance.spawn_bread(self.env)
        bread_locs, bread_meshes = self.env_instance.collect_bread_locations_and_meshes(bread)
        
        
        self.UR3_robot.bread_movement(bread_locs, [2.5, 1, 1], bread_meshes)

        placed_meshes = bread_meshes + [tray_mesh]
        
        # Process food order
        self.meat_robot_ctrl.process_food_order(meat_locs, [2.5, 1, 1], meat_meshes)
        # Move to position before sliding
        success = self.meat_robot_ctrl.move_to_start_position(
        self.meat_robot_ctrl.robot,
        self.env,
        target_pos=[2.5 + offset, 1, 1],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )
        # Slide order
        self.meat_robot_ctrl.rmrc_vertical_movement(self.meat_robot_ctrl.robot,
            self.env,
            SE3(2.45, 1, 1).t, SE3(1.75, 1, 1).t,
            tool_orientation=[pi,0,0],
            mesh_list=meat_meshes + bread_meshes + [tray_mesh])
        
        # Move to position after sliding
        success = self.meat_robot_ctrl.move_to_start_position(
        self.meat_robot_ctrl.robot,
        self.env,
        target_pos=[2.5, 1, 1],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )

        print("Processing veggies with IRB...")


        self.veggie_robot_ctrl.process_food_order(veggie_locs, [1.5, 1, 1], veggie_meshes)
        
        # Move to position before sliding
        success = self.veggie_robot_ctrl.move_to_start_position(
        self.veggie_robot_ctrl.robot,
        self.env,
        target_pos=[1.5 + offset, 1, 1],
        tool_orientation=[pi, 0, 0],
        mesh_list=None
    )
        
        self.veggie_robot_ctrl.rmrc_vertical_movement(self.veggie_robot_ctrl.robot,
            self.env,
            SE3(1.5, 1.0, 1).t, SE3(1.3, 1.0, 1).t,
            tool_orientation=[pi,0,0],
            mesh_list=meat_meshes + veggie_meshes + bread_meshes + [tray_mesh])

        # print("Processing with Cobot320...")
        # self.cobot_ctrl.other_ik_solver(
        #     pick_pose=SE3(1.3, 1.0, 1.1),
        #     place_pose=SE3(0.9, 0.55, 1.1),
        #     mesh=meat_meshes + veggie_meshes + bread_meshes + [tray_mesh],
        #     gripper_down_orientation_pick=None,
        #     gripper_down_orientation_place=None
        # )


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
