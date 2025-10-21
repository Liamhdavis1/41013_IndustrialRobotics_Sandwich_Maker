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

    def process_sandwich_individually(self, meat_locs, meat_meshes, veggie_locs, veggie_meshes, bread_locs, bread_meshes, tray_locs, tray_meshes):
        self.UR3_robot.process_food_order(tray_locs, [3, 1, 1], tray_meshes)
        # self.UR3_robot.process_food_order(bread_locs, [3, 1, 1], bread_meshes)

        

        # self.meat_robot_ctrl.rmrc_vertical_movement(
        #     self.meat_robot_ctrl.robot,
        #     self.env, start_pos, end_pos,
        #     tool_orientation=[pi, 0, 0],
        #     mesh_list=bread_meshes + tray_meshes
        # )

        self.meat_robot_ctrl.process_food_order(meat_locs, [2.45, 1, 1], meat_meshes)
        self.meat_robot_ctrl.rmrc_vertical_movement(self.meat_robot_ctrl.robot,
            self.env,
            SE3(2.45, 1, 1).t, SE3(1.75, 0.88, 1).t,
            tool_orientation=[pi,0,0],
            mesh_list=meat_meshes + bread_meshes + tray_meshes)

        print("Processing veggies with IRB...")
        self.veggie_robot_ctrl.process_food_order(veggie_locs, [1.5, 1, 1], veggie_meshes)
        self.veggie_robot_ctrl.rmrc_vertical_movement(self.veggie_robot_ctrl.robot,
            self.env,
            SE3(1.5, 1.0, 1).t, SE3(1.3, 1.0, 1).t,
            tool_orientation=[pi,0,0],
            mesh_list=meat_meshes + veggie_meshes + bread_meshes + tray_meshes)

        print("Processing with Cobot320...")
        self.cobot_ctrl.other_ik_solver(
            pick_pose=SE3(1.3, 1.0, 1.1),
            place_pose=SE3(0.9, 0.55, 1.1),
            mesh=meat_meshes + veggie_meshes + bread_meshes + tray_meshes,
            gripper_down_orientation_pick=None,
            gripper_down_orientation_place=None
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
                print(f"⚠️ Collision detected! Movement stopped.")
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
                # print("❌ No collision detected along trajectory. Adjust joint angles to force collision.")

            time.sleep(0.05)

        print("❌ No collision detected along trajectory. Adjust joint angles to force collision.")
        return False
    
    # def reset_robot(self):
    #     steps = 50
    #     traj_back = rtb.jtraj(self.meat_robot_ctrl.robot.q, self.q_original, steps).q
    #     print("Resetting robot to original position...")
    #     for q in traj_back:
    #         self.meat_robot_ctrl.robot.q = q
    #         self.env.draw(self.meat_robot_ctrl.robot)
    #         time.sleep(0.05)

    #     print("✅ Robot reset complete.")
