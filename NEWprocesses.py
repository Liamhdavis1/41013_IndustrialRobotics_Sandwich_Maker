from math import pi
from spatialmath import SE3
from NEWspawnEnv import RobotEnvironment
from NEWcollisionDetection import CollsionDetection
from ir_support import EllipsoidRobot
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

        self.ellipsoid_meat = EllipsoidRobot(self.meat_robot_ctrl.robot)
        self.ellipsoid_veggie = EllipsoidRobot(self.veggie_robot_ctrl.robot)
        self.ellipsoid_cobot = EllipsoidRobot(self.cobot_ctrl.robot)
        self.ellipsoid_UR3 = EllipsoidRobot(self.UR3_robot.robot)

    def process_sandwich_individually(self, meat_locs, meat_meshes, veggie_locs, veggie_meshes, bread_locs, bread_meshes):
        print("Processing meats with XArm6...")
        tray_mesh = spawn_ingredient(self.env, "tray", SE3(1.3, 0.45, 1) * SE3(*ENV_OFFSET))

        bread = self.env_instance.spawn_bread(self.env)
        bread_locs, bread_meshes = self.env_instance.collect_bread_locations_and_meshes(bread)
        self.UR3_robot.bread_movement(bread_locs, [3, 0.8, 1.0], bread_meshes)

        placed_meshes = bread_meshes + [tray_mesh]
        start_pos, end_pos = SE3(3, 0.6, 1).t, SE3(2.45, 0.8, 1).t

        self.meat_robot_ctrl.rmrc_vertical_movement(
            self.meat_robot_ctrl.robot,
            self.env, start_pos, end_pos,
            tool_orientation=[pi, 0, 0],
            mesh_list=placed_meshes
        )

        self.meat_robot_ctrl.process_food_order(meat_locs, [2.45, 1, 1], meat_meshes)
        self.meat_robot_ctrl.rmrc_vertical_movement(self.meat_robot_ctrl.robot,
            self.env,
            SE3(2.45, 1, 1).t, SE3(1.75, 0.88, 1).t,
            tool_orientation=[pi,0,0],
            mesh_list=meat_meshes + bread_meshes + [tray_mesh])

        print("Processing veggies with IRB...")
        self.veggie_robot_ctrl.process_food_order(veggie_locs, [1.5, 1, 1], veggie_meshes)
        self.veggie_robot_ctrl.rmrc_vertical_movement(self.veggie_robot_ctrl.robot,
            self.env,
            SE3(1.5, 1.0, 1).t, SE3(1.3, 1.0, 1).t,
            tool_orientation=[pi,0,0],
            mesh_list=meat_meshes + veggie_meshes + bread_meshes + [tray_mesh])

        print("Processing with Cobot320...")
        self.cobot_ctrl.other_ik_solver(
            pick_pose=SE3(1.3, 1.0, 1.1),
            place_pose=SE3(0.9, 0.55, 1.1),
            mesh=meat_meshes + veggie_meshes,
            gripper_down_orientation_pick=None,
            gripper_down_orientation_place=None
        )


    def force_collision(self):
        # Simple forced collision with bench by forcing joint configuration into collision pose
        collision_q = self.meat_robot_ctrl.robot.q.copy()
        # Example values that likely cause collision (tune for your robot)
        collision_q[0] += 1.7
        collision_q[1] += 1.7

        self.meat_robot_ctrl.robot.q = collision_q
        self.ellipsoid_meat.ellipsoid_for_robot_links(collision_q)

        if CollsionDetection.detect_collisions(self.ellipsoid_meat, self.bench_points):
            print("⚠️ Collision successfully forced for meat robot.")
            return True
        else:
            print("❌ Failed to force collision. Adjust joint angles.")
            return False

