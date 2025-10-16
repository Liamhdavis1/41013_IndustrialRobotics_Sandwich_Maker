from spatialmath import SE3
from math import pi
import numpy as np
import roboticstoolbox as rtb


class Sandwich:
    def __init__(self):
        pass

    def Bot1(self):
        BreadBottomPick = SE3(-1, 0.25, 1)
        BreadBottomPlace = SE3(-1, 0.25, 1)
        trajectory_pairs = [
            (BreadBottomPick, BreadBottomPlace)
        ]
        return trajectory_pairs

    def Bot2Classic(self):
        def Classic():
            lettucePick = SE3(0.25, -0.35, 0.92)
            lettucePlace = SE3(-1.01, 0.25, 1.04) @ SE3.Rz(pi / 2)
            tomatoPick = SE3(0.25, -0.35, 0.92)
            tomatoPlace = SE3(-0.98, 0.34, 1.04)
            trajectory_pairs = [
                (lettucePick, lettucePlace),
                (lettucePlace, tomatoPick),
                (tomatoPick, tomatoPlace),
            ]
            return trajectory_pairs

        def Mediteranian():
            tomatoPick = SE3(0.25, -0.35, 0.92)
            tomatoPlace = SE3(-0.98, 0.34, 1.04)
            trajectory_pairs = [
                (tomatoPick, tomatoPlace)
            ]
            return trajectory_pairs
        
        def Italian():
            tomatoPick = SE3(1.2, -0.1, 0.92)
            tomatoPlace = SE3(-1.02, 0.2, 1.03) @ SE3.Rz(pi / 2)
            trajectory_pairs = [
                (tomatoPick, tomatoPlace)
            ]
            return trajectory_pairs
        
        def BeefAndBeetroot():
            beefPick = SE3(1.2, -0.1, 0.92)
            beefPlace = SE3(-1.02, 0.2, 1.03) @ SE3.Rz(pi / 2)
            lettucePick = SE3(0.25, -0.35, 0.92)
            lettucePlace = SE3(-1.01, 0.25, 1.04) @ SE3.Rz(pi / 2)
            trajectory_pairs = [

            ]
            return trajectory_pairs
        
        def SmokeyChicken():
            chickenPick = SE3(1.2, -0.1, 0.92)
            chickenPlace = SE3(-1.02, 0.2, 1.03) @ SE3.Rz(pi / 2)
            lettucePick = SE3(0.25, -0.35, 0.92)
            lettucePlace = SE3(-1.01, 0.25, 1.04) @ SE3.Rz(pi / 2)
            trajectory_pairs = [
            ]
            return trajectory_pairs
        
        def Veggie():
            lettucePick = SE3(0.25, -0.35, 0.92)
            lettucePlace = SE3(-1.01, 0.25, 1.04) @ SE3.Rz(pi / 2)
            tomatoPick = SE3(0.25, -0.35, 0.92)
            tomatoPlace = SE3(-0.98, 0.34, 1.04)
            beetrootPick = SE3(0.25, -0.35, 0.92)
            beetrootPlace = SE3(-1.01, 0.25, 1.04) @ SE3.Rz(pi / 2)
            trajectory_pairs = [
                (lettucePick, lettucePlace),
                (lettucePlace, tomatoPick),
                (tomatoPick, tomatoPlace),
                (tomatoPlace, beetrootPick),
                (beetrootPick, beetrootPlace),
            ]
            return trajectory_pairs

        return {
            "Classic": Classic(),
            "Mediteranian": Mediteranian(),
            "Italian": Italian(),
            "Beef & Beetroot": BeefAndBeetroot(),
            "Smokey Chicken": SmokeyChicken(),
            "Veggie": Veggie()
        }

    def Bot3(self):
        def Classic():
            
            return trajectory_pairs

        def Mediteranian():
            
            return trajectory_pairs
        
        def Italian():
            
            return trajectory_pairs
        
        def BeefAndBeetroot():
            
            return trajectory_pairs
        
        def SmokeyChicken():
            
            return trajectory_pairs
        
        def Veggie():
            
            return trajectory_pairs

        return {
            "Classic": Classic(),
            "Mediteranian": Mediteranian(),
            "Italian": Italian(),
            "Beef & Beetroot": BeefAndBeetroot(),
            "Smokey Chicken": SmokeyChicken(),
            "Veggie": Veggie()
        }

    def Bot4(self):
        BreadTopPick = SE3(-0.7, 0.25, 1.0)
        BreadTopPlace = SE3(-0.7, 0.25, 1.0)
        trajectory_pairs = [
            (BreadTopPick, BreadTopPlace)
        ]
        return trajectory_pairs

    def Make_Sandwich(self, robot, trajectory_pairs):
        for pick_pose, place_pose in trajectory_pairs:
            gripper_down_orientation = SE3.Rx(pi)

            safe_pose_above_pick = pick_pose * gripper_down_orientation
            safe_pose_above_place = place_pose * gripper_down_orientation

            pick_pose_above_brick = pick_pose * gripper_down_orientation
            place_pose_above_brick = place_pose * gripper_down_orientation

            q0_pick = np.array([0, 0, 0, pi / 2, 0, 0, 0])
            q0_place = np.array([0, 0, 0, pi / 2, 0, 0, 0])

            sol_safe_pick = robot.ikine_LM(safe_pose_above_pick, q0=q0_pick)
            sol_safe_place = robot.ikine_LM(safe_pose_above_place, q0=q0_place)

            q_safe_pick = np.clip(sol_safe_pick.q, robot.qlim[0], robot.qlim[1])
            q_safe_place = np.clip(sol_safe_place.q, robot.qlim[0], robot.qlim[1])

            sol_pick = robot.ikine_LM(pick_pose_above_brick, q0=q_safe_pick)
            sol_place = robot.ikine_LM(place_pose_above_brick, q0=q_safe_place)

            q_pick = np.clip(sol_pick.q, robot.qlim[0], robot.qlim[1])
            q_place = np.clip(sol_place.q, robot.qlim[0], robot.qlim[1])

            trajectory_segments = [
                (q_safe_pick, q_pick),
                (q_pick, q_safe_pick),
                (q_safe_pick, q_safe_place),
                (q_safe_place, q_place),
                (q_place, q_safe_place)
            ]

            for q_start, q_end in trajectory_segments:
                traj = rtb.jtraj(q_start, q_end, 50)
                for q in traj.q:
                    q_clipped = np.clip(q, robot.qlim[0], robot.qlim[1])
                    robot.q = q_clipped


if __name__ == "__main__":
    class DummyRobot:
        def __init__(self):
            self.n = 7
            self._q = np.zeros(self.n)
            self.qlim = (np.array([-np.pi]*self.n), np.array([np.pi]*self.n))  # Joint limits

        @property
        def q(self):
            return self._q

        @q.setter
        def q(self, q_vals):
            self._q = q_vals
            print(f"Robot joint positions updated: {q_vals}")

        def ikine_LM(self, pose, q0=None):
            # Dummy IK solver - replace with your own
            class Result:
                def __init__(self, q):
                    self.q = q
            return Result(np.zeros(self.n))

    robot = DummyRobot()
    sandwich = Sandwich()

    # Example usage for Bot1
    traj1 = sandwich.Bot1()
    sandwich.Make_Sandwich(robot, traj1)

    # Example usage for Bot2Classic with choice
    all_bot2_trajs = sandwich.Bot2Classic()
    # Choose 'Classic'
    traj2 = all_bot2_trajs["Classic"]
    sandwich.Make_Sandwich(robot, traj2)

    # Choose 'Mediteranian'
    traj2_med = all_bot2_trajs["Mediteranian"]
    sandwich.Make_Sandwich(robot, traj2_med)

    # Other bots
    traj3 = sandwich.Bot3Classic()
    sandwich.Make_Sandwich(robot, traj3)

    traj4 = sandwich.Bot4()
    sandwich.Make_Sandwich(robot, traj4)
from spatialmath import SE3
from math import pi
import numpy as np
import roboticstoolbox as rtb


class Sandwich:
    def __init__(self):
        pass

    def Bot1(self):
        BreadBottomPick = SE3(-1, 0.25, 1)
        BreadBottomPlace = SE3(-1, 0.25, 1)
        trajectory_pairs = [
            (BreadBottomPick, BreadBottomPlace)
        ]
        return trajectory_pairs

    def Bot2Classic(self):
        def Classic():
            lettucePick = SE3(0.25, -0.35, 0.92)
            lettucePlace = SE3(-1.01, 0.25, 1.04) @ SE3.Rz(pi / 2)
            tomatoPick = SE3(0.25, -0.35, 0.92)
            tomatoPlace = SE3(-0.98, 0.34, 1.04)
            trajectory_pairs = [
                (lettucePick, lettucePlace),
                (lettucePlace, tomatoPick),
                (tomatoPick, tomatoPlace),
            ]
            return trajectory_pairs

        def Mediteranian():
            tomatoPick = SE3(0.25, -0.35, 0.92)
            tomatoPlace = SE3(-0.98, 0.34, 1.04)
            trajectory_pairs = [
                (tomatoPick, tomatoPlace)
            ]
            return trajectory_pairs

        return {
            "Classic": Classic(),
            "Mediteranian": Mediteranian()
        }

    def Bot3Classic(self):
        hamPick = SE3(1.2, -0.35, 0.92)
        hamPlace = SE3(-1.03, 0.14, 1.01)
        trajectory_pairs = [
            (hamPick, hamPlace)
        ]
        return trajectory_pairs

    def Bot4(self):
        BreadTopPick = SE3(-0.7, 0.25, 1.0)
        BreadTopPlace = SE3(-0.7, 0.25, 1.0)
        trajectory_pairs = [
            (BreadTopPick, BreadTopPlace)
        ]
        return trajectory_pairs

    def Make_Sandwich(self, robot, trajectory_pairs):
        for pick_pose, place_pose in trajectory_pairs:
            gripper_down_orientation = SE3.Rx(pi)

            safe_pose_above_pick = pick_pose * gripper_down_orientation
            safe_pose_above_place = place_pose * gripper_down_orientation

            pick_pose_above_brick = pick_pose * gripper_down_orientation
            place_pose_above_brick = place_pose * gripper_down_orientation

            q0_pick = np.array([0, 0, 0, pi / 2, 0, 0, 0])
            q0_place = np.array([0, 0, 0, pi / 2, 0, 0, 0])

            sol_safe_pick = robot.ikine_LM(safe_pose_above_pick, q0=q0_pick)
            sol_safe_place = robot.ikine_LM(safe_pose_above_place, q0=q0_place)

            q_safe_pick = np.clip(sol_safe_pick.q, robot.qlim[0], robot.qlim[1])
            q_safe_place = np.clip(sol_safe_place.q, robot.qlim[0], robot.qlim[1])

            sol_pick = robot.ikine_LM(pick_pose_above_brick, q0=q_safe_pick)
            sol_place = robot.ikine_LM(place_pose_above_brick, q0=q_safe_place)

            q_pick = np.clip(sol_pick.q, robot.qlim[0], robot.qlim[1])
            q_place = np.clip(sol_place.q, robot.qlim[0], robot.qlim[1])

            trajectory_segments = [
                (q_safe_pick, q_pick),
                (q_pick, q_safe_pick),
                (q_safe_pick, q_safe_place),
                (q_safe_place, q_place),
                (q_place, q_safe_place)
            ]

            for q_start, q_end in trajectory_segments:
                traj = rtb.jtraj(q_start, q_end, 50)
                for q in traj.q:
                    q_clipped = np.clip(q, robot.qlim[0], robot.qlim[1])
                    robot.q = q_clipped


if __name__ == "__main__":
    class DummyRobot:
        def __init__(self):
            self.n = 7
            self._q = np.zeros(self.n)
            self.qlim = (np.array([-np.pi]*self.n), np.array([np.pi]*self.n))  # Joint limits

        @property
        def q(self):
            return self._q

        @q.setter
        def q(self, q_vals):
            self._q = q_vals
            print(f"Robot joint positions updated: {q_vals}")

        def ikine_LM(self, pose, q0=None):
            # Dummy IK solver - replace with your own
            class Result:
                def __init__(self, q):
                    self.q = q
            return Result(np.zeros(self.n))

    robot = DummyRobot()
    sandwich = Sandwich()

    # Example usage for Bot1
    traj1 = sandwich.Bot1()
    sandwich.Make_Sandwich(robot, traj1)

    # Example usage for Bot2Classic with choice
    all_bot2_trajs = sandwich.Bot2Classic()
    # Choose 'Classic'
    traj2 = all_bot2_trajs["Classic"]
    sandwich.Make_Sandwich(robot, traj2)

    # Choose 'Mediteranian'
    traj2_med = all_bot2_trajs["Mediteranian"]
    sandwich.Make_Sandwich(robot, traj2_med)

    # Other bots
    traj3 = sandwich.Bot3Classic()
    sandwich.Make_Sandwich(robot, traj3)

    traj4 = sandwich.Bot4()
    sandwich.Make_Sandwich(robot, traj4)
