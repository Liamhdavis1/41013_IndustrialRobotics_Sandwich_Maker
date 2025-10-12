from Liams_robot.Cobot320 import Cobot320
from Lilys_robot.XArm6 import XArm6
import swift
from spatialmath import SE3
import time

def create_environment():
    # Launch Swift environment
    env = swift.Swift()
    env.launch(realtime=True)

    # Create robots
    Liam_robot = Cobot320()
    Liam_robot.base = SE3(0, 0, 0)
    Liam_robot.add_to_env(env)

    # Lily_robot = XArm6()
    # Lily_robot.base = SE3(2, 0, 0)
    # Lily_robot.add_to_env(env)

    # Micah_robot = VeggieVS068()
    # Micah_robot.base = SE3(4, 0, 0)
    # Micah_robot.add_to_env(env)

    return env, Liam_robot

env, Liam_robot = create_environment()
