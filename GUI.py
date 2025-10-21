import tkinter as tk
from tkinter import ttk
import os
import threading
# from IngredientsLiam import RobotEnvironment
from NEWspawnEnv import RobotEnvironment
from NEWprocesses import processes
import matplotlib.pyplot as plt
from CollisionDetectionInEnvronment import CollsionDetection
from ir_support import EllipsoidRobot

ingredients = ["ham", "tomato", "lettuce", "salami", "beef", "chicken", "cucumber", "beetroot"]
meat_ingredients = {"ham", "salami", "beef", "chicken"}
veggie_ingredients = {"tomato", "lettuce", "cucumber", "beetroot"}


class IngredientSelector(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Ingredients Selector")

        self.ingredient_vars = {}
        self.create_widgets()

        # Initialize robot environment once
        self.robot_env = RobotEnvironment()

        # Correctly instantiate processes passing env
        self.process_handler = processes(self.robot_env)

        self.estop_active = False
        self.process_thread = None
        plt.close('all')


    def create_widgets(self):
        frame = ttk.LabelFrame(self, text="Select Ingredients")
        frame.pack(padx=10, pady=10, ipadx=10, ipady=10)

        for i, ing in enumerate(ingredients):
            var = tk.BooleanVar(value=False)
            chk = ttk.Checkbutton(frame, text=ing.capitalize(), variable=var)
            chk.grid(row=i // 2, column=i % 2, sticky=tk.W, padx=10, pady=5)
            self.ingredient_vars[ing] = var

        ttk.Button(self, text="Confirm Selection", command=self.start_process).pack(pady=5)

        self.force_collision_button = ttk.Button(self, text="Force Collision", command=self.force_collision)
        self.force_collision_button.pack(pady=5)

        self.estop_button = ttk.Button(self, text="Emergency Stop", command=self.toggle_estop)
        self.estop_button.pack(pady=5)


    # def start_process(self):
    #     if self.process_thread and self.process_thread.is_alive():
    #         print("Process is already running.")
    #         return

    #     selected = [ing for ing, var in self.ingredient_vars.items() if var.get()]
    #     meat_selection = [ing for ing in selected if ing in meat_ingredients]
    #     veggie_selection = [ing for ing in selected if ing in veggie_ingredients]
    #     bread_selection = ["bread_top", "bread_bottom"]

    #     bread_locs, bread_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.bread_piles, bread_selection)
    #     meat_locs, meat_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.piles, meat_selection)
    #     veggie_locs, veggie_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.piles, veggie_selection)

    #     # Clear estop
    #     self.set_estop_for_all(False)

    #     # Start thread
    #     self.process_thread = threading.Thread(
    #         target=self.robot_env.process_sandwich_individually,
    #         args=(meat_locs, meat_meshes, veggie_locs, veggie_meshes, bread_locs, bread_meshes),
    #         daemon=True)
    #     self.process_thread.start()

    def start_process(self):
        if self.process_thread and self.process_thread.is_alive():
            print("Process is already running.")
            return

        selected = [ing for ing, var in self.ingredient_vars.items() if var.get()]
        meat_selection = [ing for ing in selected if ing in meat_ingredients]
        veggie_selection = [ing for ing in selected if ing in veggie_ingredients]
        bread_selection = ["bread_top", "bread_bottom"]

        bread_locs, bread_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.bread_piles, bread_selection)
        meat_locs, meat_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.piles, meat_selection)
        veggie_locs, veggie_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.piles, veggie_selection)

        self.set_estop_for_all(False)

        # Use the process_handler instance method here
        self.process_thread = threading.Thread(
            target=self.process_handler.process_sandwich_individually,
            args=(meat_locs, meat_meshes, veggie_locs, veggie_meshes, bread_locs, bread_meshes),
            daemon=True)
        self.process_thread.start()

    def force_collision(self):
        if self.process_thread and self.process_thread.is_alive():
            print("Another process is running. Please wait.")
            return
        self.process_thread = threading.Thread(target=self.process_handler.force_collision, daemon=True)
        self.process_thread.start()

    def toggle_estop(self):
        self.estop_active = not self.estop_active
        self.set_estop_for_all(self.estop_active)

        # Toggle button text
        if self.estop_active:
            self.estop_button.config(text="Resume")
        else:
            self.estop_button.config(text="Emergency Stop")

    def set_estop_for_all(self, status):
        for robot_ctrl in [
            self.process_handler.meat_robot_ctrl,
            self.process_handler.veggie_robot_ctrl,
            self.process_handler.cobot_ctrl,
            self.process_handler.UR3_robot
        ]:
            robot_ctrl.set_estop(status)

    # def Collsion(robot):
    #     stl_path = os.path.join(os.path.dirname(__file__), "env", "benchv2.stl")
    #     bench_points = CollsionDetection.load_mesh_points(stl_path, num_points=8000)
    #     ellipsoid_robot = EllipsoidRobot(robot)
    #     ellipsoid_robot.ellipsoid_for_robot_links(robot.q)
    #     collisions = CollsionDetection.detect_collisions(ellipsoid_robot, bench_points)

        
