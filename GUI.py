import tkinter as tk
from tkinter import ttk
from IngredientsLiam import RobotEnvironment  # Import the environment module

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

    def create_widgets(self):
        frame = ttk.LabelFrame(self, text="Select Ingredients")
        frame.pack(padx=10, pady=10, ipadx=10, ipady=10)

        for i, ing in enumerate(ingredients):
            var = tk.BooleanVar(value=False)
            chk = ttk.Checkbutton(frame, text=ing.capitalize(), variable=var)
            chk.grid(row=i // 2, column=i % 2, sticky=tk.W, padx=10, pady=5)
            self.ingredient_vars[ing] = var

        ttk.Button(self, text="Confirm Selection", command=self.ingredient_selected).pack(pady=5)

    def ingredient_selected(self):
        selected = [ing for ing, var in self.ingredient_vars.items() if var.get()]
        meat_selection = [ing for ing in selected if ing in meat_ingredients]
        veggie_selection = [ing for ing in selected if ing in veggie_ingredients]
        bread_selection = ["bread_top", "bread_bottom"]

        bread_locs, bread_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.bread_piles, bread_selection)
        meat_locs, meat_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.piles, meat_selection)
        veggie_locs, veggie_meshes = self.robot_env.collect_ingredient_locations(self.robot_env.piles, veggie_selection)

        self.robot_env.process_sandwich_individually(meat_locs, meat_meshes, veggie_locs, veggie_meshes, bread_locs, bread_meshes)
