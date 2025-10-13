import tkinter as tk
from tkinter import ttk

root = tk.Tk()
root.title("Ingredients Selector")

ingredients_frame = ttk.LabelFrame(root, text="Sandwich Ingredients")
ingredients_frame.pack(pady=10, ipadx=10, padx=10)

ingredients = ["ham", "lettuce", "tomato", "salami", "beef", "chicken", "cucumber", "beetroot"]
ingredient_vars = {}

for i, ing in enumerate(ingredients):
    var = tk.IntVar()
    chk = ttk.Checkbutton(ingredients_frame, text=ing, variable=var)
    chk.grid(row=i//2, column=i%2, sticky=tk.W, padx=10, pady=5)
    ingredient_vars[ing] = var

make_sandwich = ttk.Button(root, text="Make Sandwich")
make_sandwich.pack(pady=10, ipadx=10, padx=10)

# plain_text_label = ttk.Label(root, text="Select your ingredients and build your sandwich!")
# plain_text_label.pack(pady=5)

root.mainloop()
