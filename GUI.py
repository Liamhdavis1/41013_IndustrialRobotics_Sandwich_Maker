import tkinter as tk
from tkinter import ttk
from sandwich import Sandwich  # Import the Sandwich class

def ingredient_clicked(Sandwiches):
    print("Button clicked:", Sandwiches)
    # Use the bread object to make sandwich based on choice
    bread.Make_Sandwich(Sandwiches)

root = tk.Tk()
root.title("Ingredients Selector")

sandwich_frame = ttk.LabelFrame(root, text="What sandwich would you like?")
sandwich_frame.pack(pady=10, ipadx=10, padx=10)

Sandwiches = ["Classic", "Mediteranian", "Italian", "Beef & Beetroot", "Smokey Chicken", "Veggie"]

bread = Sandwich()

for i, ing in enumerate(Sandwiches):
    btn = ttk.Button(sandwich_frame, text=ing, width=20,
                     command=lambda ing=ing: ingredient_clicked(ing))
    btn.grid(row=i//2, column=i%2, sticky=tk.W, padx=10, pady=5)

root.mainloop()
